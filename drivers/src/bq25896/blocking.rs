use super::*;
use alloc::{format, string::String};
use embedded_hal::i2c::I2c;
use libm::{log, round};

/// Blocking (sync) implementation of the BQ25896 driver.
///
/// See <https://github.com/Xinyuan-LilyGO/LilyGo-AMOLED-Series/blob/master/libdeps/XPowersLib/src/PowersBQ25896.tpp>
impl<I2C> BQ25896<I2C>
where
    I2C: I2c,
{
    // ---- I2C helpers ----

    fn read_register(&mut self, register: u8) -> Result<u8, PmuSensorError> {
        let mut buffer = [0u8];
        self.i2c.write_read(self.adr, &[register], &mut buffer)?;
        Ok(buffer[0])
    }

    fn write_register(&mut self, register_and_data: &[u8]) -> Result<(), PmuSensorError> {
        self.i2c.write(self.adr, register_and_data)?;
        Ok(())
    }

    fn set_register_bit(&mut self, register: u8, bit: u8) -> Result<(), PmuSensorError> {
        let val = self.read_register(register)?;
        self.write_register(&[register, val | (1 << bit)])
    }

    fn get_register_bit(&mut self, register: u8, bit: u8) -> Result<bool, PmuSensorError> {
        let val = self.read_register(register)?;
        Ok((val & (1 << bit)) != 0)
    }

    fn clear_register_bit(&mut self, register: u8, bit: u8) -> Result<(), PmuSensorError> {
        let val = self.read_register(register)?;
        self.write_register(&[register, val & !(1 << bit)])
    }

    // ---- Public API ----

    /// Probes the device to verify communication.
    pub fn init(&mut self) -> Result<(), PmuSensorError> {
        self.i2c
            .write(self.adr, &[self.adr])
            .map_err(|_| PmuSensorError::Init)
    }

    // Register 0x00: Enable HIZ Mode, Enable ILIM Pin, Input Current Limit

    pub fn set_hiz_mode(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x00, 7)
    }

    pub fn exit_hiz_mode(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x00, 7)
    }

    pub fn is_hiz_mode(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x00, 7)
    }

    pub fn enable_current_limit_pin(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x00, 6)
    }

    pub fn disable_current_limit_pin(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x00, 6)
    }

    pub fn is_enable_current_limit_pin(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x00, 6)
    }

    pub fn set_input_current_limit(&mut self, milliampere: u16) -> Result<(), PmuSensorError> {
        if !milliampere.is_multiple_of(IN_CURRENT_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid50);
        }
        let ma = milliampere.clamp(IN_CURRENT_MIN, IN_CURRENT_MAX);
        let mut reg_val = self.read_register(0x00)?;
        reg_val &= 0xC0;
        reg_val |= ((ma - IN_CURRENT_MIN) / IN_CURRENT_STEP) as u8;
        self.write_register(&[0x00, reg_val])
    }

    pub fn get_input_current_limit(&mut self) -> Result<u16, PmuSensorError> {
        let reg_val = self.read_register(0x00)?;
        Ok((reg_val & 0x3F) as u16 * IN_CURRENT_STEP + IN_CURRENT_MIN)
    }

    // Register 0x01: Boost Hot/Cold Temp Threshold, Input Voltage Limit Offset

    pub fn set_boost_mode_hot_temp_threshold(
        &mut self,
        threshold: BoostHotThreshold,
    ) -> Result<(), PmuSensorError> {
        let val = self.read_register(0x01)?;
        self.write_register(&[0x01, (val & 0x3F) | ((threshold as u8) << 6)])
    }

    pub fn set_boost_mode_cold_temp_threshold(
        &mut self,
        threshold: BoostColdThreshold,
    ) -> Result<(), PmuSensorError> {
        let val = self.read_register(0x01)?;
        self.write_register(&[0x01, (val & 0xDF) | ((threshold as u8) << 5)])
    }

    pub fn set_input_voltage_limit_offset(
        &mut self,
        mut millivolt: u16,
    ) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(IN_CURRENT_OFFSET_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid100);
        }
        if millivolt > IN_CURRENT_OFFSET_MAX {
            millivolt = IN_CURRENT_OFFSET_MAX;
        }
        let steps = millivolt / IN_CURRENT_OFFSET_STEP;
        let val = self.read_register(0x01)?;
        self.write_register(&[0x01, (val & 0xE0) | steps as u8])
    }

    // Register 0x02: ADC, Boost Freq, ICO, Input Detection

    pub fn set_adc_enabled(&mut self) -> Result<(), PmuSensorError> {
        let mut data = self.read_register(0x02)?;
        data |= 1 << 7;
        data |= 1 << 6;
        self.write_register(&[0x02, data])
    }

    pub fn set_adc_disabled(&mut self) -> Result<(), PmuSensorError> {
        let mut data = self.read_register(0x02)?;
        data &= !(1 << 7);
        self.write_register(&[0x02, data])
    }

    pub fn set_boost_freq(&mut self, freq: BoostFreq) -> Result<(), PmuSensorError> {
        match freq {
            BoostFreq::Freq500KHz => self.set_register_bit(0x02, 5),
            BoostFreq::Freq1500KHz => self.clear_register_bit(0x02, 5),
        }
    }

    pub fn get_boost_freq(&mut self) -> Result<BoostFreq, PmuSensorError> {
        Ok(if self.get_register_bit(0x02, 5)? {
            BoostFreq::Freq500KHz
        } else {
            BoostFreq::Freq1500KHz
        })
    }

    pub fn enable_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x02, 4)
    }

    pub fn disable_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x02, 4)
    }

    pub fn enable_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x02, 1)
    }

    pub fn disable_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x02, 1)
    }

    pub fn is_input_detection_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x02, 1)
    }

    pub fn force_dpdm_detection(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x02, 1)
    }

    pub fn enable_automatic_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x02, 0)
    }

    pub fn disable_automatic_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x02, 0)
    }

    pub fn is_automatic_input_detection_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x02, 0)
    }

    // Register 0x03: Battery Load, Watchdog Reset, OTG, Charge Enable, SYS Voltage, Boost Exit

    pub fn is_bat_load_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x03, 7)
    }

    pub fn disable_bat_load(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x03, 7)
    }

    pub fn enable_bat_load(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x03, 7)
    }

    pub fn feed_watchdog(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x03, 6)
    }

    pub fn is_otg_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x03, 5)
    }

    pub fn disable_otg(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x03, 5)?;
        if !self.user_disable_charge {
            self.set_register_bit(0x03, 4)?;
        }
        Ok(())
    }

    pub fn enable_otg(&mut self) -> Result<bool, PmuSensorError> {
        if self.is_vbus_in()? {
            return Ok(false);
        }
        self.set_register_bit(0x03, 5)?;
        Ok(true)
    }

    pub fn set_charge_enabled(&mut self) -> Result<(), PmuSensorError> {
        self.user_disable_charge = false;
        self.set_register_bit(0x03, 4)
    }

    pub fn set_charge_disabled(&mut self) -> Result<(), PmuSensorError> {
        self.user_disable_charge = true;
        self.clear_register_bit(0x03, 4)
    }

    pub fn is_charge_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x03, 4)
    }

    pub fn set_sys_power_down_voltage(&mut self, millivolt: u16) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(SYS_VOL_STEPS) {
            return Err(PmuSensorError::VoltageStepInvalid100);
        }
        if !(SYS_VOFF_VOL_MIN..=SYS_VOFF_VOL_MAX).contains(&millivolt) {
            return Err(PmuSensorError::PowerDownVoltageInvalid);
        }
        let mut val = self.read_register(0x03)?;
        val &= 0xF1;
        val |= ((millivolt - SYS_VOFF_VOL_MIN) / SYS_VOL_STEPS) as u8;
        val <<= 1;
        self.write_register(&[0x03, val])
    }

    pub fn get_sys_power_down_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x03)?;
        Ok(((val & 0x0E) >> 1) as u16 * SYS_VOL_STEPS + SYS_VOFF_VOL_MIN)
    }

    pub fn set_exit_boost_mode_voltage(
        &mut self,
        voltage: ExitBoostModeVolt,
    ) -> Result<(), PmuSensorError> {
        match voltage {
            ExitBoostModeVolt::MiniVolt2V9 => self.clear_register_bit(0x03, 0),
            ExitBoostModeVolt::MiniVolt2V5 => self.set_register_bit(0x03, 0),
        }
    }

    // Register 0x04: Current Pulse Control, Fast Charge Current Limit

    pub fn set_current_pulse_control_enabled(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x04, 7)
    }

    pub fn set_current_pulse_control_disabled(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x04, 7)
    }

    pub fn set_fast_charge_current_limit(
        &mut self,
        milliampere: u16,
    ) -> Result<(), PmuSensorError> {
        if !milliampere.is_multiple_of(FAST_CHG_CUR_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid64);
        }
        let current = milliampere.min(FAST_CHG_CURRENT_MAX);
        let mut val = self.read_register(0x04)?;
        val &= 0x80;
        val |= (current / FAST_CHG_CUR_STEP) as u8;
        self.write_register(&[0x04, val])
    }

    pub fn get_fast_charge_current_limit(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x04)?;
        Ok((val & 0x7F) as u16 * FAST_CHG_CUR_STEP)
    }

    // Register 0x05: Precharge Current Limit, Termination Current Limit

    pub fn set_precharge_current(&mut self, milliampere: u16) -> Result<(), PmuSensorError> {
        if !milliampere.is_multiple_of(PRE_CHG_CUR_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid64);
        }
        let current = milliampere.clamp(PRE_CHG_CURRENT_MIN, PRE_CHG_CURRENT_MAX);
        let mut val = self.read_register(0x05)?;
        val &= 0x0F;
        val |= (((current - PRE_CHG_CUR_BASE) / PRE_CHG_CUR_STEP) as u8) << 4;
        self.write_register(&[0x05, val])
    }

    pub fn get_precharge_current(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x05)?;
        Ok(PRE_CHG_CUR_STEP + ((val & 0xF0) >> 4) as u16 * PRE_CHG_CUR_STEP)
    }

    pub fn set_termination_current(&mut self, milliampere: u16) -> Result<(), PmuSensorError> {
        if !milliampere.is_multiple_of(TERM_CHG_CUR_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid64);
        }
        let current = milliampere.clamp(TERM_CHG_CURRENT_MIN, TERM_CHG_CURRENT_MAX);
        let mut val = self.read_register(0x05)?;
        val &= 0xF0;
        val |= ((current - TERM_CHG_CUR_BASE) / TERM_CHG_CUR_STEP) as u8;
        self.write_register(&[0x05, val])
    }

    pub fn get_termination_current(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x05)?;
        Ok(TERM_CHG_CUR_STEP + (val & 0x0F) as u16 * TERM_CHG_CUR_STEP)
    }

    // Register 0x06: Charge Voltage Limit, Fast Charge Threshold, Recharge Threshold

    pub fn set_charge_target_voltage(&mut self, target_voltage: u16) -> Result<(), PmuSensorError> {
        if !target_voltage.is_multiple_of(CHG_VOL_STEP) {
            return Err(PmuSensorError::VoltageStepInvalid16);
        }
        let voltage = target_voltage.clamp(FAST_CHG_VOL_MIN, FAST_CHG_VOL_MAX);
        let mut val = self.read_register(0x06)?;
        val &= 0x03;
        val |= (((voltage - CHG_VOL_BASE) / CHG_VOL_STEP) << 2) as u8;
        self.write_register(&[0x06, val])
    }

    pub fn get_charge_target_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x06)?;
        let bits = (val & 0xFC) >> 2;
        if bits > 0x30 {
            return Ok(FAST_CHG_VOL_MAX);
        }
        Ok(CHG_VOL_BASE + bits as u16 * CHG_VOL_STEP)
    }

    pub fn set_fast_charge_threshold(
        &mut self,
        threshold: FastChargeThreshold,
    ) -> Result<(), PmuSensorError> {
        match threshold {
            FastChargeThreshold::Volt2V8 => self.clear_register_bit(0x06, 1),
            FastChargeThreshold::Volt3V0 => self.set_register_bit(0x06, 1),
        }
    }

    pub fn set_battery_recharge_threshold_offset(
        &mut self,
        offset: RechargeThresholdOffset,
    ) -> Result<(), PmuSensorError> {
        match offset {
            RechargeThresholdOffset::Offset100mV => self.clear_register_bit(0x06, 0),
            RechargeThresholdOffset::Offset200mV => self.set_register_bit(0x06, 0),
        }
    }

    // Register 0x07: Charging Termination, STAT Pin, Watchdog, Safety Timer, Fast Charge Timer, JEITA

    pub fn enable_charging_termination(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x07, 7)
    }

    pub fn disable_charging_termination(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x07, 7)
    }

    pub fn is_charging_termination_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x07, 7)
    }

    pub fn disable_stat_pin(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x07, 6)
    }

    pub fn enable_stat_pin(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x07, 6)
    }

    pub fn is_stat_pin_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x07, 6)
    }

    pub fn disable_watchdog(&mut self) -> Result<(), PmuSensorError> {
        let mut val = self.read_register(0x07)?;
        val &= 0xCF;
        self.write_register(&[0x07, val])
    }

    pub fn enable_watchdog(&mut self, config: WatchdogConfig) -> Result<(), PmuSensorError> {
        let mut val = self.read_register(0x07)?;
        val &= 0xCF;
        let bits: u8 = (config as u8 & 0x03) << 4;
        self.write_register(&[0x07, val | bits])
    }

    pub fn disable_charging_safety_timer(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x07, 3)
    }

    pub fn enable_charging_safety_timer(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x07, 3)
    }

    pub fn is_charging_safety_timer_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x07, 3)
    }

    pub fn set_fast_charge_timer(&mut self, timer: FastChargeTimer) -> Result<(), PmuSensorError> {
        let timer: u8 = timer.into();
        let mut val = self.read_register(0x07)?;
        val &= 0xF9;
        val |= (timer & 0x03) << 1;
        self.write_register(&[0x07, val])
    }

    pub fn get_fast_charge_timer(&mut self) -> Result<FastChargeTimer, PmuSensorError> {
        let val = self.read_register(0x07)?;
        Ok(FastChargeTimer::try_from((val >> 1) & 0x03).expect("Invalid timer value"))
    }

    pub fn set_jeita_low_temperature_current(
        &mut self,
        current: JeitaLowTemperatureCurrent,
    ) -> Result<(), PmuSensorError> {
        match current {
            JeitaLowTemperatureCurrent::Temp50 => self.clear_register_bit(0x07, 0),
            JeitaLowTemperatureCurrent::Temp20 => self.set_register_bit(0x07, 0),
        }
    }

    // Register 0x08: IR Compensation, Thermal Regulation Threshold

    pub fn set_ir_compensation_resistor(&mut self, milliohm: u16) -> Result<(), PmuSensorError> {
        if !milliohm.is_multiple_of(BAT_COMP_STEPS) {
            return Err(PmuSensorError::ResistanceStepInvalid20);
        }
        let resistance = milliohm.clamp(0, BAT_COMP_MAX);
        let mut val = self.read_register(0x08)?;
        val &= 0x1F;
        val |= ((resistance / BAT_COMP_STEPS) as u8) << 5;
        self.write_register(&[0x08, val])
    }

    pub fn set_ir_compensation_voltage_clamp(
        &mut self,
        millivolt: u16,
    ) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(VCLAMP_STEPS) {
            return Err(PmuSensorError::VoltageStepInvalid32);
        }
        let voltage = millivolt.clamp(0, VCLAMP_MAX);
        let mut val = self.read_register(0x08)?;
        val &= 0xE3;
        val |= ((voltage / VCLAMP_STEPS) as u8) << 2;
        self.write_register(&[0x08, val])
    }

    pub fn set_thermal_regulation_threshold(
        &mut self,
        threshold: ThermalRegThreshold,
    ) -> Result<(), PmuSensorError> {
        let threshold: u8 = threshold.into();
        let mut val = self.read_register(0x08)?;
        val &= 0xFC;
        val |= threshold;
        self.write_register(&[0x08, val])
    }

    // Register 0x09: ICO, Safety Timer, BATFET, JEITA High Temp, Various controls

    pub fn force_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x09, 7)
    }

    pub fn set_safety_timer_thermal_regulation(
        &mut self,
        slow_down: bool,
    ) -> Result<(), PmuSensorError> {
        if slow_down {
            self.set_register_bit(0x09, 6)
        } else {
            self.clear_register_bit(0x09, 6)
        }
    }

    pub fn shutdown(&mut self) -> Result<(), PmuSensorError> {
        self.disable_battery_power_path()
    }

    pub fn disable_battery_power_path(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x09, 5)
    }

    pub fn enable_battery_power_path(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x09, 5)
    }

    pub fn set_jeita_high_temp_voltage(&mut self, use_vreg: bool) -> Result<(), PmuSensorError> {
        if use_vreg {
            self.set_register_bit(0x09, 4)
        } else {
            self.clear_register_bit(0x09, 4)
        }
    }

    pub fn set_batfet_turnoff_delay(&mut self, delay: bool) -> Result<(), PmuSensorError> {
        if delay {
            self.set_register_bit(0x09, 3)
        } else {
            self.clear_register_bit(0x09, 3)
        }
    }

    pub fn set_full_system_reset(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.set_register_bit(0x09, 2)
        } else {
            self.clear_register_bit(0x09, 2)
        }
    }

    pub fn set_current_pulse_voltage_up(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.set_register_bit(0x09, 1)
        } else {
            self.clear_register_bit(0x09, 1)
        }
    }

    pub fn set_current_pulse_voltage_down(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.set_register_bit(0x09, 0)
        } else {
            self.clear_register_bit(0x09, 0)
        }
    }

    // Register 0x0A: Boost Mode Voltage, PFM, Boost Current Limit

    pub fn set_boost_voltage(&mut self, mut millivolt: u16) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(BOOST_VOL_STEP) {
            return Err(PmuSensorError::VoltageStepInvalid64);
        }
        millivolt = millivolt.clamp(BOOST_VOL_MIN, BOOST_VOL_MAX);
        let val = self.read_register(0x0A)?;
        let steps = ((millivolt - BOOST_VOL_BASE) / BOOST_VOL_STEP) as u8;
        self.write_register(&[0x0A, (val & 0xF0) | (steps << 4)])
    }

    pub fn set_boost_current_limit(
        &mut self,
        limit: BoostCurrentLimit,
    ) -> Result<(), PmuSensorError> {
        let limit: u8 = limit.into();
        let val = self.read_register(0x0A)?;
        self.write_register(&[0x0A, (val & 0x03) | limit])
    }

    pub fn set_boost_mode_pfm(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.clear_register_bit(0x0A, 3)
        } else {
            self.set_register_bit(0x0A, 3)
        }
    }

    // Register 0x0B: VBUS Status, Charge Status, Power Good, VSYS Regulation

    pub fn get_charge_status(&mut self) -> Result<ChargeStatus, PmuSensorError> {
        let val = self.read_register(0x0B)?;
        Ok(ChargeStatus::try_from((val >> 3) & 0x03).expect("Invalid charge status"))
    }

    pub fn is_vbus_in(&mut self) -> Result<bool, PmuSensorError> {
        Ok(self.get_bus_status()? != BusStatus::NoInput)
    }

    pub fn is_otg(&mut self) -> Result<bool, PmuSensorError> {
        Ok(self.get_bus_status()? == BusStatus::Otg)
    }

    pub fn is_charging(&mut self) -> Result<bool, PmuSensorError> {
        Ok(self.get_charge_status()? != ChargeStatus::NoCharge)
    }

    pub fn is_charge_done(&mut self) -> Result<bool, PmuSensorError> {
        Ok(self.get_charge_status()? == ChargeStatus::Done)
    }

    pub fn is_power_good(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x0B, 2)
    }

    pub fn get_bus_status(&mut self) -> Result<BusStatus, PmuSensorError> {
        let val = self.read_register(0x0B)?;
        Ok(BusStatus::try_from((val >> 5) & 0x07).expect("Invalid bus status"))
    }

    // Register 0x0C: Fault Status

    pub fn is_watchdog_fault(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.read_register(0x0C)?;
        Ok(((val >> 7) & 0x01) != 0)
    }

    pub fn is_boost_fault(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.read_register(0x0C)?;
        Ok(((val >> 6) & 0x01) != 0)
    }

    pub fn get_charge_fault(&mut self) -> Result<ChargeFaultStatus, PmuSensorError> {
        let val = self.read_register(0x0C)?;
        Ok(ChargeFaultStatus::try_from((val >> 4) & 0x03).expect("Invalid charge fault status"))
    }

    pub fn is_battery_fault(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.read_register(0x0C)?;
        Ok(((val >> 3) & 0x01) != 0)
    }

    pub fn is_ntc_fault(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.read_register(0x0C)?;
        Ok((val & 0x07) != 0)
    }

    pub fn get_ntc_status_string(&mut self) -> Result<&'static str, PmuSensorError> {
        let status = self.read_register(0x0C)? & 0x07;
        if self.is_otg()? {
            match status {
                x if x == NtcBoostStatus::Normal as u8 => Ok("Boost mode NTC normal"),
                x if x == NtcBoostStatus::Cold as u8 => Ok("Boost mode NTC cold"),
                x if x == NtcBoostStatus::Hot as u8 => Ok("Boost mode NTC hot"),
                _ => Ok("Unknown"),
            }
        } else {
            match status {
                x if x == NtcBuckStatus::Normal as u8 => Ok("Buck mode NTC normal"),
                x if x == NtcBuckStatus::Warm as u8 => Ok("Buck mode NTC warm"),
                x if x == NtcBuckStatus::Cold as u8 || x == NtcBuckStatus::ColdAlt as u8 => {
                    Ok("Buck mode NTC cold")
                }
                x if x == NtcBuckStatus::Hot as u8 => Ok("Buck mode NTC hot"),
                _ => Ok("Unknown"),
            }
        }
    }

    // Register 0x0D: VINDPM Threshold

    pub fn set_vindpm_threshold_method(&mut self, relative: bool) -> Result<(), PmuSensorError> {
        if relative {
            self.clear_register_bit(0x0D, 7)
        } else {
            self.set_register_bit(0x0D, 7)
        }
    }

    pub fn set_vindpm_threshold(&mut self, mut millivolt: u16) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(VINDPM_VOL_STEPS) {
            return Err(PmuSensorError::VoltageStepInvalid100);
        }
        millivolt = millivolt.clamp(VINDPM_VOL_MIN, VINDPM_VOL_MAX);
        let val = self.read_register(0x0D)?;
        let steps = ((millivolt - VINDPM_VOL_BASE) / VINDPM_VOL_STEPS) as u8;
        self.write_register(&[0x0D, (val & 0x80) | steps])
    }

    pub fn get_vindpm_threshold(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x0D)?;
        Ok((val & 0x7F) as u16 * VINDPM_VOL_STEPS + VINDPM_VOL_BASE)
    }

    // Register 0x0E: Battery Voltage ADC

    pub fn is_thermal_regulation_normal(&mut self) -> Result<bool, PmuSensorError> {
        Ok(!self.get_register_bit(0x0E, 7)?)
    }

    pub fn get_battery_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let data = self.read_register(0x0E)?;
        if data == 0 || ((data >> 7) & 0x01) != 0 {
            return Ok(0);
        }
        Ok((data & 0x7F) as u16 * 20 + 2304)
    }

    pub fn get_battery_percentage(&mut self) -> Result<u8, PmuSensorError> {
        let voltage = self.get_battery_voltage()?;
        let percentage = if voltage >= 4200 {
            100
        } else if voltage >= 4000 {
            90 + (voltage - 4000) * 10 / 200
        } else if voltage >= 3800 {
            60 + (voltage - 3800) * 30 / 200
        } else if voltage >= 3600 {
            30 + (voltage - 3600) * 30 / 200
        } else if voltage >= 3400 {
            10 + (voltage - 3400) * 20 / 200
        } else if voltage >= 3000 {
            (voltage - 3000) * 10 / 400
        } else {
            0
        };
        Ok(percentage.min(100) as u8)
    }

    // Register 0x0F: System Voltage ADC

    pub fn get_sys_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let data = self.read_register(0x0F)? & 0x7F;
        if data == 0 {
            return Ok(0);
        }
        Ok(data as u16 * 20 + 2304)
    }

    // Register 0x10: TS Voltage (NTC thermistor)

    pub fn get_temperature(&mut self) -> Result<f64, PmuSensorError> {
        let data = self.read_register(0x10)? & 0x7F;
        let ntc_percent = data as f64 * 0.465_f64 + 21_f64;
        let r_ratio = (100.0 - ntc_percent) / ntc_percent;

        fn r_to_temp(r: f64) -> f64 {
            const BETA: f64 = 3950.0;
            const T0: f64 = 298.15;
            const R0: f64 = 10000.0;
            let r_ntc = r * R0;
            let temp_kelvin = 1.0 / (1.0 / T0 + (1.0 / BETA) * log(r_ntc / R0));
            round((temp_kelvin - 273.15) * 2.0) / 2.0
        }

        Ok(r_to_temp(r_ratio))
    }

    // Register 0x11: VBUS Voltage ADC

    pub fn is_vbus_good(&mut self) -> Result<bool, PmuSensorError> {
        let data = self.read_register(0x11)?;
        Ok(((data >> 7) & 0x01) != 0)
    }

    pub fn get_vbus_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let data = self.read_register(0x11)?;
        if ((data >> 7) & 0x01) == 0 {
            return Ok(0);
        }
        Ok((data & 0x7F) as u16 * 100 + 2600)
    }

    // Register 0x12: Charge Current ADC

    pub fn get_charge_current(&mut self) -> Result<u16, PmuSensorError> {
        if self.get_charge_status()? == ChargeStatus::NoCharge {
            return Ok(0);
        }
        let val = self.read_register(0x12)?;
        Ok((val & 0x7F) as u16 * CHG_STEP_VAL)
    }

    // Register 0x13: VINDPM/IINDPM Status, ICO Current Limit

    pub fn is_dynamic_power_management(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x13, 7)
    }

    pub fn is_input_current_limit(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x13, 6)
    }

    pub fn set_input_current_limit_optimizer(
        &mut self,
        mut milliampere: u16,
    ) -> Result<(), PmuSensorError> {
        if !milliampere.is_multiple_of(IN_CURRENT_OPT_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid50);
        }
        milliampere = milliampere.clamp(IN_CURRENT_OPT_MIN, IN_CURRENT_OPT_MAX);
        let val = self.read_register(0x13)?;
        let steps = ((milliampere - IN_CURRENT_OPT_MIN) / IN_CURRENT_OPT_STEP) as u8;
        self.write_register(&[0x13, (val & 0x3F) | (steps << 6)])
    }

    pub fn get_input_current_limit_in_effect(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x13)?;
        Ok((val & 0x3F) as u16 * IN_CURRENT_OPT_STEP + IN_CURRENT_OPT_MIN)
    }

    // Register 0x14: Register Reset, ICO Status, Device Config, Temp Profile, Device Revision

    pub fn reset_default(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x14, 7)
    }

    pub fn is_input_current_optimizer(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x14, 6)
    }

    pub fn get_device_config(&mut self) -> Result<u8, PmuSensorError> {
        let val = self.read_register(0x14)?;
        Ok((val >> 3) & 0x03)
    }

    pub fn get_temperature_profile(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.read_register(0x14)?;
        Ok(((val >> 2) & 0x01) != 0)
    }

    pub fn get_chip_id(&mut self) -> Result<u8, PmuSensorError> {
        Ok(self.read_register(0x14)? & 0x03)
    }

    pub fn get_info(&mut self) -> Result<String, PmuSensorError> {
        let is_vbus_present = if self.is_vbus_in()? { "Yes" } else { "No" };
        let mut text = format!("CHG state: {}\n", self.get_charge_status()?);
        text.push_str(&format!("USB PlugIn: {is_vbus_present}\n"));
        text.push_str(&format!("Bus state: {}\n", self.get_bus_status()?));
        let battery_voltage = self.get_battery_voltage()?;
        let battery_percentage = self.get_battery_percentage()?;
        text.push_str(&format!(
            "Battery: {}mV ({}%)\n",
            battery_voltage, battery_percentage
        ));
        text.push_str(&format!(
            "Charge current: {}mA\n",
            self.get_charge_current()?
        ));
        text.push_str(&format!("Temperature: {}°C\n", self.get_temperature()?));
        text.push_str(&format!(
            "Charger fast charge curr.: {}mA\n",
            self.get_fast_charge_current_limit()?
        ));
        text.push_str(&format!(
            "Charger target voltage: {}mV\n",
            self.get_charge_target_voltage()?
        ));
        text.push_str(&format!(
            "Input curr. limit: {}mA\n",
            self.get_input_current_limit()?
        ));
        text.push_str(&format!("USB voltage: {}mV\n", self.get_vbus_voltage()?));
        text.push_str(&format!("SYS voltage: {}mV\n", self.get_sys_voltage()?));
        Ok(text)
    }
}
