use super::*;
use alloc::{format, string::String};
use embedded_hal_async::i2c::I2c;
use libm::{log, round};

/// Async implementation of the BQ25896 driver.
///
/// See <https://github.com/Xinyuan-LilyGO/LilyGo-AMOLED-Series/blob/master/libdeps/XPowersLib/src/PowersBQ25896.tpp>
impl<I2C> BQ25896<I2C>
where
    I2C: I2c,
{
    // ---- async I2C helpers ----

    async fn read_register(&mut self, register: u8) -> Result<u8, PmuSensorError> {
        let mut buffer = [0u8];
        self.i2c
            .write_read(self.adr, &[register], &mut buffer)
            .await?;
        Ok(buffer[0])
    }

    async fn write_register(&mut self, register_and_data: &[u8]) -> Result<(), PmuSensorError> {
        self.i2c.write(self.adr, register_and_data).await?;
        Ok(())
    }

    async fn set_register_bit(&mut self, register: u8, bit: u8) -> Result<(), PmuSensorError> {
        let val = self.read_register(register).await?;
        self.write_register(&[register, val | (1 << bit)]).await
    }

    async fn get_register_bit(&mut self, register: u8, bit: u8) -> Result<bool, PmuSensorError> {
        let val = self.read_register(register).await?;
        Ok((val & (1 << bit)) != 0)
    }

    async fn clear_register_bit(&mut self, register: u8, bit: u8) -> Result<(), PmuSensorError> {
        let val = self.read_register(register).await?;
        self.write_register(&[register, val & !(1 << bit)]).await
    }

    // ---- Public API ----

    /// Probes the device to verify I2C communication is working.
    pub async fn init(&mut self) -> Result<(), PmuSensorError> {
        self.i2c
            .write(self.adr, &[self.adr])
            .await
            .map_err(|_| PmuSensorError::Init)
    }

    // Register 0x00: Enable HIZ Mode, Enable ILIM Pin, Input Current Limit

    /// Put the input into high-impedance mode, disabling all input power paths.
    pub async fn set_hiz_mode(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x00, 7).await
    }

    /// Exit high-impedance mode and restore normal input power paths.
    pub async fn exit_hiz_mode(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x00, 7).await
    }

    /// Returns `true` when the device is in high-impedance (HIZ) mode.
    pub async fn is_hiz_mode(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x00, 7).await
    }

    /// Allow the ILIM pin to set the input current limit in hardware.
    pub async fn enable_current_limit_pin(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x00, 6).await
    }

    /// Ignore the ILIM pin; use the register-programmed current limit instead.
    pub async fn disable_current_limit_pin(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x00, 6).await
    }

    /// Returns `true` when the ILIM pin is controlling the input current limit.
    pub async fn is_enable_current_limit_pin(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x00, 6).await
    }

    /// Set the maximum input current from the power source.
    ///
    /// `milliampere` must be a multiple of 50 mA in the range 100–3250 mA.
    pub async fn set_input_current_limit(
        &mut self,
        milliampere: u16,
    ) -> Result<(), PmuSensorError> {
        if !milliampere.is_multiple_of(IN_CURRENT_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid50);
        }
        let ma = milliampere.clamp(IN_CURRENT_MIN, IN_CURRENT_MAX);
        let mut reg_val = self.read_register(0x00).await?;
        reg_val &= 0xC0;
        reg_val |= ((ma - IN_CURRENT_MIN) / IN_CURRENT_STEP) as u8;
        self.write_register(&[0x00, reg_val]).await
    }

    /// Read the programmed input current limit in mA (50 mA resolution, 100–3250 mA range).
    pub async fn get_input_current_limit(&mut self) -> Result<u16, PmuSensorError> {
        let reg_val = self.read_register(0x00).await?;
        Ok((reg_val & 0x3F) as u16 * IN_CURRENT_STEP + IN_CURRENT_MIN)
    }

    // Register 0x01: Boost Hot/Cold Temp Threshold, Input Voltage Limit Offset

    /// Set the NTC threshold that triggers over-temperature protection in boost mode.
    pub async fn set_boost_mode_hot_temp_threshold(
        &mut self,
        threshold: BoostHotThreshold,
    ) -> Result<(), PmuSensorError> {
        let val = self.read_register(0x01).await?;
        self.write_register(&[0x01, (val & 0x3F) | ((threshold as u8) << 6)])
            .await
    }

    /// Set the NTC threshold that triggers under-temperature protection in boost mode.
    pub async fn set_boost_mode_cold_temp_threshold(
        &mut self,
        threshold: BoostColdThreshold,
    ) -> Result<(), PmuSensorError> {
        let val = self.read_register(0x01).await?;
        self.write_register(&[0x01, (val & 0xDF) | ((threshold as u8) << 5)])
            .await
    }

    /// Set the input voltage limit as an offset above the VBUS threshold (VINDPM).
    ///
    /// `millivolt` must be a multiple of 100 mV and at most 3100 mV.
    pub async fn set_input_voltage_limit_offset(
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
        let val = self.read_register(0x01).await?;
        self.write_register(&[0x01, (val & 0xE0) | steps as u8])
            .await
    }

    // Register 0x02: ADC, Boost Freq, ICO, Input Detection

    /// Enable the ADC for continuous conversion of battery/system/input voltages and currents.
    pub async fn set_adc_enabled(&mut self) -> Result<(), PmuSensorError> {
        let mut data = self.read_register(0x02).await?;
        data |= 1 << 7;
        data |= 1 << 6;
        self.write_register(&[0x02, data]).await
    }

    /// Disable the ADC to reduce quiescent current in low-power modes.
    pub async fn set_adc_disabled(&mut self) -> Result<(), PmuSensorError> {
        let mut data = self.read_register(0x02).await?;
        data &= !(1 << 7);
        self.write_register(&[0x02, data]).await
    }

    /// Set the boost-mode switching frequency (500 kHz or 1500 kHz).
    pub async fn set_boost_freq(&mut self, freq: BoostFreq) -> Result<(), PmuSensorError> {
        match freq {
            BoostFreq::Freq500KHz => self.set_register_bit(0x02, 5).await,
            BoostFreq::Freq1500KHz => self.clear_register_bit(0x02, 5).await,
        }
    }

    /// Read the current boost-mode switching frequency.
    pub async fn get_boost_freq(&mut self) -> Result<BoostFreq, PmuSensorError> {
        Ok(if self.get_register_bit(0x02, 5).await? {
            BoostFreq::Freq500KHz
        } else {
            BoostFreq::Freq1500KHz
        })
    }

    /// Enable the Input Current Optimizer (ICO) to automatically find the best input current.
    pub async fn enable_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x02, 4).await
    }

    /// Disable the Input Current Optimizer.
    pub async fn disable_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x02, 4).await
    }

    /// Enable D+/D− input detection for USB charger type identification.
    pub async fn enable_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x02, 1).await
    }

    /// Disable D+/D− input detection.
    pub async fn disable_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x02, 1).await
    }

    /// Returns `true` when D+/D− input detection is enabled.
    pub async fn is_input_detection_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x02, 1).await
    }

    /// Force an immediate D+/D− detection cycle regardless of whether VBUS just appeared.
    pub async fn force_dpdm_detection(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x02, 1).await
    }

    /// Enable automatic D+/D− detection every time VBUS is applied.
    pub async fn enable_automatic_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x02, 0).await
    }

    /// Disable automatic D+/D− detection on VBUS application.
    pub async fn disable_automatic_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x02, 0).await
    }

    /// Returns `true` when automatic input detection runs on every VBUS connection.
    pub async fn is_automatic_input_detection_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x02, 0).await
    }

    // Register 0x03: Battery Load, Watchdog Reset, OTG, Charge Enable, SYS Voltage, Boost Exit

    /// Returns `true` when the battery test-load MOSFET is enabled.
    pub async fn is_bat_load_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x03, 7).await
    }

    /// Disable the internal battery test-load path.
    pub async fn disable_bat_load(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x03, 7).await
    }

    /// Enable the internal battery test-load path.
    pub async fn enable_bat_load(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x03, 7).await
    }

    /// Reset the I2C watchdog timer to prevent a watchdog-triggered reset.
    pub async fn feed_watchdog(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x03, 6).await
    }

    /// Returns `true` when OTG (USB host / boost) mode is active.
    pub async fn is_otg_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x03, 5).await
    }

    /// Disable OTG boost mode and re-enable charging if not user-disabled.
    pub async fn disable_otg(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x03, 5).await?;
        if !self.user_disable_charge {
            self.set_register_bit(0x03, 4).await?;
        }
        Ok(())
    }

    /// Enable OTG boost mode to power an external USB device.
    ///
    /// Returns `false` (and does not enable OTG) when VBUS power is already present.
    pub async fn enable_otg(&mut self) -> Result<bool, PmuSensorError> {
        if self.is_vbus_in().await? {
            return Ok(false);
        }
        self.set_register_bit(0x03, 5).await?;
        Ok(true)
    }

    /// Enable battery charging and clear the user-disabled flag.
    pub async fn set_charge_enabled(&mut self) -> Result<(), PmuSensorError> {
        self.user_disable_charge = false;
        self.set_register_bit(0x03, 4).await
    }

    /// Disable battery charging and set the user-disabled flag (persists across OTG transitions).
    pub async fn set_charge_disabled(&mut self) -> Result<(), PmuSensorError> {
        self.user_disable_charge = true;
        self.clear_register_bit(0x03, 4).await
    }

    /// Returns `true` when battery charging is enabled.
    pub async fn is_charge_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x03, 4).await
    }

    /// Set the minimum system voltage (VSYSMIN) below which the SYS rail is regulated.
    ///
    /// `millivolt` must be a multiple of 100 mV in the range 3000–3700 mV.
    pub async fn set_sys_power_down_voltage(
        &mut self,
        millivolt: u16,
    ) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(SYS_VOL_STEPS) {
            return Err(PmuSensorError::VoltageStepInvalid100);
        }
        if !(SYS_VOFF_VOL_MIN..=SYS_VOFF_VOL_MAX).contains(&millivolt) {
            return Err(PmuSensorError::PowerDownVoltageInvalid);
        }
        let mut val = self.read_register(0x03).await?;
        val &= 0xF1;
        val |= ((millivolt - SYS_VOFF_VOL_MIN) / SYS_VOL_STEPS) as u8;
        val <<= 1;
        self.write_register(&[0x03, val]).await
    }

    /// Read the programmed minimum system voltage in mV (100 mV resolution, 3000–3700 mV range).
    pub async fn get_sys_power_down_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x03).await?;
        Ok(((val & 0x0E) >> 1) as u16 * SYS_VOL_STEPS + SYS_VOFF_VOL_MIN)
    }

    /// Set the minimum VBUS voltage to exit boost mode (2.9 V or 2.5 V).
    pub async fn set_exit_boost_mode_voltage(
        &mut self,
        voltage: ExitBoostModeVolt,
    ) -> Result<(), PmuSensorError> {
        match voltage {
            ExitBoostModeVolt::MiniVolt2V9 => self.clear_register_bit(0x03, 0).await,
            ExitBoostModeVolt::MiniVolt2V5 => self.set_register_bit(0x03, 0).await,
        }
    }

    // Register 0x04: Current Pulse Control, Fast Charge Current Limit

    /// Enable current-pulse voltage-up/down control (PE charging protocol).
    pub async fn set_current_pulse_control_enabled(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x04, 7).await
    }

    /// Disable current-pulse voltage control.
    pub async fn set_current_pulse_control_disabled(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x04, 7).await
    }

    /// Set the fast-charge (CC phase) current limit.
    ///
    /// `milliampere` must be a multiple of 64 mA, clamped to 0–3008 mA.
    pub async fn set_fast_charge_current_limit(
        &mut self,
        milliampere: u16,
    ) -> Result<(), PmuSensorError> {
        if !milliampere.is_multiple_of(FAST_CHG_CUR_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid64);
        }
        let current = milliampere.min(FAST_CHG_CURRENT_MAX);
        let mut val = self.read_register(0x04).await?;
        val &= 0x80;
        val |= (current / FAST_CHG_CUR_STEP) as u8;
        self.write_register(&[0x04, val]).await
    }

    /// Read the programmed fast-charge current limit in mA (64 mA resolution, 0–3008 mA range).
    pub async fn get_fast_charge_current_limit(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x04).await?;
        Ok((val & 0x7F) as u16 * FAST_CHG_CUR_STEP)
    }

    // Register 0x05: Precharge Current Limit, Termination Current Limit

    /// Set the precharge (trickle) current applied when the battery is below the fast-charge threshold.
    ///
    /// `milliampere` must be a multiple of 64 mA in the range 64–1024 mA.
    pub async fn set_precharge_current(&mut self, milliampere: u16) -> Result<(), PmuSensorError> {
        if !milliampere.is_multiple_of(PRE_CHG_CUR_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid64);
        }
        let current = milliampere.clamp(PRE_CHG_CURRENT_MIN, PRE_CHG_CURRENT_MAX);
        let mut val = self.read_register(0x05).await?;
        val &= 0x0F;
        val |= (((current - PRE_CHG_CUR_BASE) / PRE_CHG_CUR_STEP) as u8) << 4;
        self.write_register(&[0x05, val]).await
    }

    /// Read the programmed precharge current in mA (64 mA resolution, 64–1024 mA range).
    pub async fn get_precharge_current(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x05).await?;
        Ok(PRE_CHG_CUR_STEP + ((val & 0xF0) >> 4) as u16 * PRE_CHG_CUR_STEP)
    }

    /// Set the charge termination current (charge done threshold in CV phase).
    ///
    /// `milliampere` must be a multiple of 64 mA in the range 64–1024 mA.
    pub async fn set_termination_current(
        &mut self,
        milliampere: u16,
    ) -> Result<(), PmuSensorError> {
        if !milliampere.is_multiple_of(TERM_CHG_CUR_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid64);
        }
        let current = milliampere.clamp(TERM_CHG_CURRENT_MIN, TERM_CHG_CURRENT_MAX);
        let mut val = self.read_register(0x05).await?;
        val &= 0xF0;
        val |= ((current - TERM_CHG_CUR_BASE) / TERM_CHG_CUR_STEP) as u8;
        self.write_register(&[0x05, val]).await
    }

    /// Read the programmed termination current in mA (64 mA resolution, 64–1024 mA range).
    pub async fn get_termination_current(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x05).await?;
        Ok(TERM_CHG_CUR_STEP + (val & 0x0F) as u16 * TERM_CHG_CUR_STEP)
    }

    // Register 0x06: Charge Voltage Limit, Fast Charge Threshold, Recharge Threshold

    /// Set the charge target (CV phase) voltage.
    ///
    /// `target_voltage` must be a multiple of 16 mV in the range 3840–4608 mV.
    pub async fn set_charge_target_voltage(
        &mut self,
        target_voltage: u16,
    ) -> Result<(), PmuSensorError> {
        if !target_voltage.is_multiple_of(CHG_VOL_STEP) {
            return Err(PmuSensorError::VoltageStepInvalid16);
        }
        let voltage = target_voltage.clamp(FAST_CHG_VOL_MIN, FAST_CHG_VOL_MAX);
        let mut val = self.read_register(0x06).await?;
        val &= 0x03;
        val |= (((voltage - CHG_VOL_BASE) / CHG_VOL_STEP) << 2) as u8;
        self.write_register(&[0x06, val]).await
    }

    /// Read the programmed charge target voltage in mV (16 mV resolution, 3840–4608 mV range).
    pub async fn get_charge_target_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x06).await?;
        let bits = (val & 0xFC) >> 2;
        if bits > 0x30 {
            return Ok(FAST_CHG_VOL_MAX);
        }
        Ok(CHG_VOL_BASE + bits as u16 * CHG_VOL_STEP)
    }

    /// Set the minimum battery voltage to enter fast-charge (CC) phase (2.8 V or 3.0 V).
    pub async fn set_fast_charge_threshold(
        &mut self,
        threshold: FastChargeThreshold,
    ) -> Result<(), PmuSensorError> {
        match threshold {
            FastChargeThreshold::Volt2V8 => self.clear_register_bit(0x06, 1).await,
            FastChargeThreshold::Volt3V0 => self.set_register_bit(0x06, 1).await,
        }
    }

    /// Set how far the battery voltage must drop below the target before recharging begins.
    ///
    /// Select a 100 mV or 200 mV offset below the charge target voltage.
    pub async fn set_battery_recharge_threshold_offset(
        &mut self,
        offset: RechargeThresholdOffset,
    ) -> Result<(), PmuSensorError> {
        match offset {
            RechargeThresholdOffset::Offset100mV => self.clear_register_bit(0x06, 0).await,
            RechargeThresholdOffset::Offset200mV => self.set_register_bit(0x06, 0).await,
        }
    }

    // Register 0x07: Charging Termination, STAT Pin, Watchdog, Safety Timer, Fast Charge Timer, JEITA

    /// Enable charge termination so charging stops when the current drops to the termination threshold.
    pub async fn enable_charging_termination(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x07, 7).await
    }

    /// Disable charge termination (the charger will not stop based on current alone).
    pub async fn disable_charging_termination(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x07, 7).await
    }

    /// Returns `true` when charge termination is enabled.
    pub async fn is_charging_termination_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x07, 7).await
    }

    /// Disable the STAT output pin (pin goes high-impedance).
    pub async fn disable_stat_pin(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x07, 6).await
    }

    /// Enable the STAT output pin to indicate charging/fault status.
    pub async fn enable_stat_pin(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x07, 6).await
    }

    /// Returns `true` when the STAT pin is enabled (not disabled).
    pub async fn is_stat_pin_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x07, 6).await
    }

    /// Disable the I2C watchdog timer so the charger will not reset without periodic writes.
    pub async fn disable_watchdog(&mut self) -> Result<(), PmuSensorError> {
        let mut val = self.read_register(0x07).await?;
        val &= 0xCF;
        self.write_register(&[0x07, val]).await
    }

    /// Enable the I2C watchdog with the selected timeout (40, 80, or 160 seconds).
    pub async fn enable_watchdog(&mut self, config: WatchdogConfig) -> Result<(), PmuSensorError> {
        let mut val = self.read_register(0x07).await?;
        val &= 0xCF;
        let bits: u8 = (config as u8 & 0x03) << 4;
        self.write_register(&[0x07, val | bits]).await
    }

    /// Disable the charging safety timer (allows unlimited charge duration).
    pub async fn disable_charging_safety_timer(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x07, 3).await
    }

    /// Enable the charging safety timer to automatically stop charging after the programmed duration.
    pub async fn enable_charging_safety_timer(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x07, 3).await
    }

    /// Returns `true` when the charging safety timer is enabled.
    pub async fn is_charging_safety_timer_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x07, 3).await
    }

    /// Set the maximum fast-charge duration (5, 8, 12, or 20 hours).
    pub async fn set_fast_charge_timer(
        &mut self,
        timer: FastChargeTimer,
    ) -> Result<(), PmuSensorError> {
        let timer: u8 = timer.into();
        let mut val = self.read_register(0x07).await?;
        val &= 0xF9;
        val |= (timer & 0x03) << 1;
        self.write_register(&[0x07, val]).await
    }

    /// Read the programmed fast-charge safety timer value.
    pub async fn get_fast_charge_timer(&mut self) -> Result<FastChargeTimer, PmuSensorError> {
        let val = self.read_register(0x07).await?;
        Ok(FastChargeTimer::try_from((val >> 1) & 0x03).expect("Invalid timer value"))
    }

    /// Set the fast-charge current level used in the low-temperature JEITA region.
    ///
    /// `Temp50` = 50% of ICHG; `Temp20` = 20% of ICHG.
    pub async fn set_jeita_low_temperature_current(
        &mut self,
        current: JeitaLowTemperatureCurrent,
    ) -> Result<(), PmuSensorError> {
        match current {
            JeitaLowTemperatureCurrent::Temp50 => self.clear_register_bit(0x07, 0).await,
            JeitaLowTemperatureCurrent::Temp20 => self.set_register_bit(0x07, 0).await,
        }
    }

    // Register 0x08: IR Compensation, Thermal Regulation Threshold

    /// Set the battery pack IR compensation resistor value.
    ///
    /// `milliohm` must be a multiple of 20 mΩ, clamped to 0–140 mΩ.
    pub async fn set_ir_compensation_resistor(
        &mut self,
        milliohm: u16,
    ) -> Result<(), PmuSensorError> {
        if !milliohm.is_multiple_of(BAT_COMP_STEPS) {
            return Err(PmuSensorError::ResistanceStepInvalid20);
        }
        let resistance = milliohm.clamp(0, BAT_COMP_MAX);
        let mut val = self.read_register(0x08).await?;
        val &= 0x1F;
        val |= ((resistance / BAT_COMP_STEPS) as u8) << 5;
        self.write_register(&[0x08, val]).await
    }

    /// Set the IR compensation voltage clamp (maximum IR-compensation voltage added to VREG).
    ///
    /// `millivolt` must be a multiple of 32 mV, clamped to 0–224 mV.
    pub async fn set_ir_compensation_voltage_clamp(
        &mut self,
        millivolt: u16,
    ) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(VCLAMP_STEPS) {
            return Err(PmuSensorError::VoltageStepInvalid32);
        }
        let voltage = millivolt.clamp(0, VCLAMP_MAX);
        let mut val = self.read_register(0x08).await?;
        val &= 0xE3;
        val |= ((voltage / VCLAMP_STEPS) as u8) << 2;
        self.write_register(&[0x08, val]).await
    }

    /// Set the die temperature threshold at which the charger reduces current (thermal regulation).
    ///
    /// Choose 60 °C, 80 °C, 100 °C, or 120 °C.
    pub async fn set_thermal_regulation_threshold(
        &mut self,
        threshold: ThermalRegThreshold,
    ) -> Result<(), PmuSensorError> {
        let threshold: u8 = threshold.into();
        let mut val = self.read_register(0x08).await?;
        val &= 0xFC;
        val |= threshold;
        self.write_register(&[0x08, val]).await
    }

    // Register 0x09: ICO, Safety Timer, BATFET, JEITA High Temp, Various controls

    /// Force an immediate ICO (Input Current Optimizer) algorithm run.
    pub async fn force_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x09, 7).await
    }

    /// Control whether the safety timer is slowed to 2× during thermal regulation.
    ///
    /// `slow_down = true` halves the timer rate when thermal regulation is active.
    pub async fn set_safety_timer_thermal_regulation(
        &mut self,
        slow_down: bool,
    ) -> Result<(), PmuSensorError> {
        if slow_down {
            self.set_register_bit(0x09, 6).await
        } else {
            self.clear_register_bit(0x09, 6).await
        }
    }

    /// Shut down the device by disabling the battery-to-system power path.
    pub async fn shutdown(&mut self) -> Result<(), PmuSensorError> {
        self.disable_battery_power_path().await
    }

    /// Disconnect the battery from the system (BATFET off). Prevents battery from powering SYS.
    pub async fn disable_battery_power_path(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x09, 5).await
    }

    /// Reconnect the battery to the system (BATFET on).
    pub async fn enable_battery_power_path(&mut self) -> Result<(), PmuSensorError> {
        self.clear_register_bit(0x09, 5).await
    }

    /// Control the JEITA high-temperature charge voltage.
    ///
    /// `use_vreg = true` uses VREG; `false` uses VREG minus 200 mV.
    pub async fn set_jeita_high_temp_voltage(
        &mut self,
        use_vreg: bool,
    ) -> Result<(), PmuSensorError> {
        if use_vreg {
            self.set_register_bit(0x09, 4).await
        } else {
            self.clear_register_bit(0x09, 4).await
        }
    }

    /// Control the BATFET turn-off delay.
    ///
    /// `delay = true` adds a ~10 ms delay before BATFET opens; `false` opens immediately.
    pub async fn set_batfet_turnoff_delay(&mut self, delay: bool) -> Result<(), PmuSensorError> {
        if delay {
            self.set_register_bit(0x09, 3).await
        } else {
            self.clear_register_bit(0x09, 3).await
        }
    }

    /// Trigger a full system reset (equivalent to a POR). `enable = true` triggers the reset.
    pub async fn set_full_system_reset(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.set_register_bit(0x09, 2).await
        } else {
            self.clear_register_bit(0x09, 2).await
        }
    }

    /// Send one current-pulse voltage-up command (increase adapter voltage request).
    pub async fn set_current_pulse_voltage_up(
        &mut self,
        enable: bool,
    ) -> Result<(), PmuSensorError> {
        if enable {
            self.set_register_bit(0x09, 1).await
        } else {
            self.clear_register_bit(0x09, 1).await
        }
    }

    /// Send one current-pulse voltage-down command (decrease adapter voltage request).
    pub async fn set_current_pulse_voltage_down(
        &mut self,
        enable: bool,
    ) -> Result<(), PmuSensorError> {
        if enable {
            self.set_register_bit(0x09, 0).await
        } else {
            self.clear_register_bit(0x09, 0).await
        }
    }

    // Register 0x0A: Boost Mode Voltage, PFM, Boost Current Limit

    /// Set the boost-mode output voltage on VBUS.
    ///
    /// `millivolt` must be a multiple of 64 mV in the range 4550–5510 mV.
    pub async fn set_boost_voltage(&mut self, mut millivolt: u16) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(BOOST_VOL_STEP) {
            return Err(PmuSensorError::VoltageStepInvalid64);
        }
        millivolt = millivolt.clamp(BOOST_VOL_MIN, BOOST_VOL_MAX);
        let val = self.read_register(0x0A).await?;
        let steps = ((millivolt - BOOST_VOL_BASE) / BOOST_VOL_STEP) as u8;
        self.write_register(&[0x0A, (val & 0xF0) | (steps << 4)])
            .await
    }

    /// Set the maximum current the boost regulator can draw from the battery.
    pub async fn set_boost_current_limit(
        &mut self,
        limit: BoostCurrentLimit,
    ) -> Result<(), PmuSensorError> {
        let limit: u8 = limit.into();
        let val = self.read_register(0x0A).await?;
        self.write_register(&[0x0A, (val & 0x03) | limit]).await
    }

    /// Enable or disable PFM (Pulse-Frequency Modulation) in boost mode for better light-load efficiency.
    pub async fn set_boost_mode_pfm(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.clear_register_bit(0x0A, 3).await
        } else {
            self.set_register_bit(0x0A, 3).await
        }
    }

    // Register 0x0B: VBUS Status, Charge Status, Power Good, VSYS Regulation

    /// Read the current charge phase (Not Charging, Pre-Charge, Fast Charge, or Done).
    pub async fn get_charge_status(&mut self) -> Result<ChargeStatus, PmuSensorError> {
        let val = self.read_register(0x0B).await?;
        Ok(ChargeStatus::try_from((val >> 3) & 0x03).expect("Invalid charge status"))
    }

    /// Returns `true` when a valid VBUS power source is connected.
    pub async fn is_vbus_in(&mut self) -> Result<bool, PmuSensorError> {
        Ok(self.get_bus_status().await? != BusStatus::NoInput)
    }

    /// Returns `true` when the device is operating as a USB OTG host (boost mode).
    pub async fn is_otg(&mut self) -> Result<bool, PmuSensorError> {
        Ok(self.get_bus_status().await? == BusStatus::Otg)
    }

    /// Returns `true` when the battery is currently being charged (any phase).
    pub async fn is_charging(&mut self) -> Result<bool, PmuSensorError> {
        Ok(self.get_charge_status().await? != ChargeStatus::NoCharge)
    }

    /// Returns `true` when charge termination has completed successfully.
    pub async fn is_charge_done(&mut self) -> Result<bool, PmuSensorError> {
        Ok(self.get_charge_status().await? == ChargeStatus::Done)
    }

    /// Returns `true` when VBUS is within the valid operating range (power-good).
    pub async fn is_power_good(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x0B, 2).await
    }

    /// Read the input source type (None, USB SDP, Adapter, or OTG).
    pub async fn get_bus_status(&mut self) -> Result<BusStatus, PmuSensorError> {
        let val = self.read_register(0x0B).await?;
        Ok(BusStatus::try_from((val >> 5) & 0x07).expect("Invalid bus status"))
    }

    // Register 0x0C: Fault Status

    /// Returns `true` when a watchdog timer expiry fault has occurred.
    pub async fn is_watchdog_fault(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.read_register(0x0C).await?;
        Ok(((val >> 7) & 0x01) != 0)
    }

    /// Returns `true` when a boost-mode fault (e.g. VBUS overload or low battery) has occurred.
    pub async fn is_boost_fault(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.read_register(0x0C).await?;
        Ok(((val >> 6) & 0x01) != 0)
    }

    /// Read the charger fault status (Normal, Input Fault, Thermal Shutdown, or Safety Timer).
    pub async fn get_charge_fault(&mut self) -> Result<ChargeFaultStatus, PmuSensorError> {
        let val = self.read_register(0x0C).await?;
        Ok(ChargeFaultStatus::try_from((val >> 4) & 0x03).expect("Invalid charge fault status"))
    }

    /// Returns `true` when a battery overvoltage fault has been latched.
    pub async fn is_battery_fault(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.read_register(0x0C).await?;
        Ok(((val >> 3) & 0x01) != 0)
    }

    /// Returns `true` when an NTC thermistor fault (cold/hot) is active.
    pub async fn is_ntc_fault(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.read_register(0x0C).await?;
        Ok((val & 0x07) != 0)
    }

    /// Read a human-readable description of the current NTC thermistor status.
    ///
    /// The result depends on whether the device is in boost or buck mode.
    pub async fn get_ntc_status_string(&mut self) -> Result<&'static str, PmuSensorError> {
        let status = self.read_register(0x0C).await? & 0x07;
        if self.is_otg().await? {
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

    /// Select the VINDPM threshold measurement method.
    ///
    /// `relative = true`: threshold is VBUS at connection plus the register offset.
    /// `relative = false`: threshold is the absolute register value.
    pub async fn set_vindpm_threshold_method(
        &mut self,
        relative: bool,
    ) -> Result<(), PmuSensorError> {
        if relative {
            self.clear_register_bit(0x0D, 7).await
        } else {
            self.set_register_bit(0x0D, 7).await
        }
    }

    /// Set the input voltage dynamic power management threshold.
    ///
    /// `millivolt` must be a multiple of 100 mV in the range 3900–15300 mV.
    pub async fn set_vindpm_threshold(&mut self, mut millivolt: u16) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(VINDPM_VOL_STEPS) {
            return Err(PmuSensorError::VoltageStepInvalid100);
        }
        millivolt = millivolt.clamp(VINDPM_VOL_MIN, VINDPM_VOL_MAX);
        let val = self.read_register(0x0D).await?;
        let steps = ((millivolt - VINDPM_VOL_BASE) / VINDPM_VOL_STEPS) as u8;
        self.write_register(&[0x0D, (val & 0x80) | steps]).await
    }

    /// Read the programmed VINDPM threshold in mV (100 mV resolution, 3900–15300 mV range).
    pub async fn get_vindpm_threshold(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x0D).await?;
        Ok((val & 0x7F) as u16 * VINDPM_VOL_STEPS + VINDPM_VOL_BASE)
    }

    // Register 0x0E: Battery Voltage ADC

    /// Returns `true` when the die temperature is below the thermal regulation threshold.
    pub async fn is_thermal_regulation_normal(&mut self) -> Result<bool, PmuSensorError> {
        Ok(!self.get_register_bit(0x0E, 7).await?)
    }

    /// Read the battery voltage measured by the internal ADC in mV (20 mV resolution, offset 2304 mV).
    ///
    /// Returns 0 if the ADC is disabled or the reading is invalid.
    pub async fn get_battery_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let data = self.read_register(0x0E).await?;
        if data == 0 || ((data >> 7) & 0x01) != 0 {
            return Ok(0);
        }
        Ok((data & 0x7F) as u16 * 20 + 2304)
    }

    /// Estimate the battery state-of-charge as a percentage (0–100 %) from the ADC voltage.
    ///
    /// Uses a piecewise-linear approximation calibrated for a typical LiPo discharge curve.
    pub async fn get_battery_percentage(&mut self) -> Result<u8, PmuSensorError> {
        let voltage = self.get_battery_voltage().await?;
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

    /// Read the system (SYS) rail voltage measured by the internal ADC in mV (20 mV resolution, offset 2304 mV).
    ///
    /// Returns 0 if the ADC reading is zero.
    pub async fn get_sys_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let data = self.read_register(0x0F).await? & 0x7F;
        if data == 0 {
            return Ok(0);
        }
        Ok(data as u16 * 20 + 2304)
    }

    // Register 0x10: TS Voltage (NTC thermistor)

    /// Read the battery pack temperature estimated from the NTC thermistor voltage in °C.
    ///
    /// Uses the Steinhart-Hart approximation with β = 3950, R₀ = 10 kΩ at 25 °C.
    pub async fn get_temperature(&mut self) -> Result<f64, PmuSensorError> {
        let data = self.read_register(0x10).await? & 0x7F;
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

    /// Returns `true` when the VBUS voltage is within the good range (above VVBUS_GOOD threshold).
    pub async fn is_vbus_good(&mut self) -> Result<bool, PmuSensorError> {
        let data = self.read_register(0x11).await?;
        Ok(((data >> 7) & 0x01) != 0)
    }

    /// Read the VBUS voltage measured by the internal ADC in mV (100 mV resolution, offset 2600 mV).
    ///
    /// Returns 0 when VBUS is not present or the ADC flag indicates no valid reading.
    pub async fn get_vbus_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let data = self.read_register(0x11).await?;
        if ((data >> 7) & 0x01) == 0 {
            return Ok(0);
        }
        Ok((data & 0x7F) as u16 * 100 + 2600)
    }

    // Register 0x12: Charge Current ADC

    /// Read the actual charge current measured by the internal ADC in mA (50 mA resolution).
    ///
    /// Returns 0 when no charging is active.
    pub async fn get_charge_current(&mut self) -> Result<u16, PmuSensorError> {
        if self.get_charge_status().await? == ChargeStatus::NoCharge {
            return Ok(0);
        }
        let val = self.read_register(0x12).await?;
        Ok((val & 0x7F) as u16 * CHG_STEP_VAL)
    }

    // Register 0x13: VINDPM/IINDPM Status, ICO Current Limit

    /// Returns `true` when the charger is actively limiting input current due to VINDPM regulation.
    pub async fn is_dynamic_power_management(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x13, 7).await
    }

    /// Returns `true` when the charger is actively limiting current due to IINDPM regulation.
    pub async fn is_input_current_limit(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x13, 6).await
    }

    /// Set the input current limit used by the ICO (Input Current Optimizer) algorithm.
    ///
    /// `milliampere` must be a multiple of 50 mA in the range 100–3250 mA.
    pub async fn set_input_current_limit_optimizer(
        &mut self,
        mut milliampere: u16,
    ) -> Result<(), PmuSensorError> {
        if !milliampere.is_multiple_of(IN_CURRENT_OPT_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid50);
        }
        milliampere = milliampere.clamp(IN_CURRENT_OPT_MIN, IN_CURRENT_OPT_MAX);
        let val = self.read_register(0x13).await?;
        let steps = ((milliampere - IN_CURRENT_OPT_MIN) / IN_CURRENT_OPT_STEP) as u8;
        self.write_register(&[0x13, (val & 0x3F) | (steps << 6)])
            .await
    }

    /// Read the input current limit currently in effect (from ICO result) in mA.
    pub async fn get_input_current_limit_in_effect(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.read_register(0x13).await?;
        Ok((val & 0x3F) as u16 * IN_CURRENT_OPT_STEP + IN_CURRENT_OPT_MIN)
    }

    // Register 0x14: Register Reset, ICO Status, Device Config, Temp Profile, Device Revision

    /// Reset all registers to their default values (self-clearing bit).
    pub async fn reset_default(&mut self) -> Result<(), PmuSensorError> {
        self.set_register_bit(0x14, 7).await
    }

    /// Returns `true` when the ICO algorithm has completed optimization.
    pub async fn is_input_current_optimizer(&mut self) -> Result<bool, PmuSensorError> {
        self.get_register_bit(0x14, 6).await
    }

    /// Read the device configuration bits (PN field) identifying the chip variant.
    pub async fn get_device_config(&mut self) -> Result<u8, PmuSensorError> {
        let val = self.read_register(0x14).await?;
        Ok((val >> 3) & 0x03)
    }

    /// Returns `true` when the JEITA temperature profile is active.
    pub async fn get_temperature_profile(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.read_register(0x14).await?;
        Ok(((val >> 2) & 0x01) != 0)
    }

    /// Read the device revision bits from register 0x14.
    pub async fn get_chip_id(&mut self) -> Result<u8, PmuSensorError> {
        Ok(self.read_register(0x14).await? & 0x03)
    }

    /// Return a detailed human-readable summary of all charger status and configuration fields.
    pub async fn get_info(&mut self) -> Result<String, PmuSensorError> {
        let is_vbus_present = if self.is_vbus_in().await? {
            "Yes"
        } else {
            "No"
        };
        let text = format!(
            "═══ POWER STATUS ═══\n\
            CHG state: {}\n\
            USB PlugIn: {}\n\
            Bus state: {}\n\
            Power good: {}\n\
            Charging enabled: {}\n\
            \n\
            ═══ BATTERY ═══\n\
            Voltage: {}mV ({}%)\n\
            Charge current: {}mA\n\
            Temperature: {:.1}°C\n\
            NTC status: {}\n\
            \n\
            ═══ CHARGER CONFIG ═══\n\
            Fast charge limit: {}mA\n\
            Precharge current: {}mA\n\
            Termination current: {}mA\n\
            Target voltage: {}mV\n\
            Fast charge timer: {}\n\
            \n\
            ═══ INPUT ═══\n\
            USB voltage: {}mV\n\
            USB good: {}\n\
            Input curr. limit: {}mA\n\
            ICO in progress: {}\n\
            ICO limit in effect: {}mA\n\
            DPM active: {}\n\
            Input curr. limit active: {}\n\
            \n\
            ═══ SYSTEM ═══\n\
            SYS voltage: {}mV\n\
            Power down voltage: {}mV\n\
            Thermal regulation: {}\n\
            \n\
            ═══ MODES & FEATURES ═══\n\
            OTG enabled: {}\n\
            HIZ mode: {}\n\
            Battery load enabled: {}\n\
            Boost frequency: {}\n\
            Charging safety timer: {}\n\
            Charging termination: {}\n\
            Auto input detection: {}\n\
            \n\
            ═══ FAULTS ═══\n\
            Watchdog fault: {}\n\
            Boost fault: {}\n\
            Charge fault: {:?}\n\
            Battery fault: {}\n\
            NTC fault: {}\n\
            \n\
            ═══ DEVICE INFO ═══\n\
            Chip ID: {}\n\
            Device config: {}\n\
            Temperature profile: {}\n",
            self.get_charge_status().await?,
            is_vbus_present,
            self.get_bus_status().await?,
            self.is_power_good().await?,
            self.is_charge_enabled().await?,
            self.get_battery_voltage().await?,
            self.get_battery_percentage().await?,
            self.get_charge_current().await?,
            self.get_temperature().await?,
            self.get_ntc_status_string().await?,
            self.get_fast_charge_current_limit().await?,
            self.get_precharge_current().await?,
            self.get_termination_current().await?,
            self.get_charge_target_voltage().await?,
            self.get_fast_charge_timer().await?,
            self.get_vbus_voltage().await?,
            self.is_vbus_good().await?,
            self.get_input_current_limit().await?,
            self.is_input_current_optimizer().await?,
            self.get_input_current_limit_in_effect().await?,
            self.is_dynamic_power_management().await?,
            self.is_input_current_limit().await?,
            self.get_sys_voltage().await?,
            self.get_sys_power_down_voltage().await?,
            if self.is_thermal_regulation_normal().await? {
                "Normal"
            } else {
                "Active"
            },
            self.is_otg_enabled().await?,
            self.is_hiz_mode().await?,
            self.is_bat_load_enabled().await?,
            self.get_boost_freq().await?,
            self.is_charging_safety_timer_enabled().await?,
            self.is_charging_termination_enabled().await?,
            self.is_automatic_input_detection_enabled().await?,
            self.is_watchdog_fault().await?,
            self.is_boost_fault().await?,
            self.get_charge_fault().await?,
            self.is_battery_fault().await?,
            self.is_ntc_fault().await?,
            self.get_chip_id().await?,
            self.get_device_config().await?,
            if self.get_temperature_profile().await? {
                "JEITA"
            } else {
                "Standard"
            },
        );
        Ok(text)
    }
}
