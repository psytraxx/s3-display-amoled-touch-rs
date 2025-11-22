use alloc::{format, string::String};
use core::fmt::{self, Display, Formatter};
use embedded_hal::i2c::{Error, I2c};
use libm::{log, round};
use num_enum::{IntoPrimitive, TryFromPrimitive};

use crate::AsynRegisterDevice;
/// <https://github.com/Xinyuan-LilyGO/LilyGo-AMOLED-Series/blob/master/libdeps/XPowersLib/src/PowersBQ25896.tpp>
///
/// BQ25896 battery charging and power path management IC driver.
/// Provides battery charging control, system power path management,
/// and monitoring capabilities.
///
/// # Features
/// - Battery voltage monitoring
/// - System voltage monitoring
/// - USB/adapter input detection
/// - Temperature monitoring
/// - Charge status reporting
#[derive(Debug)]
pub struct BQ25896<I2C> {
    dev: AsynRegisterDevice<I2C>,
    user_disable_charge: bool,
}

const IN_CURRENT_STEP: u16 = 50;
const IN_CURRENT_MIN: u16 = 100;
const IN_CURRENT_MAX: u16 = 3250;

const IN_CURRENT_OFFSET_STEP: u16 = 100;
const IN_CURRENT_OFFSET_MAX: u16 = 3100;

const CHG_STEP_VAL: u16 = 50;

const FAST_CHG_CUR_STEP: u16 = 64;
const FAST_CHG_CURRENT_MAX: u16 = 3008;

const CHG_VOL_BASE: u16 = 3840;
const CHG_VOL_STEP: u16 = 16;
const FAST_CHG_VOL_MIN: u16 = 3840;
const FAST_CHG_VOL_MAX: u16 = 4608;

const PRE_CHG_CUR_BASE: u16 = 64;
const PRE_CHG_CUR_STEP: u16 = 64;
const PRE_CHG_CURRENT_MIN: u16 = 64;
const PRE_CHG_CURRENT_MAX: u16 = 1024;

const SYS_VOL_STEPS: u16 = 100;
const SYS_VOFF_VOL_MIN: u16 = 3000;
const SYS_VOFF_VOL_MAX: u16 = 3700;

const TERM_CHG_CUR_BASE: u16 = 64;
const TERM_CHG_CUR_STEP: u16 = 64;
const TERM_CHG_CURRENT_MIN: u16 = 64;
const TERM_CHG_CURRENT_MAX: u16 = 1024;

const BAT_COMP_STEPS: u16 = 20;
const BAT_COMP_MAX: u16 = 140;

const VCLAMP_STEPS: u16 = 32;
const VCLAMP_MAX: u16 = 224;

const BOOST_VOL_BASE: u16 = 4550;
const BOOST_VOL_STEP: u16 = 64;
const BOOST_VOL_MIN: u16 = 4550;
const BOOST_VOL_MAX: u16 = 5510;

const VINDPM_VOL_BASE: u16 = 4550;
const VINDPM_VOL_STEPS: u16 = 100;
const VINDPM_VOL_MIN: u16 = 3900;
const VINDPM_VOL_MAX: u16 = 15300;

const IN_CURRENT_OPT_STEP: u16 = 50;
const IN_CURRENT_OPT_MIN: u16 = 100;
const IN_CURRENT_OPT_MAX: u16 = 3250;

impl<I2C> BQ25896<I2C>
where
    I2C: I2c,
{
    /// Creates a new BQ25896 driver instance
    ///
    /// # Arguments
    /// * `i2c` - I2C bus instance
    /// * `adr` - I2C device address (typically 0x6B)
    pub fn new(i2c: I2C, adr: u8) -> Result<Self, PmuSensorError> {
        let mut instance = Self {
            dev: AsynRegisterDevice::new(i2c, adr),
            user_disable_charge: false,
        };
        if instance.dev.write_register(&[adr]).is_err() {
            return Err(PmuSensorError::Init);
        }
        Ok(instance)
    }

    // Register 0x00
    // Enable HIZ Mode, Enable ILIM Pin, Input Current Limit

    /// Enables HIZ mode
    pub fn set_hiz_mode(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x00, 7)?;
        Ok(())
    }

    /// Exits HIZ mode
    pub fn exit_hiz_mode(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x00, 7)?;
        Ok(())
    }

    /// Checks if HIZ mode is enabled
    pub fn is_hiz_mode(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x00, 7)?;
        Ok(result)
    }

    /// Enables the current limit pin
    pub fn enable_current_limit_pin(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x00, 6)?;
        Ok(())
    }

    /// Disables the current limit pin
    pub fn disable_current_limit_pin(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x00, 6)?;
        Ok(())
    }

    /// Checks if the current limit pin is enabled
    pub fn is_enable_current_limit_pin(&mut self) -> Result<bool, PmuSensorError> {
        let result: bool = self.dev.get_register_bit(0x00, 6)?;
        Ok(result)
    }

    /// Sets the input current limit
    ///
    /// # Arguments
    /// * `milliampere` - Current limit in mA
    pub fn set_input_current_limit(&mut self, milliampere: u16) -> Result<(), PmuSensorError> {
        // Validate input is multiple of step size
        if !milliampere.is_multiple_of(IN_CURRENT_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid50);
        }

        // Clamp value to valid range
        let ma = milliampere.clamp(IN_CURRENT_MIN, IN_CURRENT_MAX);

        // Read current register value
        let mut reg_val = self.dev.read_register(0x00)?;

        // Clear bottom 6 bits while preserving top 2 bits
        reg_val &= 0xC0;

        // Calculate new value (offset from minimum, divided by step size)
        let current_bits = ((ma - IN_CURRENT_MIN) / IN_CURRENT_STEP) as u8;

        // Combine preserved bits with new value
        reg_val |= current_bits;

        // Write back to register
        self.dev.write_register(&[0x00, reg_val])?;
        Ok(())
    }

    /// Gets the input current limit
    pub fn get_input_current_limit(&mut self) -> Result<u16, PmuSensorError> {
        let reg_val = self.dev.read_register(0x00)?;
        let current_bits = reg_val & 0x3F;
        let ma = (current_bits as u16 * IN_CURRENT_STEP) + IN_CURRENT_MIN;
        Ok(ma)
    }

    // REGISTER 0x01
    // Boost Mode Hot Temperature Monitor Threshold , Boost Mode Cold Temperature Monitor Threshold,
    // Input Voltage Limit Offset

    /// Sets the boost mode hot temperature monitor threshold
    ///
    /// # Arguments
    /// * `threshold` - Boost hot temperature threshold
    pub fn set_boost_mode_hot_temp_threshold(
        &mut self,
        threshold: BoostHotThreshold,
    ) -> Result<(), PmuSensorError> {
        let val = self.dev.read_register(0x01)?;
        let data = (val & 0x3F) | ((threshold as u8) << 6);
        self.dev.write_register(&[0x01, data])?;
        Ok(())
    }

    /// Sets the boost mode cold temperature monitor threshold
    ///
    /// # Arguments
    /// * `threshold` - Boost cold temperature threshold
    pub fn set_boost_mode_cold_temp_threshold(
        &mut self,
        threshold: BoostColdThreshold,
    ) -> Result<(), PmuSensorError> {
        let val = self.dev.read_register(0x01)?;
        let data = (val & 0xDF) | ((threshold as u8) << 5);
        self.dev.write_register(&[0x01, data])?;
        Ok(())
    }

    /// Sets the input voltage limit offset
    /// Default: 600mV (00110)
    /// Range: 0mV – 3100mV
    /// Minimum VINDPM threshold is clamped at 3.9V
    /// Maximum VINDPM threshold is clamped at 15.3V
    /// When VBUS at noLoad is ≤ 6V, the VINDPM_OS is used to calculate VINDPM threshold
    /// When VBUS at noLoad is > 6V, the VINDPM_OS multiple by 2 is used to calculate VINDPM threshold.
    ///
    /// # Arguments
    /// * `millivolt` - Voltage limit offset in mV
    pub fn set_input_voltage_limit_offset(
        &mut self,
        mut millivolt: u16,
    ) -> Result<(), PmuSensorError> {
        // Validate step size
        if !millivolt.is_multiple_of(IN_CURRENT_OFFSET_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid100);
        }

        if millivolt > IN_CURRENT_OFFSET_MAX {
            millivolt = IN_CURRENT_OFFSET_MAX;
        }

        let steps = millivolt / IN_CURRENT_OFFSET_STEP;
        let val = self.dev.read_register(0x01)?;
        let data = (val & 0xE0) | (steps as u8);
        self.dev.write_register(&[0x01, data])?;
        Ok(())
    }

    // REGISTER 0x02
    // ADC Conversion Start Control, ADC Conversion Rate Selection, Boost Mode Frequency Selection
    // Input Current Optimizer (ICO) Enable, Force Input Detection, Automatic Input Detection Enable

    /// Enables ADC conversion for voltage and current monitoring
    pub fn set_adc_enabled(&mut self) -> Result<(), PmuSensorError> {
        let mut data = self.dev.read_register(0x02)?;
        data |= 1 << 7; // Start ADC conversion
        data |= 1 << 6; // Set continuous conversion
        self.dev.write_register(&[0x02, data])?;
        Ok(())
    }

    /// Disables ADC conversion
    pub fn set_adc_disabled(&mut self) -> Result<(), PmuSensorError> {
        let mut data = self.dev.read_register(0x02)?;
        data &= !(1 << 7); // Clear ADC conversion bit
        self.dev.write_register(&[0x02, data])?;
        Ok(())
    }

    /// Sets the boost frequency
    ///
    /// # Arguments
    /// * `freq` - Boost frequency
    pub fn set_boost_freq(&mut self, freq: BoostFreq) -> Result<(), PmuSensorError> {
        match freq {
            BoostFreq::Freq500KHz => self.dev.set_register_bit(0x02, 5)?,
            BoostFreq::Freq1500KHz => self.dev.clear_register_bit(0x02, 5)?,
        }
        Ok(())
    }

    /// Gets the boost frequency
    pub fn get_boost_freq(&mut self) -> Result<BoostFreq, PmuSensorError> {
        let bit = self.dev.get_register_bit(0x02, 5)?;
        Ok(if bit {
            BoostFreq::Freq500KHz
        } else {
            BoostFreq::Freq1500KHz
        })
    }

    /// Enables the input current optimizer
    pub fn enable_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x02, 4)?;
        Ok(())
    }

    /// Disables the input current optimizer
    pub fn disable_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x02, 4)?;
        Ok(())
    }

    /// Enables input detection
    pub fn enable_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x02, 1)?;
        Ok(())
    }

    /// Disables input detection
    pub fn disable_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x02, 1)?;
        Ok(())
    }

    /// Checks if input detection is enabled
    pub fn is_input_detection_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x02, 1)?;
        Ok(result)
    }

    /// Forces D+/D- detection when AUTO_DPDM_EN is disabled
    pub fn force_dpdm_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x02, 1)?;
        Ok(())
    }

    /// Enables automatic input detection
    pub fn enable_automatic_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x02, 0)?;
        Ok(())
    }

    /// Disables automatic input detection
    pub fn disable_automatic_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x02, 0)?;
        Ok(())
    }

    /// Checks if automatic input detection is enabled
    pub fn is_automatic_input_detection_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x02, 0)?;
        Ok(result)
    }

    // REGISTER 0x03
    // Battery Load (IBATLOAD) Enable, I2C Watchdog Timer Reset, Boost (OTG) Mode Configuration
    // Charge Enable Configuration,  Minimum System Voltage Limit, Minimum Battery Voltage (falling) to exit boost mode

    /// Checks if battery load is enabled
    pub fn is_bat_load_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x03, 7)?;
        Ok(result)
    }

    /// Disables battery load
    pub fn disable_bat_load(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x03, 7)?;
        Ok(())
    }

    /// Enables battery load
    pub fn enable_bat_load(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x03, 7)?;
        Ok(())
    }

    /// Feeds the watchdog timer
    pub fn feed_watchdog(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x03, 6)?;
        Ok(())
    }

    /// Checks if OTG mode is enabled
    pub fn is_otg_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x03, 5)?;
        Ok(result)
    }

    /// Disables OTG mode
    pub fn disable_otg(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x03, 5)?;
        // Re-enable charging if it wasn't explicitly disabled by user
        if !self.user_disable_charge {
            self.dev.set_register_bit(0x03, 4)?;
        }
        Ok(())
    }

    /// Enables OTG mode
    pub fn enable_otg(&mut self) -> Result<bool, PmuSensorError> {
        if self.is_vbus_in()? {
            return Ok(false);
        }
        self.dev.set_register_bit(0x03, 5)?;
        Ok(true)
    }

    /// Enables charging
    pub fn set_charge_enabled(&mut self) -> Result<(), PmuSensorError> {
        self.user_disable_charge = false;
        self.dev.set_register_bit(0x03, 4)?;
        Ok(())
    }

    /// Disables charging
    pub fn set_charge_disabled(&mut self) -> Result<(), PmuSensorError> {
        self.user_disable_charge = true;
        self.dev.clear_register_bit(0x03, 4)?;
        Ok(())
    }

    /// Checks if charging is enabled
    pub fn is_charge_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x03, 4)?;
        Ok(result)
    }

    /// Sets the system power down voltage
    ///
    /// # Arguments
    /// * `millivolt` - Power down voltage in mV
    pub fn set_sys_power_down_voltage(&mut self, millivolt: u16) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(SYS_VOL_STEPS) {
            return Err(PmuSensorError::VoltageStepInvalid100);
        }
        if !(SYS_VOFF_VOL_MIN..=SYS_VOFF_VOL_MAX).contains(&millivolt) {
            return Err(PmuSensorError::PowerDownVoltageInvalid);
        }

        let mut val = self.dev.read_register(0x03)?;
        val &= 0xF1;
        val |= ((millivolt - SYS_VOFF_VOL_MIN) / SYS_VOL_STEPS) as u8;
        val <<= 1;
        self.dev.write_register(&[0x03, val])?;
        Ok(())
    }

    /// Gets the system power down voltage
    pub fn get_sys_power_down_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.dev.read_register(0x03)?;
        let val = (val & 0x0E) >> 1;
        Ok((val as u16 * SYS_VOL_STEPS) + SYS_VOFF_VOL_MIN)
    }

    /// Sets the exit boost mode voltage
    ///
    /// # Arguments
    /// * `voltage` - Exit boost mode voltage
    pub fn set_exit_boost_mode_voltage(
        &mut self,
        voltage: ExitBoostModeVolt,
    ) -> Result<(), PmuSensorError> {
        match voltage {
            ExitBoostModeVolt::MiniVolt2V9 => self.dev.clear_register_bit(0x03, 0)?,
            ExitBoostModeVolt::MiniVolt2V5 => self.dev.set_register_bit(0x03, 0)?,
        }
        Ok(())
    }

    // REGISTER 0x04
    // Current pulse control Enable, Fast Charge Current Limit

    /// Enables current pulse control
    pub fn set_current_pulse_control_enabled(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x04, 7)?;
        Ok(())
    }

    /// Disables current pulse control
    pub fn set_current_pulse_control_disabled(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x04, 7)?;
        Ok(())
    }

    /// Sets the fast charge current limit
    ///
    /// # Arguments
    /// * `milliampere` - Fast charge current limit in mA
    pub fn set_fast_charge_current_limit(
        &mut self,
        milliampere: u16,
    ) -> Result<(), PmuSensorError> {
        // Check if current is multiple of step size
        if !milliampere.is_multiple_of(FAST_CHG_CUR_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid64);
        }

        // Clamp to max value
        let current = milliampere.min(FAST_CHG_CURRENT_MAX);

        // Read current register value
        let mut val = self.dev.read_register(0x04)?;

        // Clear bits 6:0, keep bit 7
        val &= 0x80;

        // Calculate and set new current bits
        val |= (current / FAST_CHG_CUR_STEP) as u8;

        // Write back to register
        self.dev.write_register(&[0x04, val])?;
        Ok(())
    }

    /// Gets the fast charge current limit
    pub fn get_fast_charge_current_limit(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.dev.read_register(0x04)?;
        let bits = val & 0x7F; // Extract bits 6:0

        Ok(bits as u16 * FAST_CHG_CUR_STEP)
    }

    // REGISTER 0x05
    // Precharge Current Limit, Termination Current Limit

    /// Sets the precharge current
    ///
    /// # Arguments
    /// * `milliampere` - Precharge current in mA
    pub fn set_precharge_current(&mut self, milliampere: u16) -> Result<(), PmuSensorError> {
        // Validate step size
        if !milliampere.is_multiple_of(PRE_CHG_CUR_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid64);
        }

        // Clamp to valid range
        let current = milliampere.clamp(PRE_CHG_CURRENT_MIN, PRE_CHG_CURRENT_MAX);

        // Read current register value
        let mut val = self.dev.read_register(0x05)?;

        // Clear bits 7:4, keep bits 3:0
        val &= 0x0F;

        // Calculate new current bits and shift to position
        let current_bits = ((current - PRE_CHG_CUR_BASE) / PRE_CHG_CUR_STEP) as u8;
        val |= current_bits << 4;

        // Write back to register
        self.dev.write_register(&[0x05, val])?;
        Ok(())
    }

    /// Gets the precharge current
    pub fn get_precharge_current(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.dev.read_register(0x05)?;
        let bits = (val & 0xF0) >> 4;

        Ok(PRE_CHG_CUR_STEP + (bits as u16 * PRE_CHG_CUR_STEP))
    }

    /// Sets the termination current
    ///
    /// # Arguments
    /// * `milliampere` - Termination current in mA
    pub fn set_termination_current(&mut self, milliampere: u16) -> Result<(), PmuSensorError> {
        // Validate step size
        if !milliampere.is_multiple_of(TERM_CHG_CUR_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid64);
        }

        // Clamp to valid range
        let current = milliampere.clamp(TERM_CHG_CURRENT_MIN, TERM_CHG_CURRENT_MAX);

        // Read current register value
        let mut val = self.dev.read_register(0x05)?;

        // Clear bits 3:0, keep bits 7:4
        val &= 0xF0;

        // Calculate new current bits
        let current_bits = ((current - TERM_CHG_CUR_BASE) / TERM_CHG_CUR_STEP) as u8;
        val |= current_bits;

        // Write back to register
        self.dev.write_register(&[0x05, val])?;
        Ok(())
    }

    /// Gets the termination current
    pub fn get_termination_current(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.dev.read_register(0x05)?;
        let bits = val & 0x0F;
        Ok(TERM_CHG_CUR_STEP + (bits as u16 * TERM_CHG_CUR_STEP))
    }

    // REGISTER 0x06
    // Charge Voltage Limit, Battery Precharge to Fast Charge Threshold, Battery Recharge Threshold Offset

    /// Sets the charge target voltage
    ///
    /// # Arguments
    /// * `target_voltage` - Charge target voltage in mV
    pub fn set_charge_target_voltage(&mut self, target_voltage: u16) -> Result<(), PmuSensorError> {
        // Check if voltage is multiple of step size
        if !target_voltage.is_multiple_of(CHG_VOL_STEP) {
            return Err(PmuSensorError::VoltageStepInvalid16);
        }

        // Clamp voltage to valid range
        let voltage = target_voltage.clamp(FAST_CHG_VOL_MIN, FAST_CHG_VOL_MAX);

        // Read current register value
        let mut val = self.dev.read_register(0x06)?;

        // Clear bits 7:2, keep bits 1:0
        val &= 0x03;

        // Calculate and set new voltage bits
        val |= (((voltage - CHG_VOL_BASE) / CHG_VOL_STEP) << 2) as u8;

        // Write back to register
        self.dev.write_register(&[0x06, val])?;
        Ok(())
    }

    /// Gets the charge target voltage
    pub fn get_charge_target_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.dev.read_register(0x06)?;
        let bits = (val & 0xFC) >> 2;

        if bits > 0x30 {
            return Ok(FAST_CHG_VOL_MAX);
        }

        Ok(CHG_VOL_BASE + (bits as u16 * CHG_VOL_STEP))
    }

    /// Sets the fast charge threshold
    ///
    /// # Arguments
    /// * `threshold` - Fast charge threshold
    pub fn set_fast_charge_threshold(
        &mut self,
        threshold: FastChargeThreshold,
    ) -> Result<(), PmuSensorError> {
        match threshold {
            FastChargeThreshold::Volt2V8 => self.dev.clear_register_bit(0x06, 1)?,
            FastChargeThreshold::Volt3V0 => self.dev.set_register_bit(0x06, 1)?,
        }
        Ok(())
    }

    /// Sets the battery recharge threshold offset
    ///
    /// # Arguments
    /// * `offset` - Recharge threshold offset
    pub fn set_battery_recharge_threshold_offset(
        &mut self,
        offset: RechargeThresholdOffset,
    ) -> Result<(), PmuSensorError> {
        match offset {
            RechargeThresholdOffset::Offset100mV => self.dev.clear_register_bit(0x06, 0)?,
            RechargeThresholdOffset::Offset200mV => self.dev.set_register_bit(0x06, 0)?,
        }
        Ok(())
    }

    // REGISTER 0x07
    // Charging Termination Enable, STAT Pin Disable , I2C Watchdog Timer Setting, Charging Safety Timer Enable
    // Fast Charge Timer Setting, JEITA Low Temperature Current Setting

    /// Enables charging termination
    pub fn enable_charging_termination(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x07, 7)?;
        Ok(())
    }

    /// Disables charging termination
    pub fn disable_charging_termination(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x07, 7)?;
        Ok(())
    }

    /// Checks if charging termination is enabled
    pub fn is_charging_termination_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x07, 7)?;
        Ok(result)
    }

    /// Disables the STAT pin
    pub fn disable_stat_pin(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x07, 6)?;
        Ok(())
    }

    /// Enables the STAT pin
    pub fn enable_stat_pin(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x07, 6)?;
        Ok(())
    }

    /// Checks if the STAT pin is enabled
    pub fn is_stat_pin_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x07, 6)?;
        Ok(result)
    }

    /// Disables the watchdog timer
    pub fn disable_watchdog(&mut self) -> Result<(), PmuSensorError> {
        let mut val = self.dev.read_register(0x07)?;
        val &= 0xCF;
        self.dev.write_register(&[0x07, val])?;
        Ok(())
    }

    /// Enables the watchdog timer
    ///
    /// # Arguments
    /// * `config` - Watchdog timer configuration
    pub fn enable_watchdog(&mut self, config: WatchdogConfig) -> Result<(), PmuSensorError> {
        let mut val = self.dev.read_register(0x07)?;
        // Clear the 4th and 5th bits (watchdog timer setting bits)
        val &= 0xCF; // 0xCF = 1100_1111, clears bits 4 & 5
                     // Map the low two bits of config into bits 4-5
        let bits: u8 = (config as u8 & 0x03) << 4;
        self.dev.write_register(&[0x07, val | bits])?;
        Ok(())
    }

    /// Disables the charging safety timer
    pub fn disable_charging_safety_timer(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x07, 3)?;
        Ok(())
    }

    /// Enables the charging safety timer
    pub fn enable_charging_safety_timer(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x07, 3)?;
        Ok(())
    }

    /// Checks if the charging safety timer is enabled
    pub fn is_charging_safety_timer_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x07, 3)?;
        Ok(result)
    }

    /// Sets the fast charge timer
    ///
    /// # Arguments
    /// * `timer` - Fast charge timer
    pub fn set_fast_charge_timer(&mut self, timer: FastChargeTimer) -> Result<(), PmuSensorError> {
        let timer: u8 = timer.into();
        let mut val = self.dev.read_register(0x07)?;
        // Clear the second and third bits (bit1 & bit2)
        val &= 0xF9; // 0xF9 = 1111_1001 (clears bits 1 and 2)
                     // Set the fast charge timer in bits 1-2
        val |= (timer & 0x03) << 1;
        self.dev.write_register(&[0x07, val])?;
        Ok(())
    }

    /// Gets the fast charge timer
    pub fn get_fast_charge_timer(&mut self) -> Result<FastChargeTimer, PmuSensorError> {
        let val = self.dev.read_register(0x07)?;
        let timer_val = (val >> 1) & 0x03;
        let timer = FastChargeTimer::try_from(timer_val).expect("Invalid timer value");
        Ok(timer)
    }

    /// Sets the JEITA low temperature current
    ///
    /// # Arguments
    /// * `current` - JEITA low temperature current
    pub fn set_jeita_low_temperature_current(
        &mut self,
        current: JeitaLowTemperatureCurrent,
    ) -> Result<(), PmuSensorError> {
        match current {
            JeitaLowTemperatureCurrent::Temp50 => self.dev.clear_register_bit(0x07, 0)?,
            JeitaLowTemperatureCurrent::Temp20 => self.dev.set_register_bit(0x07, 0)?,
        }
        Ok(())
    }

    // REGISTER 0x08
    // IR Compensation Resistor Setting, IR Compensation Voltage Clamp, Thermal Regulation Threshold

    /// Sets the IR compensation resistor
    ///
    /// # Arguments
    /// * `milliohm` - IR compensation resistor in mOhm
    pub fn set_ir_compensation_resistor(&mut self, milliohm: u16) -> Result<(), PmuSensorError> {
        if !milliohm.is_multiple_of(BAT_COMP_STEPS) {
            return Err(PmuSensorError::ResistanceStepInvalid20);
        }

        let resistance = milliohm.clamp(0, BAT_COMP_MAX);
        let mut val = self.dev.read_register(0x08)?;
        val &= 0x1F;
        val |= ((resistance / BAT_COMP_STEPS) as u8) << 5;
        self.dev.write_register(&[0x08, val])?;
        Ok(())
    }

    /// Sets the IR compensation voltage clamp
    ///
    /// # Arguments
    /// * `millivolt` - IR compensation voltage clamp in mV
    pub fn set_ir_compensation_voltage_clamp(
        &mut self,
        millivolt: u16,
    ) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(VCLAMP_STEPS) {
            return Err(PmuSensorError::VoltageStepInvalid32);
        }

        let voltage = millivolt.clamp(0, VCLAMP_MAX);
        let mut val = self.dev.read_register(0x08)?;
        val &= 0xE3;
        val |= ((voltage / VCLAMP_STEPS) as u8) << 2;
        self.dev.write_register(&[0x08, val])?;
        Ok(())
    }

    /// Sets the thermal regulation threshold
    ///
    /// # Arguments
    /// * `threshold` - Thermal regulation threshold
    pub fn set_thermal_regulation_threshold(
        &mut self,
        threshold: ThermalRegThreshold,
    ) -> Result<(), PmuSensorError> {
        let threshold: u8 = threshold.into();
        let mut val = self.dev.read_register(0x08)?;
        val &= 0xFC;
        val |= threshold;
        self.dev.write_register(&[0x08, val])?;
        Ok(())
    }

    // REGISTER 0x09
    // Force Start Input Current Optimizer (ICO), Safety Timer Setting during DPM or Thermal Regulation,Force BATFET off to enable ship mode
    // JEITA High Temperature Voltage Setting, BATFET turn off delay control, BATFET full system reset enable, Current pulse control voltage up enable
    // Current pulse control voltage down enable

    /// Forces the start of the input current optimizer (ICO)
    pub fn force_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x09, 7)?;
        Ok(())
    }

    /// Sets the safety timer behavior during DPM or thermal regulation
    ///
    /// # Arguments
    /// * `slow_down` - If true, safety timer is slowed by 2X during input DPM/thermal regulation (default)
    pub fn set_safety_timer_thermal_regulation(
        &mut self,
        slow_down: bool,
    ) -> Result<(), PmuSensorError> {
        if slow_down {
            self.dev.set_register_bit(0x09, 6)?;
        } else {
            self.dev.clear_register_bit(0x09, 6)?;
        }
        Ok(())
    }

    /// Shuts down the device
    pub fn shutdown(&mut self) -> Result<(), PmuSensorError> {
        self.disable_battery_power_path()
    }

    /// Disables the battery power path
    pub fn disable_battery_power_path(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x09, 5)?;
        Ok(())
    }

    /// Enables the battery power path
    pub fn enable_battery_power_path(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x09, 5)?;
        Ok(())
    }

    /// Sets the JEITA high temperature voltage
    ///
    /// # Arguments
    /// * `use_vreg` - If true, use VREG instead of VREG-200mV during JEITA high temperature
    pub fn set_jeita_high_temp_voltage(&mut self, use_vreg: bool) -> Result<(), PmuSensorError> {
        if use_vreg {
            self.dev.set_register_bit(0x09, 4)?;
        } else {
            self.dev.clear_register_bit(0x09, 4)?;
        }
        Ok(())
    }

    /// Sets the BATFET turn off delay
    ///
    /// # Arguments
    /// * `delay` - If true, BATFET turns off with tSM_DLY delay when BATFET_DIS is set
    pub fn set_batfet_turnoff_delay(&mut self, delay: bool) -> Result<(), PmuSensorError> {
        if delay {
            self.dev.set_register_bit(0x09, 3)?;
        } else {
            self.dev.clear_register_bit(0x09, 3)?;
        }
        Ok(())
    }

    /// Enables or disables the BATFET full system reset
    ///
    /// # Arguments
    /// * `enable` - If true, enables full system reset
    pub fn set_full_system_reset(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.dev.set_register_bit(0x09, 2)?;
        } else {
            self.dev.clear_register_bit(0x09, 2)?;
        }
        Ok(())
    }

    /// Enables or disables the current pulse control voltage up
    ///
    /// # Arguments
    /// * `enable` - If true, enables current pulse control voltage up
    pub fn set_current_pulse_voltage_up(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.dev.set_register_bit(0x09, 1)?;
        } else {
            self.dev.clear_register_bit(0x09, 1)?;
        }
        Ok(())
    }

    /// Enables or disables the current pulse control voltage down
    ///
    /// # Arguments
    /// * `enable` - If true, enables current pulse control voltage down
    pub fn set_current_pulse_voltage_down(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.dev.set_register_bit(0x09, 0)?;
        } else {
            self.dev.clear_register_bit(0x09, 0)?;
        }
        Ok(())
    }

    // REGISTER 0x0A
    // Boost Mode Voltage Regulation, PFM mode allowed in boost mode , Boost Mode Current Limit

    /// Sets the boost mode voltage regulation
    ///
    /// # Arguments
    /// * `millivolt` - Boost mode voltage regulation in mV
    pub fn set_boost_voltage(&mut self, mut millivolt: u16) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(BOOST_VOL_STEP) {
            return Err(PmuSensorError::VoltageStepInvalid64);
        }

        millivolt = millivolt.clamp(BOOST_VOL_MIN, BOOST_VOL_MAX);
        let val = self.dev.read_register(0x0A)?;
        let steps = ((millivolt - BOOST_VOL_BASE) / BOOST_VOL_STEP) as u8;
        let new_val = (val & 0xF0) | (steps << 4);
        self.dev.write_register(&[0x0A, new_val])?;
        Ok(())
    }

    /// Sets the boost mode current limit
    ///
    /// # Arguments
    /// * `limit` - Boost mode current limit
    pub fn set_boost_current_limit(
        &mut self,
        limit: BoostCurrentLimit,
    ) -> Result<(), PmuSensorError> {
        let limit: u8 = limit.into();
        let val = self.dev.read_register(0x0A)?;
        let bits = (val & 0x03) | limit;
        self.dev.write_register(&[0x0A, bits])?;
        Ok(())
    }

    /// Configures PFM mode in boost mode
    ///
    /// # Arguments
    /// * `enable` - If true, allow PFM in boost mode (default)
    pub fn set_boost_mode_pfm(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.dev.clear_register_bit(0x0A, 3)?;
        } else {
            self.dev.set_register_bit(0x0A, 3)?;
        }
        Ok(())
    }

    // REGISTER 0x0B
    // VBUS Status register, N/A Charging Status, Power Good Status , VSYS Regulation Status

    /// Gets the charge status
    pub fn get_charge_status(&mut self) -> Result<ChargeStatus, PmuSensorError> {
        let val = self.dev.read_register(0x0B)?;
        let result = (val >> 3) & 0x03;
        let result = ChargeStatus::try_from(result).expect("Invalid charge status");
        Ok(result)
    }

    /// Checks if VBUS is present
    pub fn is_vbus_in(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.get_bus_status()?;
        Ok(val != BusStatus::NoInput)
    }

    /// Checks if device is in OTG mode
    pub fn is_otg(&mut self) -> Result<bool, PmuSensorError> {
        Ok(self.get_bus_status()? == BusStatus::Otg)
    }

    /// Checks if device is currently charging
    pub fn is_charging(&mut self) -> Result<bool, PmuSensorError> {
        Ok(self.get_charge_status()? != ChargeStatus::NoCharge)
    }

    /// Checks if charging is complete
    pub fn is_charge_done(&mut self) -> Result<bool, PmuSensorError> {
        Ok(self.get_charge_status()? == ChargeStatus::Done)
    }

    /// Checks power good status
    pub fn is_power_good(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x0B, 2)?;
        Ok(result)
    }

    /// Gets the bus status
    pub fn get_bus_status(&mut self) -> Result<BusStatus, PmuSensorError> {
        let val = self.dev.read_register(0x0B)?;
        let result = (val >> 5) & 0x07;
        let result = BusStatus::try_from(result).expect("Invalid bus status");
        Ok(result)
    }

    // REGISTER 0x0C TODO
    // Watchdog Fault Status, Boost Mode Fault Status, Charge Fault Status, Battery Fault Status, NTC Fault Status

    /// Checks the watchdog fault status
    pub fn is_watchdog_fault(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.dev.read_register(0x0C)?;
        Ok(((val >> 7) & 0x01) != 0)
    }

    /// Checks the boost mode fault status
    pub fn is_boost_fault(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.dev.read_register(0x0C)?;
        Ok(((val >> 6) & 0x01) != 0)
    }

    /// Gets the charge fault status
    pub fn get_charge_fault(&mut self) -> Result<ChargeFaultStatus, PmuSensorError> {
        let val = self.dev.read_register(0x0C)?;
        let fault = (val >> 4) & 0x03;
        let fault = ChargeFaultStatus::try_from(fault).expect("Invalid charge fault status");
        Ok(fault)
    }

    /// Checks the battery fault status
    pub fn is_battery_fault(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.dev.read_register(0x0C)?;
        Ok(((val >> 3) & 0x01) != 0)
    }

    /// Checks the NTC fault status
    pub fn is_ntc_fault(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.dev.read_register(0x0C)?;
        Ok((val & 0x07) != 0)
    }

    /// Gets the NTC status as a human-readable string
    pub fn get_ntc_status_string(&mut self) -> Result<&'static str, PmuSensorError> {
        let status = self.dev.read_register(0x0C)? & 0x07;

        if self.is_otg()? {
            // Boost mode
            match status {
                x if x == NtcBoostStatus::Normal as u8 => Ok("Boost mode NTC normal"),
                x if x == NtcBoostStatus::Cold as u8 => Ok("Boost mode NTC cold"),
                x if x == NtcBoostStatus::Hot as u8 => Ok("Boost mode NTC hot"),
                _ => Ok("Unknown"),
            }
        } else {
            // Buck mode
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

    // REGISTER 0x0D
    // VINDPM Threshold Setting Method, bsolute VINDPM Threshold

    /// Sets the VINDPM threshold setting method
    ///
    /// # Arguments
    /// * `relative` - true for relative (default), false for absolute
    pub fn set_vindpm_threshold_method(&mut self, relative: bool) -> Result<(), PmuSensorError> {
        if relative {
            self.dev.clear_register_bit(0x0D, 7)?;
        } else {
            self.dev.set_register_bit(0x0D, 7)?;
        }
        Ok(())
    }

    /// Sets the absolute VINDPM threshold voltage
    ///
    /// # Arguments
    /// * `millivolt` - Voltage in mV (2600-15300mV in 100mV steps)
    pub fn set_vindpm_threshold(&mut self, mut millivolt: u16) -> Result<(), PmuSensorError> {
        if !millivolt.is_multiple_of(VINDPM_VOL_STEPS) {
            return Err(PmuSensorError::VoltageStepInvalid100);
        }

        millivolt = millivolt.clamp(VINDPM_VOL_MIN, VINDPM_VOL_MAX);
        let val = self.dev.read_register(0x0D)?;
        let steps = ((millivolt - VINDPM_VOL_BASE) / VINDPM_VOL_STEPS) as u8;
        let new_val = (val & 0x80) | steps;
        self.dev.write_register(&[0x0D, new_val])?;
        Ok(())
    }

    /// Gets the absolute VINDPM threshold voltage
    pub fn get_vindpm_threshold(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.dev.read_register(0x0D)?;
        let steps = val & 0x7F;
        Ok((steps as u16 * VINDPM_VOL_STEPS) + VINDPM_VOL_BASE)
    }

    // REGISTER 0x0E
    // ADC conversion of Battery Voltage (VBAT)

    /// Checks if thermal regulation is normal
    /// Returns true for normal operation, false if in thermal regulation
    pub fn is_thermal_regulation_normal(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x0E, 7)?;
        Ok(!result)
    }

    /// Gets the battery voltage in millivolts
    pub fn get_battery_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let data: u8 = self.dev.read_register(0x0E)?;
        if data == 0 {
            return Ok(0);
        }
        let thermal_regulation = ((data >> 7) & 0x01) != 0;
        if thermal_regulation {
            return Ok(0);
        }
        let data = data & 0x7F;
        let vbat = (data as u16) * 20 + 2304;

        Ok(vbat)
    }

    /// Estimates battery percentage based on voltage
    /// Uses a simplified Li-ion voltage curve
    /// Returns percentage (0-100)
    pub fn get_battery_percentage(&mut self) -> Result<u8, PmuSensorError> {
        let voltage = self.get_battery_voltage()?;

        // Li-ion voltage discharge curve (approximate)
        // 4.2V = 100%, 4.0V = ~90%, 3.8V = ~60%, 3.6V = ~30%, 3.4V = ~10%, 3.0V = 0%
        let percentage = if voltage >= 4200 {
            100
        } else if voltage >= 4000 {
            90 + ((voltage - 4000) * 10 / 200) // 4.0V-4.2V: 90-100%
        } else if voltage >= 3800 {
            60 + ((voltage - 3800) * 30 / 200) // 3.8V-4.0V: 60-90%
        } else if voltage >= 3600 {
            30 + ((voltage - 3600) * 30 / 200) // 3.6V-3.8V: 30-60%
        } else if voltage >= 3400 {
            10 + ((voltage - 3400) * 20 / 200) // 3.4V-3.6V: 10-30%
        } else if voltage >= 3000 {
            (voltage - 3000) * 10 / 400 // 3.0V-3.4V: 0-10%
        } else {
            0
        };

        Ok(percentage.min(100) as u8)
    }

    // REGISTER 0x0F
    // ADDC conversion of System Voltage (VSYS)

    /// Gets the system voltage in millivolts
    pub fn get_sys_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let data = self.dev.read_register(0x0F)?;
        let data = data & 0x7F;
        if data == 0 {
            return Ok(0);
        }
        let vsys = (data as u16) * 20 + 2304;

        Ok(vsys)
    }

    // REGISTER 0x10
    // ADC conversion of TS Voltage (TS) as percentage of REGN

    /// Gets the battery temperature in Celsius from the NTC thermistor
    pub fn get_temperature(&mut self) -> Result<f64, PmuSensorError> {
        let data = self.dev.read_register(0x10)?;
        let data = data & 0x7F;
        let ntc_percent = (data as f64) * 0.465_f64 + 21_f64;

        // Convert percentage to resistance ratio
        let r_ratio = (100.0 - ntc_percent) / ntc_percent;

        // Converts NTC thermistor resistance ratio to temperature
        // using Steinhart-Hart equation
        fn r_to_temp(r: f64) -> f64 {
            const BETA: f64 = 3950.0; // Beta value for typical 10k NTC
            const T0: f64 = 298.15; // 25°C in Kelvin
            const R0: f64 = 10000.0; // 10k NTC reference resistance

            // Calculate actual resistance using R0
            let r_ntc = r * R0;

            // Steinhart-Hart simplified equation with actual resistance
            let temp_kelvin = 1.0 / (1.0 / T0 + (1.0 / BETA) * log(r_ntc / R0));
            let temp_celsius = temp_kelvin - 273.15;

            // Round to nearest 0.5°C
            round(temp_celsius * 2.0) / 2.0
        }

        Ok(r_to_temp(r_ratio))
    }

    // REGISTER 0x11
    // ADC conversion of VBUS voltage (VBUS) , VBUS Good Status

    /// Checks if VBUS is good (within regulation)
    pub fn is_vbus_good(&mut self) -> Result<bool, PmuSensorError> {
        let data = self.dev.read_register(0x11)?;
        Ok(((data >> 7) & 0x01) != 0)
    }

    /// Gets the USB bus voltage in millivolts
    pub fn get_vbus_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let data = self.dev.read_register(0x11)?;

        let vbus_attached = ((data >> 7) & 0x01) == 0;
        if vbus_attached {
            return Ok(0);
        }
        let data = data & 0x7F;
        let vbus = (data as u16) * 100 + 2600;

        Ok(vbus)
    }

    // REGISTER 0x12
    // ADC conversion of Charge Current (IBAT) when VBAT > VBATSHORT

    /// Gets the charge current in mA when VBAT > VBATSHORT
    /// Note: If the charger is disconnected, the register retains the last value
    pub fn get_charge_current(&mut self) -> Result<u16, PmuSensorError> {
        // If not charging, return 0
        if self.get_charge_status()? == ChargeStatus::NoCharge {
            return Ok(0);
        }

        // Read register 0x12, which contains the ADC conversion result for charge current.
        let val = self.dev.read_register(0x12)?;

        // In the C code:
        // volt = BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB
        // where:
        //    BQ2589X_ICHGR_MASK  = 0x7F
        //    BQ2589X_ICHGR_SHIFT = 0
        //    BQ2589X_ICHGR_BASE  = 0
        //    BQ2589X_ICHGR_LSB   = 50
        //
        // Thus the current in mA is:
        let current = ((val & 0x7F) as u16) * CHG_STEP_VAL;
        Ok(current)
    }

    // REGISTER 0x13
    // VINDPM Status, IINDPM Status, Input Current Limit in effect while Input Current Optimizer

    /// Checks if Dynamic Power Management is active
    pub fn is_dynamic_power_management(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x13, 7)?;
        Ok(result)
    }

    /// Checks if Input Current Limit is active
    pub fn is_input_current_limit(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x13, 6)?;
        Ok(result)
    }

    /// Sets the Input Current Limit for the optimizer (100-3250mA)
    ///
    /// # Arguments
    /// * `milliampere` - Current limit in mA
    pub fn set_input_current_limit_optimizer(
        &mut self,
        mut milliampere: u16,
    ) -> Result<(), PmuSensorError> {
        if !milliampere.is_multiple_of(IN_CURRENT_OPT_STEP) {
            return Err(PmuSensorError::CurrentStepInvalid50);
        }

        milliampere = milliampere.clamp(IN_CURRENT_OPT_MIN, IN_CURRENT_OPT_MAX);
        let val = self.dev.read_register(0x13)?;
        let steps = ((milliampere - IN_CURRENT_OPT_MIN) / IN_CURRENT_OPT_STEP) as u8;
        let new_val = (val & 0x3F) | (steps << 6);
        self.dev.write_register(&[0x13, new_val])?;
        Ok(())
    }

    /// Gets the Input Current Limit in effect while ICO is enabled
    /// This is the detected current limit from the Input Current Optimizer
    pub fn get_input_current_limit_in_effect(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.dev.read_register(0x13)?;
        let steps = val & 0x3F;
        Ok((steps as u16 * IN_CURRENT_OPT_STEP) + IN_CURRENT_OPT_MIN)
    }

    // REGISTER 0x14
    // Register Reset, Input Current Optimizer (ICO) Status , Device Configuration, Temperature Profile, Device Revision: 10

    /// Resets the device to the default configuration
    pub fn reset_default(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x14, 7)?;
        Ok(())
    }

    /// Checks if Input Current Optimization is in progress
    pub fn is_input_current_optimizer(&mut self) -> Result<bool, PmuSensorError> {
        let result = self.dev.get_register_bit(0x14, 6)?;
        Ok(result)
    }

    /// Gets the device configuration
    pub fn get_device_config(&mut self) -> Result<u8, PmuSensorError> {
        let val = self.dev.read_register(0x14)?;
        Ok((val >> 3) & 0x03)
    }

    /// Gets the temperature profile setting
    /// Returns true if JEITA profile is enabled
    pub fn get_temperature_profile(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.dev.read_register(0x14)?;
        Ok(((val >> 2) & 0x01) != 0)
    }

    /// Gets the chip ID
    pub fn get_chip_id(&mut self) -> Result<u8, PmuSensorError> {
        let result = self.dev.read_register(0x14)?;
        Ok(result & 0x03)
    }

    /// Gets information about the PMU
    pub fn get_info(&mut self) -> Result<String, PmuSensorError> {
        let is_vbus_present = self.is_vbus_in()?;

        let is_vbus_present = match is_vbus_present {
            true => "Yes",
            false => "No",
        };

        let mut text = format!("CHG state: {}\n", self.get_charge_status()?);

        text.push_str(&format!("USB PlugIn: {is_vbus_present}\n"));
        text.push_str(&format!("Bus state: {}\n", self.get_bus_status()?));

        let battery_voltage = self.get_battery_voltage()?;
        let battery_percentage = self.get_battery_percentage()?;
        text.push_str(&format!(
            "Battery: {}mV ({}%)\n",
            battery_voltage, battery_percentage
        ));
        text.push_str(&format!("Charge current: {}mA\n", self.get_charge_current()?));
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

/// Errors that can occur when interacting with the BQ25896
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PmuSensorError {
    /// Failed to initialize the device
    Init,
    // Voltage step invalid (must be multiple of 16)
    VoltageStepInvalid16,
    // Voltage step invalid (must be multiple of 100)
    VoltageStepInvalid100,
    // Voltage step invalid (must be multiple of 32)
    VoltageStepInvalid32,
    // Voltage step invalid (must be multiple of 64)
    VoltageStepInvalid64,
    // Current step invalid (must be multiple of 64)
    CurrentStepInvalid64,
    // Current step invalid (must be multiple of 100)
    CurrentStepInvalid100,
    // Current step invalid (must be multiple of 50)
    CurrentStepInvalid50,
    // Needs to be inbetween  3000 and 3700mV
    PowerDownVoltageInvalid,
    // Steps of 20mOhm
    ResistanceStepInvalid20,
    // Invalid register value
    I2CError,
}

impl<E> From<E> for PmuSensorError
where
    E: Error,
{
    fn from(_: E) -> Self {
        PmuSensorError::I2CError
    }
}
/// Status of the power input source
#[derive(Debug, Clone, Copy, PartialEq, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum BusStatus {
    /// No power input connected
    NoInput = 0,
    /// Standard USB host (500mA max)
    UsbSdp = 1,
    /// AC/DC power adapter
    Adapter = 2,
    /// USB OTG mode - device is power source
    Otg = 3,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BoostFreq {
    Freq500KHz,
    Freq1500KHz,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FastChargeThreshold {
    Volt2V8,
    Volt3V0,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RechargeThresholdOffset {
    Offset100mV,
    Offset200mV,
}

#[derive(Debug, Clone, Copy, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WatchdogConfig {
    TimerOut40Sec = 0x10,
    TimerOut80Sec = 0x20,
    TimerOut160Sec = 0x30,
}

#[derive(Debug, Clone, Copy, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FastChargeTimer {
    Hours5 = 0,
    Hours8 = 1,
    Hours12 = 2,
    Hours20 = 3,
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum JeitaLowTemperatureCurrent {
    Temp50,
    Temp20,
}

#[derive(Debug, Clone, Copy, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ThermalRegThreshold {
    Celsius60 = 0,
    Celsius80 = 1,
    Celsius100 = 2,
    Celsius120 = 3,
}

#[derive(Debug, Clone, Copy, IntoPrimitive, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum BoostCurrentLimit {
    Limit500mA = 0x00,
    Limit750mA = 0x01,
    Limit1200mA = 0x02,
    Limit1400mA = 0x03,
    Limit1650mA = 0x04,
    Limit1875mA = 0x05,
    Limit2150mA = 0x06,
}

#[derive(Debug, Clone, Copy, PartialEq, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum NtcBoostStatus {
    Normal = 0,
    Cold = 1,
    Hot = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum NtcBuckStatus {
    Normal = 0,
    Warm = 2,
    Cold = 3,
    ColdAlt = 5,
    Hot = 6,
}

#[derive(Debug, Clone, Copy, PartialEq, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChargeFaultStatus {
    Normal = 0x00,
    InputFault = 0x01,
    ThermalShutdown = 0x02,
    SafetyTimer = 0x03,
}

impl Display for BusStatus {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        match self {
            BusStatus::NoInput => write!(f, "No input"),
            BusStatus::UsbSdp => write!(f, "USB Host SDP"),
            BusStatus::Adapter => write!(f, "Adapter"),
            BusStatus::Otg => write!(f, "OTG"),
        }
    }
}

impl Display for FastChargeTimer {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        match self {
            FastChargeTimer::Hours5 => write!(f, "5 hours"),
            FastChargeTimer::Hours8 => write!(f, "8 hours"),
            FastChargeTimer::Hours12 => write!(f, "12 hours"),
            FastChargeTimer::Hours20 => write!(f, "20 hours"),
        }
    }
}

impl Display for BoostFreq {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        match self {
            BoostFreq::Freq500KHz => write!(f, "500 kHz"),
            BoostFreq::Freq1500KHz => write!(f, "1500 kHz"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChargeStatus {
    NoCharge,
    PreCharge,
    FastCharge,
    Done,
    Unknown,
}

impl Display for ChargeStatus {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        match self {
            ChargeStatus::NoCharge => write!(f, "Not charging"),
            ChargeStatus::PreCharge => write!(f, "Pre-charge"),
            ChargeStatus::FastCharge => write!(f, "Fast charging"),
            ChargeStatus::Done => write!(f, "Charge Termination Done"),
            ChargeStatus::Unknown => write!(f, "Unknown"),
        }
    }
}

/// Boost mode hot temperature monitor threshold
#[derive(Debug, Clone, Copy, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BoostHotThreshold {
    VBHOT1 = 0x00,   // Threshold (34.75%) (default)
    VBHOT0 = 0x01,   // Threshold (Typ. 37.75%)
    VBHOT2 = 0x02,   // Threshold (Typ. 31.25%)
    Disabled = 0x03, // Disable boost mode thermal protection
}

#[derive(Debug, Clone, Copy, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BoostColdThreshold {
    VBCOLD0 = 0x00, // (Typ. 77%) (default)
    VBCOLD1 = 0x01, // (Typ. 80%)
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ExitBoostModeVolt {
    MiniVolt2V9,
    MiniVolt2V5,
}
