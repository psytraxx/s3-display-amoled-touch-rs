use alloc::{format, string::String};
use core::fmt::{self, Display, Formatter};
use embedded_hal::i2c::I2c;
use libm::{log, round};
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
    dev: BQ25896Device<I2C>,
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
            dev: BQ25896Device::new(i2c, adr),
            user_disable_charge: false,
        };
        instance.detect_pmu(adr)?;
        Ok(instance)
    }

    /// Detects the PMU
    ///
    /// # Arguments
    /// * `adr` - I2C device address
    fn detect_pmu(&mut self, adr: u8) -> Result<(), PmuSensorError> {
        if self.dev.write_register(&[adr]).is_ok() {
            Ok(())
        } else {
            Err(PmuSensorError::Init)
        }
    }

    // Register 0x00
    // Enable HIZ Mode, Enable ILIM Pin, Input Current Limit

    /// Enables HIZ mode
    pub fn set_hiz_mode(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x00, 7)
    }

    /// Exits HIZ mode
    pub fn exit_hiz_mode(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x00, 7)
    }

    /// Checks if HIZ mode is enabled
    pub fn is_hiz_mode(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x00, 7)
    }

    /// Enables the current limit pin
    pub fn enable_current_limit_pin(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x00, 6)
    }

    /// Disables the current limit pin
    pub fn disable_current_limit_pin(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x00, 6)
    }

    /// Checks if the current limit pin is enabled
    pub fn is_enable_current_limit_pin(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x00, 6)
    }

    /// Sets the input current limit
    ///
    /// # Arguments
    /// * `milliampere` - Current limit in mA
    pub fn set_input_current_limit(&mut self, milliampere: u16) -> Result<(), PmuSensorError> {
        // Validate input is multiple of step size
        if milliampere % IN_CURRENT_STEP != 0 {
            return Err(PmuSensorError::CurrentStepInvalid50);
        }

        // Clamp value to valid range
        let ma = milliampere.clamp(IN_CURRENT_MIN, IN_CURRENT_MAX);

        // Read current register value
        let mut reg_val = self.dev.read_register(0x00)?;

        // Clear bottom 6 bits while preserving top 2 bits
        reg_val &= 0xC0;

        // Calculate new value (offset from minimum, divided by step size)
        let current_bits = ((ma - IN_CURRENT_MIN) / IN_CURRENT_MAX) as u8;

        // Combine preserved bits with new value
        reg_val |= current_bits;

        // Write back to register
        self.dev.write_register(&[0x00, reg_val])
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
        self.dev.write_register(&[0x01, data])
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
        self.dev.write_register(&[0x01, data])
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
        if millivolt % IN_CURRENT_OFFSET_STEP != 0 {
            return Err(PmuSensorError::CurrentStepInvalid100);
        }

        if millivolt > IN_CURRENT_OFFSET_MAX {
            millivolt = IN_CURRENT_OFFSET_MAX;
        }

        let steps = millivolt / IN_CURRENT_OFFSET_STEP;
        let val = self.dev.read_register(0x01)?;
        let data = (val & 0xE0) | (steps as u8);
        self.dev.write_register(&[0x01, data])
    }

    // REGISTER 0x02
    // ADC Conversion Start Control, ADC Conversion Rate Selection, Boost Mode Frequency Selection
    // Input Current Optimizer (ICO) Enable, Force Input Detection, Automatic Input Detection Enable

    /// Enables ADC conversion for voltage and current monitoring
    pub fn set_adc_enabled(&mut self) -> Result<(), PmuSensorError> {
        let mut data = self.dev.read_register(0x02)?;
        data |= 1 << 7; // Start ADC conversion
        data |= 1 << 6; // Set continuous conversion
        self.dev.write_register(&[0x02, data])
    }

    /// Disables ADC conversion
    pub fn set_adc_disabled(&mut self) -> Result<(), PmuSensorError> {
        let mut data = self.dev.read_register(0x02)?;
        data &= !(1 << 7); // Clear ADC conversion bit
        self.dev.write_register(&[0x02, data])
    }

    /// Sets the boost frequency
    ///
    /// # Arguments
    /// * `freq` - Boost frequency
    pub fn set_boost_freq(&mut self, freq: BoostFreq) -> Result<(), PmuSensorError> {
        match freq {
            BoostFreq::Freq500KHz => self.dev.set_register_bit(0x02, 5),
            BoostFreq::Freq1500KHz => self.dev.clear_register_bit(0x02, 5),
        }
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
        self.dev.set_register_bit(0x02, 4)
    }

    /// Disables the input current optimizer
    pub fn disable_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x02, 4)
    }

    /// Enables input detection
    pub fn enable_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x02, 1)
    }

    /// Disables input detection
    pub fn disable_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x02, 1)
    }

    /// Checks if input detection is enabled
    pub fn is_input_detection_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x02, 1)
    }

    /// Enables automatic input detection
    pub fn enable_automatic_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x02, 0)
    }

    /// Disables automatic input detection
    pub fn disable_automatic_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x02, 0)
    }

    /// Checks if automatic input detection is enabled
    pub fn is_automatic_input_detection_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x02, 0)
    }

    // REGISTER 0x03
    // Battery Load (IBATLOAD) Enable, I2C Watchdog Timer Reset, Boost (OTG) Mode Configuration
    // Charge Enable Configuration,  Minimum System Voltage Limit, Minimum Battery Voltage (falling) to exit boost mode

    /// Checks if battery load is enabled
    pub fn is_bat_load_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x03, 7)
    }

    /// Disables battery load
    pub fn disable_bat_load(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x03, 7)
    }

    /// Enables battery load
    pub fn enable_bat_load(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x03, 7)
    }

    /// Feeds the watchdog timer
    pub fn feed_watchdog(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x03, 6)
    }

    /// Checks if OTG mode is enabled
    pub fn is_otg_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x03, 5)
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
        self.dev.set_register_bit(0x03, 4)
    }

    /// Disables charging
    pub fn set_charge_disabled(&mut self) -> Result<(), PmuSensorError> {
        self.user_disable_charge = true;
        self.dev.clear_register_bit(0x03, 4)
    }

    /// Checks if charging is enabled
    pub fn is_charge_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x03, 4)
    }

    /// Sets the system power down voltage
    ///
    /// # Arguments
    /// * `millivolt` - Power down voltage in mV
    pub fn set_sys_power_down_voltage(&mut self, millivolt: u16) -> Result<(), PmuSensorError> {
        if millivolt % SYS_VOL_STEPS != 0 {
            return Err(PmuSensorError::VoltageStepInvalid100);
        }
        if !(SYS_VOFF_VOL_MIN..=SYS_VOFF_VOL_MAX).contains(&millivolt) {
            return Err(PmuSensorError::PowerDownVoltageInvalid);
        }

        let mut val = self.dev.read_register(0x03)?;
        val &= 0xF1;
        val |= ((millivolt - SYS_VOFF_VOL_MIN) / SYS_VOL_STEPS) as u8;
        val <<= 1;
        self.dev.write_register(&[0x03, val])
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
            ExitBoostModeVolt::MiniVolt2V9 => self.dev.clear_register_bit(0x03, 0),
            ExitBoostModeVolt::MiniVolt2V5 => self.dev.set_register_bit(0x03, 0),
        }
    }

    // REGISTER 0x04
    // Current pulse control Enable, Fast Charge Current Limit

    /// Enables current pulse control
    pub fn set_current_pulse_control_enabled(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x04, 7)
    }

    /// Disables current pulse control
    pub fn set_current_pulse_control_disabled(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x04, 7)
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
        if milliampere % FAST_CHG_CUR_STEP != 0 {
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
        self.dev.write_register(&[0x04, val])
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
        if milliampere % PRE_CHG_CUR_STEP != 0 {
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
        self.dev.write_register(&[0x05, val])
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
        if milliampere % TERM_CHG_CUR_STEP != 0 {
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
        self.dev.write_register(&[0x05, val])
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
        if target_voltage % CHG_VOL_STEP != 0 {
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
        self.dev.write_register(&[0x06, val])
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
            FastChargeThreshold::Volt2V8 => self.dev.clear_register_bit(0x06, 1),
            FastChargeThreshold::Volt3V0 => self.dev.set_register_bit(0x06, 1),
        }
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
            RechargeThresholdOffset::Offset100mV => self.dev.clear_register_bit(0x06, 0),
            RechargeThresholdOffset::Offset200mV => self.dev.set_register_bit(0x06, 0),
        }
    }

    // REGISTER 0x07
    // Charging Termination Enable, STAT Pin Disable , I2C Watchdog Timer Setting, Charging Safety Timer Enable
    // Fast Charge Timer Setting, JEITA Low Temperature Current Setting

    /// Enables charging termination
    pub fn enable_charging_termination(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x07, 7)
    }

    /// Disables charging termination
    pub fn disable_charging_termination(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x07, 7)
    }

    /// Checks if charging termination is enabled
    pub fn is_charging_termination_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x07, 7)
    }

    /// Disables the STAT pin
    pub fn disable_stat_pin(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x07, 6)
    }

    /// Enables the STAT pin
    pub fn enable_stat_pin(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x07, 6)
    }

    /// Checks if the STAT pin is enabled
    pub fn is_stat_pin_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x07, 6)
    }

    /// Disables the watchdog timer
    pub fn disable_watchdog(&mut self) -> Result<(), PmuSensorError> {
        let mut val = self.dev.read_register(0x07)?;
        val &= 0xCF;
        self.dev.write_register(&[0x07, val])
    }

    /// Enables the watchdog timer
    ///
    /// # Arguments
    /// * `config` - Watchdog timer configuration
    pub fn enable_watchdog(&mut self, config: WatchdogConfig) -> Result<(), PmuSensorError> {
        let mut val = self.dev.read_register(0x07)?;
        val &= 0xCF;
        let bits = match config {
            WatchdogConfig::TimerOut40Sec => 0x10,
            WatchdogConfig::TimerOut80Sec => 0x20,
            WatchdogConfig::TimerOut160Sec => 0x30,
        };
        self.dev.write_register(&[0x07, val | bits])
    }

    /// Disables the charging safety timer
    pub fn disable_charging_safety_timer(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x07, 3)
    }

    /// Enables the charging safety timer
    pub fn enable_charging_safety_timer(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x07, 3)
    }

    /// Checks if the charging safety timer is enabled
    pub fn is_charging_safety_timer_enabled(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x07, 3)
    }

    /// Sets the fast charge timer
    ///
    /// # Arguments
    /// * `timer` - Fast charge timer
    pub fn set_fast_charge_timer(&mut self, timer: FastChargeTimer) -> Result<(), PmuSensorError> {
        let mut val = self.dev.read_register(0x07)?;
        val &= 0xF1;
        val |= (timer as u8) << 1;
        self.dev.write_register(&[0x07, val])
    }

    /// Gets the fast charge timer
    pub fn get_fast_charge_timer(&mut self) -> Result<FastChargeTimer, PmuSensorError> {
        let val = self.dev.read_register(0x07)?;
        let timer_val = (val & 0x0E) >> 1;
        Ok(match timer_val {
            0 => FastChargeTimer::Hours5,
            1 => FastChargeTimer::Hours8,
            2 => FastChargeTimer::Hours12,
            _ => FastChargeTimer::Hours20,
        })
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
            JeitaLowTemperatureCurrent::Temp50 => self.dev.clear_register_bit(0x07, 0),
            JeitaLowTemperatureCurrent::Temp20 => self.dev.set_register_bit(0x07, 0),
        }
    }

    // REGISTER 0x08
    // IR Compensation Resistor Setting, IR Compensation Voltage Clamp, Thermal Regulation Threshold

    /// Sets the IR compensation resistor
    ///
    /// # Arguments
    /// * `milliohm` - IR compensation resistor in mOhm
    pub fn set_ir_compensation_resistor(&mut self, milliohm: u16) -> Result<(), PmuSensorError> {
        if milliohm % BAT_COMP_STEPS != 0 {
            return Err(PmuSensorError::ResistanceStepInvalid20);
        }

        let resistance = milliohm.clamp(0, BAT_COMP_MAX);
        let mut val = self.dev.read_register(0x08)?;
        val &= 0x1F;
        val |= ((resistance / BAT_COMP_STEPS) as u8) << 5;
        self.dev.write_register(&[0x08, val])
    }

    /// Sets the IR compensation voltage clamp
    ///
    /// # Arguments
    /// * `millivolt` - IR compensation voltage clamp in mV
    pub fn set_ir_compensation_voltage_clamp(
        &mut self,
        millivolt: u16,
    ) -> Result<(), PmuSensorError> {
        if millivolt % VCLAMP_STEPS != 0 {
            return Err(PmuSensorError::VoltageStepInvalid32);
        }

        let voltage = millivolt.clamp(0, VCLAMP_MAX);
        let mut val = self.dev.read_register(0x08)?;
        val &= 0xE3;
        val |= ((voltage / VCLAMP_STEPS) as u8) << 2;
        self.dev.write_register(&[0x08, val])
    }

    /// Sets the thermal regulation threshold
    ///
    /// # Arguments
    /// * `threshold` - Thermal regulation threshold
    pub fn set_thermal_regulation_threshold(
        &mut self,
        threshold: ThermalRegThreshold,
    ) -> Result<(), PmuSensorError> {
        let mut val = self.dev.read_register(0x08)?;
        val &= 0xFC;
        val |= threshold as u8;
        self.dev.write_register(&[0x08, val])
    }

    // REGISTER 0x09
    // Force Start Input Current Optimizer (ICO), Safety Timer Setting during DPM or Thermal Regulation,Force BATFET off to enable ship mode
    // JEITA High Temperature Voltage Setting, BATFET turn off delay control, BATFET full system reset enable, Current pulse control voltage up enable
    // Current pulse control voltage down enable

    /// Forces the start of the input current optimizer (ICO)
    pub fn force_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x09, 7)
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
            self.dev.set_register_bit(0x09, 6)
        } else {
            self.dev.clear_register_bit(0x09, 6)
        }
    }

    /// Shuts down the device
    pub fn shutdown(&mut self) -> Result<(), PmuSensorError> {
        self.disable_battery_power_path()
    }

    /// Disables the battery power path
    pub fn disable_battery_power_path(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x09, 5)
    }

    /// Enables the battery power path
    pub fn enable_battery_power_path(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x09, 5)
    }

    /// Sets the JEITA high temperature voltage
    ///
    /// # Arguments
    /// * `use_vreg` - If true, use VREG instead of VREG-200mV during JEITA high temperature
    pub fn set_jeita_high_temp_voltage(&mut self, use_vreg: bool) -> Result<(), PmuSensorError> {
        if use_vreg {
            self.dev.set_register_bit(0x09, 4)
        } else {
            self.dev.clear_register_bit(0x09, 4)
        }
    }

    /// Sets the BATFET turn off delay
    ///
    /// # Arguments
    /// * `delay` - If true, BATFET turns off with tSM_DLY delay when BATFET_DIS is set
    pub fn set_batfet_turnoff_delay(&mut self, delay: bool) -> Result<(), PmuSensorError> {
        if delay {
            self.dev.set_register_bit(0x09, 3)
        } else {
            self.dev.clear_register_bit(0x09, 3)
        }
    }

    /// Enables or disables the BATFET full system reset
    ///
    /// # Arguments
    /// * `enable` - If true, enables full system reset
    pub fn set_full_system_reset(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.dev.set_register_bit(0x09, 2)
        } else {
            self.dev.clear_register_bit(0x09, 2)
        }
    }

    /// Enables or disables the current pulse control voltage up
    ///
    /// # Arguments
    /// * `enable` - If true, enables current pulse control voltage up
    pub fn set_current_pulse_voltage_up(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.dev.set_register_bit(0x09, 1)
        } else {
            self.dev.clear_register_bit(0x09, 1)
        }
    }

    /// Enables or disables the current pulse control voltage down
    ///
    /// # Arguments
    /// * `enable` - If true, enables current pulse control voltage down
    pub fn set_current_pulse_voltage_down(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.dev.set_register_bit(0x09, 0)
        } else {
            self.dev.clear_register_bit(0x09, 0)
        }
    }

    // REGISTER 0x0A
    // Boost Mode Voltage Regulation, PFM mode allowed in boost mode , Boost Mode Current Limit

    /// Sets the boost mode voltage regulation
    ///
    /// # Arguments
    /// * `millivolt` - Boost mode voltage regulation in mV
    pub fn set_boost_voltage(&mut self, mut millivolt: u16) -> Result<(), PmuSensorError> {
        if millivolt % BOOST_VOL_STEP != 0 {
            return Err(PmuSensorError::VoltageStepInvalid64);
        }

        millivolt = millivolt.clamp(BOOST_VOL_MIN, BOOST_VOL_MAX);
        let val = self.dev.read_register(0x0A)?;
        let steps = ((millivolt - BOOST_VOL_BASE) / BOOST_VOL_STEP) as u8;
        let new_val = (val & 0xF0) | (steps << 4);
        self.dev.write_register(&[0x0A, new_val])
    }

    /// Sets the boost mode current limit
    ///
    /// # Arguments
    /// * `limit` - Boost mode current limit
    pub fn set_boost_current_limit(
        &mut self,
        limit: BoostCurrentLimit,
    ) -> Result<(), PmuSensorError> {
        let val = self.dev.read_register(0x0A)?;
        let new_val = (val & 0x03) | (limit as u8);
        self.dev.write_register(&[0x0A, new_val])
    }

    /// Configures PFM mode in boost mode
    ///
    /// # Arguments
    /// * `enable` - If true, allow PFM in boost mode (default)
    pub fn set_boost_mode_pfm(&mut self, enable: bool) -> Result<(), PmuSensorError> {
        if enable {
            self.dev.clear_register_bit(0x0A, 3)
        } else {
            self.dev.set_register_bit(0x0A, 3)
        }
    }

    // REGISTER 0x0B
    // VBUS Status register, N/A Charging Status, Power Good Status , VSYS Regulation Status

    /// Gets the charge status
    pub fn get_charge_status(&mut self) -> Result<ChargeStatus, PmuSensorError> {
        let val = self.dev.read_register(0x0B)?;
        let result = (val >> 3) & 0x03;
        Ok(result.into())
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
        self.dev.get_register_bit(0x0B, 2)
    }

    /// Gets the bus status
    pub fn get_bus_status(&mut self) -> Result<BusStatus, PmuSensorError> {
        let val = self.dev.read_register(0x0B)?;
        let result = (val >> 5) & 0x07;
        Ok(result.into())
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
        Ok(match fault {
            0 => ChargeFaultStatus::Normal,
            1 => ChargeFaultStatus::InputFault,
            2 => ChargeFaultStatus::ThermalShutdown,
            3 => ChargeFaultStatus::SafetyTimer,
            _ => ChargeFaultStatus::Normal,
        })
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
            self.dev.clear_register_bit(0x0D, 7)
        } else {
            self.dev.set_register_bit(0x0D, 7)
        }
    }

    /// Sets the absolute VINDPM threshold voltage
    ///
    /// # Arguments
    /// * `millivolt` - Voltage in mV (2600-15300mV in 100mV steps)
    pub fn set_vindpm_threshold(&mut self, mut millivolt: u16) -> Result<(), PmuSensorError> {
        if millivolt % VINDPM_VOL_STEPS != 0 {
            return Err(PmuSensorError::VoltageStepInvalid100);
        }

        millivolt = millivolt.clamp(VINDPM_VOL_MIN, VINDPM_VOL_MAX);
        let val = self.dev.read_register(0x0D)?;
        let steps = ((millivolt - VINDPM_VOL_BASE) / VINDPM_VOL_STEPS) as u8;
        let new_val = (val & 0x80) | steps;
        self.dev.write_register(&[0x0D, new_val])
    }

    // REGISTER 0x0E
    // ADC conversion of Battery Voltage (VBAT)

    /// Checks if thermal regulation is normal
    /// Returns true for normal operation, false if in thermal regulation
    pub fn is_thermal_regulation_normal(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x0E, 7).map(|b| !b)
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

        /// Converts NTC thermistor resistance ratio to temperature
        /// using Steinhart-Hart equation
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
        // Return 0 if not charging
        if self.get_charge_status()? == ChargeStatus::NoCharge {
            return Ok(0);
        }

        // Read and validate register
        let val = self.dev.read_register(0x12)?;
        if val == 0 {
            return Ok(0);
        }

        // Calculate current in mA
        let current = ((val & 0x7F) as u16) * CHG_STEP_VAL;
        Ok(current)
    }

    // REGISTER 0x13
    // VINDPM Status, IINDPM Status, Input Current Limit in effect while Input Current Optimizer

    /// Checks if Dynamic Power Management is active
    pub fn is_dynamic_power_management(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x13, 7)
    }

    /// Checks if Input Current Limit is active
    pub fn is_input_current_limit(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x13, 6)
    }

    /// Sets the Input Current Limit for the optimizer (100-3250mA)
    ///
    /// # Arguments
    /// * `milliampere` - Current limit in mA
    pub fn set_input_current_limit_optimizer(
        &mut self,
        mut milliampere: u16,
    ) -> Result<(), PmuSensorError> {
        if milliampere % IN_CURRENT_OPT_STEP != 0 {
            return Err(PmuSensorError::CurrentStepInvalid50);
        }

        milliampere = milliampere.clamp(IN_CURRENT_OPT_MIN, IN_CURRENT_OPT_MAX);
        let val = self.dev.read_register(0x13)?;
        let steps = ((milliampere - IN_CURRENT_OPT_MIN) / IN_CURRENT_OPT_STEP) as u8;
        let new_val = (val & 0x3F) | (steps << 6);
        self.dev.write_register(&[0x13, new_val])
    }

    // REGISTER 0x14
    // Register Reset, Input Current Optimizer (ICO) Status , Device Configuration, Temperature Profile, Device Revision: 10

    /// Resets the device to the default configuration
    pub fn reset_default(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x14, 7)
    }

    /// Checks if Input Current Optimization is in progress
    pub fn is_input_current_optimizer(&mut self) -> Result<bool, PmuSensorError> {
        self.dev.get_register_bit(0x14, 6)
    }

    /// Gets the device configuration
    pub fn get_device_config(&mut self) -> Result<u8, PmuSensorError> {
        let val = self.dev.read_register(0x14)?;
        Ok((val >> 3) & 0x03)
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

        text.push_str(&format!("USB PlugIn: {}\n", is_vbus_present));
        text.push_str(&format!("Bus state: {}\n", self.get_bus_status()?));
        text.push_str(&format!(
            "Battery voltage: {}mV\n",
            self.get_battery_voltage()?
        ));
        text.push_str(&format!("USB voltage: {}mV\n", self.get_vbus_voltage()?));
        text.push_str(&format!("SYS voltage: {}mV\n", self.get_sys_voltage()?));
        text.push_str(&format!("Temperature: {}°C\n", self.get_temperature()?));
        text.push_str(&format!(
            "Charger fast charge curr.: {}mA\n",
            self.get_fast_charge_current_limit()?
        ));
        text.push_str(&format!(
            "Charger target voltage: {}mV\n",
            self.get_charge_target_voltage()?
        ));
        text.push_str(&format!("Boost frequency: {}\n", self.get_boost_freq()?));
        text.push_str(&format!(
            "Fast charge timer: {}\n",
            self.get_fast_charge_timer()?
        ));
        text.push_str(&format!(
            "Input curr. limit: {}mA\n",
            self.get_input_current_limit()?
        ));
        text.push_str(&format!(
            "Termination curr.: {}mA\n",
            self.get_termination_current()?
        ));
        text.push_str(&format!(
            "Power down voltage: {}mV\n",
            self.get_sys_power_down_voltage()?
        ));
        text.push_str(&format!(
            "Pre charge curr.: {}mA\n",
            self.get_precharge_current()?
        ));
        text.push_str(&format!("Charge curr.: {}mA\n", self.get_charge_current()?));
        text.push_str(&format!("HIZ mode: {}\n", self.is_hiz_mode()?));
        text.push_str(&format!(
            "Automatic input detection: {}\n",
            self.is_automatic_input_detection_enabled()?
        ));
        text.push_str(&format!(
            "Charging safety timer: {}\n",
            self.is_charging_safety_timer_enabled()?
        ));
        text.push_str(&format!(
            "Input det. enabled: {}\n",
            self.is_input_detection_enabled()?
        ));
        text.push_str(&format!(
            "Input curr. optimizer: {}\n",
            self.is_input_current_optimizer()?
        ));
        text.push_str(&format!("Chip Id: {}\n", self.get_chip_id()?));

        Ok(text)
    }
}

/// Errors that can occur when interacting with the BQ25896
#[derive(Debug)]
pub enum PmuSensorError {
    /// Failed to initialize the device
    Init,
    /// Failed to read from register
    ReadRegister,
    /// Failed to write to register
    WriteRegister,
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
}

/// Status of the power input source
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BusStatus {
    /// No power input connected
    NoInput,
    /// Standard USB host (500mA max)
    UsbSdp,
    /// AC/DC power adapter
    Adapter,
    /// USB OTG mode - device is power source
    Otg,
    /// Unknown input status
    Unknown,
}

#[derive(Debug, Clone, Copy)]
pub enum BoostFreq {
    Freq500KHz,
    Freq1500KHz,
}

#[derive(Debug, Clone, Copy)]
pub enum FastChargeThreshold {
    Volt2V8,
    Volt3V0,
}

#[derive(Debug, Clone, Copy)]
pub enum RechargeThresholdOffset {
    Offset100mV,
    Offset200mV,
}

#[derive(Debug, Clone, Copy)]
pub enum WatchdogConfig {
    TimerOut40Sec,
    TimerOut80Sec,
    TimerOut160Sec,
}

#[derive(Debug, Clone, Copy)]
pub enum FastChargeTimer {
    Hours5 = 0,
    Hours8 = 1,
    Hours12 = 2,
    Hours20 = 3,
}

#[derive(Debug, Clone, Copy)]
pub enum JeitaLowTemperatureCurrent {
    Temp50,
    Temp20,
}

#[derive(Debug, Clone, Copy)]
pub enum ThermalRegThreshold {
    Celsius60 = 0,
    Celsius80 = 1,
    Celsius100 = 2,
    Celsius120 = 3,
}

#[derive(Debug, Clone, Copy)]
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

#[derive(Debug, Clone, Copy, PartialEq)]
enum NtcBoostStatus {
    Normal = 0,
    Cold = 1,
    Hot = 2,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum NtcBuckStatus {
    Normal = 0,
    Warm = 2,
    Cold = 3,
    ColdAlt = 5,
    Hot = 6,
}

#[derive(Debug, Clone, Copy, PartialEq)]
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
            BusStatus::Unknown => write!(f, "Unknown"),
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

impl From<u8> for BusStatus {
    fn from(val: u8) -> Self {
        match val {
            0 => BusStatus::NoInput,
            1 => BusStatus::UsbSdp,  // Standard USB port (500mA max)
            2 => BusStatus::Adapter, // AC/DC power adapter
            3 => BusStatus::Otg,     // On-The-Go -it powers other devices on usb
            _ => BusStatus::Unknown,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
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

impl From<u8> for ChargeStatus {
    fn from(val: u8) -> Self {
        match val {
            0 => ChargeStatus::NoCharge,
            1 => ChargeStatus::PreCharge,
            2 => ChargeStatus::FastCharge,
            3 => ChargeStatus::Done,
            _ => ChargeStatus::Unknown,
        }
    }
}

/// Boost mode hot temperature monitor threshold
#[derive(Debug, Clone, Copy)]
pub enum BoostHotThreshold {
    VBHOT1 = 0x00,   // Threshold (34.75%) (default)
    VBHOT0 = 0x01,   // Threshold (Typ. 37.75%)
    VBHOT2 = 0x02,   // Threshold (Typ. 31.25%)
    Disabled = 0x03, // Disable boost mode thermal protection
}

#[derive(Debug, Clone, Copy)]
pub enum BoostColdThreshold {
    VBCOLD0 = 0x00, // (Typ. 77%) (default)
    VBCOLD1 = 0x01, // (Typ. 80%)
}

#[derive(Debug, Clone, Copy)]
pub enum ExitBoostModeVolt {
    MiniVolt2V9,
    MiniVolt2V5,
}

#[derive(Debug)]
struct BQ25896Device<I2C> {
    i2c: I2C,
    adr: u8,
}

impl<I2C> BQ25896Device<I2C>
where
    I2C: I2c,
{
    fn new(i2c: I2C, adr: u8) -> Self {
        Self { i2c, adr }
    }

    fn read_register(&mut self, register: u8) -> Result<u8, PmuSensorError> {
        let mut buffer = [0u8];

        self.i2c
            .write_read(self.adr, &[register], &mut buffer)
            .map_err(|_| PmuSensorError::ReadRegister)?;
        Ok(buffer[0])
    }

    fn write_register(&mut self, register_and_data: &[u8]) -> Result<(), PmuSensorError> {
        self.i2c
            .write(self.adr, register_and_data)
            .map_err(|_| PmuSensorError::WriteRegister)
    }

    fn set_register_bit(&mut self, register: u8, bit: u8) -> Result<(), PmuSensorError> {
        let val = self.read_register(register)?;
        let data = val | (1 << bit);
        self.write_register(&[register, data])
    }

    fn get_register_bit(&mut self, register: u8, bit: u8) -> Result<bool, PmuSensorError> {
        let val = self.read_register(register)?;
        Ok((val & (1 << bit)) != 0)
    }

    fn clear_register_bit(&mut self, register: u8, bit: u8) -> Result<(), PmuSensorError> {
        let val = self.read_register(register)?;
        let data = val & !(1 << bit);
        self.write_register(&[register, data])
    }
}
