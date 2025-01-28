use core::fmt::{self, Display, Formatter};

use alloc::string::String;
use embedded_hal::i2c::I2c;
use libm::{log, round};
/// https://github.com/Xinyuan-LilyGO/LilyGo-AMOLED-Series/blob/master/libdeps/XPowersLib/src/PowersBQ25896.tpp
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

    fn detect_pmu(&mut self, adr: u8) -> Result<(), PmuSensorError> {
        if self.dev.write_register(&[adr]).is_ok() {
            Ok(())
        } else {
            Err(PmuSensorError::Init)
        }
    }

    // Register 0x00
    // Enable HIZ Mode, Enable ILIM Pin, Input Current Limit

    pub fn set_hiz_mode(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x00, 7)
    }

    pub fn exit_hiz_mode(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x00, 7)
    }

    pub fn is_hiz_mode(&mut self) -> Result<bool, PmuSensorError> {
        let bit = self.dev.get_register_bit(0x00, 7)?;
        Ok(bit != 0)
    }

    pub fn enable_current_limit_pin(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x00, 6)
    }

    pub fn disable_current_limit_pin(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x00, 6)
    }

    pub fn is_enable_current_limit_pin(&mut self) -> Result<bool, PmuSensorError> {
        let bit = self.dev.get_register_bit(0x00, 6)?;
        Ok(bit != 0)
    }

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
    pub fn set_boost_mode_hot_temp_threshold(
        &mut self,
        threshold: BoostHotThreshold,
    ) -> Result<(), PmuSensorError> {
        let val = self.dev.read_register(0x01)?;
        let data = (val & 0x3F) | ((threshold as u8) << 6);
        self.dev.write_register(&[0x01, data])
    }

    /// Sets the boost mode cold temperature monitor threshold
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

    // Disables ADC conversion
    pub fn set_adc_disabled(&mut self) -> Result<(), PmuSensorError> {
        let mut data = self.dev.read_register(0x02)?;
        data &= !(1 << 7); // Clear ADC conversion bit
        self.dev.write_register(&[0x02, data])
    }

    // Set boost frequency
    pub fn set_boost_freq(&mut self, freq: BoostFreq) -> Result<(), PmuSensorError> {
        match freq {
            BoostFreq::Freq500KHz => self.dev.set_register_bit(0x02, 5),
            BoostFreq::Freq1500KHz => self.dev.clear_register_bit(0x02, 5),
        }
    }

    // Get boost frequency
    pub fn get_boost_freq(&mut self) -> Result<BoostFreq, PmuSensorError> {
        let bit = self.dev.get_register_bit(0x02, 5)?;
        Ok(if bit != 0 {
            BoostFreq::Freq500KHz
        } else {
            BoostFreq::Freq1500KHz
        })
    }

    // Input Current Optimizer controls
    pub fn enable_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x02, 4)
    }

    pub fn disable_input_current_optimizer(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x02, 4)
    }

    // Input Detection controls
    pub fn enable_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x02, 1)
    }

    pub fn disable_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x02, 1)
    }

    pub fn is_input_detection_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let bit = self.dev.get_register_bit(0x02, 1)?;
        Ok(bit != 0)
    }

    // Automatic Input Detection controls
    pub fn enable_automatic_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x02, 0)
    }

    pub fn disable_automatic_input_detection(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x02, 0)
    }

    pub fn is_automatic_input_detection_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let bit = self.dev.get_register_bit(0x02, 0)?;
        Ok(bit != 0)
    }

    // REGISTER 0x03
    // Battery Load (IBATLOAD) Enable, I2C Watchdog Timer Reset, Boost (OTG) Mode Configuration
    // Charge Enable Configuration,  Minimum System Voltage Limit, Minimum Battery Voltage (falling) to exit boost mode

    pub fn is_bat_load_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let bit = self.dev.get_register_bit(0x03, 7)?;
        Ok(bit != 0)
    }

    pub fn disable_bat_load(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x03, 7)
    }

    pub fn enable_bat_load(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x03, 7)
    }

    pub fn feed_watchdog(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x03, 6)
    }

    pub fn is_otg_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let bit = self.dev.get_register_bit(0x03, 5)?;
        Ok(bit != 0)
    }

    pub fn disable_otg(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x03, 5)?;
        // Re-enable charging if it wasn't explicitly disabled by user
        if !self.user_disable_charge {
            self.dev.set_register_bit(0x03, 4)?;
        }
        Ok(())
    }

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

    /// Get charging status
    pub fn is_charge_enabled(&mut self) -> Result<bool, PmuSensorError> {
        let bit = self.dev.get_register_bit(0x03, 4)?;
        Ok(bit != 0)
    }

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

    pub fn get_sys_power_down_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.dev.read_register(0x03)?;
        let val = (val & 0x0E) >> 1;
        Ok((val as u16 * SYS_VOL_STEPS) + SYS_VOFF_VOL_MIN)
    }

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

    pub fn set_current_pulse_control_enabled(&mut self) -> Result<(), PmuSensorError> {
        self.dev.set_register_bit(0x04, 7)
    }

    pub fn set_current_pulse_control_disabled(&mut self) -> Result<(), PmuSensorError> {
        self.dev.clear_register_bit(0x04, 7)
    }

    /// Sets the charger constant current
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

    /// Gets the charger constant current
    pub fn get_fast_charge_current_limit(&mut self) -> Result<u16, PmuSensorError> {
        let val = self.dev.read_register(0x04)?;
        let bits = val & 0x7F; // Extract bits 6:0

        Ok(bits as u16 * FAST_CHG_CUR_STEP)
    }

    // REGISTER 0x05
    // Precharge Current Limit, Termination Current Limit

    /// Sets the precharge current
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

    // REGISTER 0x06
    // Charge Voltage Limit, Battery Precharge to Fast Charge Threshold, Battery Recharge Threshold Offset

    /// Sets the charge target voltage
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

    // REGISTER 0x07
    // Charging Termination Enable, STAT Pin Disable , I2C Watchdog Timer Setting, Charging Safety Timer Enable
    // Fast Charge Timer Setting, JEITA Low Temperature Current Setting

    // REGISTER 0x08
    // IR Compensation Resistor Setting, IR Compensation Voltage Clamp, Thermal Regulation Threshold

    // REGISTER 0x09
    // Force Start Input Current Optimizer (ICO), Safety Timer Setting during DPM or Thermal Regulation,Force BATFET off to enable ship mode
    // JEITA High Temperature Voltage Setting, BATFET turn off delay control, BATFET full system reset enable, Current pulse control voltage up enable
    // Current pulse control voltage down enable

    // REGISTER 0x0A
    // Boost Mode Voltage Regulation, PFM mode allowed in boost mode , Boost Mode Current Limit

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

    /// Gets the bus status
    pub fn get_bus_status(&mut self) -> Result<BusStatus, PmuSensorError> {
        let val = self.dev.read_register(0x0B)?;
        let result = (val >> 5) & 0x07;
        Ok(result.into())
    }

    // REGISTER 0x0C
    // Watchdog Fault Status, Boost Mode Fault Status, Charge Fault Status, Battery Fault Status, NTC Fault Status

    // REGISTER 0x0E

    // REGISTER 0x0D
    // VINDPM Threshold Setting Method, bsolute VINDPM Threshold

    // REGISTER 0x0E
    // ADC conversion of Battery Voltage (VBAT)

    /// Gets battery voltage in millivolts
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

    /// Gets system voltage in millivolts
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

    /// Gets battery temperature in Celsius from NTC thermistor
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

    /// Gets USB bus voltage in millivolts
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

    // REGISTER 0x13
    // VINDPM Status, IINDPM Status, Input Current Limit in effect while Input Current Optimizer

    // REGISTER 0x14
    // Register Reset, Input Current Optimizer (ICO) Status , Device Configuration, Temperature Profile, Device Revision: 10

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
            "Battery voltage: {}mv\n",
            self.get_battery_voltage()?
        ));
        text.push_str(&format!("USB voltage: {}mv\n", self.get_vbus_voltage()?));
        text.push_str(&format!("SYS voltage: {}mv\n", self.get_sys_voltage()?));
        text.push_str(&format!("Temperature: {}°C\n", self.get_temperature()?));
        text.push_str(&format!(
            "Charger constant current: {}mA\n",
            self.get_fast_charge_current_limit()?
        ));
        text.push_str(&format!(
            "Charger target voltage: {}mv\n",
            self.get_charge_target_voltage()?
        ));

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
    // Current step invalid (must be multiple of 64)
    CurrentStepInvalid64,
    // Current step invalid (must be multiple of 100)
    CurrentStepInvalid100,
    // Current step invalid (must be multiple of 50)
    CurrentStepInvalid50,
    // Needs to be inbetween  3000 and 3700mV
    PowerDownVoltageInvalid,
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

    fn get_register_bit(&mut self, register: u8, bit: u8) -> Result<u8, PmuSensorError> {
        let val = self.read_register(register)?;
        Ok(val & (1 << bit))
    }

    fn clear_register_bit(&mut self, register: u8, bit: u8) -> Result<(), PmuSensorError> {
        let val = self.read_register(register)?;
        let data = val & !(1 << bit);
        self.write_register(&[register, data])
    }
}
