use core::fmt::{self, Display, Formatter};

use defmt::Format;
use embedded_hal::i2c::I2c;
use libm::{log, round};

/// Handles all operations on/with Mpu6050
/// https://github.com/andhieSetyabudi/BQ25896/blob/master/BQ25896.h
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
pub struct BQ25896<I2C> {
    i2c: I2C,
    adr: u8,
}

// Register address constants with documentation
/// ADC control register - enables ADC features
const ADC_CTRL: u8 = 0x02;
/// System control register - charge enable/disable
const SYS_CTRL: u8 = 0x03;
/// Battery voltage register
const BATV: u8 = 0x0E;
/// System voltage register  
const SYSV: u8 = 0x0F;
/// Bus status register - input source detection
const VBUS_STAT: u8 = 0x0B;
/// Temperature register - NTC readings
const TSPCT: u8 = 0x10;
/// USB bus voltage register
const VBUSV: u8 = 0x11;

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
        let mut instance = Self { i2c, adr };
        instance.detect_pmu(adr)?;
        Ok(instance)
    }

    fn detect_pmu(&mut self, adr: u8) -> Result<(), PmuSensorError> {
        if self.i2c.write(adr, &[]).is_ok() {
            Ok(())
        } else {
            Err(PmuSensorError::Init)
        }
    }

    fn read_register(&mut self, reg: &[u8]) -> Result<u8, PmuSensorError> {
        let mut buffer = [0u8];

        self.i2c
            .write_read(self.adr, reg, &mut buffer)
            .map_err(|_| PmuSensorError::ReadRegister)?;
        Ok(buffer[0])
    }

    fn write_register(&mut self, reg: u8, data: u8) -> Result<(), PmuSensorError> {
        self.i2c
            .write(self.adr, &[reg, data])
            .map_err(|_| PmuSensorError::WriteRegister)
    }

    /// Enables ADC conversion for voltage and current monitoring
    pub fn set_adc_enabled(&mut self) -> Result<(), PmuSensorError> {
        let mut data = self.read_register(&[ADC_CTRL])?;
        data |= 1 << 7; // Start ADC conversion
        data |= 1 << 6; // Set continuous conversion
        self.write_register(ADC_CTRL, data)
    }

    pub fn get_chip_id(&mut self) -> Result<u8, PmuSensorError> {
        let result = self.read_register(&[])?;
        Ok(result & 0x03)
    }

    pub fn get_charge_status(&mut self) -> Result<ChargeStatus, PmuSensorError> {
        let val = self.read_register(&[VBUS_STAT])?;
        let result = (val >> 3) & 0x03;
        Ok(result.into())
    }

    pub fn set_charge_enable(&mut self, enabled: bool) -> Result<(), PmuSensorError> {
        let mut data = self.read_register(&[SYS_CTRL])?;

        match enabled {
            false => data &= !(1 << 4),
            true => data |= 1 << 4,
        }

        self.write_register(SYS_CTRL, data)
    }

    pub fn get_bus_status(&mut self) -> Result<BusStatus, PmuSensorError> {
        let val = self.read_register(&[VBUS_STAT])?;
        let result = (val >> 5) & 0x07;
        Ok(result.into())
    }

    pub fn is_vbus_in(&mut self) -> Result<bool, PmuSensorError> {
        let val = self.get_bus_status()?;
        Ok(val != BusStatus::NoInput)
    }

    /// Gets battery voltage in millivolts
    /// Returns voltage_mv
    pub fn get_battery_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let data = self.read_register(&[BATV])?;

        let thermal_regulation = ((data >> 7) & 0x01) != 0;
        if thermal_regulation {
            return Ok(0);
        }
        let data = data & 0x7F;
        let vbat = (data as u16) * 20 + 2304;

        Ok(vbat)
    }

    /// Returns the USB voltage
    /// Gets USB bus voltage in millivolts

    pub fn get_vbus_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let data = self.read_register(&[VBUSV])?;

        let vbus_attached = ((data >> 7) & 0x01) == 0;
        if vbus_attached {
            return Ok(0);
        }
        let data = data & 0x7F;
        let vbus = (data as u16) * 100 + 2600;

        Ok(vbus)
    }

    /// Gets system voltage in millivolts
    pub fn get_sys_voltage(&mut self) -> Result<u16, PmuSensorError> {
        let data = self.read_register(&[SYSV])?;
        let data = data & 0x7F;
        if data == 0 {
            return Ok(0);
        }
        let vsys = (data as u16) * 20 + 2304;

        Ok(vsys)
    }

    /// Converts NTC thermistor resistance ratio to temperature
    /// using Steinhart-Hart equation
    fn r_to_temp(&self, r: f64) -> f64 {
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

    /// Gets battery temperature in Celsius from NTC thermistor
    pub fn get_temperature(&mut self) -> Result<f64, PmuSensorError> {
        let data = self.read_register(&[TSPCT])?;
        let data = data & 0x7F;
        let ntc_percent = (data as f64) * 0.465_f64 + 21_f64;

        // Convert percentage to resistance ratio
        let r_ratio = (100.0 - ntc_percent) / ntc_percent;

        Ok(self.r_to_temp(r_ratio))
    }

    /*pub fn set_min_vbus(&mut self, voltage: f32) -> Result<(), PmuSensorError> {
        let voltage = if voltage > 2.6 { voltage - 2.6 } else { 0.0 };
        let data = (voltage * 10.0) as u8;
        self.write_register(0x0D, data)
    }

    pub fn set_fast_charge_current_limit(&mut self, current: f32) -> Result<(), PmuSensorError> {
        const POWERS_PPM_REG_04H: u8 = 0x04;
        let mut data = self.read_register(&[POWERS_PPM_REG_04H])?;
        let current = (current / 0.064).min(127.0) as u8;
        data = (data & 0x80) | current;
        self.write_register(0x04, data)
    }*/
}

/// Errors that can occur when interacting with the BQ25896
#[derive(Debug, Format)]
pub enum PmuSensorError {
    /// Failed to initialize the device
    Init,
    /// Failed to read from register
    ReadRegister,
    /// Failed to write to register
    WriteRegister,
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
