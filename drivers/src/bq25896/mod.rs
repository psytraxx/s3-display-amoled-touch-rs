use core::fmt::{self, Display, Formatter};
use embedded_hal::i2c::Error;
use num_enum::{IntoPrimitive, TryFromPrimitive};

pub(crate) const IN_CURRENT_STEP: u16 = 50;
pub(crate) const IN_CURRENT_MIN: u16 = 100;
pub(crate) const IN_CURRENT_MAX: u16 = 3250;

pub(crate) const IN_CURRENT_OFFSET_STEP: u16 = 100;
pub(crate) const IN_CURRENT_OFFSET_MAX: u16 = 3100;

pub(crate) const CHG_STEP_VAL: u16 = 50;

pub(crate) const FAST_CHG_CUR_STEP: u16 = 64;
pub(crate) const FAST_CHG_CURRENT_MAX: u16 = 3008;

pub(crate) const CHG_VOL_BASE: u16 = 3840;
pub(crate) const CHG_VOL_STEP: u16 = 16;
pub(crate) const FAST_CHG_VOL_MIN: u16 = 3840;
pub(crate) const FAST_CHG_VOL_MAX: u16 = 4608;

pub(crate) const PRE_CHG_CUR_BASE: u16 = 64;
pub(crate) const PRE_CHG_CUR_STEP: u16 = 64;
pub(crate) const PRE_CHG_CURRENT_MIN: u16 = 64;
pub(crate) const PRE_CHG_CURRENT_MAX: u16 = 1024;

pub(crate) const SYS_VOL_STEPS: u16 = 100;
pub(crate) const SYS_VOFF_VOL_MIN: u16 = 3000;
pub(crate) const SYS_VOFF_VOL_MAX: u16 = 3700;

pub(crate) const TERM_CHG_CUR_BASE: u16 = 64;
pub(crate) const TERM_CHG_CUR_STEP: u16 = 64;
pub(crate) const TERM_CHG_CURRENT_MIN: u16 = 64;
pub(crate) const TERM_CHG_CURRENT_MAX: u16 = 1024;

pub(crate) const BAT_COMP_STEPS: u16 = 20;
pub(crate) const BAT_COMP_MAX: u16 = 140;

pub(crate) const VCLAMP_STEPS: u16 = 32;
pub(crate) const VCLAMP_MAX: u16 = 224;

pub(crate) const BOOST_VOL_BASE: u16 = 4550;
pub(crate) const BOOST_VOL_STEP: u16 = 64;
pub(crate) const BOOST_VOL_MIN: u16 = 4550;
pub(crate) const BOOST_VOL_MAX: u16 = 5510;

pub(crate) const VINDPM_VOL_BASE: u16 = 4550;
pub(crate) const VINDPM_VOL_STEPS: u16 = 100;
pub(crate) const VINDPM_VOL_MIN: u16 = 3900;
pub(crate) const VINDPM_VOL_MAX: u16 = 15300;

pub(crate) const IN_CURRENT_OPT_STEP: u16 = 50;
pub(crate) const IN_CURRENT_OPT_MIN: u16 = 100;
pub(crate) const IN_CURRENT_OPT_MAX: u16 = 3250;

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
pub(crate) enum NtcBoostStatus {
    Normal = 0,
    Cold = 1,
    Hot = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, TryFromPrimitive, IntoPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum NtcBuckStatus {
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

#[cfg(feature = "async")]
pub mod asynch;
pub mod blocking;
