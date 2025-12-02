use core::fmt::Display;

#[cfg(feature = "defmt")]
use defmt::Format;
use num_enum::{IntoPrimitive, TryFromPrimitive};

// Constants
pub(crate) const LD2410_BUFFER_SIZE: usize = 256;

pub(crate) const DATA_HEADER: [u8; 4] = [0xF4, 0xF3, 0xF2, 0xF1];
pub(crate) const DATA_TAIL: [u8; 4] = [0xF8, 0xF7, 0xF6, 0xF5];

pub(crate) const CMD_HEADER: [u8; 4] = [0xFD, 0xFC, 0xFB, 0xFA];
pub(crate) const CMD_TAIL: [u8; 4] = [0x04, 0x03, 0x02, 0x01];

// Data Types
#[derive(Debug, Clone, Copy)]
pub struct FirmwareVersion {
    pub major: u8,
    pub minor: u8,
    pub bugfix: u32,
}

impl Display for FirmwareVersion {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.bugfix)
    }
}

#[cfg(feature = "defmt")]
impl Format for FirmwareVersion {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{}.{}.{}", self.major, self.minor, self.bugfix)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RadarConfiguration {
    pub max_gate: u8,
    pub max_moving_gate: u8,
    pub max_stationary_gate: u8,
    pub motion_sensitivity: [u8; 9],
    pub stationary_sensitivity: [u8; 9],
    pub sensor_idle_time: u16,
}

impl Display for RadarConfiguration {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "RadarConfiguration {{ max_gate: {}, max_moving_gate: {}, max_stationary_gate: {}, idle_time: {}s, motion_sensitivity: {:?}, stationary_sensitivity: {:?} }}",
            self.max_gate,
            self.max_moving_gate,
            self.max_stationary_gate,
            self.sensor_idle_time,
            &self.motion_sensitivity[..],
            &self.stationary_sensitivity[..]
        )
    }
}

#[cfg(feature = "defmt")]
impl Format for RadarConfiguration {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "RadarConfiguration {{ max_gate: {}, max_moving_gate: {}, max_stationary_gate: {}, idle_time: {}s, motion_sensitivity: {:?}, stationary_sensitivity: {:?} }}",
            self.max_gate,
            self.max_moving_gate,
            self.max_stationary_gate,
            self.sensor_idle_time,
            &self.motion_sensitivity[..],
            &self.stationary_sensitivity[..]
        );
    }
}

impl TargetState {
    /// Check if any target is detected
    pub fn has_target(&self) -> bool {
        *self != TargetState::None
    }

    /// Check if a moving target is detected
    pub fn has_moving(&self) -> bool {
        matches!(self, TargetState::Moving | TargetState::Both)
    }

    /// Check if a stationary target is detected
    pub fn has_stationary(&self) -> bool {
        matches!(self, TargetState::Stationary | TargetState::Both)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub enum TargetState {
    /// No target detected (value: 0x00)
    None = 0x00,
    /// Moving target detected (value: 0x01)
    Moving = 0x01,
    /// Stationary target detected (value: 0x02)
    Stationary = 0x02,
    /// Both moving and stationary targets detected (value: 0x03)
    Both = 0x03,
}

#[cfg(feature = "defmt")]
impl Format for TargetState {
    fn format(&self, f: defmt::Formatter) {
        match self {
            TargetState::None => defmt::write!(f, "No Target"),
            TargetState::Moving => defmt::write!(f, "Moving Target"),
            TargetState::Stationary => defmt::write!(f, "Stationary Target"),
            TargetState::Both => defmt::write!(f, "Moving & Stationary Targets"),
        }
    }
}

impl core::fmt::Display for TargetState {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            TargetState::None => write!(f, "No Target"),
            TargetState::Moving => write!(f, "Moving Target"),
            TargetState::Stationary => write!(f, "Stationary Target"),
            TargetState::Both => write!(f, "Moving & Stationary Targets"),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct RadarData {
    pub target_state: TargetState,
    pub movement_target_distance: u16,
    pub stationary_target_distance: u16,
    pub detection_distance: u16,
    pub movement_target_energy: u8,
    pub stationary_target_energy: u8,
}

impl RadarData {
    /// Check if a moving target is detected
    pub fn moving_target_detected(&self) -> bool {
        self.target_state.has_moving()
            && self.movement_target_distance > 0
            && self.movement_target_energy > 0
    }

    /// Check if any target (moving or stationary) is detected
    pub fn presence_detected(&self) -> bool {
        self.target_state.has_target()
    }

    /// Check if a stationary target is detected
    pub fn stationary_target_detected(&self) -> bool {
        self.target_state.has_stationary()
            && self.stationary_target_distance > 0
            && self.stationary_target_energy > 0
    }
}

#[cfg(feature = "defmt")]
impl Format for RadarData {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "movement: {}cm, stationary: {}cm, detection: {}cm, energy: movement: {}, stationary: {}",
            self.movement_target_distance,
            self.stationary_target_distance,
            self.detection_distance,
            self.movement_target_energy,
            self.stationary_target_energy,
        )
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    EnterConfigMode,
    ExitConfigMode,
    RequestFirmware,
    RequestRestart,
    RequestFactoryReset,
    RequestCurrentConfig,
    RequestStartEngineeringMode,
    RequestEndEngineeringMode,
}

impl Command {
    pub(crate) fn payload(&self) -> &'static [u8] {
        match self {
            Command::EnterConfigMode => &[0x04, 0x00, 0xFF, 0x00, 0x01, 0x00],
            Command::ExitConfigMode => &[0x02, 0x00, 0xFE, 0x00],
            Command::RequestFirmware => &[0x02, 0x00, 0xA0, 0x00],
            Command::RequestRestart => &[0x02, 0x00, 0xA3, 0x00],
            Command::RequestFactoryReset => &[0x02, 0x00, 0xA2, 0x00],
            Command::RequestCurrentConfig => &[0x02, 0x00, 0x61, 0x00],
            Command::RequestStartEngineeringMode => &[0x02, 0x00, 0x62, 0x00],
            Command::RequestEndEngineeringMode => &[0x02, 0x00, 0x63, 0x00],
        }
    }

    pub(crate) fn expected_ack(&self) -> u8 {
        match self {
            Command::EnterConfigMode => 0xFF,
            Command::ExitConfigMode => 0xFE,
            Command::RequestFirmware => 0xA0,
            Command::RequestRestart => 0xA3,
            Command::RequestFactoryReset => 0xA2,
            Command::RequestCurrentConfig => 0x61,
            Command::RequestStartEngineeringMode => 0x62,
            Command::RequestEndEngineeringMode => 0x63,
        }
    }
}

pub struct PollingConfig {
    pub max_attempts: u8,
    pub delay_ms: u32,
    pub timeout_ms: u32,
}

impl Default for PollingConfig {
    fn default() -> Self {
        Self {
            max_attempts: 10,
            delay_ms: 10,
            timeout_ms: 100,
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LD2410Error {
    IoError,
    InvalidHeader,
    InvalidTail,
    TooLarge(usize),
    InvalidData,
}

impl<E> From<embedded_io::ReadExactError<E>> for LD2410Error
where
    E: embedded_io::Error,
{
    fn from(_: embedded_io::ReadExactError<E>) -> Self {
        LD2410Error::IoError
    }
}

pub mod asynch;
pub mod blocking;
