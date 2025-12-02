use core::fmt::{Display, Formatter};

// https://github.com/fbiego/CST816S
use bitflags::bitflags;
use embedded_hal::i2c::Error;
use num_enum::{IntoPrimitive, TryFromPrimitive};

pub(crate) const CST816S_ADDRESS: u8 = 0x15;

/// Number of bytes for a single touch event
pub const RAW_TOUCH_EVENT_LEN: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Gesture {
    None = 0x00,
    SwipeUp = 0x01,
    SwipeDown = 0x02,
    SwipeLeft = 0x03,
    SwipeRight = 0x04,
    SingleClick = 0x05,
    DoubleClick = 0x0B,
    LongPress = 0x0C,
}

#[derive(Debug, Clone, Copy, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    Down = 0,
    Up = 1,
    Contact = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ChipID {
    CST816S = 0xB4,
    CST816T = 0xB5,
}

impl Display for ChipID {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            ChipID::CST816S => write!(f, "CST816S"),
            ChipID::CST816T => write!(f, "CST816T"),
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TouchData {
    pub gesture: Gesture,
    pub points: u8,
    pub event: Event,
    pub x: u16,
    pub y: u16,
}

bitflags! {
    pub struct IrqControl: u8 {
        /// Bit 7: EnTest (Enable test, periodically sends low pulses)
        const EN_TEST   = 1 << 7;
        /// Bit 6: EnTouch (Sends low pulse on touch detection)
        const EN_TOUCH  = 1 << 6;
        /// Bit 5: EnChange (Sends low pulse on touch state change)
        const EN_CHANGE = 1 << 5;
        /// Bit 4: EnMotion (Sends low pulse on gesture detection)
        const EN_MOTION = 1 << 4;
        /// Bit 0: OnceWLP (Sends one low pulse on long press)
        const ONCE_WLP  = 1 << 0;
    }
}

impl Default for IrqControl {
    fn default() -> Self {
        // Set default to have EN_TOUCH, EN_CHANGE, and EN_MOTION enabled
        IrqControl::EN_TOUCH | IrqControl::EN_CHANGE | IrqControl::EN_MOTION
    }
}

bitflags! {
    pub struct MotionMask: u8 {
        /// Enable double-click detection
        const DOUBLE_CLICK = 1 << 0;
        /// Enable continuous up/down swipe
        const CONTINUOUS_UPDOWN = 1 << 1;
        /// Enable continuous left/right swipe
        const CONTINUOUS_LEFTRIGHT = 1 << 2;
    }
}

impl Default for MotionMask {
    fn default() -> Self {
        // Default to enabling double-click, matching the C++ library's begin()
        MotionMask::DOUBLE_CLICK
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for MotionMask {
    fn format(&self, f: defmt::Formatter) {
        self.iter_names().for_each(|name| {
            defmt::write!(f, "{}", name);
        });
    }
}

/// Errors that can occur when interacting with the BQ25896
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TouchSensorError {
    UnknownGesture(u8),
    InvalidLongPressTime,
    I2CError,
    PinError,
}

impl<E> From<E> for TouchSensorError
where
    E: Error,
{
    fn from(_: E) -> Self {
        TouchSensorError::I2CError
    }
}

pub mod asynch;
pub mod blocking;
