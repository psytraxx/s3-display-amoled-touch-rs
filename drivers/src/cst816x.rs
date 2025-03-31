// https://github.com/fbiego/CST816S
use bitflags::bitflags;
use embedded_hal::i2c::I2c;
use embedded_hal::{digital::InputPin, i2c::Error};
use num_enum::{IntoPrimitive, TryFromPrimitive};

use crate::AsynRegisterDevice;

const CST816S_ADDRESS: u8 = 0x15;

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
    #[derive(Default)]
    pub struct MotionMask: u8 {
        /// Enable double-click detection
        const DOUBLE_CLICK = 1 << 0;
        /// Enable continuous up/down swipe
        const CONTINUOUS_UPDOWN = 1 << 1;
        /// Enable continuous left/right swipe
        const CONTINUOUS_LEFTRIGHT = 1 << 2;
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
#[derive(Debug)]
pub struct CST816x<I2C, PIN> {
    dev: AsynRegisterDevice<I2C>,
    touch_int: PIN,
}

impl<I2C, PIN> CST816x<I2C, PIN>
where
    I2C: I2c,
    PIN: InputPin,
{
    /// Create a new CST816S instance
    pub fn new(i2c: I2C, touch_int: PIN) -> Self {
        Self {
            dev: AsynRegisterDevice::new(i2c, CST816S_ADDRESS),
            touch_int,
        }
    }

    /// Sets the auto-reset time after detecting a touch with no valid gesture.
    ///
    /// AutoReset register (0xFB):
    /// - 0: Auto-reset is disabled.
    /// - 1 to 5: The number of seconds before an auto-reset occurs (default is 5 seconds).
    ///
    /// # Arguments
    ///
    /// * `seconds` - Must be 0 (to disable) or 1 to 5 for the desired auto-reset delay.
    ///
    pub fn enable_auto_reset(&mut self, seconds: u8) -> Result<(), TouchSensorError> {
        let buffer = [0xFB, seconds];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    pub fn get_version(&mut self) -> Result<u8, TouchSensorError> {
        let result = self.dev.read_register(0xA9)?;
        Ok(result)
    }

    pub fn get_chip_id(&mut self) -> Result<ChipID, TouchSensorError> {
        let result = self.dev.read_register(0xA7)?;
        let result = ChipID::try_from(result).expect("Unknown chip ID");
        Ok(result)
    }

    /// Disable auto sleep mode
    pub fn disable_auto_sleep(&mut self) -> Result<(), TouchSensorError> {
        let buffer = [0xFE, 0x01];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    /// Enable auto sleep mode
    pub fn enable_auto_sleep(&mut self) -> Result<(), TouchSensorError> {
        let buffer = [0xFE, 0x00];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    /// Sets the interrupt pulse width in 0.1 ms units.
    ///
    /// IrqPulseWidth register (0xED):
    /// - Represents the interrupt pulse width in increments of 0.1 ms.
    /// - Default is 10 (i.e. 1.0 ms).
    /// - Valid values: 1 to 200.
    ///
    /// # Arguments
    ///
    /// * `width` - The desired pulse width, in 0.1 ms units. Values outside of 1–200 will be clamped.
    pub fn set_irq_pulse_width(&mut self, mut width: u8) -> Result<(), TouchSensorError> {
        // Clamp the width to the valid range 1 to 200.
        width = width.clamp(1, 200);
        // If you want to ensure the value is 8-bit:
        let buffer = [0xED, width];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    /// Gets the current interrupt pulse width setting, in 0.1 ms units.
    pub fn get_irq_pulse_width(&mut self) -> Result<u8, TouchSensorError> {
        let value = self.dev.read_register(0xED)?;
        Ok(value)
    }

    /// Sets the normal scan period in 10 ms units.
    ///
    /// NorScanPer register (0xEE):
    /// - Represents the normal scan period in increments of 10 ms.
    /// - Default is 1 (i.e. 10 ms).
    /// - Valid values: 1 to 30.
    ///
    /// # Arguments
    ///
    /// * `period` - The desired scan period in 10 ms units. Values outside of 1–30 will be clamped.
    pub fn set_nor_scan_period(&mut self, mut period: u8) -> Result<(), TouchSensorError> {
        // Clamp the period to the valid range 1 to 30.
        period = period.clamp(1, 30);
        let buffer = [0xEE, period];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    /// Gets the current normal scan period setting, in 10 ms units.
    pub fn get_nor_scan_period(&mut self) -> Result<u8, TouchSensorError> {
        let value = self.dev.read_register(0xEE)?;
        Ok(value)
    }

    /// Set auto sleep time
    ///
    /// AutoSleepTime register (0xF9):
    /// - This register defines the time in seconds before the sensor enters standby mode after inactivity.
    /// - Default is 2 seconds.
    /// - Valid values: 1 to 255 seconds.
    ///
    /// # Arguments
    ///
    /// * `seconds` - The number of seconds for auto sleep. Values outside of 1–255 will be clamped.
    pub fn set_auto_sleep_time(&mut self, mut seconds: i32) -> Result<(), TouchSensorError> {
        seconds = seconds.clamp(1, 255);
        let buffer = [0xF9, seconds as u8];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    /// Reads the long press time setting from the sensor.
    pub fn get_long_press_time(&mut self) -> Result<u8, TouchSensorError> {
        let result = self.dev.read_register(0xFC)?;
        Ok(result)
    }

    // Add these new methods
    /// Read the current interrupt control settings
    pub fn get_irq_control(&mut self) -> Result<IrqControl, TouchSensorError> {
        let result = self.dev.read_register(0xFA)?;
        Ok(IrqControl::from_bits_truncate(result))
    }

    /// Write interrupt control settings
    ///
    /// IrqCtl (register 0xFA): Control of interrupt behavior.
    /// - Bit 7: EnTest (Enable test, periodically sends low pulses)
    /// - Bit 6: EnTouch (Sends low pulse on touch detection)
    /// - Bit 5: EnChange (Sends low pulse on touch state change)
    /// - Bit 4: EnMotion (Sends low pulse on gesture detection)
    /// - Bit 0: OnceWLP (Sends one low pulse on long press)
    pub fn set_irq_control(&mut self, control: &IrqControl) -> Result<(), TouchSensorError> {
        let buffer = [0xFA, control.bits()];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    /// Writes the long press time setting to the sensor.
    ///
    /// # Arguments
    ///
    /// * `time` - The long press duration in seconds (0 to disable, 1-10 for duration).
    pub fn set_long_press_time(&mut self, time: u8) -> Result<(), TouchSensorError> {
        if time > 10 {
            return Err(TouchSensorError::InvalidLongPressTime);
        }

        let buffer = [0xFC, time];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    /// Get current motion mask settings from register 0xEC
    ///
    /// MotionMask register (0xEC):
    /// - Bit 0: EnDClick (enable double-click)
    /// - Bit 1: EnConUD (enable continuous up/down swipe)
    /// - Bit 2: EnConLR (enable continuous left/right swipe)
    pub fn get_motion_mask(&mut self) -> Result<MotionMask, TouchSensorError> {
        let result = self.dev.read_register(0xEC)?;
        Ok(MotionMask::from_bits_truncate(result))
    }

    /// Set motion mask settings to register 0xEC
    ///
    /// MotionMask register (0xEC):
    /// - Bit 0: EnDClick (enable double-click)
    /// - Bit 1: EnConUD (enable continuous up/down swipe)
    /// - Bit 2: EnConLR (enable continuous left/right swipe)
    ///
    /// # Arguments
    ///
    /// * `mask` - A `MotionMask` struct specifying which gestures to enable.
    pub fn set_motion_mask(&mut self, mask: &MotionMask) -> Result<(), TouchSensorError> {
        let buffer = [0xEC, mask.bits()];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    pub fn is_touch_available(&mut self) -> Result<bool, TouchSensorError> {
        self.touch_int
            .is_low()
            .map_err(|_| TouchSensorError::PinError)
    }

    pub fn read_touch(&mut self) -> Result<TouchData, TouchSensorError> {
        let mut buffer = [0u8; 13];

        self.dev.read_register_buffer(0x00, &mut buffer)?;

        let gesture = Gesture::try_from(buffer[1]).expect("Unknown gesture");
        let points = buffer[2] & 0x0F;

        let x_high = buffer[3] & 0x0f;
        let x_low = buffer[4];

        let y_high = buffer[5] & 0x0f;
        let y_low = buffer[6];

        let x: u16 = (u16::from(x_high) << 8) | u16::from(x_low);
        let y: u16 = (u16::from(y_high) << 8) | u16::from(y_low);

        let event = Event::try_from(buffer[3] >> 6).expect("Unknown event");

        let data = TouchData {
            gesture,
            points,
            event,
            x,
            y,
        };
        Ok(data)
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
