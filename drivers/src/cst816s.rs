// https://github.com/fbiego/CST816S
use embedded_hal::digital::InputPin;
use embedded_hal_async::i2c::I2c;

use crate::AsynRegisterDevice;

const CST816S_ADDRESS: u8 = 0x15;

/// Number of bytes for a single touch event
pub const RAW_TOUCH_EVENT_LEN: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq)]
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

impl From<u8> for Gesture {
    fn from(value: u8) -> Self {
        match value {
            0x00 => Gesture::None,
            0x01 => Gesture::SwipeUp,
            0x02 => Gesture::SwipeDown,
            0x03 => Gesture::SwipeLeft,
            0x04 => Gesture::SwipeRight,
            0x05 => Gesture::SingleClick,
            0x0B => Gesture::DoubleClick,
            0x0C => Gesture::LongPress,
            _ => Gesture::None,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    Down = 0,
    Up = 1,
    Contact = 2,
}

impl From<u8> for Event {
    fn from(value: u8) -> Self {
        match value {
            0 => Event::Down,
            1 => Event::Up,
            2 => Event::Contact,
            _ => panic!("Unknown event: {}", value),
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

// Add after existing enums
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IrqControl {
    /// Bit 7: EnTest (Enable test, periodically sends low pulses)
    pub en_test: bool,
    /// Bit 6: EnTouch (Sends low pulse on touch detection)
    pub en_touch: bool,
    /// Bit 5: EnChange (Sends low pulse on touch state change
    pub en_change: bool,
    /// Bit 4:  Bit 4: EnMotion (Sends low pulse on gesture detection)
    pub en_motion: bool,
    /// Bit 0: OnceWLP (Sends one low pulse on long press)
    pub once_wlp: bool,
}

impl Default for IrqControl {
    fn default() -> Self {
        Self {
            en_test: false,
            en_touch: true,
            en_change: true,
            en_motion: true,
            once_wlp: false,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotionMask {
    /// Enable double-click detection
    pub double_click: bool,
    /// Enable continuous up/down swipe
    pub continuous_updown: bool,
    /// Enable continuous left/right swipe
    pub continuous_leftright: bool,
}
#[derive(Debug)]
pub struct CST816S<I2C, PIN> {
    dev: AsynRegisterDevice<I2C>,
    touch_int: PIN,
}

impl<I2C, PIN> CST816S<I2C, PIN>
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
    pub async fn enable_auto_reset(&mut self, seconds: u8) -> Result<(), TouchSensorError> {
        let buffer = [0xFB, seconds];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    pub async fn get_version(&mut self) -> Result<u8, TouchSensorError> {
        let result = self.dev.read_register(0xA9).await?;
        Ok(result)
    }

    pub async fn get_chip_id(&mut self) -> Result<u8, TouchSensorError> {
        let result = self.dev.read_register(0xA7).await?;
        Ok(result)
    }

    /// Disable auto sleep mode
    pub async fn disable_auto_sleep(&mut self) -> Result<(), TouchSensorError> {
        let buffer = [0xFE, 0x01];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    /// Enable auto sleep mode
    pub async fn enable_auto_sleep(&mut self) -> Result<(), TouchSensorError> {
        let buffer = [0xFE, 0x00];
        self.dev.write_register(&buffer).await?;
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
    pub async fn set_irq_pulse_width(&mut self, mut width: u8) -> Result<(), TouchSensorError> {
        // Clamp the width to the valid range 1 to 200.
        width = width.clamp(1, 200);
        // If you want to ensure the value is 8-bit:
        let buffer = [0xED, width];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    /// Gets the current interrupt pulse width setting, in 0.1 ms units.
    pub async fn get_irq_pulse_width(&mut self) -> Result<u8, TouchSensorError> {
        let value = self.dev.read_register(0xED).await?;
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
    pub async fn set_nor_scan_period(&mut self, mut period: u8) -> Result<(), TouchSensorError> {
        // Clamp the period to the valid range 1 to 30.
        period = period.clamp(1, 30);
        let buffer = [0xEE, period];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    /// Gets the current normal scan period setting, in 10 ms units.
    pub async fn get_nor_scan_period(&mut self) -> Result<u8, TouchSensorError> {
        let value = self.dev.read_register(0xEE).await?;
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
    pub async fn set_auto_sleep_time(&mut self, mut seconds: i32) -> Result<(), TouchSensorError> {
        seconds = seconds.clamp(1, 255);
        let buffer = [0xF9, seconds as u8];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    /// Reads the long press time setting from the sensor.
    pub async fn get_long_press_time(&mut self) -> Result<u8, TouchSensorError> {
        let result = self.dev.read_register(0xFC).await?;
        Ok(result)
    }

    // Add these new methods
    /// Read the current interrupt control settings
    pub async fn get_irq_control(&mut self) -> Result<IrqControl, TouchSensorError> {
        let result = self.dev.read_register(0xFA).await?;

        Ok(IrqControl {
            en_test: (result & (1 << 7)) != 0,   // Bit 7
            en_touch: (result & (1 << 6)) != 0,  // Bit 6
            en_change: (result & (1 << 5)) != 0, // Bit 5
            en_motion: (result & (1 << 4)) != 0, // Bit 4
            once_wlp: (result & (1 << 0)) != 0,  // Bit 0
        })
    }

    /// Write interrupt control settings
    ///
    /// IrqCtl (register 0xFA): Control of interrupt behavior.
    /// - Bit 7: EnTest (Enable test, periodically sends low pulses)
    /// - Bit 6: EnTouch (Sends low pulse on touch detection)
    /// - Bit 5: EnChange (Sends low pulse on touch state change)
    /// - Bit 4: EnMotion (Sends low pulse on gesture detection)
    /// - Bit 0: OnceWLP (Sends one low pulse on long press)
    pub async fn set_irq_control(&mut self, control: &IrqControl) -> Result<(), TouchSensorError> {
        let value = (if control.en_test { 1 << 7 } else { 0 })
            | (if control.en_touch { 1 << 6 } else { 0 })
            | (if control.en_change { 1 << 5 } else { 0 })
            | (if control.en_motion { 1 << 4 } else { 0 })
            | (if control.once_wlp { 1 << 0 } else { 0 });

        let buffer = [0xFA, value];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    /// Writes the long press time setting to the sensor.
    ///
    /// # Arguments
    ///
    /// * `time` - The long press duration in seconds (0 to disable, 1-10 for duration).
    pub async fn set_long_press_time(&mut self, time: u8) -> Result<(), TouchSensorError> {
        if time > 10 {
            return Err(TouchSensorError::InvalidLongPressTime);
        }

        let buffer = [0xFC, time];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    /// Get current motion mask settings from register 0xEC
    ///
    /// MotionMask register (0xEC):
    /// - Bit 0: EnDClick (enable double-click)
    /// - Bit 1: EnConUD (enable continuous up/down swipe)
    /// - Bit 2: EnConLR (enable continuous left/right swipe)
    pub async fn get_motion_mask(&mut self) -> Result<MotionMask, TouchSensorError> {
        let result = self.dev.read_register(0xEC).await?;
        Ok(MotionMask {
            double_click: (result & (1 << 0)) != 0,      // Bit 0: EnDClick
            continuous_updown: (result & (1 << 1)) != 0, // Bit 1: EnConUD
            continuous_leftright: (result & (1 << 2)) != 0, // Bit 2: EnConLR
        })
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
    pub async fn set_motion_mask(&mut self, mask: &MotionMask) -> Result<(), TouchSensorError> {
        // Use bit shifting and bitwise OR to combine the flags into a single value
        let value = (if mask.double_click { 1 << 0 } else { 0 })
            | (if mask.continuous_updown { 1 << 1 } else { 0 })
            | (if mask.continuous_leftright { 1 << 2 } else { 0 });

        let buffer = [0xEC, value];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    fn is_touch_available(&mut self) -> Result<bool, TouchSensorError> {
        self.touch_int
            .is_low()
            .map_err(|_| TouchSensorError::PinError)
    }

    pub async fn read_touch(
        &mut self,
        check_int_pin: bool,
    ) -> Result<Option<TouchData>, TouchSensorError> {
        // return if no touch detected
        if check_int_pin && !self.is_touch_available()? {
            return Ok(None);
        }

        let mut buffer = [0u8; 13];

        self.dev.read_register_buffer(0x00, &mut buffer).await?;

        let gesture = Gesture::from(buffer[1]);
        let points = buffer[2] & 0x0F;

        let x_high = buffer[3] & 0x0f;
        let x_low = buffer[4];

        let y_high = buffer[5] & 0x0f;
        let y_low = buffer[6];

        let x: u16 = (u16::from(x_high) << 8) | u16::from(x_low);
        let y: u16 = (u16::from(y_high) << 8) | u16::from(y_low);

        let event = Event::from(buffer[3] >> 6);

        let data = TouchData {
            gesture,
            points,
            event,
            x,
            y,
        };
        Ok(Some(data))
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
    E: embedded_hal_async::i2c::Error,
{
    fn from(_: E) -> Self {
        TouchSensorError::I2CError
    }
}
