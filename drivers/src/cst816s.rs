// https://github.com/fbiego/CST816S
// https://github.com/mjdonders/CST816_TouchLib/blob/main/src/CST816Touch.cpp
// https://github.com/IniterWorker/cst816s
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

    pub async fn enable_auto_reset(&mut self) -> Result<(), TouchSensorError> {
        let buffer = [0xFB, 0x01];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    pub async fn get_version(&mut self) -> Result<u8, TouchSensorError> {
        let result = self.dev.read_register(0xA9).await?;
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

    /// Set auto sleep time
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
            en_test: (result & 0b1000_0000) != 0,
            en_touch: (result & 0b0100_0000) != 0,
            en_change: (result & 0b0010_0000) != 0,
            en_motion: (result & 0b0001_0000) != 0,
            once_wlp: (result & 0b0000_0001) != 0,
        })
    }

    /// Write interrupt control settings
    pub async fn set_irq_control(&mut self, control: &IrqControl) -> Result<(), TouchSensorError> {
        let value = (if control.en_test { 0b1000_0000 } else { 0 })
            | (if control.en_touch { 0b0100_0000 } else { 0 })
            | (if control.en_change { 0b0010_0000 } else { 0 })
            | (if control.en_motion { 0b0001_0000 } else { 0 })
            | (if control.once_wlp { 0b0000_0001 } else { 0 });

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

    /// Get current motion mask settings
    pub async fn get_motion_mask(&mut self) -> Result<MotionMask, TouchSensorError> {
        let result = self.dev.read_register(0xEC).await?;

        Ok(MotionMask {
            double_click: (result & 0b0000_0001) != 0,
            continuous_updown: (result & 0b0000_0010) != 0,
            continuous_leftright: (result & 0b0000_0100) != 0,
        })
    }

    /// Set motion mask settings
    pub async fn set_motion_mask(&mut self, mask: &MotionMask) -> Result<(), TouchSensorError> {
        let value = (if mask.double_click { 0b0000_0001 } else { 0 })
            | (if mask.continuous_updown {
                0b0000_0010
            } else {
                0
            })
            | (if mask.continuous_leftright {
                0b0000_0100
            } else {
                0
            });

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
