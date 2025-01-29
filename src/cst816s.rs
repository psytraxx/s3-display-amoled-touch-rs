// https://github.com/fbiego/CST816S
// https://github.com/mjdonders/CST816_TouchLib/blob/main/src/CST816Touch.cpp
#![allow(dead_code)]
use embedded_hal::{digital::InputPin, i2c::I2c};

const CST816S_ADDRESS: u8 = 0x15;
const ONE_EVENT_LEN: usize = 6 + 3; // RAW_TOUCH_EVENT_LEN + GESTURE_HEADER_LEN
/// Number of bytes for a single touch event
pub const RAW_TOUCH_EVENT_LEN: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Gesture {
    #[default]
    None = 0x00,
    SwipeUp = 0x01,
    SwipeDown = 0x02,
    SwipeLeft = 0x03,
    SwipeRight = 0x04,
    SingleClick = 0x05,
    DoubleClick = 0x0B,
    LongPress = 0x0C,
}

impl TryFrom<u8> for Gesture {
    type Error = TouchSensorError;
    fn try_from(value: u8) -> Result<Self, TouchSensorError> {
        match value {
            0x00 => Ok(Gesture::None),
            0x01 => Ok(Gesture::SwipeUp),
            0x02 => Ok(Gesture::SwipeDown),
            0x03 => Ok(Gesture::SwipeLeft),
            0x04 => Ok(Gesture::SwipeRight),
            0x05 => Ok(Gesture::SingleClick),
            0x0B => Ok(Gesture::DoubleClick),
            0x0C => Ok(Gesture::LongPress),
            _ => Err(TouchSensorError::UnknownGesture(value)),
        }
    }
}

#[derive(Debug)]
pub struct TouchData {
    pub gesture: Gesture,
    pub points: u8,
    pub event: u8,
    pub x: i32,
    pub y: i32,
    pub pressure: u8,
    pub area: u8,
}

// Add after existing enums
#[derive(Debug, Clone, Copy)]
pub struct IrqControl {
    pub en_test: bool,   // Bit 7: Enable test mode (periodic low pulses)
    pub en_touch: bool,  // Bit 6: Enable touch detection interrupt
    pub en_change: bool, // Bit 5: Enable touch state change interrupt
    pub en_motion: bool, // Bit 4: Enable gesture detection interrupt
    pub once_wlp: bool,  // Bit 0: Enable single pulse on long press
}

impl Default for IrqControl {
    fn default() -> Self {
        Self {
            en_test: false,
            en_touch: true,  // Enable touch by default
            en_change: true, // Enable change by default
            en_motion: true, // Enable motion by default
            once_wlp: false,
        }
    }
}

#[derive(Debug)]
pub struct CST816S<I2C, PIN> {
    dev: CST816SDevice<I2C>,
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
            dev: CST816SDevice::new(i2c, CST816S_ADDRESS),
            touch_int,
        }
    }

    /// Enable double click
    pub fn enable_double_click(&mut self) -> Result<(), TouchSensorError> {
        self.dev.write_register(0xEC, 0x01)
    }

    pub fn enable_auto_reset(&mut self) -> Result<(), TouchSensorError> {
        self.dev.write_register(0xFB, 0x01)
    }

    pub fn get_version(&mut self) -> Result<u8, TouchSensorError> {
        let mut buffer = [0u8; 1];
        self.dev.read_register(0xA9, &mut buffer)?;
        Ok(buffer[0])
    }

    /// Disable auto sleep mode
    pub fn disable_auto_sleep(&mut self) -> Result<(), TouchSensorError> {
        self.dev.write_register(0xFE, 0x01)
    }

    /// Enable auto sleep mode
    pub fn enable_auto_sleep(&mut self) -> Result<(), TouchSensorError> {
        self.dev.write_register(0xFE, 0x00)
    }

    /// Set auto sleep time
    pub fn set_auto_sleep_time(&mut self, mut seconds: i32) -> Result<(), TouchSensorError> {
        seconds = seconds.clamp(1, 255);

        self.dev.write_register(0xF9, seconds as u8)
    }

    pub fn read_touch(
        &mut self,
        check_int_pin: bool,
    ) -> Result<Option<TouchData>, TouchSensorError> {
        // return if no touch detected
        if check_int_pin && !self.is_touch_available() {
            return Ok(None);
        }

        let mut buffer = [0u8; ONE_EVENT_LEN];

        self.dev
            .read_register(0x01, &mut buffer)
            .map_err(|_| TouchSensorError::WriteError)?;

        let points = buffer[1];

        // return if no points detected
        if points == 0 {
            return Ok(None);
        }

        let gesture = Gesture::try_from(buffer[0])?;
        let touch_x_h_and_action = buffer[2];
        let touch_y_h_and_finger = buffer[4];
        let x = (buffer[3] as i32) | (((touch_x_h_and_action & 0x0F) as i32) << 8);
        let y = (buffer[5] as i32) | (((touch_y_h_and_finger & 0x0F) as i32) << 8);
        let event = touch_x_h_and_action >> 6;
        let points = touch_y_h_and_finger >> 4;
        let pressure = buffer[6];
        let area = buffer[7];
        let data = TouchData {
            gesture,
            points,
            event,
            x,
            y,
            pressure,
            area,
        };
        Ok(Some(data))
    }

    /// Reads the long press time setting from the sensor.
    pub fn read_long_press_time(&mut self) -> Result<u8, TouchSensorError> {
        let mut buffer = [0];
        self.dev
            .read_register(0xFC, &mut buffer)
            .map_err(|_| TouchSensorError::ReadError)?;
        Ok(buffer[0])
    }

    // Add these new methods
    /// Read the current interrupt control settings
    pub fn read_irq_control(&mut self) -> Result<IrqControl, TouchSensorError> {
        let mut buffer = [0u8];
        self.dev.read_register(0xFA, &mut buffer)?;

        Ok(IrqControl {
            en_test: (buffer[0] & 0b1000_0000) != 0,
            en_touch: (buffer[0] & 0b0100_0000) != 0,
            en_change: (buffer[0] & 0b0010_0000) != 0,
            en_motion: (buffer[0] & 0b0001_0000) != 0,
            once_wlp: (buffer[0] & 0b0000_0001) != 0,
        })
    }

    /// Write interrupt control settings
    pub fn write_irq_control(&mut self, control: &IrqControl) -> Result<(), TouchSensorError> {
        let value = (if control.en_test { 0b1000_0000 } else { 0 })
            | (if control.en_touch { 0b0100_0000 } else { 0 })
            | (if control.en_change { 0b0010_0000 } else { 0 })
            | (if control.en_motion { 0b0001_0000 } else { 0 })
            | (if control.once_wlp { 0b0000_0001 } else { 0 });

        self.dev.write_register(0xFA, value)
    }

    /// Writes the long press time setting to the sensor.
    ///
    /// # Arguments
    ///
    /// * `time` - The long press duration in seconds (0 to disable, 1-10 for duration).
    pub fn write_long_press_time(&mut self, time: u8) -> Result<(), TouchSensorError> {
        if time > 10 {
            return Err(TouchSensorError::InvalidLongPressTime);
        }
        self.dev
            .write_register(0xFC, time)
            .map_err(|_| TouchSensorError::WriteError)?;
        Ok(())
    }

    fn is_touch_available(&mut self) -> bool {
        self.touch_int.is_low().unwrap()
    }
}

#[derive(Debug)]
struct CST816SDevice<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> CST816SDevice<I2C>
where
    I2C: I2c,
{
    fn new(i2c: I2C, adr: u8) -> Self {
        Self { i2c, address: adr }
    }

    fn write_register(&mut self, register_address: u8, data: u8) -> Result<(), TouchSensorError> {
        let mut buffer = [0u8; 2]; // Assuming 1 byte register + 1 byte data
        buffer[0] = register_address;
        buffer[1] = data;

        self.i2c
            .write(self.address, &buffer)
            .map_err(|_| TouchSensorError::WriteError)
    }

    fn read_register(
        &mut self,
        register_address: u8,
        buffer: &mut [u8],
    ) -> Result<(), TouchSensorError>
    where
        I2C: I2c,
    {
        self.i2c
            .write_read(self.address, &[register_address], buffer)
            .map_err(|_| TouchSensorError::ReadError)?;
        Ok(())
    }
}

/// Errors that can occur when interacting with the BQ25896
#[derive(Debug)]
pub enum TouchSensorError {
    WriteError,
    ReadError,
    UnknownGesture(u8),
    InvalidLongPressTime,
}
