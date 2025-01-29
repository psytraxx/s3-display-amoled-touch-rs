// https://github.com/fbiego/CST816S

#![allow(dead_code)]
use embedded_hal::{digital::InputPin, i2c::I2c};

const CST816S_ADDRESS: u8 = 0x15;
const ONE_EVENT_LEN: usize = 6 + 3; // RAW_TOUCH_EVENT_LEN + GESTURE_HEADER_LEN

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

#[derive(Debug, Default)]
pub struct TouchData {
    pub gesture: Gesture,
    pub points: u8,
    pub event: u8,
    pub x: u16,
    pub y: u16,
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
        let mut data = TouchData::default();
        let mut buffer = [0u8; ONE_EVENT_LEN];

        if !check_int_pin || self.is_touch_available() {
            self.dev
                .read_register(0x01, &mut buffer)
                .map_err(|_| TouchSensorError::WriteError)?;
            data.gesture = Gesture::try_from(buffer[0])?;
            data.points = buffer[1];
            data.event = buffer[2] >> 6;
            data.x = (((buffer[2] & 0xF) as u16) << 8) + buffer[3] as u16;
            data.y = (((buffer[4] & 0xF) as u16) << 8) + buffer[5] as u16;
            Ok(Some(data))
        } else {
            Ok(None)
        }
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
}
