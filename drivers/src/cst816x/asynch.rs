use super::{
    ChipID, Event, Gesture, IrqControl, MotionMask, TouchData, TouchSensorError, CST816S_ADDRESS,
};
use crate::AsyncRegisterDevice;
use embedded_hal::digital::InputPin;

#[derive(Debug)]
pub struct CST816xAsync<I2C, PIN> {
    dev: AsyncRegisterDevice<I2C>,
    touch_int: PIN,
}

impl<I2C, PIN> CST816xAsync<I2C, PIN>
where
    I2C: embedded_hal_async::i2c::I2c,
    PIN: InputPin,
{
    /// Create a new CST816S instance
    pub fn new(i2c: I2C, touch_int: PIN) -> Self {
        Self {
            dev: AsyncRegisterDevice::new(i2c, CST816S_ADDRESS),
            touch_int,
        }
    }

    pub async fn enable_auto_reset(&mut self, seconds: u8) -> Result<(), TouchSensorError> {
        let buffer = [0xFB, seconds];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    pub async fn get_version(&mut self) -> Result<u8, TouchSensorError> {
        let result = self.dev.read_register(0xA9).await?;
        Ok(result)
    }

    pub async fn get_chip_id(&mut self) -> Result<ChipID, TouchSensorError> {
        let result = self.dev.read_register(0xA7).await?;
        let result = ChipID::try_from(result).expect("Unknown chip ID");
        Ok(result)
    }

    pub async fn disable_auto_sleep(&mut self) -> Result<(), TouchSensorError> {
        let buffer = [0xFE, 0x01];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    pub async fn enable_auto_sleep(&mut self) -> Result<(), TouchSensorError> {
        let buffer = [0xFE, 0x00];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    pub async fn set_irq_pulse_width(&mut self, mut width: u8) -> Result<(), TouchSensorError> {
        width = width.clamp(1, 200);
        let buffer = [0xED, width];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    pub async fn get_irq_pulse_width(&mut self) -> Result<u8, TouchSensorError> {
        let value = self.dev.read_register(0xED).await?;
        Ok(value)
    }

    pub async fn set_nor_scan_period(&mut self, mut period: u8) -> Result<(), TouchSensorError> {
        period = period.clamp(1, 30);
        let buffer = [0xEE, period];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    pub async fn get_nor_scan_period(&mut self) -> Result<u8, TouchSensorError> {
        let value = self.dev.read_register(0xEE).await?;
        Ok(value)
    }

    pub async fn set_auto_sleep_time(&mut self, mut seconds: i32) -> Result<(), TouchSensorError> {
        seconds = seconds.clamp(1, 255);
        let buffer = [0xF9, seconds as u8];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    pub async fn get_long_press_time(&mut self) -> Result<u8, TouchSensorError> {
        let result = self.dev.read_register(0xFC).await?;
        Ok(result)
    }

    pub async fn get_irq_control(&mut self) -> Result<IrqControl, TouchSensorError> {
        let result = self.dev.read_register(0xFA).await?;
        Ok(IrqControl::from_bits_truncate(result))
    }

    pub async fn set_irq_control(&mut self, control: &IrqControl) -> Result<(), TouchSensorError> {
        let buffer = [0xFA, control.bits()];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    pub async fn set_long_press_time(&mut self, time: u8) -> Result<(), TouchSensorError> {
        if time > 10 {
            return Err(TouchSensorError::InvalidLongPressTime);
        }
        let buffer = [0xFC, time];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    pub async fn get_motion_mask(&mut self) -> Result<MotionMask, TouchSensorError> {
        let result = self.dev.read_register(0xEC).await?;
        Ok(MotionMask::from_bits_truncate(result))
    }

    pub async fn set_motion_mask(&mut self, mask: &MotionMask) -> Result<(), TouchSensorError> {
        let buffer = [0xEC, mask.bits()];
        self.dev.write_register(&buffer).await?;
        Ok(())
    }

    pub fn is_touch_available(&mut self) -> Result<bool, TouchSensorError> {
        self.touch_int
            .is_low()
            .map_err(|_| TouchSensorError::PinError)
    }

    pub async fn read_touch(&mut self) -> Result<TouchData, TouchSensorError> {
        let mut buffer = [0u8; 7];
        self.dev.read_register_buffer(0x01, &mut buffer).await?;

        let gesture = Gesture::try_from(buffer[0]).expect("Unknown gesture");
        let points = buffer[1] & 0x0F;
        let x_high = buffer[2] & 0x0f;
        let x_low = buffer[3];
        let x: u16 = (u16::from(x_high) << 8) | u16::from(x_low);
        let y_high = buffer[4] & 0x0f;
        let y_low = buffer[5];
        let y: u16 = (u16::from(y_high) << 8) | u16::from(y_low);
        let event = Event::try_from(buffer[2] >> 6).expect("Unknown event");

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
