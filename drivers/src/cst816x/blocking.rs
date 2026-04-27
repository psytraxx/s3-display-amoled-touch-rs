use super::{
    CST816S_ADDRESS, ChipID, Event, Gesture, IrqControl, MotionMask, TouchData, TouchSensorError,
};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::I2c;

/// Blocking (sync) implementation of the CST816x driver.
impl<I2C, PIN, RST, DELAY> super::CST816x<I2C, PIN, RST, DELAY>
where
    I2C: I2c,
    PIN: InputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    // ---- I2C helpers ----

    fn read_register(&mut self, register: u8) -> Result<u8, TouchSensorError> {
        let mut buffer = [0u8];
        self.i2c
            .write_read(CST816S_ADDRESS, &[register], &mut buffer)?;
        Ok(buffer[0])
    }

    fn read_register_buffer(
        &mut self,
        register: u8,
        buffer: &mut [u8],
    ) -> Result<(), TouchSensorError> {
        self.i2c
            .write_read(CST816S_ADDRESS, &[register], buffer)?;
        Ok(())
    }

    fn write_register(&mut self, register_and_data: &[u8]) -> Result<(), TouchSensorError> {
        self.i2c.write(CST816S_ADDRESS, register_and_data)?;
        Ok(())
    }

    // ---- Public API ----

    pub fn begin(&mut self) -> Result<(), TouchSensorError> {
        self.reset()?;
        let _ = self.get_chip_id()?;
        self.set_irq_control(&IrqControl::default())?;
        self.set_motion_mask(&MotionMask::default())?;
        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), TouchSensorError> {
        if let Some(rst) = &mut self.rst_pin {
            rst.set_low().map_err(|_| TouchSensorError::PinError)?;
            self.delay.delay_ms(20);
            rst.set_high().map_err(|_| TouchSensorError::PinError)?;
            self.delay.delay_ms(100);
        }
        self.gesture_state.reset();
        Ok(())
    }

    pub fn sleep(&mut self) -> Result<(), TouchSensorError> {
        self.write_register(&[0xA5, 0x03])
    }

    pub fn enable_auto_reset(&mut self, seconds: u8) -> Result<(), TouchSensorError> {
        self.write_register(&[0xFB, seconds])
    }

    pub fn get_version(&mut self) -> Result<u8, TouchSensorError> {
        self.read_register(0xA9)
    }

    pub fn get_chip_id(&mut self) -> Result<ChipID, TouchSensorError> {
        let result = self.read_register(0xA7)?;
        Ok(ChipID::try_from(result).expect("Unknown chip ID"))
    }

    pub fn disable_auto_sleep(&mut self) -> Result<(), TouchSensorError> {
        self.write_register(&[0xFE, 0x01])
    }

    pub fn enable_auto_sleep(&mut self) -> Result<(), TouchSensorError> {
        self.write_register(&[0xFE, 0x00])
    }

    pub fn set_irq_pulse_width(&mut self, mut width: u8) -> Result<(), TouchSensorError> {
        width = width.clamp(1, 200);
        self.write_register(&[0xED, width])
    }

    pub fn get_irq_pulse_width(&mut self) -> Result<u8, TouchSensorError> {
        self.read_register(0xED)
    }

    pub fn set_nor_scan_period(&mut self, mut period: u8) -> Result<(), TouchSensorError> {
        period = period.clamp(1, 30);
        self.write_register(&[0xEE, period])
    }

    pub fn get_nor_scan_period(&mut self) -> Result<u8, TouchSensorError> {
        self.read_register(0xEE)
    }

    pub fn set_auto_sleep_time(&mut self, mut seconds: i32) -> Result<(), TouchSensorError> {
        seconds = seconds.clamp(1, 255);
        self.write_register(&[0xF9, seconds as u8])
    }

    pub fn get_long_press_time(&mut self) -> Result<u8, TouchSensorError> {
        self.read_register(0xFC)
    }

    pub fn get_irq_control(&mut self) -> Result<IrqControl, TouchSensorError> {
        let result = self.read_register(0xFA)?;
        Ok(IrqControl::from_bits_truncate(result))
    }

    pub fn set_irq_control(&mut self, control: &IrqControl) -> Result<(), TouchSensorError> {
        self.write_register(&[0xFA, control.bits()])
    }

    pub fn set_long_press_time(&mut self, time: u8) -> Result<(), TouchSensorError> {
        if time > 10 {
            return Err(TouchSensorError::InvalidLongPressTime);
        }
        self.write_register(&[0xFC, time])
    }

    pub fn get_motion_mask(&mut self) -> Result<MotionMask, TouchSensorError> {
        let result = self.read_register(0xEC)?;
        Ok(MotionMask::from_bits_truncate(result))
    }

    pub fn set_motion_mask(&mut self, mask: &MotionMask) -> Result<(), TouchSensorError> {
        self.write_register(&[0xEC, mask.bits()])
    }

    pub fn is_touch_available(&mut self) -> Result<bool, TouchSensorError> {
        self.touch_int
            .is_low()
            .map_err(|_| TouchSensorError::PinError)
    }

    pub fn read_touch(&mut self) -> Result<TouchData, TouchSensorError> {
        let mut buffer = [0u8; 7];
        self.read_register_buffer(0x01, &mut buffer)?;

        let gesture = Gesture::try_from(buffer[0]).unwrap_or(Gesture::None);
        let points = buffer[1] & 0x0F;
        let x_high = buffer[2] & 0x0f;
        let x_low = buffer[3];
        let x: u16 = (u16::from(x_high) << 8) | u16::from(x_low);
        let y_high = buffer[4] & 0x0f;
        let y_low = buffer[5];
        let y: u16 = (u16::from(y_high) << 8) | u16::from(y_low);
        let event = Event::try_from(buffer[2] >> 6).unwrap_or(Event::Down);

        let mut data = TouchData {
            gesture,
            points,
            event,
            x,
            y,
        };

        self.gesture_state.process(&mut data, &self.gesture_config);

        Ok(data)
    }
}
