use super::{
    ChipID, Event, Gesture, GestureState, IrqControl, MotionMask, SoftwareGestureConfig, TouchData,
    TouchSensorError, CST816S_ADDRESS,
};
use crate::BlockingRegisterDevice;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::i2c::I2c;

#[derive(Debug)]
pub struct CST816x<I2C, PIN, RST, DELAY> {
    dev: BlockingRegisterDevice<I2C>,
    touch_int: PIN,
    rst_pin: Option<RST>,
    delay: DELAY,
    last_buffer: [u8; 7],
    gesture_state: GestureState,
    gesture_config: SoftwareGestureConfig,
}

impl<I2C, PIN, RST, DELAY> CST816x<I2C, PIN, RST, DELAY>
where
    I2C: I2c,
    PIN: InputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    /// Create a new CST816S instance
    pub fn new(i2c: I2C, touch_int: PIN, rst_pin: Option<RST>, delay: DELAY) -> Self {
        Self {
            dev: BlockingRegisterDevice::new(i2c, CST816S_ADDRESS),
            touch_int,
            rst_pin,
            delay,
            last_buffer: [0u8; 7],
            gesture_state: GestureState::default(),
            gesture_config: SoftwareGestureConfig::default(),
        }
    }

    /// Configure software gesture detection
    pub fn set_software_gesture_config(&mut self, config: SoftwareGestureConfig) {
        self.gesture_config = config;
    }

    /// Initialize the touch controller
    pub fn begin(&mut self) -> Result<(), TouchSensorError> {
        self.reset()?;

        // Verify communication
        let _ = self.get_chip_id()?;

        // Configure default settings
        let irq_control = IrqControl::default();
        self.set_irq_control(&irq_control)?;

        let motion_mask = MotionMask::default();
        self.set_motion_mask(&motion_mask)?;

        Ok(())
    }

    /// Reset the chip via the reset pin
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

    /// Put the chip to sleep
    pub fn sleep(&mut self) -> Result<(), TouchSensorError> {
        let buffer = [0xA5, 0x03]; // TOUCH_REGISTER_SLEEP, TOUCH_CMD_SLEEP
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    /// Sets the auto-reset time after detecting a touch with no valid gesture.
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

    pub fn set_irq_pulse_width(&mut self, mut width: u8) -> Result<(), TouchSensorError> {
        width = width.clamp(1, 200);
        let buffer = [0xED, width];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    pub fn get_irq_pulse_width(&mut self) -> Result<u8, TouchSensorError> {
        let value = self.dev.read_register(0xED)?;
        Ok(value)
    }

    pub fn set_nor_scan_period(&mut self, mut period: u8) -> Result<(), TouchSensorError> {
        period = period.clamp(1, 30);
        let buffer = [0xEE, period];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    pub fn get_nor_scan_period(&mut self) -> Result<u8, TouchSensorError> {
        let value = self.dev.read_register(0xEE)?;
        Ok(value)
    }

    pub fn set_auto_sleep_time(&mut self, mut seconds: i32) -> Result<(), TouchSensorError> {
        seconds = seconds.clamp(1, 255);
        let buffer = [0xF9, seconds as u8];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    pub fn get_long_press_time(&mut self) -> Result<u8, TouchSensorError> {
        let result = self.dev.read_register(0xFC)?;
        Ok(result)
    }

    pub fn get_irq_control(&mut self) -> Result<IrqControl, TouchSensorError> {
        let result = self.dev.read_register(0xFA)?;
        Ok(IrqControl::from_bits_truncate(result))
    }

    pub fn set_irq_control(&mut self, control: &IrqControl) -> Result<(), TouchSensorError> {
        let buffer = [0xFA, control.bits()];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    pub fn set_long_press_time(&mut self, time: u8) -> Result<(), TouchSensorError> {
        if time > 10 {
            return Err(TouchSensorError::InvalidLongPressTime);
        }

        let buffer = [0xFC, time];
        self.dev.write_register(&buffer)?;
        Ok(())
    }

    pub fn get_motion_mask(&mut self) -> Result<MotionMask, TouchSensorError> {
        let result = self.dev.read_register(0xEC)?;
        Ok(MotionMask::from_bits_truncate(result))
    }

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

    pub fn read_touch(&mut self) -> Result<Option<TouchData>, TouchSensorError> {
        // Read 7 bytes starting from register 0x01, matching C++ implementation
        let mut buffer = [0u8; 7];
        self.dev.read_register_buffer(0x01, &mut buffer)?;

        // Duplicate check
        if buffer == self.last_buffer {
            return Ok(None);
        }
        self.last_buffer = buffer;

        // Indices adjusted based on reading from 0x01
        // If the hardware reports an invalid gesture ID (or 0x00), treat it as None.
        // This allows the software gesture engine to attempt detection in the process() call below.
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

        // Process software gestures
        self.gesture_state.process(&mut data, &self.gesture_config);

        Ok(Some(data))
    }
}
