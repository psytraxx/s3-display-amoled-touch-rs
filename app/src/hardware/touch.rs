//! Touchpad hardware initialization module
//!
//! This module handles the initialization and configuration of the CST816x
//! capacitive touch controller via I2C interface.

use drivers::cst816x::asynch::CST816xAsync;
use drivers::cst816x::IrqControl;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Delay;
use esp_hal::gpio::{Input, InputConfig, Output, Pull};
use esp_hal::i2c::master::I2c;
use esp_hal::peripherals::GPIO21;
use esp_hal::Async;
use log::info;

/// Type alias for the CST816x touchpad driver instance
pub type Touchpad = CST816xAsync<
    I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
    Input<'static>,
    Output<'static>,
    Delay,
>;

/// Initializes and configures the CST816x capacitive touchpad.
///
/// This function:
/// - Configures the touch IRQ GPIO pin
/// - Initializes the CST816x driver via I2C
/// - Configures interrupts for touch, change, and motion events
/// - Enables auto-reset functionality
/// - Sets IRQ pulse width
/// - Logs configuration details
///
/// # Arguments
///
/// * `i2c_device` - Acquired I2C device handle from the shared bus
/// * `touch` - GPIO pin for touch interrupt
///
/// # Returns
///
/// Returns an instance of the touchpad driver.
///
/// # Panics
///
/// Panics if any initialization step fails.
pub async fn initialize_touchpad(
    i2c_device: I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
    touch: GPIO21<'static>,
) -> Touchpad {
    // Configure the GPIO pin used for touch input (no pull-up/down)
    let touch_pin = Input::new(touch, InputConfig::default().with_pull(Pull::None));
    let mut touchpad = CST816xAsync::new(i2c_device, touch_pin, None, Delay);
    touchpad.begin().await.expect("Failed to begin touchpad");
    let irq_config = IrqControl::EN_TOUCH | IrqControl::EN_CHANGE | IrqControl::EN_MOTION;
    touchpad
        .set_irq_control(&irq_config)
        .await
        .expect("Failed to set IRQ control");
    touchpad
        .enable_auto_reset(5)
        .await
        .expect("Failed to enable auto-reset");
    let irq_config = touchpad
        .get_irq_control()
        .await
        .expect("Failed to get IRQ control");
    info!("IRQ control: 0x{:X}", irq_config.bits());
    let chip_id = touchpad.get_chip_id().await.expect("Failed to get chip ID");
    info!("Touchpad chip ID: {chip_id}");
    let motion_mask = touchpad
        .get_motion_mask()
        .await
        .expect("Failed to get motion mask");
    info!("Motion mask: 0x{motion_mask:X}");
    touchpad
        .set_irq_pulse_width(10)
        .await
        .expect("Failed to set pulse width");
    let pulse_config = touchpad
        .get_irq_pulse_width()
        .await
        .expect("Failed to get pulse config");
    info!("Pulse width: {pulse_config:?}");

    touchpad
}
