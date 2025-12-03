//! Radar sensor hardware initialization module
//!
//! This module handles the initialization of the LD2410 human presence radar
//! sensor via UART interface.

use drivers::ld2410::asynch::LD2410Async;
use embassy_time::Delay;
use esp_hal::peripherals::{GPIO43, GPIO44, UART0};
use esp_hal::uart::{Config as UartConfig, Parity, StopBits, Uart};
use esp_hal::Async;

/// Type alias for the LD2410 radar sensor driver instance
pub type RadarSensor = LD2410Async<Uart<'static, Async>, Delay>;

/// Creates and initializes the LD2410 radar sensor interface using UART.
///
/// This function:
/// - Configures UART0 with 256000 baud rate, no parity, 1 stop bit
/// - Associates RX and TX GPIO pins
/// - Initializes the LD2410 driver in async mode
///
/// # Arguments
///
/// * `uart1` - UART0 peripheral instance
/// * `rx_pin` - GPIO pin for UART RX (receiving data from radar)
/// * `tx_pin` - GPIO pin for UART TX (sending commands to radar)
///
/// # Returns
///
/// Returns the configured radar sensor instance.
///
/// # Panics
///
/// Panics if UART initialization fails.
pub fn initialize_radar(
    uart1: UART0<'static>,
    rx_pin: GPIO44<'static>,
    tx_pin: GPIO43<'static>,
) -> RadarSensor {
    // Set UART configuration including baud rate, parity, and stop bits
    let config = UartConfig::default()
        .with_baudrate(256000)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);

    // Initialize the UART instance for the radar module communication
    let uart0 = Uart::new(uart1, config).expect("Failed to initialize UART0");

    // Associate the UART with its designated RX and TX GPIO pins
    let uart0 = uart0.with_rx(rx_pin).with_tx(tx_pin).into_async();

    // Construct the radar driver with the configured UART and a delay provider
    LD2410Async::new(uart0, Delay)
}
