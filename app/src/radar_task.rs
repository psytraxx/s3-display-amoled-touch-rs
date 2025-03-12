use defmt::info;
use drivers::ld2410::LD2410;
use embassy_time::{Delay, Timer};
use esp_hal::{
    gpio::GpioPin,
    peripherals::UART0,
    uart::{Config, Uart},
};

#[embassy_executor::task()]
pub async fn render_task(rx_pin: GpioPin<44>, tx_pin: GpioPin<43>, uart0: UART0) {
    let config = Config::default()
        .with_baudrate(256000)
        .with_parity(esp_hal::uart::Parity::None)
        .with_stop_bits(esp_hal::uart::StopBits::_1);

    let uart0 = Uart::new(uart0, config).expect("Failed to initialize UART0");

    let uart0 = uart0.with_rx(rx_pin).with_tx(tx_pin);

    // Create the LD2410 radar instance
    let mut radar = LD2410::new(uart0, Delay);

    /*
    let version = radar
        .request_factory_reset()
        .expect("Failed to request factory reset");
    info!("Factory reset: {:?}", version); */

    let version = radar
        .get_firmware_version()
        .expect("Failed to request radar restart");

    if let Some(v) = version {
        info!("Firmware version: {}", v);
    }

    let config = radar
        .get_configuration()
        .expect("Failed to request current configuration");

    if let Some(c) = config {
        info!("Current configuration: {}", c);
    }
    loop {
        if let Ok(Some(event)) = radar.get_radar_data() {
            info!("Radar data: {:?}", event);
        }
        Timer::after_millis(100).await
    }
}
