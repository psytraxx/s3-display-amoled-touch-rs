use defmt::info;
use drivers::ld2410::LD2410;
use embassy_time::{Delay, Timer};
use esp_hal::uart::Uart;

#[embassy_executor::task()]
pub async fn radar_task(mut radar: LD2410<Uart<'static, esp_hal::Blocking>, Delay>) {
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
