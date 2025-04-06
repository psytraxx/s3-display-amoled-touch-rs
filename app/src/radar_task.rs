use embassy_time::Timer;
use esp_println::println;

use crate::RadarSensor;

#[embassy_executor::task()]
pub async fn radar_task(mut radar: RadarSensor) {
    /*
    let version = radar
        .request_factory_reset()
        .expect("Failed to request factory reset");
    println!("Factory reset: {:?}", version); */

    let version = radar
        .get_firmware_version()
        .expect("Failed to request radar restart");

    if let Some(v) = version {
        println!("Firmware version: {}", v);
    }

    let config = radar
        .get_configuration()
        .expect("Failed to request current configuration");

    if let Some(c) = config {
        println!("Current configuration: {}", c);
    }
    loop {
        if let Ok(Some(event)) = radar.get_radar_data() {
            println!("Radar data: {:?}", event);
        }
        Timer::after_millis(100).await
    }
}
