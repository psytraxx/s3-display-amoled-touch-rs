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

    match radar.get_firmware_version() {
        Ok(Some(v)) => println!("Firmware version: {}", v),
        Ok(None) => println!("Could not retrieve firmware version (sensor returned None)."),
        Err(e) => println!("Failed to get firmware version: {:?}", e),
    }

    match radar.get_configuration() {
        Ok(Some(c)) => println!("Current configuration: {}", c),
        Ok(None) => println!("Could not retrieve configuration (sensor returned None)."),
        Err(e) => println!("Failed to get configuration: {:?}", e),
    }

    loop {
        match radar.get_radar_data() {
            Ok(event) => {
                println!("Radar data: {:?}", event);
            }
            Err(e) => {
                // Log other specific errors
                println!("Radar error: {:?}", e);
            }
        }
        Timer::after_millis(100).await;
    }
}
