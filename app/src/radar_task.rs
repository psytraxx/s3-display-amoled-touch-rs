use embassy_time::Timer;
use log::info;

use crate::RadarSensor;

#[embassy_executor::task()]
pub async fn radar_task(mut radar: RadarSensor) {
    /*
    let version = radar
        .request_factory_reset()
        .expect("Failed to request factory reset");
    info!("Factory reset: {:?}", version); */

    let version = radar
        .get_firmware_version()
        .expect("Failed to request radar restart");

    if let Some(v) = version {
        info!("Firmware version: {v}");
    }

    let config = radar
        .get_configuration()
        .expect("Failed to request current configuration");

    if let Some(c) = config {
        info!("Current configuration: {c}");
    }
    loop {
        if let Ok(Some(event)) = radar.get_radar_data() {
            info!("Radar data: {event:?}");
        }
        Timer::after_millis(200).await
    }
}
