use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Timer;
use log::info;

use crate::RadarSensor;
use drivers::ld2410::RadarData;

pub type RadarDataChannelType = Channel<CriticalSectionRawMutex, RadarData, 2>;

pub static RADAR_DATA: RadarDataChannelType = Channel::new();

#[embassy_executor::task()]
pub async fn radar_task(mut radar: RadarSensor) {
    /*
    let version = radar
        .request_factory_reset()
        .await
        .expect("Failed to request factory reset");
    info!("Factory reset: {:?}", version); */

    let version = radar
        .get_firmware_version()
        .await
        .expect("Failed to request radar restart");

    if let Some(v) = version {
        info!("Firmware version: {v}");
    }

    let config = radar
        .get_configuration()
        .await
        .expect("Failed to request current configuration");

    if let Some(c) = config {
        info!("Current configuration: {c}");
    }
    loop {
        if let Ok(Some(event)) = radar.get_radar_data().await {
            // Send radar data to UI (non-blocking)
            let _ = RADAR_DATA.try_send(event);
        }
        Timer::after_millis(10).await
    }
}
