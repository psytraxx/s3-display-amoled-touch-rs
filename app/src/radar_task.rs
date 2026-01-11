use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::Timer;
use log::{debug, error, info, warn};

use crate::RadarSensor;
use drivers::ld2410::RadarData;

pub type RadarDataChannelType = Channel<CriticalSectionRawMutex, RadarData, 2>;

pub static RADAR_DATA: RadarDataChannelType = Channel::new();

#[embassy_executor::task()]
pub async fn radar_task(mut radar: RadarSensor) {
    use embassy_time::{with_timeout, Duration};

    info!("Radar task starting...");
    info!("Waiting 100ms for radar sensor to power up...");
    Timer::after_millis(100).await;

    // Try factory reset with timeout
    info!("Requesting factory reset...");
    match with_timeout(Duration::from_secs(2), radar.request_factory_reset()).await {
        Ok(Ok(success)) => {
            info!("Factory reset complete: {:?}", success);
        }
        Ok(Err(e)) => {
            error!("Factory reset failed: {:?}", e);
            error!("Radar sensor may not be connected or responding. Task will exit.");
            return;
        }
        Err(_) => {
            error!("Factory reset timed out after 2 seconds");
            error!("Radar sensor is not responding. Check connections:");
            error!("  - Verify radar is connected to UART0");
            error!("  - Check GPIO43 (TX) and GPIO44 (RX) wiring");
            error!("  - Verify baud rate is 256000");
            error!("  - Check if radar sensor needs external power");
            error!("Task will exit.");
            return;
        }
    }

    // Try getting firmware version with timeout
    info!("Requesting firmware version...");
    match with_timeout(Duration::from_secs(2), radar.get_firmware_version()).await {
        Ok(Ok(Some(v))) => {
            info!("Firmware version: {v}");
        }
        Ok(Ok(None)) => {
            warn!("No firmware version received");
        }
        Ok(Err(e)) => {
            error!("Failed to get firmware version: {:?}", e);
        }
        Err(_) => {
            error!("Firmware version request timed out");
        }
    }

    // Try getting configuration with timeout
    info!("Requesting current configuration...");
    match with_timeout(Duration::from_secs(2), radar.get_configuration()).await {
        Ok(Ok(Some(c))) => {
            info!("Current configuration: {c}");
        }
        Ok(Ok(None)) => {
            warn!("No configuration received");
        }
        Ok(Err(e)) => {
            error!("Failed to get configuration: {:?}", e);
        }
        Err(_) => {
            error!("Configuration request timed out");
        }
    }

    info!("Starting radar data reading loop...");
    let mut read_count = 0u32;
    let mut error_count = 0u32;
    let mut none_count = 0u32;
    let mut success_count = 0u32;

    loop {
        read_count += 1;

        // Use timeout for data reads too
        match with_timeout(Duration::from_millis(100), radar.get_radar_data()).await {
            Ok(Ok(Some(event))) => {
                success_count += 1;
                debug!("Radar data received: {:?}", event);

                // Send radar data to UI (non-blocking)
                if let Err(e) = RADAR_DATA.try_send(event) {
                    warn!("Failed to send radar data to channel (full?): {:?}", e);
                }

                // Log periodic stats
                if success_count.is_multiple_of(100) {
                    info!(
                        "Radar stats - reads: {}, successes: {}, none: {}, errors: {}",
                        read_count, success_count, none_count, error_count
                    );
                }
            }
            Ok(Ok(None)) => {
                none_count += 1;
                if none_count % 50 == 1 {
                    warn!("Radar data frame decoded to None (count: {})", none_count);
                }
            }
            Ok(Err(e)) => {
                error_count += 1;
                error!("Radar error (count: {}): {:?}", error_count, e);

                // If errors are too frequent, log stats
                if error_count.is_multiple_of(10) {
                    error!(
                        "Radar error stats - reads: {}, successes: {}, none: {}, errors: {}",
                        read_count, success_count, none_count, error_count
                    );
                }
            }
            Err(_) => {
                error_count += 1;
                if error_count % 50 == 1 {
                    warn!(
                        "Radar data read timed out (count: {}/{})",
                        error_count, read_count
                    );
                }
            }
        }

        Timer::after_millis(10).await
    }
}
