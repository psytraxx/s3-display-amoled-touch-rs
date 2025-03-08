## Power Management Unit (BQ25896)

- High efficiency single-cell Li-Ion/Li-polymer battery charger
- Input voltage range: 3.9V to 14V
- Charging current up to 3A
- I2C programmable
- Battery temperature monitoring
- Multiple charging modes:
  - Pre-charge
  - Constant current
  - Constant voltage

## HLK-LD2410 24Ghz Radar Human Presence Sensor

This Rust driver module provides support for the Hi-Link LD2410 24GHz FMCW radar sensor. It abstracts the low-level UART
communication with the sensor and implements the command protocol used for configuration and data retrieval.

### Features

- **Command-based Interaction:**  
  The driver supports standard commands to enter and exit configuration mode, request firmware version, restart the sensor, perform a factory reset, and query the current sensor configuration.

- **Frame Parsing:**  
  Data and command frames are parsed based on defined headers and tails. The driver validates the frames and decodes them into high-level structures like `RadarData` and `RadarConfiguration`.

- **Error Handling:**  
  Robust error handling is provided via the `LD2410Error` enum, ensuring that unexpected data or protocol mismatches are caught.

- **Configurable Polling:**  
  The driver allows adjustment of timeout values and polling delays through a `PollingConfig` struct.

### Usage Example

```rust
// Create an instance of the driver with your UART and delay implementations.
let mut radar = LD2410::new(uart, delay);

// Request firmware version.
match radar.request_firmware_version() {
    Ok(Some(firmware)) => defmt::info!("Firmware version: {}", firmware),
    Ok(None) => defmt::warn!("No firmware information received."),
    Err(e) => defmt::error!("Failed to retrieve firmware version: {:?}", e),
}
```

### Generate documentation

    cargo doc -p drivers --no-deps  --open
