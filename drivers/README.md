## Drivers Overview

This project includes several driver modules that provide support for essential hardware components in embedded systems.

### Power Management Unit (BQ25896)

- Single-cell Li-Ion/Li-polymer battery charger with high efficiency.
- I2C communication for flexible charging control.
- Configurable charging current, voltage limits, and safety features.
- Battery temperature monitoring and multiple charging modes:
  - Pre-charge
  - Constant current
  - Constant voltage
- Supports advanced features such as HIZ mode, ADC enabling, watchdog timer configuration, and boost mode settings.

### 24GHz Radar Sensor (HLK-LD2410)

This module handles UART communication with the Hi-Link LD2410 radar sensor. It provides:

- Command-based operation to enter and exit configuration mode.
- Robust frame parsing with dedicated headers and tails.
- Decoding of radar data into structures such as `RadarData` and `RadarConfiguration`.
- Error handling via a comprehensive `LD2410Error` enum.
- Configurable polling and timeout settings through a `PollingConfig` struct.
- Facilities for firmware version queries, sensor restart, factory reset, and mode transitions.

### Touch Sensor (CST816S)

The CST816S driver supports capacitive touch functionality over I2C. Key features include:

- Gesture recognition (swipe up/down/left/right, single click, double click, and long press).
- Detection and interpretation of raw touch events and coordinates.
- Auto sleep mode management and adjustable long press duration.
- Configurable interrupt control for various touch and motion events.
- Customizable motion mask options to enable or disable specific gesture interactions.

### Usage Example

```rust
// Example of initializing and using the drivers in an embedded application.

use drivers::{BQ25896, LD2410, CST816S};

// --- Power Management Unit (BQ25896) ---
let mut pmu = BQ25896::new(i2c_bus, 0x6B)?;
pmu.set_input_current_limit(1000)?;
let current_limit = pmu.get_input_current_limit()?;
defmt::println!("Input current limit: {} mA", current_limit);

// --- 24GHz Radar Sensor (HLK-LD2410) ---
let mut radar = LD2410::new(uart, delay);
// Request firmware version from the radar sensor.
match radar.request_firmware_version() {
    Ok(Some(firmware)) => defmt::println!("Radar firmware version: {}", firmware),
    Ok(None) => defmt::warn!("No radar firmware info received."),
    Err(e) => defmt::println!("Radar error: {:?}", e),
}

// --- Touch Sensor (CST816S) ---
let mut touch_sensor = CST816S::new(i2c_bus, touch_int_pin);
if let Ok(Some(touch_data)) = touch_sensor.read_touch(true) {
    defmt::println!("Touch event: {:?}", touch_data);
} else {
    defmt::debug!("No touch detected at this time.");
}
```

### Generate Documentation

To build and view the documentation for all drivers:

    cargo doc -p drivers --no-deps --open
