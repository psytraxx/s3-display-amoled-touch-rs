# T-Display-S3 AMOLED Plus

Rust example for the LilyGo T-Display-S3 AMOLED Plus development board.


![LilyGo T-Display-S3 AMOLED Plus](https://lilygo.cc/cdn/shop/files/LILYGO-NEW-BOARDS.png)

## Hardware

- [LilyGo T-Display-S3 AMOLED Plus](https://lilygo.cc/products/t-display-s3-amoled-plus)
- ESP32-S3 microcontroller
- 1.91" AMOLED display (RM67162 controller)
- CST816S touch controller
- BQ25896 PMU (Power Management Unit)

## Features

- Display driver for RM67162 AMOLED controller
- Touch input support via CST816S
- Power management via BQ25896
- SPI display interface
- Touch gesture detection
- Battery monitoring

## Dependencies

- [esp-backtrace](https://crates.io/crates/esp-backtrace) - Provides backtrace support for ESP32-S3.
- [esp-hal](https://crates.io/crates/esp-hal) - Hardware abstraction layer for the ESP32-S3.
- [esp-alloc](https://crates.io/crates/esp-alloc) - Memory allocator for ESP32.
- [embedded-hal](https://crates.io/crates/embedded-hal) - Traits for embedded hardware abstractions.
- [embassy-embedded-hal](https://crates.io/crates/embassy-embedded-hal) - Async implementations of embedded-hal traits.
- [esp-hal-embassy](https://crates.io/crates/esp-hal-embassy) - Integration of esp-hal with Embassy.
- [embassy-executor](https://crates.io/crates/embassy-executor) - Task executor for async/await in embedded systems.
- [mipidsi](https://crates.io/crates/mipidsi) - Driver for MIPI DSI displays.
- [embedded-graphics-core](https://crates.io/crates/embedded-graphics-core) - Core traits for embedded-graphics.
- [display-interface](https://crates.io/crates/display-interface) - Abstraction for display communication interfaces.
- [display-interface-spi](https://crates.io/crates/display-interface-spi) - SPI implementation for display-interface.
- [embedded-hal-bus](https://crates.io/crates/embedded-hal-bus) - Bus abstractions for embedded-hal.
- [critical-section](https://crates.io/crates/critical-section) - Critical section management.
- [libm](https://crates.io/crates/libm) - Math library for embedded systems.
- [slint](https://crates.io/crates/slint) - GUI toolkit for embedded systems.
- [slint-build](https://crates.io/crates/slint-build) - Build script support for Slint.

## Building

1. Build the project:
```bash
./scripts/build.sh
```

2. Flash to device:
```bash
./scripts/flash.sh
```

## Usage

- The display will show PMU status information including:
  - Charge status
  - Bus status  
  - Battery voltage
  - Temperature
- Touch gestures supported:
  - Single tap
  - Double tap
  - Long press
  - Swipe (up/down/left/right)

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License

MIT License - see [LICENSE](./LICENSE) file for details
