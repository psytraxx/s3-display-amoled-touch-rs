# S3 Display Touch SPI RS - Rust Example for LilyGo T-Display-S3 AMOLED Plus

Rust example for the LilyGo T-Display-S3 AMOLED Plus development board.

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

- [esp-hal](https://crates.io/crates/esp-hal)
- [embedded-graphics](https://crates.io/crates/embedded-graphics)
- [embedded-text](https://crates.io/crates/embedded-text)
- [mipidsi](https://crates.io/crates/mipidsi)
- [embassy-executor](https://crates.io/crates/embassy-executor)
- [defmt](https://crates.io/crates/defmt)
- [defmt-rtt](https://crates.io/crates/defmt-rtt)
- [slint](https://crates.io/crates/slint)
- [cst816s](https://crates.io/crates/cst816s)
- [esp-hal-embassy](https://crates.io/crates/esp-hal-embassy)

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
- Touch points are displayed in green

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

## License

MIT License - see LICENSE file for details
