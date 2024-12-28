# S3 Display Touch SPI RS

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
- SPI DMA display interface
- Touch gesture detection
- Battery monitoring

## Dependencies

- esp-hal
- embedded-graphics
- embedded-text
- mipidsi
- embassy-executor
- defmt for logging

## Building

1. Build the project:
```bash
cargo build --release
```

2. Flash to device:
```bash
cargo espflash flash --release --monitor
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



