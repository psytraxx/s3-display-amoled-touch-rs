# T-Display-S3 AMOLED Plus

This repository contains the Rust example application for the LilyGo T-Display-S3 AMOLED Plus development board. It integrates the display driver and system-level functionality to showcase how to build and deploy applications using the ESP32-S3 microcontroller with the onboard AMOLED display, touch controller, and power management.

## Overview

The project demonstrates:

- High-performance display control using the RM67162 AMOLED controller.
- Touch input handling via the CST816S touch controller.
- Power management monitoring with the BQ25896 PMU.
- Application-level UI components built with Slint for a seamless embedded GUI experience.

For details on the low-level driver implementations, please refer to the corresponding driver documentation.

## Building and Flashing

1. Build the project:

```bash
./scripts/build.sh
```

2. Flash the firmware to your device:

```bash
./scripts/flash.sh
```

## Generating Documentation

Generate the project documentation with the following command:

```bash
cargo doc -p app --no-deps --open
```

## Contributing

Contributions are welcome! Please open an issue beforehand for major enhancements or changes.

## License

MIT License – see [LICENSE](./LICENSE) for details.
