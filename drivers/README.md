# LILYGO T-Display-S3 AMOLED Drivers

This repository contains Rust drivers for the LILYGO T-Display-S3 AMOLED development board components.

## Display Driver (RM67162)

The display uses a RM67162 AMOLED controller with the following specifications:

- Resolution: 240x536 pixels
- Color Format: RGB565 (16-bit)
- Interface: SPI with DMA support
- Notable features:
  - Brightness control (0-255)
  - Sleep mode support
  - 16-bit color depth

## Power Management Unit (BQ25896)

The board uses a BQ25896 power management IC with these capabilities:

- High efficiency single-cell Li-Ion/Li-polymer battery charger
- Input voltage range: 3.9V to 14V
- Charging current up to 3A
- I2C programmable
- Battery temperature monitoring
- Multiple charging modes:
  - Pre-charge
  - Constant current
  - Constant voltage

## Touch Controller (CST816S)

The CST816S capacitive touch controller features:

- Single touch detection
- Gesture support
- I2C interface

## References

- [LILYGO AMOLED Series Documentation](https://github.com/Xinyuan-LilyGO/LilyGo-AMOLED-Series)
- [Product Page](https://www.lilygo.cc/products/t-display-s3-amoled)
