# T-Display-S3 AMOLED Plus

Rust application for the LilyGo T-Display-S3 AMOLED Plus development board featuring the ESP32-S3 microcontroller with integrated AMOLED display, capacitive touch, 24GHz radar sensor, and power management.

## Features

- **Display**: RM67162 AMOLED controller (536×240, RGB565)
- **Touch**: CST816S capacitive touch with gesture support
- **Radar**: HLK-LD2410 24GHz human presence sensor
- **Power**: BQ25896 battery charger and PMU
- **UI**: Slint-based embedded GUI with async rendering

## Quick Start

```bash
# Build and flash to device
./scripts/build.sh
./scripts/flash.sh
```

## Development

For detailed build instructions, architecture documentation, and development guidelines, see [CLAUDE.md](./CLAUDE.md).

## Contributing

Contributions are welcome! Please open an issue beforehand for major enhancements or changes.

## License

MIT License – see [LICENSE](./LICENSE) for details.
