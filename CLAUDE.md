# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an embedded Rust project for the LilyGo T-Display-S3 AMOLED Plus development board. It targets the ESP32-S3 microcontroller (xtensa architecture) and integrates display, touch, radar sensor, and power management capabilities with a GUI built using Slint.

**Target Platform**: ESP32-S3 (xtensa-esp32s3-none-elf)
**Toolchain**: ESP Rust toolchain (channel = "esp")
**Main Dependencies**: esp-hal, esp-rtos, embassy, Slint UI framework

## Build Commands

Build and flash require sourcing ESP toolchain environment first (`source ~/export-esp.sh`).

### Build
```bash
# Release build (default)
./scripts/build.sh
# or
cargo build --release

# Debug build
./scripts/build.sh debug
# or
cargo build
```

### Flash to Device
```bash
# Flash release build (default)
./scripts/flash.sh

# Flash debug build
./scripts/flash.sh debug
```

The flash script uses `cargo run` which automatically builds and flashes via espflash (configured in `.cargo/config.toml`).

### Documentation
```bash
# Generate and open app documentation
cargo doc -p app --no-deps --open

# Generate and open drivers documentation
cargo doc -p drivers --no-deps --open
```

### Testing
```bash
# Run tests (drivers module has unit tests)
cargo test -p drivers
```

## Project Architecture

### Workspace Structure

This is a Cargo workspace with three members:

1. **`app/`** - Main application binary for ESP32-S3
2. **`drivers/`** - Hardware driver library (BQ25896 PMU, CST816S touch, LD2410 radar)
3. **`slint-generated/`** - Slint UI compilation package

### Core Application Architecture

The application uses an **async task-based architecture** with Embassy executor:

- **Main loop** (`main.rs`): Initializes hardware and spawns async tasks
- **Controller** (`controller.rs`): Central event loop that processes user actions and sensor data using Embassy channels
- **Render Task** (`render_task.rs`): Dedicated task for Slint UI rendering and touch event processing
- **Radar Task** (`radar_task.rs`): Background task for reading LD2410 radar sensor data

### Communication Pattern

The app uses **Embassy channels** for inter-task communication:

- `ACTION` channel (in `controller.rs`): UI callbacks send actions to controller
- `RADAR_DATA` channel (in `radar_task.rs`): Radar task sends data to controller
- Controller's main loop uses `embassy_futures::select` to handle events from both channels

### Hardware Initialization

Hardware modules are organized in `app/src/hardware/`:

- **`display.rs`**: RM67162 AMOLED display via SPI with DMA (536x240 resolution)
- **`touch.rs`**: CST816x capacitive touch controller via I2C
- **`pmu.rs`**: BQ25896 battery charger/PMU via I2C
- **`radar.rs`**: LD2410 24GHz radar sensor via UART

All I2C devices share a single bus using `embassy_embedded_hal::shared_bus` with mutex protection.

### Display and UI

- **UI Framework**: Slint with software renderer (RGB565)
- **UI Files**: Located in `slint-generated/ui/` (`.slint` files)
- **Build Process**: Slint files are compiled via `build.rs` in slint-generated package
- **Rendering**: Line-by-line rendering using `DisplayLineBuffer` to minimize memory usage
- **Touch Mapping**: Touch coordinates are transformed and inverted to match display orientation (see `render_task.rs`)

### Memory Configuration

- **Heap**: Custom DRAM allocation (73744 bytes in `.dram2_uninit` section)
- **PSRAM**: Octal PSRAM enabled for additional memory (configured via `ESP_HAL_CONFIG_PSRAM_MODE`)
- **Profile**: Even debug builds use optimization level "s" for size (ESP32 debug is too slow)

## Important Implementation Details

### Async/Sync Boundary

UI callbacks in Slint are synchronous, but the controller runs async. The `send_action()` function uses **non-blocking `try_send()`** to bridge this gap. If the channel is full, the action is dropped with an error log.

### Touch Event Handling

Touch events require careful state management:
- Down events clear any previous unreleased touch state before registering new touch
- Up events use last tracked position for reliable release when finger goes off-screen
- Coordinates are clamped and transformed: `x = DISPLAY_WIDTH - point.x`, `y = DISPLAY_HEIGHT - point.y`

### Driver Features

The `drivers` crate provides both sync and async APIs:
- Use `async` feature flag to enable async methods
- Both blocking (`BlockingRegisterDevice`) and async (`AsyncRegisterDevice`) register access patterns

### Linker Configuration

Custom linker setup in `app/build.rs` provides helpful error messages for common issues (missing defmt, scheduler problems, etc.). The `-Tlinkall.x` linker script must be last.

## Development Environment Setup

1. Install ESP Rust toolchain and espflash
2. Ensure `~/export-esp.sh` exists and exports ESP toolchain environment
3. Use nightly Rust with `static_cell` nightly features

## Hardware Notes

- Board detection via I2C (address 0x15 for presence, 0x51 for SPI vs QSPI variant)
- GPIO38 (PMICEN) must be set high to enable the power management IC
- SPI driver is placed in RAM for performance (`ESP_HAL_PLACE_SPI_DRIVER_IN_RAM`)
