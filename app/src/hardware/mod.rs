//! Hardware initialization modules
//!
//! This module provides organized initialization functions for all hardware
//! components used in the ESP32-S3 touch display application:
//!
//! - **Display**: RM67162 display controller via SPI with DMA
//! - **Touchpad**: CST816x capacitive touch controller via I2C
//! - **Radar**: LD2410 human presence sensor via UART
//! - **PMU**: BQ25896 battery charger via I2C

pub mod display;
pub mod pmu;
pub mod radar;
pub mod touch;

// Re-export commonly used types and functions for convenience
pub use display::{initialize_display, TouchDisplay, DISPLAY_HEIGHT, DISPLAY_WIDTH};
pub use pmu::{initialize_pmu, Charger};
pub use radar::{initialize_radar, RadarSensor};
pub use touch::{initialize_touchpad, Touchpad};
