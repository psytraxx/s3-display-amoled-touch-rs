#![no_std]
//! Drivers for LilyGo T-Display-S3 AMOLED Plus peripherals.
//!
//! Provides BQ25896 PMU, CST816x touch, and LD2410 radar sensor drivers.
//! Each driver exposes a single generic struct with blocking and async impl blocks.

/// BQ25896 battery charging and power path management IC driver.
pub mod bq25896;

/// CST816S capacitive touch sensor driver.
pub mod cst816x;

/// LD2410 radar sensor driver.
pub mod ld2410;

extern crate alloc;
