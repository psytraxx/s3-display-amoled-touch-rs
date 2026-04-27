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

#[cfg(test)]
mod tests {
    use crate::bq25896::{BusStatus, ChargeStatus};
    use alloc::format;
    use num_enum::TryFromPrimitive;

    #[test]
    fn test_bus_status_display() {
        assert_eq!(format!("{}", BusStatus::NoInput), "No input");
        assert_eq!(format!("{}", BusStatus::UsbSdp), "USB Host SDP");
        assert_eq!(format!("{}", BusStatus::Adapter), "Adapter");
        assert_eq!(format!("{}", BusStatus::Otg), "OTG");
    }

    #[test]
    fn test_charge_status_display() {
        assert_eq!(format!("{}", ChargeStatus::NoCharge), "Not charging");
        assert_eq!(format!("{}", ChargeStatus::PreCharge), "Pre-charge");
        assert_eq!(format!("{}", ChargeStatus::FastCharge), "Fast charging");
        assert_eq!(format!("{}", ChargeStatus::Done), "Charge Termination Done");
        assert_eq!(format!("{}", ChargeStatus::Unknown), "Unknown");
    }

    #[test]
    fn test_bus_status_try_from() {
        assert_eq!(
            BusStatus::try_from_primitive(0u8).unwrap(),
            BusStatus::NoInput
        );
        assert_eq!(
            BusStatus::try_from_primitive(1u8).unwrap(),
            BusStatus::UsbSdp
        );
        assert_eq!(
            BusStatus::try_from_primitive(2u8).unwrap(),
            BusStatus::Adapter
        );
        assert_eq!(BusStatus::try_from_primitive(3u8).unwrap(), BusStatus::Otg);
        assert!(BusStatus::try_from_primitive(4u8).is_err());
    }

    #[test]
    fn test_charge_status_try_from() {
        assert_eq!(
            ChargeStatus::try_from_primitive(0u8).unwrap(),
            ChargeStatus::NoCharge
        );
        assert_eq!(
            ChargeStatus::try_from_primitive(1u8).unwrap(),
            ChargeStatus::PreCharge
        );
        assert_eq!(
            ChargeStatus::try_from_primitive(2u8).unwrap(),
            ChargeStatus::FastCharge
        );
        assert_eq!(
            ChargeStatus::try_from_primitive(3u8).unwrap(),
            ChargeStatus::Done
        );
        assert_eq!(
            ChargeStatus::try_from_primitive(4u8).unwrap(),
            ChargeStatus::Unknown
        );
    }
}
