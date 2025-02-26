#![no_std]
//! BQ25896 Battery Charging and Power Path Management Library
//!
//! This library provides an interface to the BQ25896 IC for battery charging
//! and power path management. It includes functions for configuring and
//! monitoring the charging process.

/// BQ25896 battery charging and power path management IC driver.
pub mod bq25896;

/// CST816S capacitive touch sensor driver.
pub mod cst816s;

extern crate alloc;

#[cfg(test)]
mod tests {
    use crate::bq25896::{BusStatus, ChargeStatus};
    use alloc::format;
    #[test]
    fn test_bus_status_display() {
        assert_eq!(format!("{}", BusStatus::NoInput), "No input");
        assert_eq!(format!("{}", BusStatus::UsbSdp), "USB Host SDP");
        assert_eq!(format!("{}", BusStatus::Adapter), "Adapter");
        assert_eq!(format!("{}", BusStatus::Otg), "OTG");
        assert_eq!(format!("{}", BusStatus::Unknown), "Unknown");
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
    fn test_bus_status_from() {
        assert_eq!(BusStatus::from(0), BusStatus::NoInput);
        assert_eq!(BusStatus::from(1), BusStatus::UsbSdp);
        assert_eq!(BusStatus::from(2), BusStatus::Adapter);
        assert_eq!(BusStatus::from(3), BusStatus::Otg);
        assert_eq!(BusStatus::from(4), BusStatus::Unknown);
    }

    #[test]
    fn test_charge_status_from() {
        assert_eq!(ChargeStatus::from(0), ChargeStatus::NoCharge);
        assert_eq!(ChargeStatus::from(1), ChargeStatus::PreCharge);
        assert_eq!(ChargeStatus::from(2), ChargeStatus::FastCharge);
        assert_eq!(ChargeStatus::from(3), ChargeStatus::Done);
        assert_eq!(ChargeStatus::from(4), ChargeStatus::Unknown);
    }
}
