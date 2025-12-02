#![no_std]
//! BQ25896 Battery Charging and Power Path Management Library
//!
//! This library provides an interface to the BQ25896 IC for battery charging
//! and power path management. It includes functions for configuring and
//! monitoring the charging process.

use embedded_hal::i2c::I2c;

/// BQ25896 battery charging and power path management IC driver.
pub mod bq25896;

/// CST816S capacitive touch sensor driver.
pub mod cst816x;

/// LD2410 radar sensor driver.
pub mod ld2410;

extern crate alloc;

#[derive(Debug)]
struct BlockingRegisterDevice<I2C> {
    i2c: I2C,
    adr: u8,
}

impl<I2C> BlockingRegisterDevice<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C, adr: u8) -> Self {
        Self { i2c, adr }
    }

    pub fn read_register_buffer(
        &mut self,
        register: u8,
        buffer: &mut [u8],
    ) -> Result<(), I2C::Error> {
        self.i2c.write_read(self.adr, &[register], buffer)?;
        Ok(())
    }

    pub fn read_register(&mut self, register: u8) -> Result<u8, I2C::Error> {
        let mut buffer = [0];
        self.i2c.write_read(self.adr, &[register], &mut buffer)?;
        Ok(buffer[0])
    }

    pub fn write_register(&mut self, register_and_data: &[u8]) -> Result<(), I2C::Error> {
        self.i2c.write(self.adr, register_and_data)
    }

    pub fn set_register_bit(&mut self, register: u8, bit: u8) -> Result<(), I2C::Error> {
        let val = self.read_register(register)?;
        let data = val | (1 << bit);
        self.write_register(&[register, data])
    }

    pub fn get_register_bit(&mut self, register: u8, bit: u8) -> Result<bool, I2C::Error> {
        let val = self.read_register(register)?;
        Ok((val & (1 << bit)) != 0)
    }

    pub fn clear_register_bit(&mut self, register: u8, bit: u8) -> Result<(), I2C::Error> {
        let val = self.read_register(register)?;
        let data = val & !(1 << bit);
        self.write_register(&[register, data])
    }
}

#[derive(Debug)]
struct AsyncRegisterDevice<I2C> {
    i2c: I2C,
    adr: u8,
}

impl<I2C> AsyncRegisterDevice<I2C>
where
    I2C: embedded_hal_async::i2c::I2c,
{
    pub fn new(i2c: I2C, adr: u8) -> Self {
        Self { i2c, adr }
    }

    pub async fn read_register_buffer(
        &mut self,
        register: u8,
        buffer: &mut [u8],
    ) -> Result<(), I2C::Error> {
        self.i2c.write_read(self.adr, &[register], buffer).await?;
        Ok(())
    }

    pub async fn read_register(&mut self, register: u8) -> Result<u8, I2C::Error> {
        let mut buffer = [0];
        self.i2c
            .write_read(self.adr, &[register], &mut buffer)
            .await?;
        Ok(buffer[0])
    }

    pub async fn write_register(&mut self, register_and_data: &[u8]) -> Result<(), I2C::Error> {
        self.i2c.write(self.adr, register_and_data).await
    }

    pub async fn set_register_bit(&mut self, register: u8, bit: u8) -> Result<(), I2C::Error> {
        let val = self.read_register(register).await?;
        let data = val | (1 << bit);
        self.write_register(&[register, data]).await
    }

    pub async fn get_register_bit(&mut self, register: u8, bit: u8) -> Result<bool, I2C::Error> {
        let val = self.read_register(register).await?;
        Ok((val & (1 << bit)) != 0)
    }

    pub async fn clear_register_bit(&mut self, register: u8, bit: u8) -> Result<(), I2C::Error> {
        let val = self.read_register(register).await?;
        let data = val & !(1 << bit);
        self.write_register(&[register, data]).await
    }
}

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
