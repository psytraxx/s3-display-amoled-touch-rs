use defmt::Format;
use embedded_hal::i2c::I2c;
use libm::log;

/// Handles all operations on/with Mpu6050
pub struct BQ25896<I2C> {
    i2c: I2C,
    adr: u8,
}

impl<I2C> BQ25896<I2C>
where
    I2C: I2c,
{
    pub fn new(i2c: I2C, adr: u8) -> Result<Self, PmuSensorError> {
        let mut instance = Self { i2c, adr };
        instance.detect_pmu(adr)?;
        Ok(instance)
    }

    fn detect_pmu(&mut self, adr: u8) -> Result<(), PmuSensorError> {
        if self.i2c.write(adr, &[]).is_ok() {
            Ok(())
        } else {
            Err(PmuSensorError::InitError)
        }
    }

    fn read_register(&mut self, reg: &[u8]) -> Result<u8, PmuSensorError> {
        let mut buffer = [0u8];

        self.i2c
            .write_read(self.adr, reg, &mut buffer)
            .map_err(|_| PmuSensorError::ReadRegisterError)?;
        Ok(buffer[0])
    }

    pub fn get_chip_id(&mut self) -> Result<u8, PmuSensorError> {
        let result = self.read_register(&[])?;
        Ok(result & 0x03)
    }

    pub fn get_charge_status(&mut self) -> Result<ChargeStatus, PmuSensorError> {
        const POWERS_PPM_REG_0BH: u8 = 0x0B;
        let val = self.read_register(&[POWERS_PPM_REG_0BH])?;
        let result = (val >> 3) & 0x03;
        Ok(result.into())
    }

    pub fn get_bus_status(&mut self) -> Result<BusStatus, PmuSensorError> {
        const POWERS_PPM_REG_0BH: u8 = 0x0B;
        let val = self.read_register(&[POWERS_PPM_REG_0BH])?;
        let result = (val >> 5) & 0x07;
        Ok(result.into())
    }

    pub fn get_battery_voltage(&mut self) -> Result<f32, PmuSensorError> {
        const POWERS_PPM_REG_0EH: u8 = 0x0E;

        let data = self.read_register(&[POWERS_PPM_REG_0EH])?;
        let vbat = (data & 0x7F) as f32 * 0.02 + 2.304;
        Ok(vbat)
    }

    pub fn get_vbus_voltage(&mut self) -> Result<f32, PmuSensorError> {
        const POWERS_PPM_REG_11H: u8 = 0x11;
        let data = self.read_register(&[POWERS_PPM_REG_11H])?;
        let vbus_attached = (data >> 7) & 0x01 == 1;
        let vbus = 2.6 + (data & 0x7F) as f32 * 0.1;
        if vbus_attached {
            Ok(vbus)
        } else {
            Ok(0.0)
        }
    }

    fn r_to_temp(&self, r: f64) -> f64 {
        let temperature = r / 10000.0;
        let temperature = log(temperature) / 3950.0 + 1.0 / 298.15;
        1.0 / temperature - 273.15
    }

    pub fn get_temperature(&mut self) -> Result<f64, PmuSensorError> {
        const POWERS_PPM_REG_10H: u8 = 0x10;
        let tspct = self.read_register(&[POWERS_PPM_REG_10H])? as f64 * 0.465 + 21.0;
        let vts = 5.0 * tspct / 100.0;
        let rp = (vts * 5230.0) / (5.0 - vts);
        let ntc = (rp * 30100.0) / (30100.0 - rp);
        Ok(self.r_to_temp(ntc))
    }
}

/// A clock error
#[derive(Debug, Format)]
pub enum PmuSensorError {
    InitError,
    ReadRegisterError,
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum BusStatus {
    NoInput,
    UsbSdp,
    Adapter,
    Otg,
    Unknown,
}

impl From<u8> for BusStatus {
    fn from(val: u8) -> Self {
        match val {
            0 => BusStatus::NoInput,
            1 => BusStatus::UsbSdp,
            2 => BusStatus::Adapter,
            3 => BusStatus::Otg,
            _ => BusStatus::Unknown,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum ChargeStatus {
    NoCharge,
    PreCharge,
    FastCharge,
    Done,
    Unknown,
}

impl From<u8> for ChargeStatus {
    fn from(val: u8) -> Self {
        match val {
            0 => ChargeStatus::NoCharge,
            1 => ChargeStatus::PreCharge,
            2 => ChargeStatus::FastCharge,
            3 => ChargeStatus::Done,
            _ => ChargeStatus::Unknown,
        }
    }
}
