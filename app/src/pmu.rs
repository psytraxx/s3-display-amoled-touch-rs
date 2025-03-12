use alloc::string::String;
use defmt::{error, info};
use drivers::bq25896::{PmuSensorError, BQ25896};
use embedded_hal::i2c::I2c as I2CBus;
use embedded_hal_bus::{i2c::AtomicDevice, util::AtomicCell};
use esp_hal::{i2c::master::I2c, Blocking};

use crate::{
    controller::Pmu, BQ25896_SLAVE_ADDRESS, PMU_CHARGE_TARGET_VOLTAGE, PMU_CONSTANT_CHARGE_CURRENT,
    PMU_PRECHARGE_CURRENT,
};

type PmuDevice = BQ25896<AtomicDevice<'static, I2c<'static, Blocking>>>;

pub struct PmuImpl {
    pmu: PmuDevice,
}

impl PmuImpl {
    pub fn new(i2c_ref_cell: &'static AtomicCell<I2c<'static, Blocking>>) -> Self {
        // Detect SPI model
        detect_spi_model(AtomicDevice::new(i2c_ref_cell));

        let mut pmu: BQ25896<AtomicDevice<'_, I2c<'_, Blocking>>> =
            init_pmu(i2c_ref_cell).expect("PMU initialization failed");

        let fast_charge_current_limit = pmu
            .get_fast_charge_current_limit()
            .expect("get_fast_charge_current_limit failed");

        info!("Fast charge current limit: {}", fast_charge_current_limit);

        let precharge_current = pmu
            .get_precharge_current()
            .expect("get_precharge_current failed");

        info!("Precharge current: {}", precharge_current);

        let charge_target_voltage = pmu
            .get_charge_target_voltage()
            .expect("get_charge_target_voltage failed");

        info!("Charge target voltage: {}", charge_target_voltage);

        info!("PMU chip id: {}", pmu.get_chip_id().unwrap());

        Self { pmu }
    }
}

impl Pmu for PmuImpl {
    fn get_pmu_info(&mut self) -> String {
        self.pmu.get_info().expect("get_info failed")
    }

    fn toggle_pmu_charger(&mut self, enable_charging: bool) -> bool {
        if enable_charging {
            self.pmu
                .set_charge_enabled()
                .expect("set_charge_enable failed");
        } else {
            self.pmu
                .set_charge_disabled()
                .expect("set_charge_disabled failed");
        }

        self.pmu
            .is_charge_enabled()
            .expect("is_charge_enabled failed")
    }
}

fn init_pmu(
    i2c_ref_cell: &'static AtomicCell<esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>>,
) -> Result<
    BQ25896<AtomicDevice<'static, esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>>>,
    PmuSensorError,
> {
    // Initialize BQ25896 charger
    let mut pmu = BQ25896::new(AtomicDevice::new(i2c_ref_cell), BQ25896_SLAVE_ADDRESS)?;

    // Set the charging target voltage, Range:3840 ~ 4608mV ,step:16 mV
    pmu.set_charge_target_voltage(PMU_CHARGE_TARGET_VOLTAGE)?;

    // Set the precharge current , Range: 64mA ~ 1024mA ,step:64mA
    pmu.set_precharge_current(PMU_PRECHARGE_CURRENT)?;

    // The premise is that Limit Pin is disabled, or it will only follow the maximum charging current set by Limit Pin.
    // Set the charging current , Range:0~5056mA ,step:64mA
    pmu.set_fast_charge_current_limit(PMU_CONSTANT_CHARGE_CURRENT)?;

    pmu.set_adc_enabled()?;

    Ok(pmu)
}

/// Detects the display board model by probing I2C addresses
/// Supports 1.91" SPI and QSPI variants
fn detect_spi_model<I2C>(mut i2c: I2C)
where
    I2C: I2CBus,
{
    // Try to find 1.91 inch i2c devices
    if i2c.write(0x15, &[]).is_ok() {
        // Check RTC Slave address
        if i2c.write(0x51, &[]).is_ok() {
            info!("Detect 1.91-inch SPI board model!");
        } else {
            info!("Detect 1.91-inch QSPI board model!");
        }
    } else {
        error!("Unable to detect 1.91-inch touch board model!");
    }
}
