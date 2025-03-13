use alloc::string::String;
use defmt::{error, info};
use drivers::bq25896::{PmuSensorError, BQ25896};
use embedded_hal::i2c::I2c;
use embedded_hal_bus::{i2c::AtomicDevice, util::AtomicCell};
use esp_hal::gpio::{GpioPin, Level, Output, OutputConfig};

use crate::controller::Pmu;

/// I2C address of BQ25896 PMU
const BQ25896_SLAVE_ADDRESS: u8 = 0x6B;
/// Charging target voltage in mV for BQ25896
const PMU_CHARGE_TARGET_VOLTAGE: u16 = 4208;
/// Precharge current in mA for BQ25896
const PMU_PRECHARGE_CURRENT: u16 = 128;
/// Constant charging current in mA for BQ25896
const PMU_CONSTANT_CHARGE_CURRENT: u16 = 1536;

pub struct PmuImpl<'a, BUS>
where
    BUS: I2c,
{
    pmu: BQ25896<AtomicDevice<'a, BUS>>,
}

impl<BUS> PmuImpl<'_, BUS>
where
    BUS: I2c,
{
    pub fn new(i2c_ref_cell: &'static AtomicCell<BUS>, power_pin: GpioPin<38>) -> Self {
        // Initialize PMICEN pin to enable power management IC
        let mut pmicen = Output::new(power_pin, Level::Low, OutputConfig::default());
        pmicen.set_high();
        info!("PMICEN set high");

        // Detect board model
        Self::detect_spi_model(i2c_ref_cell);

        let mut pmu = Self::init_pmu(i2c_ref_cell).expect("PMU initialization failed");

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

    fn detect_spi_model(i2c_ref_cell: &'static AtomicCell<BUS>) {
        let mut i2c = AtomicDevice::new(i2c_ref_cell);

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

    fn init_pmu(
        i2c_ref_cell: &'static AtomicCell<BUS>,
    ) -> Result<BQ25896<AtomicDevice<'static, BUS>>, PmuSensorError> {
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
}

impl<BUS> Pmu for PmuImpl<'_, BUS>
where
    BUS: I2c,
{
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
