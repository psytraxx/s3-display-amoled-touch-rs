//! Power Management Unit (PMU) hardware initialization module
//!
//! This module handles the initialization and configuration of the BQ25896
//! battery charger IC via I2C interface.

use drivers::bq25896::asynch::BQ25896Async;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use esp_hal::i2c::master::I2c;
use esp_hal::Async;
use log::info;

/// BQ25896 I2C slave address
const BQ25896_SLAVE_ADDRESS: u8 = 0x6B;

/// Battery charge target voltage in millivolts (4.208V)
const PMU_CHARGE_TARGET_VOLTAGE: u16 = 4208;

/// Precharge current in milliamperes (128mA)
const PMU_PRECHARGE_CURRENT: u16 = 128;

/// Constant (fast) charge current in milliamperes (1536mA)
const PMU_CONSTANT_CHARGE_CURRENT: u16 = 1536;

/// Type alias for the BQ25896 PMU driver instance
pub type Charger = BQ25896Async<I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>>;

/// Initializes and configures the BQ25896 power management unit for battery charging.
///
/// This function:
/// - Initializes the BQ25896 driver via I2C
/// - Configures charging parameters (voltage, current limits)
/// - Enables ADC for power measurement
/// - Logs comprehensive configuration details
///
/// # Charging Configuration
///
/// - **Target voltage**: 4.208V (typical for Li-ion batteries)
/// - **Precharge current**: 128mA (for deeply discharged batteries)
/// - **Fast charge current**: 1536mA (main charging current)
///
/// # Arguments
///
/// * `i2c_device` - Acquired I2C device handle from the shared bus
///
/// # Returns
///
/// Returns the configured PMU instance.
///
/// # Panics
///
/// Panics if any initialization or configuration step fails.
pub async fn initialize_pmu(
    i2c_device: I2cDevice<'static, CriticalSectionRawMutex, I2c<'static, Async>>,
) -> Charger {
    // Create a new PMU instance on the I2C bus at the designated slave address
    let mut pmu = BQ25896Async::new(i2c_device, BQ25896_SLAVE_ADDRESS);
    pmu.init().await.expect("Failed to initialize BQ25896");

    // Set the battery charger target voltage
    pmu.set_charge_target_voltage(PMU_CHARGE_TARGET_VOLTAGE)
        .await
        .expect("set_charge_target_voltage failed");

    // Set the precharge current for battery charging
    pmu.set_precharge_current(PMU_PRECHARGE_CURRENT)
        .await
        .expect("set_precharge_current failed");

    // Set the fast (constant) charge current limit
    pmu.set_fast_charge_current_limit(PMU_CONSTANT_CHARGE_CURRENT)
        .await
        .expect("set_fast_charge_current_limit failed");

    // Enable ADC for power measurement in the PMU
    pmu.set_adc_enabled().await.expect("set_adc_enabled failed");

    info!(
        "Fast charge current limit: {}",
        pmu.get_fast_charge_current_limit()
            .await
            .expect("get_fast_charge_current_limit failed")
    );

    info!(
        "Precharge current: {}",
        pmu.get_precharge_current()
            .await
            .expect("get_precharge_current failed")
    );

    info!(
        "Charge target voltage: {}",
        pmu.get_charge_target_voltage()
            .await
            .expect("get_charge_target_voltage failed")
    );

    info!(
        "Boost frequency:  {}",
        pmu.get_boost_freq().await.expect("get_boost_freq failed")
    );

    info!(
        "Fast charge timer: {}",
        pmu.get_fast_charge_timer()
            .await
            .expect("get_fast_charge_timer failed")
    );

    info!(
        "Termination curr.: {}mA",
        pmu.get_termination_current()
            .await
            .expect("get_termination_current failed")
    );

    info!(
        "Power down voltage: {}mV",
        pmu.get_sys_power_down_voltage()
            .await
            .expect("get_sys_power_down_voltage failed")
    );

    info!(
        "Automatic input detection: {}",
        pmu.is_automatic_input_detection_enabled()
            .await
            .expect("is_automatic_input_detection_enabled failed")
    );

    info!(
        "HIZ mode: {}",
        pmu.is_hiz_mode().await.expect("is_hiz_mode failed")
    );

    info!(
        "Charging safety timer: {}",
        pmu.is_charging_safety_timer_enabled()
            .await
            .expect("is_charging_safety_timer_enabled failed")
    );

    info!(
        "Input detection enabled: {}",
        pmu.is_input_detection_enabled()
            .await
            .expect("is_input_detection_enabled failed")
    );

    info!(
        "Input current optimizer: {}",
        pmu.is_input_current_optimizer()
            .await
            .expect("is_input_current_optimizer failed")
    );

    info!(
        "PMU chip id: {}",
        pmu.get_chip_id().await.expect("get_chip_id failed")
    );

    info!(
        "Charge current: {}mA",
        pmu.get_charge_current()
            .await
            .expect("get_charge_current failed")
    );

    pmu
}
