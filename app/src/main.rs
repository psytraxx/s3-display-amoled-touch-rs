#![no_std]
#![no_main]

use alloc::boxed::Box;
use controller::Controller;
use defmt::info;
use embassy_executor::Spawner;
use embedded_hal_bus::util::AtomicCell;
use esp_alloc::psram_allocator;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::xtensa_lx::singleton;
use esp_hal_embassy::main;
use pmu::PmuImpl;
use radar_task::radar_task;
use render_task::{render_task, RenderTaskPeriphals};
use slint::platform::software_renderer::{MinimalSoftwareWindow, RepaintBufferType};
use slint::{ComponentHandle, PhysicalSize};
use slint_backend::Backend;
use {defmt_rtt as _, esp_backtrace as _};

slint::include_modules!();

extern crate alloc;

mod controller;
mod draw_buffer;
mod pmu;
mod radar_task;
mod render_task;
mod slint_backend;

pub const DISPLAY_HEIGHT: u16 = 240;
/// Display resolution width in pixels
pub const DISPLAY_WIDTH: u16 = 536;
/// I2C address of BQ25896 PMU
const BQ25896_SLAVE_ADDRESS: u8 = 0x6B;
/// Charging target voltage in mV for BQ25896
const PMU_CHARGE_TARGET_VOLTAGE: u16 = 4208;
/// Precharge current in mA for BQ25896
const PMU_PRECHARGE_CURRENT: u16 = 128;
/// Constant charging current in mA for BQ25896
const PMU_CONSTANT_CHARGE_CURRENT: u16 = 1536;

#[main]
async fn main(spawner: Spawner) {
    // Initialize peripherals
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::_240MHz));

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    info!("Embassy initialized!");

    // Initialize PSRAM allocator
    psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    // Initialize I2C bus
    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO3)
        .with_scl(peripherals.GPIO2);

    let i2c_ref_cell = singleton!(:AtomicCell<I2c<'_, esp_hal::Blocking>> = AtomicCell::new(i2c))
        .expect("Failed to create I2C mutex");

    // Initialize PMICEN pin to enable power management IC
    let mut pmicen = Output::new(peripherals.GPIO38, Level::Low, OutputConfig::default());
    pmicen.set_high();
    info!("PMICEN set high");

    let window = MinimalSoftwareWindow::new(RepaintBufferType::ReusedBuffer);
    let size = PhysicalSize::new(DISPLAY_WIDTH.into(), DISPLAY_HEIGHT.into());
    window.set_size(size);

    // Set the platform for Slint
    let backend = Box::new(Backend::new(window.clone()));
    slint::platform::set_platform(backend).expect("set_platform failed");

    let p = RenderTaskPeriphals {
        touch_pin: peripherals.GPIO21,
        reset_pin: peripherals.GPIO17,
        dc_pin: peripherals.GPIO7,
        sck_pin: peripherals.GPIO47,
        mosi_pin: peripherals.GPIO18,
        cs_pin: peripherals.GPIO6,
        dma_ch0: peripherals.DMA_CH0,
        spi2: peripherals.SPI2,
    };

    // TASK: run the gui render loop
    spawner.spawn(render_task(window, p, i2c_ref_cell)).ok();

    // TASK: run the radar task
    spawner
        .spawn(radar_task(
            peripherals.GPIO44,
            peripherals.GPIO43,
            peripherals.UART0,
        ))
        .ok();

    // Initialize UI
    let app_window = AppWindow::new().expect("UI init failed");
    app_window.show().expect("UI show failed");

    // Initialize power management unit
    let pmu = PmuImpl::new(i2c_ref_cell);

    // run the controller event loop
    let mut controller = Controller::new(&app_window, pmu);
    controller.run().await;
}
