#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

// Import core allocator utilities and modules
extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

use alloc::boxed::Box;
use controller::Controller;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use esp_alloc::psram_allocator;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::peripherals::{GPIO2, GPIO3, I2C0};
use esp_hal::Async;
use log::info;
use radar_task::radar_task;
use render_task::render_task;
use slint::platform::software_renderer::{MinimalSoftwareWindow, RepaintBufferType};
use slint::{ComponentHandle, PhysicalSize};
use slint_backend::Backend;
use slint_generated::AppWindow;
use static_cell::StaticCell;

// Hardware initialization modules
mod hardware;
use hardware::*;

mod controller;
mod display_line_buffer;
mod radar_task;
mod render_task;
mod slint_backend;

/// Main entry point for the application
#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();

    // Initialize peripherals and configure the CPU clock
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::_240MHz));

    // Reserve memory for dynamic allocations
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 73744);

    // Set up the timer group for the embassy executor
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);
    info!("Embassy initialized!");

    // Initialize the PSRAM allocator for extra memory requirements
    psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    // Enable the power management IC by setting the PMICEN pin high
    let mut pmicen = Output::new(peripherals.GPIO38, Level::Low, OutputConfig::default());
    pmicen.set_high();
    info!("PMICEN set high");

    // Initialize the I2C bus used by several peripherals
    let i2c_bus = initialize_i2c(peripherals.I2C0, peripherals.GPIO3, peripherals.GPIO2);

    // Detect the connected SPI board model via I2C communication
    {
        detect_spi_model(&mut I2cDevice::new(i2c_bus)).await;
    }

    // Create the GUI window for Slint's minimal software renderer
    let window = MinimalSoftwareWindow::new(RepaintBufferType::ReusedBuffer);
    let size = PhysicalSize::new(DISPLAY_WIDTH.into(), DISPLAY_HEIGHT.into());
    window.set_size(size);

    // Set up the Slint rendering platform backend
    let backend = Box::new(Backend::new(window.clone()));
    slint::platform::set_platform(backend).expect("set_platform failed");

    // Initialize the touchpad interface for user interactions
    let touchpad = { initialize_touchpad(I2cDevice::new(i2c_bus), peripherals.GPIO21).await };

    // Initialize the display via SPI with DMA support
    let display = initialize_display(
        peripherals.GPIO17,
        peripherals.GPIO7,
        peripherals.GPIO47,
        peripherals.GPIO18,
        peripherals.GPIO6,
        peripherals.SPI2,
        peripherals.DMA_CH0,
    );

    // Launch the GUI render task asynchronously
    spawner.spawn(render_task(window, display, touchpad)).ok();

    // Initialize the radar (LD2410) sensor interface via UART
    let radar = initialize_radar(peripherals.UART0, peripherals.GPIO44, peripherals.GPIO43);

    // Launch the radar task asynchronously
    spawner.spawn(radar_task(radar)).ok();

    // Create and show the application window UI
    let app_window = AppWindow::new().expect("UI init failed");
    app_window.show().expect("UI show failed");

    // Initialize the PMU for battery charging control
    let mut pmu = { initialize_pmu(I2cDevice::new(i2c_bus)).await };
    // Populate initial battery percentage on the main screen
    if let Ok(percentage) = pmu.get_battery_percentage().await {
        app_window.set_battery_percentage(percentage as i32);
    }
    // Set initial charging state
    if let Ok(charging_enabled) = pmu.is_charging().await {
        app_window.set_charging(charging_enabled);
    }

    // Start the main event loop in the controller with the UI and PMU
    let mut controller = Controller::new(&app_window, pmu);
    controller.run().await;
}

/// Type alias for the shared I2C bus wrapped in a Mutex
type SharedI2cBus = Mutex<CriticalSectionRawMutex, I2c<'static, Async>>;

/// Initialize the I2C bus used to communicate with external devices.
/// Returns a shared I2C bus wrapped in a Mutex for safe concurrent access.
fn initialize_i2c(
    i2c: I2C0<'static>,
    sda: GPIO3<'static>,
    scl: GPIO2<'static>,
) -> &'static SharedI2cBus {
    // Create a new I2C master instance with default configuration
    let i2c = I2c::new(i2c, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl)
        .into_async();

    // Wrap it in a Mutex for sharing between devices
    static I2C_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, Async>>> =
        StaticCell::new();
    I2C_BUS.init(Mutex::new(i2c))
}

/// Detects the model of the connected SPI board via I2C communication.
///
/// This function probes known I2C addresses to identify the board variant.
/// It helps in determining the correct driver configuration at runtime.
///
/// # Detection Strategy
///
/// - Checks address `0x15` first to confirm board presence
/// - Then checks address `0x51` to distinguish between SPI and QSPI variants
/// - Logs the detected model or error message
///
/// # Arguments
///
/// * `i2c` - An acquired I2C device handle from the shared bus
async fn detect_spi_model<I: embedded_hal_async::i2c::I2c>(i2c: &mut I) {
    // Try to communicate with a known I2C address to identify the board model
    if i2c.write(0x15, &[]).await.is_ok() {
        // Check a secondary address to distinguish between SPI and QSPI models
        if i2c.write(0x51, &[]).await.is_ok() {
            info!("Detected 1.91-inch SPI board model!");
        } else {
            info!("Detected 1.91-inch QSPI board model!");
        }
    } else {
        log::error!("Unable to detect 1.91-inch touch board model!");
    }
}
