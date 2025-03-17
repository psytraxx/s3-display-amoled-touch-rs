#![no_std]
#![no_main]

// Import core allocator utilities and modules
extern crate alloc;
use alloc::boxed::Box;
use controller::Controller;
use defmt::{error, info};
use drivers::bq25896::BQ25896;
use drivers::cst816s::CST816S;
use drivers::ld2410::LD2410;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use embedded_hal_async::i2c::I2c as I2cTrait;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_alloc::psram_allocator;
use esp_hal::clock::CpuClock;
use esp_hal::dma::{DmaChannel0, DmaTxBuf};
use esp_hal::gpio::{GpioPin, Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::i2c::master::I2c;
use esp_hal::peripherals::{I2C0, SPI2, UART0};
use esp_hal::spi::master::{Config as SpiConfig, Spi, SpiDmaBus};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::uart::{Config as UartConfig, Parity, StopBits, Uart};
use esp_hal::{dma_buffers, uart, Async, Blocking};
use esp_hal_embassy::main;
use mipidsi::interface::SpiInterface;
use mipidsi::models::RM67162;
use mipidsi::options::{Orientation, Rotation};
use mipidsi::{Builder, Display};
use radar_task::radar_task;
use render_task::render_task;
use slint::platform::software_renderer::{MinimalSoftwareWindow, RepaintBufferType};
use slint::{ComponentHandle, PhysicalSize};
use slint_backend::Backend;
use slint_generated::AppWindow;
use static_cell::StaticCell;
use {defmt_rtt as _, esp_backtrace as _};

mod controller;
mod display_line_buffer;
mod radar_task;
mod render_task;
mod slint_backend;

/// Display resolution height in pixels
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

/// Type alias for the RM67162 display instance using SPI interface
pub type TouchDisplay = Display<
    SpiInterface<
        'static,
        ExclusiveDevice<SpiDmaBus<'static, Blocking>, Output<'static>, NoDelay>,
        Output<'static>,
    >,
    RM67162,
    Output<'static>,
>;

pub type I2C0Bus = Mutex<NoopRawMutex, I2c<'static, Async>>;

pub type Charger = BQ25896<I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>>;

pub type RadarSensor = LD2410<Uart<'static, Async>, Delay>;

pub type Touchpad = CST816S<I2cDevice<'static, NoopRawMutex, I2c<'static, Async>>, Input<'static>>;

/// Main entry point for the application
#[main]
async fn main(spawner: Spawner) {
    // Initialize peripherals and set the CPU clock configuration
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::_240MHz));

    // Set aside memory for dynamic allocations
    esp_alloc::heap_allocator!(size: 72 * 1024);

    // Initialize timer group for the embassy executor
    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);
    info!("Embassy initialized!");

    // Initialize PSRAM allocator to support extra memory requirements
    psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    // Enable the power management IC by setting PMICEN pin high
    let mut pmicen = Output::new(peripherals.GPIO38, Level::Low, OutputConfig::default());
    pmicen.set_high();
    info!("PMICEN set high");

    // Initialize the I2C bus used by several peripherals
    let i2c_bus = initialize_i2c(peripherals.I2C0, peripherals.GPIO3, peripherals.GPIO2);

    // Try to detect the connected SPI model based on I2C responses
    detect_spi_model(i2c_bus).await;

    // Create the GUI window used by Slint's minimal software renderer
    let window = MinimalSoftwareWindow::new(RepaintBufferType::ReusedBuffer);
    let size = PhysicalSize::new(DISPLAY_WIDTH.into(), DISPLAY_HEIGHT.into());
    window.set_size(size);

    // Set the Slint rendering platform backend
    let backend = Box::new(Backend::new(window.clone()));
    slint::platform::set_platform(backend).expect("set_platform failed");

    // Initialize the touchpad interface for user interactions
    let touchpad = initialize_touchpad(i2c_bus, peripherals.GPIO21);

    // Initialize the display connected via SPI with DMA support
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

    // Initialize the power management unit for battery/charger control
    let pmu = initialize_pmu(i2c_bus).await;

    // Start the main event loop in the controller with the UI and PMU
    let mut controller = Controller::new(&app_window, pmu);
    controller.run().await;
}

/// Initialize the I2C bus used to communicate with external devices.
/// Returns a shared, thread-safe reference (AtomicCell) for the I2C instance.
fn initialize_i2c(i2c: I2C0, sda: GpioPin<3>, scl: GpioPin<2>) -> &'static mut I2C0Bus {
    // Create a new I2C master instance with default configuration
    let i2c = I2c::new(i2c, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl)
        .into_async();

    static I2C_BUS: StaticCell<I2C0Bus> = StaticCell::new();
    I2C_BUS.init(Mutex::new(i2c))
}

/// Initialize the touchpad by configuring the relevant GPIO pin and wrapping
/// the CST816S touch sensor driver.
/// Returns a boxed trait object to abstract over the touch input interface.
fn initialize_touchpad(i2c_bus: &'static I2C0Bus, touch: GpioPin<21>) -> Touchpad {
    // Configure the GPIO pin for the touch input with no pull-up/down
    let touch_pin = Input::new(touch, InputConfig::default().with_pull(Pull::None));
    let i2c_device = I2cDevice::new(i2c_bus);
    // Initialize the touch sensor driver using the I2C bus and touch input pin
    CST816S::new(i2c_device, touch_pin)
}

/// Create and initialize the radar sensor (LD2410) interface using UART.
/// Returns the configured radar instance.
fn initialize_radar(uart1: UART0, rx_pin: GpioPin<44>, tx_pin: GpioPin<43>) -> RadarSensor {
    // Configure UART options including baud rate, parity, and stop bits
    let config = UartConfig::default()
        .with_baudrate(256000)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);

    // Create the UART instance for communication with the radar module
    let uart0 = uart::Uart::new(uart1, config).expect("Failed to initialize UART0");

    // Associate the UART with its designated RX and TX GPIO pins
    let uart0 = uart0.with_rx(rx_pin).with_tx(tx_pin).into_async();

    // Construct the radar driver with the configured UART and a delay provider
    LD2410::new(uart0, Delay)
}

/// Initializes the SPI-connected display and configures its DMA buffers.
/// Returns an instance of the RM67162 display driver.
fn initialize_display(
    reset: GpioPin<17>,
    dc: GpioPin<7>,
    sck: GpioPin<47>,
    mosi: GpioPin<18>,
    cs: GpioPin<6>,
    spi: SPI2,
    dma: DmaChannel0,
) -> TouchDisplay {
    // Set up GPIO pins for display control signals (Data/Command, Chip Select, Reset, Clock, MOSI)
    let dc = Output::new(dc, Level::Low, OutputConfig::default());
    let cs = Output::new(cs, Level::High, OutputConfig::default());
    let reset_pin = Output::new(reset, Level::High, OutputConfig::default());
    let sck = Output::new(sck, Level::Low, OutputConfig::default());
    let mosi = Output::new(mosi, Level::Low, OutputConfig::default());

    // Create the SPI instance with DMA support and desired communication settings
    let spi_dma = Spi::new(
        spi,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(75))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sck)
    .with_mosi(mosi)
    .with_dma(dma);

    // Configure the DMA buffers for SPI communication
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = esp_hal::dma::DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // Create the SPI DMA bus from the SPI instance and DMA buffers
    let spi = SpiDmaBus::new(spi_dma, dma_rx_buf, dma_tx_buf);

    // Attach the SPI device with a CS control pin (using no delay for simplicity)
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    // Allocate a buffer for display initialization commands
    let buffer = Box::new([0_u8; 512]);
    // Create the SPI interface for the display driver using the device, DC pin, and buffer
    let di = SpiInterface::new(spi_device, dc, Box::leak(buffer).as_mut());

    // Initialize and configure the RM67162 display with orientation and reset pin settings
    Builder::new(RM67162, di)
        .orientation(Orientation {
            mirrored: false,
            rotation: Rotation::Deg270,
        })
        .reset_pin(reset_pin)
        .init(&mut esp_hal::delay::Delay::new())
        .expect("Failed to initialize display")
}

/// Detects the model of the connected SPI board via I2C communication.
/// This helps in choosing the correct driver configuration.
async fn detect_spi_model(i2c_bus: &'static I2C0Bus) {
    // Create an atomic I2C device for safe peripheral access
    let mut i2c = I2cDevice::new(i2c_bus);

    // Attempt to communicate with a known I2C address to identify the board model
    if i2c.write(0x15, &[]).await.is_ok() {
        // Further check a secondary address to distinguish between SPI and QSPI models
        if i2c.write(0x51, &[]).await.is_ok() {
            info!("Detect 1.91-inch SPI board model!");
        } else {
            info!("Detect 1.91-inch QSPI board model!");
        }
    } else {
        error!("Unable to detect 1.91-inch touch board model!");
    }
}

/// Initializes and configures the power management unit (PMU)
/// by setting charging target, precharge current, and fast charge current limits.
/// Returns the configured PMU instance
async fn initialize_pmu(i2c_bus: &'static I2C0Bus) -> Charger {
    let i2c_device = I2cDevice::new(i2c_bus);

    // Create a new PMU instance on the I2C bus at the designated slave address
    let mut pmu = BQ25896::new(i2c_device, BQ25896_SLAVE_ADDRESS)
        .await
        .expect("Failed to initialize BQ25896");

    // Configure the charging target voltage for the battery charger
    pmu.set_charge_target_voltage(PMU_CHARGE_TARGET_VOLTAGE)
        .await
        .expect("set_charge_target_voltage failed");

    // Configure the precharge current for battery charging
    pmu.set_precharge_current(PMU_PRECHARGE_CURRENT)
        .await
        .expect("set_precharge_current failed");

    // Configure the fast (constant) charge current limit
    pmu.set_fast_charge_current_limit(PMU_CONSTANT_CHARGE_CURRENT)
        .await
        .expect("set_fast_charge_current_limit failed");

    // Enable ADC for power measurement in the PMU
    pmu.set_adc_enabled().await.expect("set_adc_enabled failed");

    // Retrieve and log the configured fast charge current limit
    let fast_charge_current_limit = pmu
        .get_fast_charge_current_limit()
        .await
        .expect("get_fast_charge_current_limit failed");
    info!("Fast charge current limit: {}", fast_charge_current_limit);

    // Retrieve and log the precharge current setting
    let precharge_current = pmu
        .get_precharge_current()
        .await
        .expect("get_precharge_current failed");
    info!("Precharge current: {}", precharge_current);

    // Retrieve and log the charging target voltage
    let charge_target_voltage = pmu
        .get_charge_target_voltage()
        .await
        .expect("get_charge_target_voltage failed");
    info!("Charge target voltage: {}", charge_target_voltage);

    // Log the chip ID for debugging purposes
    info!(
        "PMU chip id: {}",
        pmu.get_chip_id().await.expect("get_chip_id failed")
    );

    pmu
}
