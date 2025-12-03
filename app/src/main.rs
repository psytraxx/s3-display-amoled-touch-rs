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
use core::cell::UnsafeCell;
use core::sync::atomic::AtomicBool;
use drivers::bq25896::asynch::BQ25896Async;
use drivers::cst816x::asynch::CST816xAsync;
use drivers::cst816x::IrqControl;
use drivers::ld2410::asynch::LD2410Async;
use embassy_executor::Spawner;
use embassy_time::Delay;
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_alloc::psram_allocator;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::dma::DmaTxBuf;
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::i2c::master::I2c;
use esp_hal::peripherals::{
    DMA_CH0, GPIO17, GPIO18, GPIO2, GPIO21, GPIO3, GPIO43, GPIO44, GPIO47, GPIO6, GPIO7, I2C0,
    SPI2, UART0,
};
use esp_hal::spi::master::{Config as SpiConfig, Spi, SpiDmaBus};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::uart::{Config as UartConfig, Parity, StopBits, Uart};
use esp_hal::{dma_buffers, uart, Async, Blocking};
use log::{error, info};
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

// Simple atomic cell for I2C bus sharing
pub struct I2cCell<T> {
    bus: UnsafeCell<T>,
    busy: AtomicBool,
}

impl<T> I2cCell<T> {
    pub const fn new(bus: T) -> Self {
        Self {
            bus: UnsafeCell::new(bus),
            busy: AtomicBool::new(false),
        }
    }
}

unsafe impl<T> Sync for I2cCell<T> {}

// Async-compatible atomic I2C device wrapper
pub struct AsyncAtomicDevice<'a, T> {
    bus: &'a I2cCell<T>,
}

impl<'a, T> AsyncAtomicDevice<'a, T> {
    pub fn new(bus: &'a I2cCell<T>) -> Self {
        Self { bus }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum AtomicI2cError<T> {
    Busy,
    I2c(T),
}

impl<T> embedded_hal::i2c::Error for AtomicI2cError<T>
where
    T: embedded_hal::i2c::Error,
{
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            AtomicI2cError::Busy => embedded_hal::i2c::ErrorKind::Other,
            AtomicI2cError::I2c(e) => e.kind(),
        }
    }
}

impl<T> embedded_hal_async::i2c::ErrorType for AsyncAtomicDevice<'_, T>
where
    T: embedded_hal_async::i2c::I2c,
{
    type Error = AtomicI2cError<T::Error>;
}

impl<T> embedded_hal_async::i2c::I2c for AsyncAtomicDevice<'_, T>
where
    T: embedded_hal_async::i2c::I2c,
{
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.bus
            .busy
            .compare_exchange(
                false,
                true,
                core::sync::atomic::Ordering::SeqCst,
                core::sync::atomic::Ordering::SeqCst,
            )
            .map_err(|_| AtomicI2cError::Busy)?;

        let result = unsafe { &mut *self.bus.bus.get() }
            .read(address, read)
            .await
            .map_err(AtomicI2cError::I2c);

        self.bus
            .busy
            .store(false, core::sync::atomic::Ordering::SeqCst);

        result
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.bus
            .busy
            .compare_exchange(
                false,
                true,
                core::sync::atomic::Ordering::SeqCst,
                core::sync::atomic::Ordering::SeqCst,
            )
            .map_err(|_| AtomicI2cError::Busy)?;

        let result = unsafe { &mut *self.bus.bus.get() }
            .write(address, write)
            .await
            .map_err(AtomicI2cError::I2c);

        self.bus
            .busy
            .store(false, core::sync::atomic::Ordering::SeqCst);

        result
    }

    async fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.bus
            .busy
            .compare_exchange(
                false,
                true,
                core::sync::atomic::Ordering::SeqCst,
                core::sync::atomic::Ordering::SeqCst,
            )
            .map_err(|_| AtomicI2cError::Busy)?;

        let result = unsafe { &mut *self.bus.bus.get() }
            .write_read(address, write, read)
            .await
            .map_err(AtomicI2cError::I2c);

        self.bus
            .busy
            .store(false, core::sync::atomic::Ordering::SeqCst);

        result
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.bus
            .busy
            .compare_exchange(
                false,
                true,
                core::sync::atomic::Ordering::SeqCst,
                core::sync::atomic::Ordering::SeqCst,
            )
            .map_err(|_| AtomicI2cError::Busy)?;

        let result = unsafe { &mut *self.bus.bus.get() }
            .transaction(address, operations)
            .await
            .map_err(AtomicI2cError::I2c);

        self.bus
            .busy
            .store(false, core::sync::atomic::Ordering::SeqCst);

        result
    }
}

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

pub type Charger = BQ25896Async<AsyncAtomicDevice<'static, I2c<'static, Async>>>;

pub type RadarSensor = LD2410Async<Uart<'static, Async>, Delay>;

pub type Touchpad = CST816xAsync<
    AsyncAtomicDevice<'static, I2c<'static, Async>>,
    Input<'static>,
    Output<'static>,
    Delay,
>;

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
    detect_spi_model(i2c_bus).await;

    // Create the GUI window for Slint's minimal software renderer
    let window = MinimalSoftwareWindow::new(RepaintBufferType::ReusedBuffer);
    let size = PhysicalSize::new(DISPLAY_WIDTH.into(), DISPLAY_HEIGHT.into());
    window.set_size(size);

    // Set up the Slint rendering platform backend
    let backend = Box::new(Backend::new(window.clone()));
    slint::platform::set_platform(backend).expect("set_platform failed");

    // Initialize the touchpad interface for user interactions
    let touchpad = initialize_touchpad(i2c_bus, peripherals.GPIO21).await;

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
    let mut pmu = initialize_pmu(i2c_bus).await;
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

/// Initialize the I2C bus used to communicate with external devices.
/// Returns a shared, thread-safe reference (I2cCell) for the I2C instance.
fn initialize_i2c(
    i2c: I2C0<'static>,
    sda: GPIO3<'static>,
    scl: GPIO2<'static>,
) -> &'static I2cCell<I2c<'static, Async>> {
    // Create a new I2C master instance with default configuration
    let i2c = I2c::new(i2c, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(scl)
        .into_async();

    // Use a StaticCell for static storage of the I2cCell wrapping the I2C instance
    static I2C_INSTANCE: StaticCell<I2cCell<I2c<'static, Async>>> = StaticCell::new();
    I2C_INSTANCE.init(I2cCell::new(i2c))
}

/// Configures the touch sensor driver (CST816S) by wrapping the I2C bus and input pin.
/// Returns an instance of the touchpad driver.
async fn initialize_touchpad(
    i2c: &'static I2cCell<I2c<'static, Async>>,
    touch: GPIO21<'static>,
) -> Touchpad {
    // Configure the GPIO pin used for touch input (no pull-up/down)
    let touch_pin = Input::new(touch, InputConfig::default().with_pull(Pull::None));
    let i2c_device = AsyncAtomicDevice::new(i2c);
    let mut touchpad = CST816xAsync::new(i2c_device, touch_pin, None, Delay);
    touchpad.begin().await.expect("Failed to begin touchpad");
    let irq_config = IrqControl::EN_TOUCH | IrqControl::EN_CHANGE | IrqControl::EN_MOTION;
    touchpad
        .set_irq_control(&irq_config)
        .await
        .expect("Failed to set IRQ control");
    touchpad
        .enable_auto_reset(5)
        .await
        .expect("Failed to enable auto-reset");
    let irq_config = touchpad
        .get_irq_control()
        .await
        .expect("Failed to get IRQ control");
    info!("IRQ control: 0x{:X}", irq_config.bits());
    let chip_id = touchpad.get_chip_id().await.expect("Failed to get chip ID");
    info!("Touchpad chip ID: {chip_id}");
    let motion_mask = touchpad
        .get_motion_mask()
        .await
        .expect("Failed to get motion mask");
    info!("Motion mask: 0x{motion_mask:X}");
    touchpad
        .set_irq_pulse_width(10)
        .await
        .expect("Failed to set pulse width");
    let pulse_config = touchpad
        .get_irq_pulse_width()
        .await
        .expect("Failed to get pulse config");
    info!("Pulse width: {pulse_config:?}");

    touchpad
}

/// Creates and initializes the radar sensor (LD2410) interface using UART.
/// Returns the configured radar sensor instance.
fn initialize_radar(
    uart1: UART0<'static>,
    rx_pin: GPIO44<'static>,
    tx_pin: GPIO43<'static>,
) -> RadarSensor {
    // Set UART configuration including baud rate, parity, and stop bits
    let config = UartConfig::default()
        .with_baudrate(256000)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);

    // Initialize the UART instance for the radar module communication
    let uart0 = uart::Uart::new(uart1, config).expect("Failed to initialize UART0");

    // Associate the UART with its designated RX and TX GPIO pins
    let uart0 = uart0.with_rx(rx_pin).with_tx(tx_pin).into_async();

    // Construct the radar driver with the configured UART and a delay provider
    LD2410Async::new(uart0, Delay)
}

/// Initializes the SPI-connected display and configures its DMA buffers.
/// Returns an instance of the RM67162 display driver.
fn initialize_display(
    reset: GPIO17<'static>,
    dc: GPIO7<'static>,
    sck: GPIO47<'static>,
    mosi: GPIO18<'static>,
    cs: GPIO6<'static>,
    spi: SPI2<'static>,
    dma: DMA_CH0<'static>,
) -> TouchDisplay {
    // Configure GPIO pins for display control signals (DC, CS, reset, clock, and MOSI)
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

    #[allow(clippy::manual_div_ceil)]
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = esp_hal::dma::DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // Create the SPI DMA bus with the configured buffers
    let spi = SpiDmaBus::new(spi_dma, dma_rx_buf, dma_tx_buf);

    // Attach the SPI device using the chip-select control pin (no delay used)
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

    // Allocate a buffer for display initialization commands
    static DISPLAY_BUFFER: StaticCell<[u8; 512]> = StaticCell::new();
    let buffer = DISPLAY_BUFFER.init([0_u8; 512]);

    // Create the SPI interface for the display driver using the SPI device, DC pin, and initialization buffer
    let di = SpiInterface::new(spi_device, dc, buffer);

    // Initialize and configure the RM67162 display with the desired orientation and reset pin handling
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
/// This helps in determining the correct driver configuration.
async fn detect_spi_model(i2c_ref_cell: &'static I2cCell<I2c<'static, Async>>) {
    // Create an I2C device instance for peripheral communication
    let mut i2c = AsyncAtomicDevice::new(i2c_ref_cell);

    // Try to communicate with a known I2C address to identify the board model
    use embedded_hal_async::i2c::I2c as _;
    if i2c.write(0x15, &[]).await.is_ok() {
        // Check a secondary address to distinguish between SPI and QSPI models
        if i2c.write(0x51, &[]).await.is_ok() {
            info!("Detected 1.91-inch SPI board model!");
        } else {
            info!("Detected 1.91-inch QSPI board model!");
        }
    } else {
        error!("Unable to detect 1.91-inch touch board model!");
    }
}

/// Initializes and configures the power management unit (PMU) for battery charging.
/// It sets the charging target voltage, precharge current, and fast charge current limits,
/// enables ADC for power measurement, and logs the configuration details.
/// Returns the configured PMU instance.
async fn initialize_pmu(i2c_ref_cell: &'static I2cCell<I2c<'static, Async>>) -> Charger {
    let i2c = AsyncAtomicDevice::new(i2c_ref_cell);

    // Create a new PMU instance on the I2C bus at the designated slave address
    let mut pmu = BQ25896Async::new(i2c, BQ25896_SLAVE_ADDRESS);
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
