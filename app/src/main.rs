#![no_std]
#![no_main]

extern crate alloc;
use alloc::boxed::Box;
use controller::Controller;
use defmt::info;
use drivers::cst816s::{TouchInput, CST816S};
use drivers::ld2410::LD2410;
use embassy_executor::Spawner;
use embassy_time::Delay;
use embedded_hal::i2c::I2c as I2CBus;
use embedded_hal_bus::i2c::AtomicDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_hal_bus::util::AtomicCell;
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
use esp_hal::xtensa_lx::singleton;
use esp_hal::{dma_buffers, uart, Blocking};
use esp_hal_embassy::main;
use mipidsi::interface::SpiInterface;
use mipidsi::models::RM67162;
use mipidsi::options::{Orientation, Rotation};
use mipidsi::{Builder, Display};
use pmu::PmuImpl;
use radar_task::radar_task;
use render_task::render_task;
use slint::platform::software_renderer::{MinimalSoftwareWindow, RepaintBufferType};
use slint::{ComponentHandle, PhysicalSize};
use slint_backend::Backend;
use slint_generated::AppWindow;
use {defmt_rtt as _, esp_backtrace as _};

mod controller;
mod display_line_buffer;
mod pmu;
mod radar_task;
mod render_task;
mod slint_backend;

pub const DISPLAY_HEIGHT: u16 = 240;
/// Display resolution width in pixels
pub const DISPLAY_WIDTH: u16 = 536;

pub type MipiDisplay = Display<
    SpiInterface<
        'static,
        ExclusiveDevice<
            SpiDmaBus<'static, Blocking>,
            Output<'static>,
            embedded_hal_bus::spi::NoDelay,
        >,
        Output<'static>,
    >,
    RM67162,
    Output<'static>,
>;

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

    // Initialize PMICEN pin to enable power management IC
    let mut pmicen = Output::new(peripherals.GPIO38, Level::Low, OutputConfig::default());
    pmicen.set_high();
    info!("PMICEN set high");

    let i2c_ref_cell = initialize_i2c(peripherals.I2C0, peripherals.GPIO3, peripherals.GPIO2);

    let window = MinimalSoftwareWindow::new(RepaintBufferType::ReusedBuffer);
    let size = PhysicalSize::new(DISPLAY_WIDTH.into(), DISPLAY_HEIGHT.into());
    window.set_size(size);

    // Set the platform for Slint
    let backend = Box::new(Backend::new(window.clone()));
    slint::platform::set_platform(backend).expect("set_platform failed");

    let touchpad = initialize_touchpad(i2c_ref_cell, peripherals.GPIO21);

    let display = initialize_display(
        peripherals.GPIO17,
        peripherals.GPIO7,
        peripherals.GPIO47,
        peripherals.GPIO18,
        peripherals.GPIO6,
        peripherals.SPI2,
        peripherals.DMA_CH0,
    );

    // TASK: run the gui render loop
    spawner.spawn(render_task(window, display, touchpad)).ok();

    let radar = initialize_radar(peripherals.UART0, peripherals.GPIO44, peripherals.GPIO43);

    // TASK: run the radar task
    spawner.spawn(radar_task(radar)).ok();

    // Initialize UI
    let app_window = AppWindow::new().expect("UI init failed");
    app_window.show().expect("UI show failed");

    // Initialize power management unit
    let pmu = PmuImpl::new(i2c_ref_cell);

    // run the controller event loop
    let mut controller = Controller::new(&app_window, pmu);
    controller.run().await;
}

// Initialize I2C bus
fn initialize_i2c(
    i2c: I2C0,
    sda: GpioPin<3>,
    pin: GpioPin<2>,
) -> &'static AtomicCell<I2c<'static, Blocking>> {
    let i2c = I2c::new(i2c, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(sda)
        .with_scl(pin);

    singleton!(:AtomicCell<I2c<'_, Blocking>> = AtomicCell::new(i2c))
        .expect("Failed to create I2C mutex")
}

// Initialize touchpad
fn initialize_touchpad<BUS>(
    i2c: &'static AtomicCell<BUS>,
    touch: GpioPin<21>,
) -> Box<dyn TouchInput>
where
    BUS: I2CBus,
{
    let touch_pin = Input::new(touch, InputConfig::default().with_pull(Pull::None));

    let touchpad = Box::new(CST816S::new(AtomicDevice::new(i2c), touch_pin));
    touchpad
}

// Create the LD2410 radar instance
fn initialize_radar<'a>(
    uart1: UART0,
    rx_pin: GpioPin<44>,
    tx_pin: GpioPin<43>,
) -> LD2410<Uart<'a, Blocking>, Delay> {
    let config = UartConfig::default()
        .with_baudrate(256000)
        .with_parity(Parity::None)
        .with_stop_bits(StopBits::_1);

    let uart0 = uart::Uart::new(uart1, config).expect("Failed to initialize UART0");

    let uart0 = uart0.with_rx(rx_pin).with_tx(tx_pin);

    LD2410::new(uart0, Delay)
}

// Initialize display
fn initialize_display(
    reset: GpioPin<17>,
    dc: GpioPin<7>,
    sck: GpioPin<47>,
    mosi: GpioPin<18>,
    cs: GpioPin<6>,
    spi: SPI2,
    dma: DmaChannel0,
) -> MipiDisplay {
    // Initialize display SPI peripherals

    let dc = Output::new(dc, Level::Low, OutputConfig::default());
    let cs = Output::new(cs, Level::High, OutputConfig::default());
    let reset_pin = Output::new(reset, Level::High, OutputConfig::default());
    let sck = Output::new(sck, Level::Low, OutputConfig::default());
    let mosi = Output::new(mosi, Level::Low, OutputConfig::default());

    // Configure SPI
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

    // Configure SPI DMA buffers
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = esp_hal::dma::DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = SpiDmaBus::new(spi_dma, dma_rx_buf, dma_tx_buf);

    // Initialize SPI device
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let buffer = Box::new([0_u8; 512]);
    let di = SpiInterface::new(spi_device, dc, Box::leak(buffer).as_mut());

    Builder::new(RM67162, di)
        .orientation(Orientation {
            mirrored: false,
            rotation: Rotation::Deg270,
        })
        .reset_pin(reset_pin)
        .init(&mut esp_hal::delay::Delay::new())
        .expect("Failed to initialize display")
}
