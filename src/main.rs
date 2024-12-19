#![no_std]
#![no_main]
#![feature(async_closure)]

use core::cell::RefCell;

use defmt::info;
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::NoopMutex;
use embedded_graphics_core::prelude::RgbColor;
use esp_alloc::psram_allocator;
use esp_hal::gpio::{Level, Output};
use esp_hal::rng::Rng;
use esp_hal::spi::master::Config;
use esp_hal::spi::SpiMode;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{delay::Delay, spi::master::Spi};
use esp_hal::{prelude::*, Blocking};
use mipidsi::options::Orientation;
use mipidsi::Builder;
use rm67162::RM67162;
use static_cell::StaticCell;
use {defmt_rtt as _, esp_backtrace as _};
extern crate alloc;

mod display;
mod rm67162;

//https://github.com/Xinyuan-LilyGO/LilyGo-AMOLED-Series/blob/8c72b786373fbaef46ce35a6db924d6e16a0c3ec/src/LilyGo_AMOLED.cpp#L806

static DISP_SPI_BUS: StaticCell<NoopMutex<RefCell<Spi<'static, Blocking>>>> = StaticCell::new();

#[main]
async fn main(_spawner: Spawner) -> ! {
    esp_alloc::heap_allocator!(72 * 1024);

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::Clock240MHz;
        config
    });

    let mut rng = Rng::new(peripherals.RNG);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    esp_hal_embassy::init(timg0.timer0);

    psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let delay = Delay::new();

    // SPI pins
    let sck = Output::new(peripherals.GPIO47, Level::Low);
    let mosi = Output::new(peripherals.GPIO18, Level::Low);
    let cs = Output::new(peripherals.GPIO6, Level::Low);

    // Configure SPI
    let spi = Spi::new_with_config(
        peripherals.SPI2,
        Config {
            frequency: 40.MHz(),
            mode: SpiMode::Mode0,
            ..Config::default()
        },
    )
    .with_sck(sck)
    .with_mosi(mosi);

    let spi_bus = NoopMutex::new(RefCell::new(spi));
    let spi_bus = DISP_SPI_BUS.init(spi_bus);
    let dc_pin = peripherals.GPIO7;
    let rst_pin = peripherals.GPIO17;

    let spi: SpiDevice<
        '_,
        embassy_sync::blocking_mutex::raw::NoopRawMutex,
        Spi<'_, Blocking>,
        Output<'_>,
    > = SpiDevice::new(spi_bus, cs);

    // Initialize the display

    //let mut driver = DisplayDriver::new(spi, delay);
    //driver.init(dc_pin, rst_pin);

    let di = SPIInterface::new(spi, Output::new(dc_pin, Level::Low));

    let mut display = Builder::new(RM67162, di)
        .orientation(Orientation::default())
        .display_size(240, 536)
        .reset_pin(Output::new(rst_pin, Level::Low))
        .init(&mut embassy_time::Delay)
        .unwrap();

    loop {
        info!("Hello world!");
        delay.delay(500.millis());
        let x = rng.random() % 240;
        let y = rng.random() % 536;
        display
            .set_pixel(x as u16, y as u16, RgbColor::MAGENTA)
            .expect("set_pixel failed");
    }
}
