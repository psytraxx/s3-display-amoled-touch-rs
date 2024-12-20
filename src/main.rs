#![no_std]
#![no_main]
#![feature(async_closure)]

use defmt::{error, info};
use embassy_executor::Spawner;
use embedded_graphics_core::prelude::RgbColor;
use esp_alloc::psram_allocator;
use esp_display_interface_spi_dma::display_interface_spi_dma::{self};
use esp_hal::dma::{Dma, DmaPriority};
use esp_hal::gpio::{Input, Level, Output};
use esp_hal::i2c::master::I2c;
use esp_hal::prelude::*;
use esp_hal::rng::Rng;
use esp_hal::spi::master::Config;
use esp_hal::spi::SpiMode;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{delay::Delay, spi::master::Spi};
use mipidsi::options::Orientation;
use mipidsi::Builder;
use rm67162::RM67162;
use {defmt_rtt as _, esp_backtrace as _};
extern crate alloc;

mod rm67162;

pub const DISPLAY_HEIGHT: u16 = 536;
pub const DISPLAY_WIDTH: u16 = 240;

pub const LCD_PIXELS: usize = (DISPLAY_HEIGHT as usize) * (DISPLAY_WIDTH as usize);

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

    let mut i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .with_sda(peripherals.GPIO3)
        .with_scl(peripherals.GPIO2);

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

    let mut delay = Delay::new();

    // SPI pins
    let sck = Output::new(peripherals.GPIO47, Level::Low);
    let mosi = Output::new(peripherals.GPIO18, Level::Low);
    let cs = Output::new(peripherals.GPIO6, Level::High);

    let _te = Input::new(peripherals.GPIO9, esp_hal::gpio::Pull::Down);

    let mut pmicen = Output::new_typed(peripherals.GPIO38, Level::Low);
    pmicen.set_high();
    info!("PMICEN set high");

    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;

    // Configure SPI
    let spi_bus = Spi::new_with_config(
        peripherals.SPI2,
        Config {
            frequency: 80.MHz(), //TODO: 40MHz
            mode: SpiMode::Mode0,
            ..Config::default()
        },
    )
    .with_sck(sck)
    .with_cs(cs)
    .with_mosi(mosi)
    .with_dma(dma_channel.configure(false, DmaPriority::Priority0));

    let dc_pin = peripherals.GPIO7;
    let rst_pin = peripherals.GPIO17;

    // Initialize the display
    let di =
        display_interface_spi_dma::new_no_cs(LCD_PIXELS, spi_bus, Output::new(dc_pin, Level::Low));

    let mut display = Builder::new(RM67162, di)
        .orientation(Orientation::default())
        .display_size(DISPLAY_WIDTH, DISPLAY_HEIGHT)
        .reset_pin(Output::new(rst_pin, Level::High))
        .init(&mut delay)
        .unwrap();

    let mut x: u16 = 0;
    let mut y: u16 = 0;

    loop {
        let x_rand = rng.random() as u16 % DISPLAY_WIDTH;
        let y_rand = rng.random() as u16 % DISPLAY_HEIGHT;
        display
            .set_pixel(x_rand, y_rand, RgbColor::MAGENTA)
            .expect("set_pixel failed");

        display
            .set_pixel(x, y, RgbColor::GREEN)
            .expect("set_pixel failed");

        x += 1;
        if x >= DISPLAY_WIDTH {
            x = 0;
            y += 1;
            if y >= DISPLAY_HEIGHT {
                y = 0;
            }
        }
    }
}
