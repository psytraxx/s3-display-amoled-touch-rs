#![no_std]
#![no_main]
#![feature(async_closure)]

use cst816s::CST816S;
use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::Delay;
use embedded_graphics_core::pixelcolor::raw::RawU16;
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::prelude::RgbColor;
use esp_alloc::psram_allocator;
use esp_display_interface_spi_dma::display_interface_spi_dma::{self};
use esp_hal::dma::{Dma, DmaPriority};
use esp_hal::gpio::{Input, Level, NoPin, Output};
use esp_hal::i2c::master::I2c;
use esp_hal::prelude::*;
use esp_hal::rng::Rng;
use esp_hal::spi::master::Config;
use esp_hal::spi::SpiMode;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{spi::master::Spi, Blocking};
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

    let mut delay = Delay;

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

    detect_spi_model(&mut i2c);

    // Detect PMU chip
    let slave_address = detect_pmu(&mut i2c);
    info!("PMU Slave address: {:?}", slave_address);

    /*static const BoardPmuPins_t AMOLED_191_SPI_PMU_PINS =  {3/*SDA*/, 2/*SCL*/, 1/*IRQ*/}; */

    const CST816_SLAVE_ADDRESS: u8 = 0x15;

    /*static const  BoardsConfigure_t BOARD_AMOLED_191_SPI = {
        static const BoardTouchPins_t AMOLED_191_TOUCH_PINS = {3 /*SDA*/, 2 /*SCL*/, 21/*IRQ*/, -1/*RST*/};
    }; */

    // Try to find touch device
    let mut touchpad = if i2c.write(CST816_SLAVE_ADDRESS, &[]).is_ok() {
        // Touch device found
        // Initialize touch driver

        let touch_int = peripherals.GPIO21;
        let touch_int = Input::new(touch_int, esp_hal::gpio::Pull::Up);

        let mut touchpad = CST816S::new(i2c, touch_int, NoPin);
        match touchpad.setup(&mut delay) {
            Ok(_) => Some(touchpad),
            Err(_) => {
                error!("Touchpad setup failed");
                None
            }
        }
    } else {
        error!(
            "Touch device not detected at address 0x{:02X}",
            CST816_SLAVE_ADDRESS
        );
        None
    };

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
    let mut x_touch_prev: i32 = 0;
    let mut y_touch_prev: i32 = 0;

    let mut background_color = RgbColor::GREEN;
    let mut foreground_color = RgbColor::MAGENTA;

    loop {
        if let Some(ref mut touchpad) = touchpad {
            if let Some(touch_event) = touchpad.read_one_touch_event(true) {
                match touch_event.gesture {
                    cst816s::TouchGesture::None => _ = (),
                    cst816s::TouchGesture::SlideDown => info!("Gesture: Slide Down"),
                    cst816s::TouchGesture::SlideUp => info!("Gesture: Slide Up"),
                    cst816s::TouchGesture::SlideLeft => info!("Gesture: Slide Left"),
                    cst816s::TouchGesture::SlideRight => info!("Gesture: Slide Right"),
                    cst816s::TouchGesture::SingleClick => info!("Gesture: Single Click"),
                    cst816s::TouchGesture::DoubleClick => info!("Gesture: Double Click"),
                    cst816s::TouchGesture::LongPress => info!("Gesture: Long Press"),
                }

                let x_touch = touch_event.x;
                let y_touch = touch_event.y;

                if (x_touch != x_touch_prev) || (y_touch != y_touch_prev) {
                    info!("Touch event: {} {}", x_touch, y_touch);

                    display
                        .set_pixel(
                            DISPLAY_WIDTH - y_touch as u16,
                            x_touch as u16,
                            RgbColor::RED,
                        )
                        .expect("set_pixel failed");
                    y_touch_prev = touch_event.y;
                    x_touch_prev = touch_event.x;

                    let rand_val = rng.random();
                    background_color = Rgb565::from(RawU16::new(rand_val as u16));
                    let rand_val = rng.random();
                    foreground_color = Rgb565::from(RawU16::new(rand_val as u16));
                }
            }
        }

        let x_rand = rng.random() as u16 % DISPLAY_WIDTH;
        let y_rand = rng.random() as u16 % DISPLAY_HEIGHT;
        display
            .set_pixel(x_rand, y_rand, foreground_color)
            .expect("set_pixel failed");

        display
            .set_pixel(x, y, background_color)
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

fn detect_spi_model(i2c: &mut I2c<Blocking>) {
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

const SY6970_SLAVE_ADDRESS: u8 = 0x6A;
const BQ25896_SLAVE_ADDRESS: u8 = 0x6B;

fn detect_pmu(i2c: &mut I2c<Blocking>) -> Option<u8> {
    // Try SY6970
    if i2c.write(SY6970_SLAVE_ADDRESS, &[]).is_ok() {
        info!("Detected SY6970 PMU chip");
        return Some(SY6970_SLAVE_ADDRESS);
    }

    // Try BQ25896
    if i2c.write(BQ25896_SLAVE_ADDRESS, &[]).is_ok() {
        info!("Detected BQ25896 PMU chip");
        return Some(BQ25896_SLAVE_ADDRESS);
    }

    // No PMU detected
    None
}
