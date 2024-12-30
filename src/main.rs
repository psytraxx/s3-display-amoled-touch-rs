#![no_std]
#![no_main]
#![feature(async_closure)]

use bq25896::BQ25896;
use core::cell::RefCell;
use cst816s::CST816S;
use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::Delay;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::mono_font::iso_8859_1::FONT_10X20 as FONT;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::prelude::Dimensions;
use embedded_graphics::Drawable;
use embedded_graphics_core::pixelcolor::Rgb565;
use embedded_graphics_core::prelude::RgbColor;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c as I2CBus;
use embedded_hal_bus::i2c::RefCellDevice;
use embedded_text::alignment::HorizontalAlignment;
use embedded_text::style::{HeightMode, TextBoxStyle, TextBoxStyleBuilder};
use embedded_text::TextBox;
use esp_alloc::psram_allocator;
use esp_hal::gpio::{Input, Level, NoPin, Output};
use esp_hal::i2c::master::I2c;
use esp_hal::prelude::*;
use esp_hal::rng::Rng;
use esp_hal::spi::master::Config;
use esp_hal::spi::master::Spi;
use esp_hal::spi::SpiMode;
use esp_hal::timer::timg::TimerGroup;
use mipidsi::options::Orientation;
use mipidsi::Builder;
use rm67162::RM67162;
use {defmt_rtt as _, esp_backtrace as _};

#[macro_use]
extern crate alloc;

mod bq25896;
mod rm67162;

pub const DISPLAY_HEIGHT: u16 = 536;

/// Display resolution width in pixels
pub const DISPLAY_WIDTH: u16 = 240;

/// Total number of pixels in display
pub const LCD_PIXELS: usize = (DISPLAY_HEIGHT as usize) * (DISPLAY_WIDTH as usize);

/// Default text box style settings
const TEXT_BOX_STYLE: TextBoxStyle = TextBoxStyleBuilder::new()
    .height_mode(HeightMode::FitToText)
    .alignment(HorizontalAlignment::Justified)
    .build();

/// Default text rendering style
const TEXT_STYLE: MonoTextStyle<Rgb565> = MonoTextStyle::new(&FONT, Rgb565::WHITE);

/// I2C address of BQ25896 PMU
const BQ25896_SLAVE_ADDRESS: u8 = 0x6B;

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

    // initalize i2c bus
    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .with_sda(peripherals.GPIO3)
        .with_scl(peripherals.GPIO2);

    let i2c_ref_cell = RefCell::new(i2c);

    // initalize bq25896 charger
    let mut pmu = BQ25896::new(RefCellDevice::new(&i2c_ref_cell), BQ25896_SLAVE_ADDRESS)
        .expect("BQ25896 init failed");

    pmu.set_adc_enabled().expect("set_adc_enabled failed");

    pmu.set_fast_charge_current_limit(2048)
        .expect("set_fast_charge_current_limit failed");

    info!("PMU chip id: {}", pmu.get_chip_id().unwrap());

    // initalize touchpad
    let touch_int = peripherals.GPIO21;
    let touch_int = Input::new(touch_int, esp_hal::gpio::Pull::Up);

    let mut touchpad = CST816S::new(RefCellDevice::new(&i2c_ref_cell), touch_int, NoPin);
    touchpad.setup(&mut delay).expect("touchpad setup failed");

    // initialize display
    let sck = Output::new(peripherals.GPIO47, Level::Low);
    let mosi = Output::new(peripherals.GPIO18, Level::Low);
    let cs = Output::new(peripherals.GPIO6, Level::High);

    let _te = Input::new(peripherals.GPIO9, esp_hal::gpio::Pull::Down);

    let mut pmicen = Output::new_typed(peripherals.GPIO38, Level::Low);
    pmicen.set_high();
    info!("PMICEN set high");

    let spi_bus = Spi::new_with_config(
        peripherals.SPI2,
        Config {
            frequency: 80.MHz(), //TODO: 40MHz
            mode: SpiMode::Mode0,
            ..Config::default()
        },
    )
    .with_sck(sck)
    .with_mosi(mosi);

    let dc_pin = peripherals.GPIO7;
    let rst_pin = peripherals.GPIO17;

    let spi_bus = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();

    let di = display_interface_spi::SPIInterface::new(spi_bus, Output::new(dc_pin, Level::Low));

    detect_spi_model(RefCellDevice::new(&i2c_ref_cell));

    let mut display = Builder::new(RM67162, di)
        .orientation(Orientation {
            rotation: mipidsi::options::Rotation::Deg90,
            mirrored: false,
        })
        .display_size(DISPLAY_WIDTH, DISPLAY_HEIGHT)
        .reset_pin(Output::new(rst_pin, Level::High))
        .init(&mut delay)
        .unwrap();

    let mut x_touch_prev: i32 = 0;
    let mut y_touch_prev: i32 = 0;

    loop {
        let rand_val = rng.random();
        let background_color = generate_dark_color(rand_val);
        display.clear(background_color).expect("clear failed");

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
                        RgbColor::WHITE,
                    )
                    .expect("set_pixel failed");
                y_touch_prev = touch_event.y;
                x_touch_prev = touch_event.x;
            }
        }

        let is_vbus_present = pmu.is_vbus_in().unwrap();

        pmu.set_charge_enable(!is_vbus_present)
            .expect("set_charge_enable failed");

        let is_vbus_present = match is_vbus_present {
            true => "Yes",
            false => "No",
        };
        let mut text = format!("CHG state: {}\n", pmu.get_charge_status().unwrap());

        text.push_str(&format!("USB PlugIn: {}\n", is_vbus_present));

        text.push_str(&format!("Bus state: {}\n", pmu.get_bus_status().unwrap()));

        text.push_str(&format!(
            "Battery voltage: {}mv\n",
            pmu.get_battery_voltage().unwrap()
        ));

        text.push_str(&format!(
            "USB voltage: {}mv\n",
            pmu.get_vbus_voltage().unwrap()
        ));

        text.push_str(&format!(
            "SYS voltage: {}mv\n",
            pmu.get_sys_voltage().unwrap()
        ));

        text.push_str(&format!(
            "Temperature: {}°C\n",
            pmu.get_temperature().unwrap()
        ));

        text.push_str(&format!(
            "Fast charge current limit: {}mv\n",
            pmu.get_fast_charge_current_limit().unwrap()
        ));

        let text_box = TextBox::with_textbox_style(
            text.as_str(),
            display.bounding_box(),
            TEXT_STYLE,
            TEXT_BOX_STYLE,
        );
        // Draw the text box.
        text_box.draw(&mut display).expect("draw failed");

        delay.delay_ms(10_000);
    }
}

/// Detects the display board model by probing I2C addresses
/// Supports 1.91" SPI and QSPI variants
fn detect_spi_model<I2C>(mut i2c: I2C)
where
    I2C: I2CBus,
{
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

/// Generates a dark RGB565 color from random value
/// Used for background color cycling
fn generate_dark_color(rand_val: u32) -> Rgb565 {
    let r = (rand_val & 0x07) as u8;
    let g = ((rand_val >> 3) & 0x07) as u8;
    let b = ((rand_val >> 6) & 0x03) as u8;
    Rgb565::new(r << 2, g << 2, b << 3) // Scale values for better distribution
}
