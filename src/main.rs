#![no_std]
#![no_main]
#![feature(async_closure)]

use core::cell::RefCell;

use defmt::{error, info};
use display_interface_spi::SPIInterface;
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::NoopMutex;
use embedded_graphics_core::prelude::RgbColor;
use esp_alloc::psram_allocator;
use esp_hal::gpio::{Input, Level, Output};
use esp_hal::i2c::master::I2c;
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

    /*static const  BoardsConfigure_t BOARD_AMOLED_191_SPI = {
        // RM67162 Driver
        RM67162_AMOLED_SPI,
        &AMOLED_191_TOUCH_PINS,     //Touch CST816T
        &AMOLED_191_SPI_PMU_PINS,   //PMU
        NULL,                       //SENSOR
        &AMOLED_191_SPI_SD_PINS,    //SDCard
        AMOLED_191_BUTTONTS,        //Button Pins
        1, //Button Number
        -1,//pixelsPins
        4, //adcPins
        38,//PMICEnPins
        false,//framebuffer
    }; */

    /*// LILYGO 1.91 Inch AMOLED(RM67162) S3R8
    // https://www.lilygo.cc/products/t-display-s3-amoled
    static const DisplayConfigure_t RM67162_AMOLED_SPI  = {
        18,//BOARD_DISP_DATA0,          //MOSI
        7,//BOARD_DISP_DATA1,           //DC
        -1,//BOARD_DISP_DATA2,
        -1,//BOARD_DISP_DATA3,
        47,//BOARD_DISP_SCK,            //SCK
        6,//BOARD_DISP_CS,              //CS
        BOARD_NONE_PIN,//DC
        17,//BOARD_DISP_RESET,          //RST
        9, //BOARD_DISP_TE,
        8, //command bit
        24,//address bit
        40000000,
        (lcd_cmd_t *)rm67162_spi_cmd,
        RM67162_INIT_SPI_SEQUENCE_LENGTH,
        RM67162_WIDTH,//width
        RM67162_HEIGHT,//height
        0,//frameBufferSize
        false //fullRefresh
    }; */

    /*typedef struct __DisplayConfigure {
        int d0;
        int d1;
        int d2;
        int d3;
        int sck;
        int cs;
        int dc;
        int rst;
        int te;
        uint8_t cmdBit;
        uint8_t addBit;
        int  freq;
        lcd_cmd_t *initSequence;
        uint32_t initSize;
        uint16_t width;
        uint16_t height;
        uint32_t frameBufferSize;
        bool fullRefresh;
    } DisplayConfigure_t; */

    /*static const  BoardsConfigure_t BOARD_AMOLED_191_SPI = {
        // RM67162 Driver
        RM67162_AMOLED_SPI,
        &AMOLED_191_TOUCH_PINS,     //Touch CST816T
        &AMOLED_191_SPI_PMU_PINS,   //PMU
        NULL,                       //SENSOR
        &AMOLED_191_SPI_SD_PINS,    //SDCard
        AMOLED_191_BUTTONTS,        //Button Pins
        1, //Button Number
        -1,//pixelsPins
        4, //adcPins
        38,//PMICEnPins
        false,//framebuffer
    }; */

    let delay = Delay::new();

    // SPI pins
    let sck = Output::new(peripherals.GPIO47, Level::Low);
    let mosi = Output::new(peripherals.GPIO18, Level::Low);
    let cs = Output::new(peripherals.GPIO6, Level::High);

    let _te = Input::new(peripherals.GPIO9, esp_hal::gpio::Pull::Down);

    let mut pmicen = Output::new_typed(peripherals.GPIO38, Level::Low);
    pmicen.set_high();
    info!("PMICEN set high");

    // Configure SPI
    let spi = Spi::new_with_config(
        peripherals.SPI2,
        Config {
            frequency: 40.MHz(), //TODO: 40MHz
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

    let spi = SpiDevice::new(spi_bus, cs);

    // Initialize the display
    let di = SPIInterface::new(spi, Output::new(dc_pin, Level::Low));

    let mut display = Builder::new(RM67162, di)
        .orientation(Orientation::default())
        .display_size(240, 536)
        .reset_pin(Output::new(rst_pin, Level::High))
        .init(&mut embassy_time::Delay)
        .unwrap();

    loop {
        delay.delay(100.micros());
        let x = rng.random() % 240;
        let y = rng.random() % 536;
        display
            .set_pixel(x as u16, y as u16, RgbColor::MAGENTA)
            .expect("set_pixel failed");
    }
}
