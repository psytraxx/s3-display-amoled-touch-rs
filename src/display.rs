use crate::driver::rm67162_pmu_driver::RM67162;
use crate::{DISPLAY_HEIGHT, DISPLAY_WIDTH};

use core::convert::Infallible;
use defmt::info;
use display_interface::DisplayError;
use embedded_graphics_core::pixelcolor::raw::RawU16;
use esp_hal::delay::Delay;
use esp_hal::gpio::{GpioPin, Level, Output};
use esp_hal::peripherals::SPI2;
use esp_hal::prelude::*;
use esp_hal::spi::master::{Config, Spi};
use esp_hal::spi::{SpiBitOrder, SpiMode};
use mipidsi::error::InitError;
use mipidsi::options::Orientation;
use mipidsi::{Builder, Display as MipiDisplay};
use slint::platform::software_renderer::{LineBufferProvider, Rgb565Pixel};

pub type MipiDisplayWrapper<'a> = MipiDisplay<
    display_interface_spi::SPIInterface<
        embedded_hal_bus::spi::ExclusiveDevice<
            Spi<'a, esp_hal::Blocking>,
            Output<'a>,
            embedded_hal_bus::spi::NoDelay,
        >,
        Output<'a>,
    >,
    RM67162,
    Output<'a>,
>;

pub struct Display<'a> {
    display: MipiDisplayWrapper<'a>,
    line_buffer: [Rgb565Pixel; DISPLAY_WIDTH as usize],
}

pub struct DisplayPeripherals {
    pub sck: GpioPin<47>,
    pub mosi: GpioPin<18>,
    pub cs: GpioPin<6>,
    pub pmicen: GpioPin<38>,
    pub dc: GpioPin<7>,
    pub rst: GpioPin<17>,
    pub spi: SPI2,
}

impl<'a> Display<'a> {
    pub fn new(p: DisplayPeripherals) -> Result<Self, Error> {
        // SPI pins
        let sck = Output::new(p.sck, Level::Low);
        let mosi = Output::new(p.mosi, Level::Low);
        let cs = Output::new(p.cs, Level::High);

        let mut pmicen = Output::new_typed(p.pmicen, Level::Low);
        pmicen.set_high();
        info!("PMICEN set high");

        // Configure SPI
        let spi_bus = Spi::new_with_config(
            p.spi,
            Config {
                frequency: 80.MHz(),
                mode: SpiMode::Mode0,
                write_bit_order: SpiBitOrder::MSBFirst,
                read_bit_order: SpiBitOrder::MSBFirst,
            },
        )
        .with_sck(sck)
        .with_mosi(mosi);

        let dc_pin = p.dc;
        let rst_pin = p.rst;

        let spi_bus = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi_bus, cs).unwrap();

        let di = display_interface_spi::SPIInterface::new(spi_bus, Output::new(dc_pin, Level::Low));

        let mut delay = Delay::new();

        let display = Builder::new(RM67162, di)
            .orientation(Orientation {
                mirrored: false,
                rotation: mipidsi::options::Rotation::Deg90,
            })
            .display_size(DISPLAY_WIDTH, DISPLAY_HEIGHT)
            .reset_pin(Output::new(rst_pin, Level::High))
            .init(&mut delay)
            .unwrap();

        let line_buffer = [Rgb565Pixel(100); DISPLAY_WIDTH as usize];

        Ok(Self {
            display,
            line_buffer,
        })
    }
}

impl<'a> LineBufferProvider for &mut Display<'a> {
    type TargetPixel = Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        let buffer = &mut self.line_buffer[range.clone()];

        render_fn(buffer);

        // We send empty data just to get the device in the right window
        self.display
            .set_pixels(
                range.start as u16,
                line as _,
                range.end as u16,
                line as u16,
                buffer.iter().map(|x| RawU16::new(x.0).into()),
            )
            .unwrap();
    }
}
/// A clock error
#[derive(Debug)]
pub enum Error {
    DisplayInterface(DisplayError),
    Infallible,
}

impl From<DisplayError> for Error {
    fn from(error: DisplayError) -> Self {
        Self::DisplayInterface(error)
    }
}

impl From<InitError<Infallible>> for Error {
    fn from(_: InitError<Infallible>) -> Self {
        Self::Infallible
    }
}
impl From<Infallible> for Error {
    fn from(_: Infallible) -> Self {
        Self::Infallible
    }
}
