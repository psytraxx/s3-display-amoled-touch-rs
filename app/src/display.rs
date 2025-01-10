use crate::DISPLAY_WIDTH;

use core::convert::Infallible;
use defmt::info;
use embedded_graphics_core::pixelcolor::raw::RawU16;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::delay::Delay;
use esp_hal::dma::{Dma, DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::{GpioPin, Level, Output};
use esp_hal::peripherals::{DMA, SPI2};
use esp_hal::spi::master::{Config, Spi, SpiDmaBus};
use esp_hal::{dma_buffers, prelude::*};
use mipidsi::interface::SpiInterface;
use mipidsi::options::{Orientation, Rotation};
use mipidsi::Builder;
use mipidsi::Display as MipiDisplay;
use rm67162::RM67162;
use s3_display_amoled_touch_drivers::rm67162;
use slint::platform::software_renderer::{LineBufferProvider, Rgb565Pixel};

pub type MipiDisplayWrapper<'a> = MipiDisplay<
    SpiInterface<
        'a,
        ExclusiveDevice<
            SpiDmaBus<'a, esp_hal::Blocking>,
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
    pub dma: DMA,
}

impl<'a> Display<'a> {
    pub fn new(p: DisplayPeripherals, buffer: &'a mut [u8]) -> Result<Self, Error> {
        // SPI pins
        let sck = Output::new(p.sck, Level::Low);
        let mosi = Output::new(p.mosi, Level::Low);
        let cs = Output::new(p.cs, Level::High);

        let mut pmicen = Output::new_typed(p.pmicen, Level::Low);
        pmicen.set_high();
        info!("PMICEN set high");

        let dma = Dma::new(p.dma);

        // Configure SPI
        let spi = Spi::new_with_config(
            p.spi,
            Config {
                frequency: 75.MHz(),
                ..Config::default()
            },
        )
        .with_sck(sck)
        .with_mosi(mosi)
        .with_dma(
            dma.channel0
                .configure(false, esp_hal::dma::DmaPriority::Priority0),
        );

        let dc_pin = p.dc;
        let rst_pin = p.rst;
        let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
        let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
        let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

        let spi = SpiDmaBus::new(spi, dma_rx_buf, dma_tx_buf);

        let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();

        let di = SpiInterface::new(spi_device, Output::new(dc_pin, Level::Low), buffer);

        let mut delay = Delay::new();

        let display = Builder::new(RM67162, di)
            .orientation(Orientation {
                mirrored: false,
                rotation: Rotation::Deg270,
            })
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
    DisplayInterface,
    Infallible,
}

impl From<Infallible> for Error {
    fn from(_: Infallible) -> Self {
        Self::Infallible
    }
}
