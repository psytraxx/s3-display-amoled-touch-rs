//! Display hardware initialization module
//!
//! This module handles the initialization of the RM67162 display controller
//! via SPI interface with DMA support for high-speed data transfer.

use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use esp_hal::dma::DmaTxBuf;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::peripherals::{DMA_CH0, GPIO17, GPIO18, GPIO47, GPIO6, GPIO7, SPI2};
use esp_hal::spi::master::{Config as SpiConfig, Spi, SpiDmaBus};
use esp_hal::spi::Mode;
use esp_hal::time::Rate;
use esp_hal::{dma_buffers, Blocking};
use mipidsi::interface::SpiInterface;
use mipidsi::models::RM67162;
use mipidsi::options::{Orientation, Rotation};
use mipidsi::{Builder, Display};
use static_cell::StaticCell;

/// Display dimensions
pub const DISPLAY_HEIGHT: u16 = 240;
pub const DISPLAY_WIDTH: u16 = 536;

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

/// Initializes the RM67162 display with SPI interface and DMA support.
///
/// This function configures:
/// - GPIO pins for display control (DC, CS, reset, SCK, MOSI)
/// - SPI bus with DMA at 75MHz
/// - Display driver with 270-degree rotation
///
/// # Arguments
///
/// * `reset` - GPIO pin for display reset
/// * `dc` - GPIO pin for data/command selection
/// * `sck` - GPIO pin for SPI clock
/// * `mosi` - GPIO pin for SPI MOSI (master out, slave in)
/// * `cs` - GPIO pin for chip select
/// * `spi` - SPI2 peripheral instance
/// * `dma` - DMA channel 0 for high-speed transfers
///
/// # Returns
///
/// Returns an instance of the RM67162 display driver.
///
/// # Panics
///
/// Panics if display initialization fails.
pub fn initialize_display(
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
