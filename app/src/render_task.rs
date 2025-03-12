use alloc::rc::Rc;
use drivers::cst816s::CST816S;
use embassy_time::Timer;
use embedded_hal_bus::{i2c::AtomicDevice, spi::ExclusiveDevice, util::AtomicCell};
use esp_hal::{
    delay::Delay,
    dma::DmaTxBuf,
    dma_buffers,
    gpio::{GpioPin, InputConfig, Level, Output, OutputConfig, Pull},
    i2c::master::I2c,
    peripherals::SPI2,
    spi::{
        master::{Config, Spi, SpiDmaBus},
        Mode,
    },
    time::Rate,
    Blocking,
};
use mipidsi::interface::SpiInterface;
use slint::{
    platform::{
        software_renderer::{MinimalSoftwareWindow, Rgb565Pixel},
        PointerEventButton, WindowEvent,
    },
    LogicalPosition,
};

use crate::{draw_buffer::DrawBuffer, DISPLAY_HEIGHT, DISPLAY_WIDTH};

pub struct RenderTaskPeriphals {
    pub touch_pin: GpioPin<21>,
    pub reset_pin: GpioPin<17>,
    pub dc_pin: GpioPin<7>,
    pub sck_pin: GpioPin<47>,
    pub mosi_pin: GpioPin<18>,
    pub cs_pin: GpioPin<6>,
    pub dma_ch0: esp_hal::dma::DmaChannel0,
    pub spi2: SPI2,
}

#[embassy_executor::task()]
pub async fn render_task(
    window: Rc<MinimalSoftwareWindow>,
    p: RenderTaskPeriphals,
    i2c_ref_cell: &'static AtomicCell<I2c<'static, Blocking>>,
) {
    // Initialize touchpad
    let touch_int =
        esp_hal::gpio::Input::new(p.touch_pin, InputConfig::default().with_pull(Pull::None));
    let mut touchpad = CST816S::new(AtomicDevice::new(i2c_ref_cell), touch_int);

    let rst = Output::new(p.reset_pin, Level::High, OutputConfig::default());

    // Initialize display SPI peripherals
    let sck = Output::new(p.sck_pin, Level::Low, OutputConfig::default());
    let mosi = Output::new(p.mosi_pin, Level::Low, OutputConfig::default());
    let cs = Output::new(p.cs_pin, Level::High, OutputConfig::default());

    // Configure SPI
    let spi = Spi::new(
        p.spi2,
        Config::default()
            .with_frequency(Rate::from_mhz(75))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sck)
    .with_mosi(mosi)
    .with_dma(p.dma_ch0);

    // Configure SPI DMA buffers
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = esp_hal::dma::DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = SpiDmaBus::new(spi, dma_rx_buf, dma_tx_buf);

    // Initialize SPI device
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let mut buffer = [0_u8; 512];
    let di = SpiInterface::new(
        spi_device,
        Output::new(p.dc_pin, Level::Low, OutputConfig::default()),
        &mut buffer,
    );

    // Initialize buffer provider
    let line_buffer = &mut [Rgb565Pixel(0); DISPLAY_WIDTH as usize];
    let mut delay = Delay::new();
    let mut buffer_provider = DrawBuffer::new(di, line_buffer, rst, &mut delay);
    let mut last_touch: Option<slint::LogicalPosition> = None;

    loop {
        // Update timers and animations
        slint::platform::update_timers_and_animations();

        // process touchscreen events
        process_touch(&mut touchpad, &mut last_touch, window.clone());

        // Draw the scene if something needs to be drawn
        let is_dirty = window.draw_if_needed(|renderer| {
            renderer.render_by_line(&mut buffer_provider);
        });

        if !is_dirty {
            Timer::after_millis(10).await
        }
    }
}

fn process_touch(
    touch: &mut CST816S<AtomicDevice<'_, I2c<'_, Blocking>>, esp_hal::gpio::Input<'_>>,
    last_touch: &mut Option<slint::LogicalPosition>,
    window: Rc<MinimalSoftwareWindow>,
) {
    // process touchscreen touch events
    match touch.read_touch(true) {
        Ok(point) => {
            let button = PointerEventButton::Left;
            let event = match point {
                Some(point) => {
                    let position = LogicalPosition::new(
                        DISPLAY_WIDTH as f32 - point.x as f32,
                        DISPLAY_HEIGHT as f32 - point.y as f32,
                    );

                    Some(match last_touch.replace(position) {
                        Some(_) => WindowEvent::PointerMoved { position },
                        None => WindowEvent::PointerPressed { position, button },
                    })
                }
                None => last_touch
                    .take()
                    .map(|position| WindowEvent::PointerReleased { position, button }),
            };

            if let Some(event) = event {
                let is_pointer_release_event = matches!(event, WindowEvent::PointerReleased { .. });
                window.dispatch_event(event);

                // removes hover state on widgets
                if is_pointer_release_event {
                    window.dispatch_event(WindowEvent::PointerExited);
                }
            }
        }
        Err(_) => {
            // ignore as these are expected NotReady messages from the touchscreen
        }
    }
}
