use embedded_graphics_core::pixelcolor::raw::RawU16;
use embedded_hal::digital::OutputPin;
use mipidsi::interface::{Interface, InterfacePixelFormat};
use mipidsi::models::Model;
use mipidsi::Display as MipiDisplay;
use s3_display_amoled_touch_drivers::rm67162::RM67162;
use slint::platform::software_renderer::{LineBufferProvider, Rgb565Pixel};

pub struct DrawBuffer<'a, DI, MODEL, RST>
where
    DI: Interface<Word = u8>,
    MODEL: Model,
    MODEL::ColorFormat: InterfacePixelFormat<DI::Word>,
    RST: OutputPin,
{
    pub display: MipiDisplay<DI, MODEL, RST>,
    pub line_buffer: &'a mut [Rgb565Pixel],
}

impl<DI, RST> LineBufferProvider for &mut DrawBuffer<'_, DI, RM67162, RST>
where
    DI: Interface<Word = u8>,
    RST: OutputPin,
{
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
            .expect("set_pixels failed");
    }
}
