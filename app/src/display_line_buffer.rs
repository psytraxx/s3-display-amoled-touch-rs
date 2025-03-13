use embedded_graphics_core::pixelcolor::raw::RawU16;
use embedded_hal::digital::OutputPin;
use mipidsi::interface::{Interface, InterfacePixelFormat};
use mipidsi::models::{Model, RM67162};
use mipidsi::Display;
use slint::platform::software_renderer::{LineBufferProvider, Rgb565Pixel};

pub struct DisplayLineBuffer<'a, DI, MODEL, RST>
where
    DI: Interface<Word = u8>,
    MODEL: Model,
    MODEL::ColorFormat: InterfacePixelFormat<DI::Word>,
    RST: OutputPin,
{
    pub display: Display<DI, MODEL, RST>,
    pub line_buffer: &'a mut [Rgb565Pixel],
}

impl<DI, RST> DisplayLineBuffer<'_, DI, RM67162, RST>
where
    DI: Interface<Word = u8>,
    RST: OutputPin,
{
    pub fn new(
        display: Display<DI, RM67162, RST>,
        line_buffer: &mut [Rgb565Pixel],
    ) -> DisplayLineBuffer<'_, DI, RM67162, RST> {
        DisplayLineBuffer {
            display,
            line_buffer,
        }
    }
}

impl<DI, RST> LineBufferProvider for &mut DisplayLineBuffer<'_, DI, RM67162, RST>
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
