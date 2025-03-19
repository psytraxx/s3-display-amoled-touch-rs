use embedded_graphics_core::pixelcolor::raw::RawU16;
use slint::platform::software_renderer::{LineBufferProvider, Rgb565Pixel};

use crate::TouchDisplay;

pub struct DisplayLineBuffer<'a> {
    pub display: TouchDisplay,
    pub line_buffer: &'a mut [Rgb565Pixel],
}

impl DisplayLineBuffer<'_> {
    pub fn new(display: TouchDisplay, line_buffer: &mut [Rgb565Pixel]) -> DisplayLineBuffer<'_> {
        DisplayLineBuffer {
            display,
            line_buffer,
        }
    }
}

impl LineBufferProvider for &mut DisplayLineBuffer<'_> {
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
