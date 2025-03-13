use alloc::{boxed::Box, rc::Rc};
use defmt::error;
use drivers::cst816s::TouchInput;
use embassy_time::Timer;
use slint::{
    platform::{
        software_renderer::{MinimalSoftwareWindow, Rgb565Pixel},
        PointerEventButton, WindowEvent,
    },
    LogicalPosition,
};

use crate::{display_line_buffer::DisplayLineBuffer, MipiDisplay, DISPLAY_HEIGHT, DISPLAY_WIDTH};

#[embassy_executor::task()]
pub async fn render_task(
    window: Rc<MinimalSoftwareWindow>,
    display: MipiDisplay,
    mut touchpad: Box<dyn TouchInput>,
) {
    // Initialize buffer provider
    let line_buffer = &mut [Rgb565Pixel(0); DISPLAY_WIDTH as usize];

    let mut buffer_provider = DisplayLineBuffer::new(display, line_buffer);
    let mut last_touch: Option<LogicalPosition> = None;

    loop {
        // Update timers and animations
        slint::platform::update_timers_and_animations();

        // process touchscreen events
        process_touch(touchpad.as_mut(), &mut last_touch, window.clone());

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
    touch: &mut dyn TouchInput,
    last_touch: &mut Option<LogicalPosition>,
    window: Rc<MinimalSoftwareWindow>,
) {
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
        Err(e) => {
            error!("Touch read error: {:?}", e);
        }
    }
}
