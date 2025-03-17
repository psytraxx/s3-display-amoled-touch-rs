use alloc::rc::Rc;
use defmt::error;
use embassy_time::Timer;
use slint::{
    platform::{
        software_renderer::{MinimalSoftwareWindow, Rgb565Pixel},
        PointerEventButton, WindowEvent,
    },
    LogicalPosition,
};

use crate::{
    display_line_buffer::DisplayLineBuffer, TouchDisplay, Touchpad, DISPLAY_HEIGHT, DISPLAY_WIDTH,
};

#[embassy_executor::task()]
pub async fn render_task(
    window: Rc<MinimalSoftwareWindow>,
    display: TouchDisplay,
    mut touchpad: Touchpad,
) {
    // Initialize buffer provider
    let line_buffer = &mut [Rgb565Pixel(0); DISPLAY_WIDTH as usize];

    let mut buffer_provider = DisplayLineBuffer::new(display, line_buffer);
    let mut last_touch: Option<LogicalPosition> = None;

    loop {
        // Update timers and animations
        slint::platform::update_timers_and_animations();

        // process touchscreen events
        process_touch(&mut touchpad, &mut last_touch, window.clone()).await;

        // Draw the scene if something needs to be drawn
        let is_dirty = window.draw_if_needed(|renderer| {
            renderer.render_by_line(&mut buffer_provider);
        });

        if !is_dirty {
            Timer::after_millis(10).await
        }
    }
}

async fn process_touch(
    touch: &mut Touchpad,
    last_touch: &mut Option<LogicalPosition>,
    window: Rc<MinimalSoftwareWindow>,
) {
    match touch.read_touch(true).await {
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
