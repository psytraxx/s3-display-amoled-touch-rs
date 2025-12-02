use alloc::rc::Rc;
use drivers::cst816x::Event;
use embassy_time::Timer;
use log::error;
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
    // Check if a touch is available
    if !touch
        .is_touch_available()
        .expect("Touch availability check failed")
    {
        return;
    }

    // Read the touch data
    let point = match touch.read_touch().await {
        Ok(point) => point,
        Err(e) => {
            error!("Touch read error: {e:?}");
            return;
        }
    };

    // Ignore spurious events with 0 points (except Up events)
    if point.points == 0 && point.event != Event::Up {
        return;
    }

    // Transform and clamp coordinates to screen bounds
    let x = (DISPLAY_WIDTH as f32 - point.x as f32).clamp(0.0, DISPLAY_WIDTH as f32 - 1.0);
    let y = (DISPLAY_HEIGHT as f32 - point.y as f32).clamp(0.0, DISPLAY_HEIGHT as f32 - 1.0);
    let position = LogicalPosition::new(x, y);

    // Map touch events to Slint pointer events
    let event = match point.event {
        Event::Down => {
            // Clean up any unreleased touch state before starting new gesture
            if let Some(old_pos) = last_touch.replace(position) {
                window.dispatch_event(WindowEvent::PointerReleased {
                    position: old_pos,
                    button: PointerEventButton::Left,
                });
            }
            WindowEvent::PointerPressed {
                position,
                button: PointerEventButton::Left,
            }
        }
        Event::Contact => {
            last_touch.replace(position);
            WindowEvent::PointerMoved { position }
        }
        Event::Up => {
            // Use last tracked position for more reliable release when finger goes off-screen
            let release_pos = last_touch.take().unwrap_or(position);
            WindowEvent::PointerReleased {
                position: release_pos,
                button: PointerEventButton::Left,
            }
        }
    };

    window.dispatch_event(event);
}
