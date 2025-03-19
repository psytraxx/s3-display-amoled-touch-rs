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
    // Check if a touch is available
    if !touch
        .is_touch_available()
        .expect("Touch availability check failed")
    {
        *last_touch = None;
        return;
    }
    // Read the touch data
    match touch.read_touch().await {
        Ok(point) => {
            let button = PointerEventButton::Left;
            let position = LogicalPosition::new(
                DISPLAY_WIDTH as f32 - point.x as f32,
                DISPLAY_HEIGHT as f32 - point.y as f32,
            );
            // Determine event based on whether we had a previous touch
            let event = if last_touch.is_some() {
                WindowEvent::PointerMoved { position }
            } else {
                WindowEvent::PointerPressed { position, button }
            };
            // Update last_touch and dispatch events
            last_touch.replace(position);
            window
                .try_dispatch_event(event)
                .expect("Event dispatch failed");
        }
        Err(e) => {
            error!("Touch read error: {:?}", e);
        }
    }

    if let Some(position) = last_touch.take() {
        let button = PointerEventButton::Left;
        window
            .try_dispatch_event(WindowEvent::PointerReleased { position, button })
            .expect("Event dispatch failed");
        window
            .try_dispatch_event(WindowEvent::PointerExited)
            .expect("Event dispatch failed");
    }
}
