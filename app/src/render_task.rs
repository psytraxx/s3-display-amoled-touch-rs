use alloc::rc::Rc;
use drivers::cst816x::Event;
use embassy_time::Timer;
use esp_println::println;
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
    match touch.read_touch() {
        Ok(point) => {
            // Ignore events with 0 points unless it's an 'Up' event
            if point.points == 0 && point.event != Event::Up {
                // Potentially spurious event, ignore or log if needed
                println!("Ignoring touch event with 0 points: {:?}", point);
                return;
            }
            let button = PointerEventButton::Left;
            // Use the coordinates from the touch data
            let position = LogicalPosition::new(
                DISPLAY_WIDTH as f32 - point.x as f32,
                DISPLAY_HEIGHT as f32 - point.y as f32,
            );

            let event = match point.event {
                Event::Down => {
                    // Only send Pressed if not already down
                    if last_touch.is_none() {
                        last_touch.replace(position);
                        Some(WindowEvent::PointerPressed { position, button })
                    } else {
                        // Already pressed, treat as move? Or ignore? Let's treat as move.
                        last_touch.replace(position);
                        Some(WindowEvent::PointerMoved { position })
                    }
                }
                Event::Contact => {
                    // Send Moved only if currently pressed
                    if last_touch.is_some() {
                        last_touch.replace(position);
                        Some(WindowEvent::PointerMoved { position })
                    } else {
                        None
                    }
                }
                Event::Up => {
                    // Send Released only if currently pressed
                    if last_touch.take().is_some() {
                        // Use the last known down position for release? Or current point's position?
                        // Slint usually expects the position where the release occurred.
                        Some(WindowEvent::PointerReleased { position, button })
                    } else {
                        // Up event without prior Down? Ignore.
                        Some(WindowEvent::PointerExited)
                    }
                }
            };

            // Dispatch the determined event, if any
            if let Some(evt) = event {
                println!("Dispatching Slint event: {:?}", evt);
                window
                    .try_dispatch_event(evt)
                    .expect("Event dispatch failed");
            }
        }
        Err(e) => {
            println!("Touch read error: {:?}", e);
        }
    }
}
