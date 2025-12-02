use alloc::rc::Rc;
use drivers::cst816x::{Event, Gesture};
use embassy_time::Timer;
use log::{error, info};
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
    match touch.read_touch().await {
        Ok(point) => {
            if point.gesture != Gesture::None {
                info!("Gesture detected: {:?}", point.gesture);
            }

            // Ignore events with 0 points unless it's an 'Up' event
            if point.points == 0 && point.event != Event::Up {
                // Potentially spurious event, ignore or log if needed
                info!("Ignoring touch event with 0 points: {point:?}");
                return;
            }
            let button = PointerEventButton::Left;

            // Transform and clamp coordinates to valid screen bounds
            let x = (DISPLAY_WIDTH as f32 - point.x as f32).clamp(0.0, DISPLAY_WIDTH as f32 - 1.0);
            let y = (DISPLAY_HEIGHT as f32 - point.y as f32).clamp(0.0, DISPLAY_HEIGHT as f32 - 1.0);
            let position = LogicalPosition::new(x, y);

            let event = match point.event {
                Event::Down => {
                    // On new Down event, always send Pressed to start a new gesture
                    // This ensures Slint's Flickable knows a new drag is starting
                    if last_touch.is_some() {
                        // We had a previous touch that wasn't properly released
                        // Send an explicit release first to clean up state
                        info!("Found unreleased touch on new Down, cleaning up");
                        if let Some(old_pos) = last_touch.replace(position) {
                            // First release the old touch
                            window
                                .try_dispatch_event(WindowEvent::PointerReleased {
                                    position: old_pos,
                                    button,
                                })
                                .ok();
                        }
                    } else {
                        last_touch.replace(position);
                    }
                    Some(WindowEvent::PointerPressed { position, button })
                }
                Event::Contact => {
                    // Send Moved only if currently pressed
                    if last_touch.is_some() {
                        last_touch.replace(position);
                        Some(WindowEvent::PointerMoved { position })
                    } else {
                        // Got Contact without Down - treat as implicit down
                        info!("Contact without Down, treating as implicit press");
                        last_touch.replace(position);
                        Some(WindowEvent::PointerPressed { position, button })
                    }
                }
                Event::Up => {
                    // Send Released using the last valid position we tracked
                    if let Some(last_pos) = last_touch.take() {
                        // Use the last tracked position for more reliable release
                        // This helps when finger slides off screen during drag
                        Some(WindowEvent::PointerReleased { position: last_pos, button })
                    } else {
                        // Up event without prior Down - send exit
                        info!("Up without Down, sending exit");
                        Some(WindowEvent::PointerExited)
                    }
                }
            };

            // Dispatch the determined event, if any
            if let Some(evt) = event {
                info!("Dispatching Slint event: {evt:?}");
                window
                    .try_dispatch_event(evt)
                    .expect("Event dispatch failed");
            }
        }
        Err(e) => {
            error!("Touch read error: {e:?}");
        }
    }
}
