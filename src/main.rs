#![no_std]
#![no_main]
#![feature(async_closure)]

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::string::ToString;
use core::cell::RefCell;
use critical_section::Mutex;
use defmt::{error, info};
use display::{Display, DisplayPeripherals};
use driver::display_bq25896::BQ25896;
use driver::touch_cst816s::CST816S;
use embassy_executor::Spawner;
use embedded_hal::i2c::I2c as I2CBus;
use embedded_hal_bus::i2c::CriticalSectionDevice;
use esp_alloc::psram_allocator;
use esp_hal::delay::Delay;
use esp_hal::i2c::master::I2c;
use esp_hal::prelude::*;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::xtensa_lx::singleton;
use slint::platform::software_renderer::{MinimalSoftwareWindow, RepaintBufferType};
use slint::platform::{Platform, PointerEventButton};
use slint::{LogicalPosition, PhysicalSize};
use {defmt_rtt as _, esp_backtrace as _};

#[macro_use]
extern crate alloc;

mod display;
mod driver;

pub const DISPLAY_HEIGHT: u16 = 240;

/// Display resolution width in pixels
pub const DISPLAY_WIDTH: u16 = 536;

/// I2C address of BQ25896 PMU
const BQ25896_SLAVE_ADDRESS: u8 = 0x6B;

#[main]
async fn main(_spawner: Spawner) -> ! {
    esp_alloc::heap_allocator!(72 * 1024);

    let delay = Delay::new();

    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::Clock240MHz;
        config
    });

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    esp_hal_embassy::init(timg0.timer0);

    psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    // initalize i2c bus
    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .with_sda(peripherals.GPIO3)
        .with_scl(peripherals.GPIO2)
        .into_async();

    let i2c_ref_cell =
        singleton!(:Mutex<RefCell<I2c<'_, esp_hal::Async>>> = Mutex::new(RefCell::new(i2c)))
            .expect("Failed to create I2C mutex");

    // initalize bq25896 charger
    let mut pmu = BQ25896::new(
        CriticalSectionDevice::new(i2c_ref_cell),
        BQ25896_SLAVE_ADDRESS,
    )
    .expect("BQ25896 init failed");

    pmu.set_adc_enabled().expect("set_adc_enabled failed");

    pmu.set_fast_charge_current_limit(2048)
        .expect("set_fast_charge_current_limit failed");

    info!("PMU chip id: {}", pmu.get_chip_id().unwrap());

    // initalize touchpad
    let touch_int = peripherals.GPIO21;

    let mut touchpad = CST816S::new(CriticalSectionDevice::new(i2c_ref_cell), touch_int, delay);

    detect_spi_model(CriticalSectionDevice::new(i2c_ref_cell));

    let display_peripherals = DisplayPeripherals {
        sck: peripherals.GPIO47,
        mosi: peripherals.GPIO18,
        cs: peripherals.GPIO6,
        dc: peripherals.GPIO7,
        rst: peripherals.GPIO17,
        pmicen: peripherals.GPIO38,
        spi: peripherals.SPI2,
    };

    let mut display = Display::new(display_peripherals).expect("Display init failed");

    let window = MinimalSoftwareWindow::new(RepaintBufferType::ReusedBuffer);
    let size = PhysicalSize::new(DISPLAY_WIDTH.into(), DISPLAY_HEIGHT.into());
    window.set_size(size);
    let backend = Backend {
        window: window.clone(),
    };

    slint::platform::set_platform(Box::new(backend)).unwrap();

    let ui = AppWindow::new().expect("UI init failed");

    ui.on_request_update({
        let ui_handle = ui.as_weak();
        move || {
            info!("update pmu readings");

            let ui = ui_handle.unwrap();
            let text = pmu.get_info().unwrap();
            ui.set_text(text.clone().into());
        }
    });

    let mut touch_registered = false;

    loop {
        slint::platform::update_timers_and_animations();

        if !window.has_active_animations() {
            // if no animation is running, wait for the next input event
            if let Some(touch_event) = touchpad.read_touch(true).expect("read touch failed") {
                let position = LogicalPosition::new(
                    DISPLAY_WIDTH as f32 - touch_event.x as f32,
                    DISPLAY_HEIGHT as f32 - touch_event.y as f32,
                );

                //info!("Touch event: {:?}", defmt::Debug2Format(&touch_event));

                if touch_event.points > 0 && !touch_registered {
                    //info!("Touch pressed");
                    window.dispatch_event(slint::platform::WindowEvent::PointerPressed {
                        position,
                        button: PointerEventButton::Left,
                    });
                    touch_registered = true;
                } else if touch_event.points > 0 && touch_registered {
                    //info!("Touch down");
                    window.dispatch_event(slint::platform::WindowEvent::PointerMoved { position });
                } else if touch_event.points == 0 && touch_registered {
                    //info!("Touch released");
                    window.dispatch_event(slint::platform::WindowEvent::PointerReleased {
                        position,
                        button: PointerEventButton::Left,
                    });
                    touch_registered = false;
                }
            }
        }

        // Draw the scene if something needs to be drawn.
        window.draw_if_needed(|renderer| {
            //info!("Drawing scene");
            renderer.render_by_line(&mut display);
        });
    }
}

/// Detects the display board model by probing I2C addresses
/// Supports 1.91" SPI and QSPI variants
fn detect_spi_model<I2C>(mut i2c: I2C)
where
    I2C: I2CBus,
{
    // Try to find 1.91 inch i2c devices
    if i2c.write(0x15, &[]).is_ok() {
        // Check RTC Slave address
        if i2c.write(0x51, &[]).is_ok() {
            info!("Detect 1.91-inch SPI board model!");
        } else {
            info!("Detect 1.91-inch QSPI board model!");
        }
    } else {
        error!("Unable to detect 1.91-inch touch board model!");
    }
}

slint::include_modules!();

struct Backend {
    window: Rc<MinimalSoftwareWindow>,
}

impl Platform for Backend {
    fn create_window_adapter(
        &self,
    ) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        // Since on MCUs, there can be only one window, just return a clone of self.window.
        // We'll also use the same window in the event loop.
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(
            SystemTimer::now() * 1_000 / SystemTimer::ticks_per_second(),
        )
    }

    // fn run_event_loop(&self) -> Result<(), slint::PlatformError>
    fn debug_log(&self, arguments: core::fmt::Arguments) {
        info!("Slint: {}", arguments.to_string().as_str());
    }
}
