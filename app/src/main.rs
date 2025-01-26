#![no_std]
#![no_main]
#![feature(async_closure)]

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::string::ToString;
use core::cell::RefCell;
use core::time::Duration;
use critical_section::Mutex;
use defmt::{error, info};
use draw_buffer::DrawBuffer;
use embedded_hal::i2c::I2c as I2CBus;
use embedded_hal_bus::i2c::CriticalSectionDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_alloc::psram_allocator;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::gpio::{Input, Level, Output, Pull};
use esp_hal::i2c::master::I2c;
use esp_hal::spi::master::{Config, Spi, SpiDmaBus};
use esp_hal::spi::Mode;
use esp_hal::time::now;
use esp_hal::time::RateExtU32;
use esp_hal::xtensa_lx::singleton;
use esp_hal::{dma_buffers, main};
use mipidsi::interface::SpiInterface;
use s3_display_amoled_touch_drivers::bq25896::ChargeStatus;
use s3_display_amoled_touch_drivers::{bq25896::BQ25896, cst816s::CST816S};
use slint::platform::software_renderer::{MinimalSoftwareWindow, RepaintBufferType, Rgb565Pixel};
use slint::platform::{Platform, PointerEventButton};
use slint::{LogicalPosition, PhysicalSize};
use {defmt_rtt as _, esp_backtrace as _};

slint::include_modules!();

extern crate alloc;

mod draw_buffer;

pub const DISPLAY_HEIGHT: u16 = 240;

/// Display resolution width in pixels
pub const DISPLAY_WIDTH: u16 = 536;

/// I2C address of BQ25896 PMU
const BQ25896_SLAVE_ADDRESS: u8 = 0x6B;

// credits to
// https://github.com/slint-ui/slint/blob/master/examples/mcu-board-support/esp32_s3_box.rs and
// https://github.com/Yandrik/kolibri-cyd-evaluation-apps/blob/b089aef31cc64e294bd64b2032356fb70789ad65/app/src/bin/exapp-slint-timer.rs#L56
// for the initial code

#[main]
fn main() -> ! {
    esp_alloc::heap_allocator!(72 * 1024);

    // Initialize delay
    let mut delay = Delay::new();

    // Initialize peripherals
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::_240MHz;
        config
    });

    // Initialize PSRAM allocator
    psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    // Initialize I2C bus
    let i2c = I2c::new(peripherals.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO3)
        .with_scl(peripherals.GPIO2);

    let i2c_ref_cell =
        singleton!(:Mutex<RefCell<I2c<'_, esp_hal::Blocking>>> = Mutex::new(RefCell::new(i2c)))
            .expect("Failed to create I2C mutex");

    // Initialize BQ25896 charger
    let mut pmu = BQ25896::new(
        CriticalSectionDevice::new(i2c_ref_cell),
        BQ25896_SLAVE_ADDRESS,
    )
    .expect("BQ25896 init failed");

    // Set the charging target voltage, Range:3840 ~ 4608mV ,step:16 mV
    pmu.set_charge_target_voltage(4208)
        .expect("set_charge_target_voltage failed");

    let charge_target_voltage = pmu
        .get_charge_target_voltage()
        .expect("get_charge_target_voltage failed");
    info!("Charge target voltage: {}", charge_target_voltage);

    // Set the precharge current , Range: 64mA ~ 1024mA ,step:64mA
    pmu.set_precharge_current(128)
        .expect("set_precharge_current_limit failed");

    let precharge_current = pmu
        .get_precharge_current()
        .expect("get_precharge_current failed");

    info!("Precharge current: {}", precharge_current);

    // The premise is that Limit Pin is disabled, or it will only follow the maximum charging current set by Limit Pin.
    // Set the charging current , Range:0~5056mA ,step:64mA
    pmu.set_charger_constantcurr(1536)
        .expect("set_charger_constantcurr failed");

    let constantcurr = pmu
        .get_charger_constantcurr()
        .expect("get_charger_constantcurr failed");

    info!("Charger constant current: {}", constantcurr);

    pmu.set_adc_enabled().expect("set_adc_enabled failed");

    pmu.set_fast_charge_current_limit(1024)
        .expect("set_fast_charge_current_limit failed");

    let fast_charge_current_limit = pmu
        .get_fast_charge_current_limit()
        .expect("get_fast_charge_current_limit failed");

    info!("Fast charge current limit: {}", fast_charge_current_limit);

    info!("PMU chip id: {}", pmu.get_chip_id().unwrap());

    // Initialize touchpad
    let touch_int = peripherals.GPIO21;
    let touch_int = Input::new(touch_int, Pull::None);
    let mut touchpad = CST816S::new(CriticalSectionDevice::new(i2c_ref_cell), touch_int, delay);

    // Detect SPI model
    detect_spi_model(CriticalSectionDevice::new(i2c_ref_cell));

    // Initialize PMICEN pin
    let mut pmicen = Output::new(peripherals.GPIO38, Level::Low);
    pmicen.set_high();
    info!("PMICEN set high");

    // Initialize display SPI peripherals
    let sck = Output::new(peripherals.GPIO47, Level::Low);
    let mosi = Output::new(peripherals.GPIO18, Level::Low);
    let cs = Output::new(peripherals.GPIO6, Level::High);

    // Configure SPI
    let spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(75_u32.MHz())
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(sck)
    .with_mosi(mosi)
    .with_dma(peripherals.DMA_CH0);

    // Configure SPI DMA buffers
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    let spi = SpiDmaBus::new(spi, dma_rx_buf, dma_tx_buf);

    // Initialize SPI device
    let spi_device = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let mut buffer = [0_u8; 512];
    let di = SpiInterface::new(
        spi_device,
        Output::new(peripherals.GPIO7, Level::Low),
        &mut buffer,
    );

    let rst = Output::new(peripherals.GPIO17, Level::High);

    // Initialize buffer provider
    let line_buffer = &mut [Rgb565Pixel(0); DISPLAY_WIDTH as usize];
    let mut buffer_provider = DrawBuffer::new(di, line_buffer, rst, &mut delay);

    let window = MinimalSoftwareWindow::new(RepaintBufferType::ReusedBuffer);
    let size = PhysicalSize::new(DISPLAY_WIDTH.into(), DISPLAY_HEIGHT.into());
    window.set_size(size);

    // Set the platform for Slint
    let backend = Box::new(Backend {
        window: window.clone(),
    });
    slint::platform::set_platform(backend).expect("set_platform failed");

    // Initialize UI
    let ui = AppWindow::new().expect("UI init failed");
    // Set up UI update callback
    ui.on_request_update({
        let ui_handle = ui.as_weak();
        move || {
            info!("update pmu readings");

            let not_charging = pmu.get_charge_status().expect("get_charge_status failed")
                == ChargeStatus::NoCharge;
            if not_charging {
                pmu.set_charge_enable().expect("set_charge_enable failed");
            } else {
                pmu.set_charge_disabled()
                    .expect("set_charge_disabled failed");
            }

            let ui = ui_handle.unwrap();
            let text = pmu.get_info().expect("get_info failed");
            ui.set_text(text.clone().into());
        }
    });

    let mut touch_registered = false;

    loop {
        // Update timers and animations
        slint::platform::update_timers_and_animations();

        if window.has_active_animations() {
            continue;
        }

        // Read touch events
        if let Some(touch_event) = touchpad.read_touch(true).expect("read touch failed") {
            let position = LogicalPosition::new(
                DISPLAY_WIDTH as f32 - touch_event.x as f32,
                DISPLAY_HEIGHT as f32 - touch_event.y as f32,
            );

            // Handle touch events
            if touch_event.points > 0 && !touch_registered {
                window.dispatch_event(slint::platform::WindowEvent::PointerPressed {
                    position,
                    button: PointerEventButton::Left,
                });
                touch_registered = true;
            } else if touch_event.points > 0 && touch_registered {
                window.dispatch_event(slint::platform::WindowEvent::PointerMoved { position });
            } else if touch_event.points == 0 && touch_registered {
                window.dispatch_event(slint::platform::WindowEvent::PointerReleased {
                    position,
                    button: PointerEventButton::Left,
                });
                touch_registered = false;
            }
        }

        // Draw the scene if something needs to be drawn
        window.draw_if_needed(|renderer| {
            renderer.render_by_line(&mut buffer_provider);
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
        // Calculate duration since start
        Duration::from_millis(now().duration_since_epoch().to_millis())
    }

    fn debug_log(&self, arguments: core::fmt::Arguments) {
        // Log debug messages
        info!("Slint: {}", arguments.to_string().as_str());
    }
}
