use alloc::rc::Rc;
use defmt::info;
use embassy_time::Instant;
use slint::{
    platform::{software_renderer::MinimalSoftwareWindow, Platform, WindowAdapter},
    PlatformError,
};

pub struct Backend {
    window: Rc<MinimalSoftwareWindow>,
}

impl Backend {
    pub fn new(window: Rc<MinimalSoftwareWindow>) -> Self {
        Self { window }
    }
}

impl Platform for Backend {
    fn create_window_adapter(&self) -> Result<Rc<dyn WindowAdapter>, PlatformError> {
        let window = self.window.clone();
        info!("Creating window adapter");
        Ok(window)
    }

    fn duration_since_start(&self) -> core::time::Duration {
        Instant::now().duration_since(Instant::from_secs(0)).into()
    }
}
