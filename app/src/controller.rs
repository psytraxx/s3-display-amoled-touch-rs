use alloc::string::String;
use defmt::{error, warn};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

use crate::AppWindow;

pub struct Controller<'a, Hardware> {
    app_window: &'a AppWindow,
    hardware: Hardware,
}

#[derive(defmt::Format, Debug, Clone)]
pub enum Action {
    RequestUpdate,
    ToggleCharger(bool),
}

type ActionChannelType = Channel<CriticalSectionRawMutex, Action, 2>;

pub static ACTION: ActionChannelType = Channel::new();

pub trait Hardware {
    fn get_pmu_info(&mut self) -> String;
    fn toggle_pmu_charger(&mut self, state: bool) -> bool;
}

impl<'a, H> Controller<'a, H>
where
    H: Hardware,
{
    pub fn new(app_window: &'a AppWindow, hardware: H) -> Self {
        Self {
            app_window,
            hardware,
        }
    }

    pub async fn run(&mut self) {
        self.set_action_event_handlers();

        loop {
            let action = ACTION.receive().await;

            match self.process_action(action).await {
                Ok(()) => {
                    // all good
                }
                Err(e) => {
                    error!("process action: {:?}", e);
                }
            }
        }
    }

    pub async fn process_action(&mut self, action: Action) -> Result<(), ()> {
        match action {
            Action::RequestUpdate => {
                let text = self.hardware.get_pmu_info();
                self.app_window.set_text(text.into());
            }
            Action::ToggleCharger(state) => {
                let status = self.hardware.toggle_pmu_charger(state);
                self.app_window.set_charging(!status);
                let text = self.hardware.get_pmu_info();
                self.app_window.set_text(text.into());
            }
        }
        Ok(())
    }

    // user initiated action event handlers
    fn set_action_event_handlers(&self) {
        self.app_window
            .on_toggle_charger(|state| send_action(Action::ToggleCharger(state)));
        self.app_window
            .on_request_update(|| send_action(Action::RequestUpdate));
    }
}

pub fn send_action(a: Action) {
    // use non-blocking try_send here because this function needs is called from sync code (the gui callbacks)
    match ACTION.try_send(a) {
        Ok(_) => {
            // see loop in `fn run()` for dequeue
        }
        Err(a) => {
            // this could happen because the controller is slow to respond or we are making too many requests
            warn!("user action queue full, could not add: {:?}", a)
        }
    }
}
