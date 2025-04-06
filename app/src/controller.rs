use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use esp_println::println;
use slint_generated::AppWindow;

use crate::Charger;

pub struct Controller<'a> {
    app_window: &'a AppWindow,
    pmu: Charger,
}

#[derive(Debug, Clone)]
pub enum Action {
    RequestUpdate,
    ToggleCharger(bool),
}

type ActionChannelType = Channel<CriticalSectionRawMutex, Action, 2>;

pub static ACTION: ActionChannelType = Channel::new();

impl<'a> Controller<'a> {
    pub fn new(app_window: &'a AppWindow, pmu: Charger) -> Self {
        Self { app_window, pmu }
    }

    pub async fn run(&mut self) {
        self.set_action_event_handlers();

        loop {
            let action = ACTION.receive().await;
            println!("process action {:?}", &action);
            match self.process_action(action).await {
                Ok(()) => {
                    // all good
                }
                Err(e) => {
                    println!("process action: {:?}", e);
                }
            }
        }
    }

    pub async fn process_action(&mut self, action: Action) -> Result<(), ()> {
        match action {
            Action::RequestUpdate => {
                let text = self.pmu.get_info().expect("failed to get info");
                self.app_window.set_text(text.into());
            }
            Action::ToggleCharger(state) => {
                if state {
                    self.pmu
                        .set_charge_enabled()
                        .expect("set_charge_enable failed");
                } else {
                    self.pmu
                        .set_charge_disabled()
                        .expect("set_charge_disabled failed");
                }

                let status = self
                    .pmu
                    .is_charge_enabled()
                    .expect("is_charge_enabled failed");

                self.app_window.set_charging(!status);
                let text = self.pmu.get_info().expect("failed to get info");
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
            println!("user action queue full, could not add: {:?}", a)
        }
    }
}
