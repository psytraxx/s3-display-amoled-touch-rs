use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use log::{error, info};
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
            info!("process action {:?}", &action);
            match self.process_action(action).await {
                Ok(()) => {
                    // all good
                }
                Err(e) => {
                    error!("process action: {e:?}");
                }
            }
        }
    }

    pub async fn process_action(&mut self, action: Action) -> Result<(), ()> {
        match action {
            Action::RequestUpdate => match self.pmu.get_info().await {
                Ok(text) => {
                    self.app_window.set_text(text.into());
                }
                Err(e) => {
                    error!("Failed to get PMU info: {e:?}");
                    self.app_window
                        .set_text("Error: Failed to read PMU data\nCheck I2C connection".into());
                    return Err(());
                }
            },
            Action::ToggleCharger(state) => {
                let result = if state {
                    self.pmu.set_charge_enabled().await
                } else {
                    self.pmu.set_charge_disabled().await
                };

                match result {
                    Ok(_) => {
                        info!("Charger state changed to: {state}");
                        // Update the display with new PMU info
                        match self.pmu.get_info().await {
                            Ok(text) => {
                                self.app_window.set_text(text.into());
                            }
                            Err(e) => {
                                error!("Failed to get PMU info after toggle: {e:?}");
                                self.app_window.set_text(
                                    "Charger toggled but info read failed\nCheck I2C connection"
                                        .into(),
                                );
                            }
                        }
                    }
                    Err(e) => {
                        error!("Failed to toggle charger to {state}: {e:?}");
                        self.app_window.set_text(
                            "Error: Failed to toggle charger\nI2C communication error\nTry again or check hardware"
                                .into(),
                        );
                        return Err(());
                    }
                }
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
            error!("user action queue full, could not add: {a:?}")
        }
    }
}
