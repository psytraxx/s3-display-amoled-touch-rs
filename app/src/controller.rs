use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use log::{error, info};
use slint_generated::AppWindow;

use crate::radar_task::RADAR_DATA;
use crate::Charger;

pub struct Controller<'a> {
    app_window: &'a AppWindow,
    pmu: Charger,
}

#[derive(Debug, Clone)]
pub enum Action {
    RequestPmuUpdate,
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
            match select(ACTION.receive(), RADAR_DATA.receive()).await {
                Either::First(action) => {
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
                Either::Second(radar_data) => {
                    // Update radar display with new data
                    let info = radar_data.get_info();
                    self.app_window.set_radar_data(info.into());
                }
            }
        }
    }

    pub async fn process_action(&mut self, action: Action) -> Result<(), ()> {
        match action {
            Action::RequestPmuUpdate => match self.pmu.get_info().await {
                Ok(text) => {
                    self.app_window.set_pmu_details(text.into());
                    // Also update battery percentage on main screen
                    match self.pmu.get_battery_percentage().await {
                        Ok(percentage) => {
                            self.app_window.set_battery_percentage(percentage as i32);
                        }
                        Err(e) => {
                            error!("Failed to get battery percentage: {e:?}");
                        }
                    }
                }
                Err(e) => {
                    error!("Failed to get PMU info: {e:?}");
                    self.app_window.set_pmu_details(
                        "Error: Failed to read PMU data\nCheck I2C connection".into(),
                    );
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
                        // Update battery percentage on main screen
                        match self.pmu.get_battery_percentage().await {
                            Ok(percentage) => {
                                self.app_window.set_battery_percentage(percentage as i32);
                            }
                            Err(e) => {
                                error!("Failed to get battery percentage after toggle: {e:?}");
                            }
                        }
                    }
                    Err(e) => {
                        error!("Failed to toggle charger to {state}: {e:?}");
                        self.app_window.set_pmu_details(
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
            .on_request_pmu_update(|| send_action(Action::RequestPmuUpdate));
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
