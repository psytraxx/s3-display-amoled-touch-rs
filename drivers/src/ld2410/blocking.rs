use super::*;
use embedded_hal::delay::DelayNs;
use embedded_io::{Read, Write};

impl<UART, DELAY> LD2410<UART, DELAY>
where
    UART: Read + Write,
    DELAY: DelayNs,
{
    /// Read one data frame and decode it. Blocks until a complete frame is available.
    pub fn get_radar_data(&mut self) -> Result<Option<RadarData>, LD2410Error> {
        let mut header = [0u8; 4];
        self.uart.read_exact(&mut header)?;
        if header != DATA_HEADER {
            return Err(LD2410Error::InvalidHeader);
        }

        let mut len_buf = [0u8; 2];
        self.uart.read_exact(&mut len_buf)?;
        let len = u16::from_le_bytes(len_buf) as usize;
        if len > self.buf.len() {
            return Err(LD2410Error::TooLarge(len));
        }

        self.uart.read_exact(&mut self.buf[..len])?;

        let mut tail = [0u8; 4];
        self.uart.read_exact(&mut tail)?;
        if tail != DATA_TAIL {
            return Err(LD2410Error::InvalidTail);
        }

        Ok(self.decode_data_frame(&self.buf[..len]))
    }

    /// Query the sensor firmware version.
    pub fn get_firmware_version(&mut self) -> Result<Option<FirmwareVersion>, LD2410Error> {
        self.config_command(Command::RequestFirmware, |data| {
            if data.len() < 8 {
                return None;
            }
            let bugfix = (data[7] as u32) * 1_000_000
                + (data[6] as u32) * 10_000
                + (data[5] as u32) * 100
                + (data[4] as u32);
            Some(FirmwareVersion {
                major: data[3],
                minor: data[2],
                bugfix,
            })
        })
        .map(|opt| opt.flatten())
    }

    /// Send a restart command to the sensor.
    pub fn request_restart(&mut self) -> Result<bool, LD2410Error> {
        let found = self
            .config_command(Command::RequestRestart, |_| ())
            .map(|opt| opt.is_some())?;
        if found {
            #[cfg(feature = "defmt")]
            defmt::info!("Restart request successful");
            #[cfg(feature = "log-04")]
            log::info!("Restart request successful");
            self.delay.delay_ms(50);
        }
        Ok(found)
    }

    /// Send a factory reset command to the sensor.
    pub fn request_factory_reset(&mut self) -> Result<bool, LD2410Error> {
        let found = self
            .config_command(Command::RequestFactoryReset, |_| ())
            .map(|opt| opt.is_some())?;
        if found {
            #[cfg(feature = "defmt")]
            defmt::info!("Factory reset request successful");
            #[cfg(feature = "log-04")]
            log::info!("Factory reset request successful");
            self.delay.delay_ms(50);
        } else {
            #[cfg(feature = "defmt")]
            defmt::warn!("Factory reset request failed");
            #[cfg(feature = "log-04")]
            log::warn!("Factory reset request failed");
        }
        Ok(found)
    }

    /// Query the current sensor gate sensitivity configuration.
    pub fn get_configuration(&mut self) -> Result<Option<RadarConfiguration>, LD2410Error> {
        self.config_command(Command::RequestCurrentConfig, |data| {
            if data.len() < 24 {
                return None;
            }
            let motion_sensitivity = match data.get(4..13).and_then(|s| s.try_into().ok()) {
                Some(array) => array,
                None => return None,
            };
            let stationary_sensitivity = match data.get(13..22).and_then(|s| s.try_into().ok()) {
                Some(array) => array,
                None => return None,
            };
            let idle_time_bytes: [u8; 2] = match data.get(22..24).and_then(|s| s.try_into().ok()) {
                Some(array) => array,
                None => return None,
            };
            Some(RadarConfiguration {
                max_gate: data[1],
                max_moving_gate: data[2],
                max_stationary_gate: data[3],
                motion_sensitivity,
                stationary_sensitivity,
                sensor_idle_time: u16::from_le_bytes(idle_time_bytes),
            })
        })
        .map(|opt| opt.flatten())
    }

    /// Request to enter engineering mode (detailed per-gate energy output).
    pub fn request_start_engineering_mode(&mut self) -> Result<bool, LD2410Error> {
        let found = self
            .execute_command(Command::RequestStartEngineeringMode)?
            .is_some();
        #[cfg(feature = "defmt")]
        if found {
            defmt::info!("Start Engineering Mode successful");
        } else {
            defmt::warn!("Start Engineering Mode failed");
        }
        #[cfg(feature = "log-04")]
        if found {
            log::info!("Start Engineering Mode successful");
        } else {
            log::warn!("Start Engineering Mode failed");
        }
        Ok(found)
    }

    /// Request to exit engineering mode.
    pub fn request_end_engineering_mode(&mut self) -> Result<bool, LD2410Error> {
        let found = self
            .execute_command(Command::RequestEndEngineeringMode)?
            .is_some();
        #[cfg(feature = "defmt")]
        if found {
            defmt::info!("End Engineering Mode successful");
        } else {
            defmt::warn!("End Engineering Mode failed");
        }
        #[cfg(feature = "log-04")]
        if found {
            log::info!("End Engineering Mode successful");
        } else {
            log::warn!("End Engineering Mode failed");
        }
        Ok(found)
    }

    // ------- private -------

    /// Execute `cmd` inside configuration mode and pass the raw response slice
    /// to `f`. Returns `Some(f(data))` on success, `None` if the command produced no ACK.
    fn config_command<F, T>(&mut self, cmd: Command, f: F) -> Result<Option<T>, LD2410Error>
    where
        F: FnOnce(&[u8]) -> T,
    {
        if !self.enter_configuration_mode()? {
            #[cfg(feature = "log-04")]
            log::warn!("Failed to enter configuration mode");
            return Ok(None);
        }
        self.delay.delay_ms(50);
        // execute_command fills self.buf; process the data before leaving config
        // mode so the borrow is released before the next &mut self call.
        let result = self.execute_command(cmd)?;
        let processed = result.map(|len| f(&self.buf[4..len]));
        self.leave_configuration_mode()?;
        Ok(processed)
    }

    /// Send `command` and poll for an ACK frame. Returns `Some(payload_end)` on
    /// success; response data is at `self.buf[4..payload_end]`.
    fn execute_command(&mut self, command: Command) -> Result<Option<usize>, LD2410Error> {
        #[cfg(feature = "log-04")]
        log::debug!("Executing command: {:?}", command);

        self.uart
            .write_all(&CMD_HEADER)
            .map_err(|_| LD2410Error::IoError)?;
        self.uart
            .write_all(command.payload())
            .map_err(|_| LD2410Error::IoError)?;
        self.uart
            .write_all(&CMD_TAIL)
            .map_err(|_| LD2410Error::IoError)?;

        let expected_ack = command.expected_ack();
        let mut elapsed_ms = 0;
        while elapsed_ms < self.config.timeout_ms {
            self.delay.delay_ms(self.config.delay_ms);
            elapsed_ms += self.config.delay_ms;
            if let Ok(Some(len)) = self.read_command_frame(expected_ack) {
                #[cfg(feature = "log-04")]
                log::debug!(
                    "Command {:?} successful, received {} bytes",
                    command,
                    len - 4
                );
                return Ok(Some(len));
            }
        }

        #[cfg(feature = "log-04")]
        log::warn!("Command {:?} timed out after {}ms", command, elapsed_ms);
        Ok(None)
    }

    /// Read one command-response frame into `self.buf`. Returns `Some(len)` where
    /// `self.buf[0..len]` is the full ACK payload and `self.buf[4..len]` is the
    /// response data (after the 4-byte ack/status prefix).
    fn read_command_frame(&mut self, expected_ack: u8) -> Result<Option<usize>, LD2410Error> {
        let mut header = [0u8; 4];
        self.uart
            .read_exact(&mut header)
            .map_err(|_| LD2410Error::IoError)?;
        if header != CMD_HEADER {
            return Err(LD2410Error::InvalidHeader);
        }

        let mut len_buf = [0u8; 2];
        self.uart
            .read_exact(&mut len_buf)
            .map_err(|_| LD2410Error::IoError)?;
        let len = u16::from_le_bytes(len_buf) as usize;
        if len > self.buf.len() {
            return Err(LD2410Error::TooLarge(len));
        }

        self.uart
            .read_exact(&mut self.buf[..len])
            .map_err(|_| LD2410Error::IoError)?;

        let mut tail = [0u8; 4];
        self.uart
            .read_exact(&mut tail)
            .map_err(|_| LD2410Error::IoError)?;
        if tail != CMD_TAIL {
            return Err(LD2410Error::InvalidTail);
        }

        if len >= 4 {
            let ack_code = self.buf[0];
            let command_success = self.buf[2] == 0x00 && self.buf[3] == 0x00;
            if command_success && ack_code == expected_ack {
                return Ok(Some(len));
            }
        }
        Ok(None)
    }

    fn decode_data_frame(&self, buf: &[u8]) -> Option<RadarData> {
        if buf.len() < 13 {
            #[cfg(feature = "log-04")]
            log::warn!("Data frame too short: {} bytes (expected 13)", buf.len());
            return None;
        }
        if buf[0] != 0x02 {
            #[cfg(feature = "log-04")]
            log::warn!("Invalid data frame type: 0x{:02X} (expected 0x02)", buf[0]);
            return None;
        }
        if buf[1] != 0xAA {
            #[cfg(feature = "log-04")]
            log::warn!("Invalid data frame head: 0x{:02X} (expected 0xAA)", buf[1]);
            return None;
        }
        if buf[11] != 0x55 {
            #[cfg(feature = "log-04")]
            log::warn!(
                "Invalid data frame tail marker: 0x{:02X} (expected 0x55)",
                buf[11]
            );
            return None;
        }
        if buf[12] != 0x00 {
            #[cfg(feature = "log-04")]
            log::warn!(
                "Invalid data frame check byte: 0x{:02X} (expected 0x00)",
                buf[12]
            );
            return None;
        }

        let target_status = buf[2];
        let movement_target_distance = u16::from_le_bytes(buf[3..5].try_into().ok()?);
        let movement_target_energy = buf[5];
        let stationary_target_distance = u16::from_le_bytes(buf[6..8].try_into().ok()?);
        let stationary_target_energy = buf[8];
        let detection_distance = u16::from_le_bytes(buf[9..11].try_into().ok()?);

        let target_state = match TargetState::try_from(target_status) {
            Ok(state) => state,
            Err(_) => {
                #[cfg(feature = "log-04")]
                log::warn!("Invalid target status: 0x{:02X}", target_status);
                return None;
            }
        };

        Some(RadarData {
            target_state,
            movement_target_distance,
            stationary_target_distance,
            detection_distance,
            movement_target_energy,
            stationary_target_energy,
        })
    }

    fn enter_configuration_mode(&mut self) -> Result<bool, LD2410Error> {
        let entered = self.execute_command(Command::EnterConfigMode)?.is_some();
        if entered {
            #[cfg(feature = "defmt")]
            defmt::debug!("Entered configuration mode");
            #[cfg(feature = "log-04")]
            log::debug!("Entered configuration mode");
        } else {
            #[cfg(feature = "defmt")]
            defmt::warn!("Failed to enter configuration mode");
            #[cfg(feature = "log-04")]
            log::warn!("Failed to enter configuration mode");
        }
        Ok(entered)
    }

    fn leave_configuration_mode(&mut self) -> Result<bool, LD2410Error> {
        let left = self.execute_command(Command::ExitConfigMode)?.is_some();
        if left {
            #[cfg(feature = "defmt")]
            defmt::debug!("Left configuration mode");
            #[cfg(feature = "log-04")]
            log::debug!("Left configuration mode");
        } else {
            #[cfg(feature = "defmt")]
            defmt::warn!("Failed to leave configuration mode");
            #[cfg(feature = "log-04")]
            log::warn!("Failed to leave configuration mode");
        }
        Ok(left)
    }
}
