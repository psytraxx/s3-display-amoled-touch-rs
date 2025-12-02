use super::*;
use alloc::vec::Vec;
use embedded_hal::delay::DelayNs;
use embedded_io::{Read, Write};

// Main driver struct
pub struct LD2410<UART, DELAY: DelayNs> {
    uart: UART,
    buf: [u8; LD2410_BUFFER_SIZE],
    delay: DELAY,
    config: PollingConfig,
}

// Implementation
impl<UART, DELAY: DelayNs> LD2410<UART, DELAY>
where
    UART: Read + Write,
{
    // Constructor
    pub fn new(uart: UART, delay: DELAY) -> Self {
        Self {
            uart,
            buf: [0; LD2410_BUFFER_SIZE],
            delay,
            config: PollingConfig::default(),
        }
    }

    // Public API
    /// Reads a data frame (header, length, payload, tail) and decodes it.
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

    pub fn get_firmware_version(&mut self) -> Result<Option<FirmwareVersion>, LD2410Error> {
        self.with_configuration_mode(|this| this.execute_command(Command::RequestFirmware))
            .map(|opt_data| {
                opt_data.and_then(|data| {
                    if data.len() < 8 {
                        return None;
                    }
                    let bugfix = (data[7] as u32) * 1000000
                        + (data[6] as u32) * 10000
                        + (data[5] as u32) * 100
                        + (data[4] as u32);

                    Some(FirmwareVersion {
                        major: data[3],
                        minor: data[2],
                        bugfix,
                    })
                })
            })
    }

    pub fn request_restart(&mut self) -> Result<bool, LD2410Error> {
        self.with_configuration_mode(|this| {
            let res = this.execute_command(Command::RequestRestart)?;
            if res.is_some() {
                #[cfg(feature = "defmt")]
                defmt::info!("Restart request successful");
                this.delay.delay_ms(50);
                #[cfg(feature = "log-04")]
                log::info!("Restart request successful");
            }
            Ok(res)
        })
        .map(|opt| opt.is_some())
    }

    pub fn request_factory_reset(&mut self) -> Result<bool, LD2410Error> {
        self.with_configuration_mode(|this| {
            let res = this.execute_command(Command::RequestFactoryReset)?;
            if res.is_some() {
                #[cfg(feature = "defmt")]
                defmt::info!("Factory reset request successful");
                this.delay.delay_ms(50);
                #[cfg(feature = "log-04")]
                log::info!("Factory reset request successful");
            } else {
                #[cfg(feature = "defmt")]
                defmt::warn!("Factory reset request failed");
                #[cfg(feature = "log-04")]
                log::warn!("Factory reset request failed");
            }
            Ok(res)
        })
        .map(|opt| opt.is_some())
    }

    pub fn get_configuration(&mut self) -> Result<Option<RadarConfiguration>, LD2410Error> {
        self.with_configuration_mode(|this| this.execute_command(Command::RequestCurrentConfig))
            .map(|opt_data| {
                opt_data.and_then(|data| {
                    // Safe bounds checking
                    if data.len() < 24 {
                        return None;
                    }

                    // Safely convert slices to arrays
                    let motion_sensitivity = match data.get(4..13).and_then(|s| s.try_into().ok()) {
                        Some(array) => array,
                        None => return None,
                    };

                    let stationary_sensitivity =
                        match data.get(13..22).and_then(|s| s.try_into().ok()) {
                            Some(array) => array,
                            None => return None,
                        };

                    let idle_time_bytes = match data.get(22..24).and_then(|s| s.try_into().ok()) {
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
            })
    }

    /// Requests to enter engineering mode.
    pub fn request_start_engineering_mode(&mut self) -> Result<bool, LD2410Error> {
        // Execute the engineering command directly without changing configuration mode.
        let res = self.execute_command(Command::RequestStartEngineeringMode)?;
        #[cfg(feature = "defmt")]
        {
            if res.is_some() {
                defmt::info!("Start Engineering Mode successful");
            } else {
                defmt::warn!("Start Engineering Mode failed");
            }
        }
        #[cfg(feature = "log-04")]
        {
            if res.is_some() {
                log::info!("Start Engineering Mode successful");
            } else {
                log::warn!("Start Engineering Mode failed");
            }
        }
        Ok(res.is_some())
    }

    /// Requests to exit engineering mode.
    pub fn request_end_engineering_mode(&mut self) -> Result<bool, LD2410Error> {
        // Execute the engineering command directly without changing configuration mode.
        let res = self.execute_command(Command::RequestEndEngineeringMode)?;
        #[cfg(feature = "defmt")]
        {
            if res.is_some() {
                defmt::info!("End Engineering Mode successful");
            } else {
                defmt::warn!("End Engineering Mode failed");
            }
        }
        #[cfg(feature = "log-04")]
        {
            if res.is_some() {
                log::info!("End Engineering Mode successful");
            } else {
                log::warn!("End Engineering Mode failed");
            }
        }
        Ok(res.is_some())
    }

    // Private helper methods
    fn decode_data_frame(&self, buf: &[u8]) -> Option<RadarData> {
        // we receive 13 bytes of data - first byte is data value byte
        // second byte is the head and 11 and 12 tail and check
        if buf.len() < 13 || buf[0] != 0x02 || buf[1] != 0xAA || buf[11] != 0x55 || buf[12] != 0x00
        {
            return None;
        }

        let target_status = buf[2];
        let movement_target_distance = u16::from_le_bytes(buf[3..5].try_into().ok()?);
        let movement_target_energy = buf[5];
        let stationary_target_distance = u16::from_le_bytes(buf[6..8].try_into().ok()?);
        let stationary_target_energy = buf[8];
        let detection_distance = u16::from_le_bytes(buf[9..11].try_into().ok()?);

        Some(RadarData {
            target_state: TargetState::try_from(target_status).expect("Invalid target state"),
            movement_target_distance,
            stationary_target_distance,
            detection_distance,
            movement_target_energy,
            stationary_target_energy,
        })
    }

    fn read_command_frame(&mut self, expected_ack: u8) -> Result<Option<&[u8]>, LD2410Error> {
        let mut header = [0u8; 4];
        self.uart.read_exact(&mut header)?;
        if header != CMD_HEADER {
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
        if tail != CMD_TAIL {
            return Err(LD2410Error::InvalidTail);
        }

        if len >= 4 {
            let ack_code = self.buf[0];
            let command_success = self.buf[2] == 0x00 && self.buf[3] == 0x00;
            if command_success && ack_code == expected_ack {
                return Ok(Some(&self.buf[4..len]));
            }
        }
        Ok(None)
    }

    fn execute_command(&mut self, command: Command) -> Result<Option<Vec<u8>>, LD2410Error> {
        self.uart
            .write_all(&CMD_HEADER)
            .expect("Failed to write command header");
        self.uart
            .write_all(command.payload())
            .expect("Failed to write command payload");
        self.uart
            .write_all(&CMD_TAIL)
            .expect("Failed to write command tail");

        let expected_ack = command.expected_ack();
        let mut elapsed_ms = 0;
        while elapsed_ms < self.config.timeout_ms {
            self.delay.delay_ms(self.config.delay_ms);
            elapsed_ms += self.config.delay_ms;
            if let Ok(Some(data)) = self.read_command_frame(expected_ack) {
                return Ok(Some(data.to_vec()));
            }
        }
        Ok(None)
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

    /// Helper to wrap an operation inside configuration mode.
    fn with_configuration_mode<F, T>(&mut self, op: F) -> Result<Option<T>, LD2410Error>
    where
        F: FnOnce(&mut Self) -> Result<Option<T>, LD2410Error>,
    {
        if self.enter_configuration_mode()? {
            self.delay.delay_ms(50);
            let res = op(self)?;
            self.leave_configuration_mode()?;
            Ok(res)
        } else {
            #[cfg(feature = "defmt")]
            defmt::warn!("Failed to enter configuration mode");
            #[cfg(feature = "log-04")]
            log::warn!("Failed to enter configuration mode");
            Ok(None)
        }
    }
}
