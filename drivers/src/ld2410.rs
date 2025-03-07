use defmt::{debug, error, info, warn, Format};
use embedded_hal::delay::DelayNs;
use embedded_io::{Read, Write};

pub struct LD2410<UART, DELAY: DelayNs> {
    uart: UART,
    buf: [u8; 256],
    delay: DELAY,
}
#[derive(Debug, Clone, Copy)]
pub struct RadarData {
    pub target_status: u8,
    pub movement_target_distance: u16,
    pub stationary_target_distance: u16,
    pub detection_distance: u16,
}

impl Format for RadarData {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "RadarData {{ status: {}, movement: {}cm, stationary: {}cm, detection: {}cm }}",
            self.target_status,
            self.movement_target_distance,
            self.stationary_target_distance,
            self.detection_distance
        )
    }
}

#[derive(Debug)]
pub enum LD2410Error<E> {
    Read(E),
    InvalidHeader,
    InvalidTail,
    TooLarge(usize),
    InvalidData,
}

impl<UART, DELAY: DelayNs> LD2410<UART, DELAY>
where
    UART: Read + Write,
{
    pub fn new(uart: UART, delay: DELAY) -> Self {
        Self {
            uart,
            buf: [0; 256],
            delay,
        }
    }

    pub fn read_data_frame(&mut self) -> Result<Option<RadarData>, LD2410Error<UART::Error>> {
        let mut header = [0u8; 4];
        Self::read_exact(&mut self.uart, &mut header).map_err(LD2410Error::Read)?;
        if header != [0xf4, 0xf3, 0xf2, 0xf1] {
            return Err(LD2410Error::InvalidHeader);
        }

        let mut len_buf = [0u8; 2];
        Self::read_exact(&mut self.uart, &mut len_buf).map_err(LD2410Error::Read)?;
        let len = u16::from_le_bytes(len_buf) as usize;
        if len > self.buf.len() {
            return Err(LD2410Error::TooLarge(len));
        }

        Self::read_exact(&mut self.uart, &mut self.buf[..len]).map_err(LD2410Error::Read)?;

        let mut tail = [0u8; 4];
        Self::read_exact(&mut self.uart, &mut tail).map_err(LD2410Error::Read)?;
        if tail != [0xf8, 0xf7, 0xf6, 0xf5] {
            return Err(LD2410Error::InvalidTail);
        }

        Ok(self.decode_data_frame(&self.buf[..len]))
    }

    fn decode_data_frame(&self, buf: &[u8]) -> Option<RadarData> {
        if buf.len() >= 13 && buf[1] == 0xAA && buf[11] == 0x55 && buf[12] == 0x00 {
            //let data_type = buf[0];
            let target_status = buf[2];
            let movement_target_distance = u16::from_le_bytes([buf[3], buf[4]]);
            let stationary_target_distance = u16::from_le_bytes([buf[6], buf[7]]);
            let detection_distance = u16::from_le_bytes([buf[9], buf[10]]);

            Some(RadarData {
                target_status,
                movement_target_distance,
                stationary_target_distance,
                detection_distance,
            })
        } else {
            error!("Invalid data format");
            None
        }
    }

    fn read_exact<T: Read>(uart: &mut T, buf: &mut [u8]) -> Result<(), T::Error> {
        let mut read = 0;
        while read < buf.len() {
            let n = uart.read(&mut buf[read..])?;
            if n == 0 {
                continue;
            }
            read += n;
        }
        Ok(())
    }
    fn send_command_preamble(&mut self) -> Result<(), LD2410Error<UART::Error>> {
        debug!("Sending command preamble");
        self.uart
            .write_all(&[0xFD, 0xFC, 0xFB, 0xFA])
            .map_err(LD2410Error::Read)
    }

    fn send_command_postamble(&mut self) -> Result<(), LD2410Error<UART::Error>> {
        debug!("Sending command postamble");
        self.uart
            .write_all(&[0x04, 0x03, 0x02, 0x01])
            .map_err(LD2410Error::Read)
    }

    fn enter_configuration_mode(&mut self) -> Result<bool, LD2410Error<UART::Error>> {
        // Example command to enter configuration mode:
        // Command: [0x04, 0x00, 0xFF, 0x00, 0x01, 0x00]
        self.send_command_preamble()?;
        self.uart
            .write_all(&[0x04, 0x00, 0xFF, 0x00, 0x01, 0x00])
            .map_err(LD2410Error::Read)?;
        self.send_command_postamble()?;
        for _ in 0..10 {
            self.delay.delay_ms(10);
            if let Ok(Some(response)) = self.read_command_frame() {
                if response == 0xFF {
                    info!("Entered configuration mode with ack {}", response);
                    return Ok(true);
                }
            }
        }
        warn!("Failed to enter configuration mode");
        Ok(false)
    }

    fn leave_configuration_mode(&mut self) -> Result<(), LD2410Error<UART::Error>> {
        // Example command to leave configuration mode:
        // Command: [0x02, 0x00, 0xFE, 0x00]
        self.send_command_preamble()?;
        info!("Leaving configuration mode");
        self.uart
            .write_all(&[0x02, 0x00, 0xFE, 0x00])
            .map_err(LD2410Error::Read)?;
        self.send_command_postamble()?;
        Ok(())
    }

    /// Requests firmware version from the sensor.
    /// Sends a firmware request command and polls for a response.
    /// Returns Ok(true) if the expected ack is received, otherwise Ok(false).
    pub fn request_firmware_version(&mut self) -> Result<bool, LD2410Error<UART::Error>> {
        if let Ok(success) = self.enter_configuration_mode() {
            if success {
                self.delay.delay_ms(50);
                // Send firmware version request command:
                // Command: [0x02, 0x00, 0xA0, 0x00]
                self.send_command_preamble()?;
                self.uart
                    .write_all(&[0x02, 0x00, 0xA0, 0x00])
                    .map_err(LD2410Error::Read)?;
                self.send_command_postamble()?;
                // Record command send time here if using a timeout mechanism
                // Poll for response (for simplicity, a fixed number of attempts are made)
                for _ in 0..10 {
                    self.delay.delay_ms(10);
                    // Try reading a frame which should contain the firmware ACK.
                    if let Ok(Some(response)) = self.read_command_frame() {
                        // In this example, an ACK frame is assumed to have target_status == 0xA0
                        if response == 0xA0 {
                            info!("Firmware version request successful with ack: {}", response);
                            self.delay.delay_ms(50);
                            // Parse version information from the buffer
                            if self.buf.len() >= 18 {
                                warn!("Buffer: {:?}", &self.buf);
                                let firmware_major_version = self.buf[7];
                                let firmware_minor_version = self.buf[6];
                                let firmware_bugfix_version = self.buf[8] as u32
                                    | ((self.buf[9] as u32) << 8)
                                    | ((self.buf[10] as u32) << 16)
                                    | ((self.buf[11] as u32) << 24);

                                info!(
                                    "Firmware version: {}.{}.{}",
                                    firmware_major_version,
                                    firmware_minor_version,
                                    firmware_bugfix_version
                                );
                            } else {
                                warn!("Buffer too small to extract firmware version");
                            }
                            self.leave_configuration_mode()?;
                            return Ok(true);
                        }
                    }
                }
                self.leave_configuration_mode()?;
            }
        }
        warn!("Failed to request firmware version");
        Ok(false)
    }

    /// Requests the radar module to restart.
    /// Sends a restart request command and polls for a response.
    /// Returns Ok(true) if the expected ack is received, otherwise Ok(false).
    pub fn request_restart(&mut self) -> Result<bool, LD2410Error<UART::Error>> {
        if let Ok(success) = self.enter_configuration_mode() {
            if success {
                self.delay.delay_ms(50);
                // Send restart request command:
                // Command: [0x02, 0x00, 0xA3, 0x00]
                self.send_command_preamble()?;
                self.uart
                    .write_all(&[0x02, 0x00, 0xA3, 0x00])
                    .map_err(LD2410Error::Read)?;
                self.send_command_postamble()?;

                // Poll for response (for simplicity, a fixed number of attempts are made)
                for _ in 0..10 {
                    self.delay.delay_ms(10);
                    // Try reading a frame which should contain the restart ACK.
                    if let Ok(Some(response)) = self.read_command_frame() {
                        info!("Restart request complete with ack: {}", response);
                        // In this example, an ACK frame is assumed to have target_status == 0xA3
                        if response == 0xA3 {
                            self.delay.delay_ms(50);
                            self.leave_configuration_mode()?;
                            return Ok(true);
                        }
                    }
                }

                self.leave_configuration_mode()?;
            }
        }

        Ok(false)
    }

    fn read_command_frame(&mut self) -> Result<Option<u8>, LD2410Error<UART::Error>> {
        // Try to read a full frame
        let mut header = [0u8; 4];
        Self::read_exact(&mut self.uart, &mut header).map_err(LD2410Error::Read)?;

        // For ACK frames, the header should be [0xFD, 0xFC, 0xFB, 0xFA]
        if header != [0xFD, 0xFC, 0xFB, 0xFA] {
            // Not an ACK frame, could be discarded or handled differently
            return Err(LD2410Error::InvalidHeader);
        }

        // Read the length bytes
        let mut len_buf = [0u8; 2];
        Self::read_exact(&mut self.uart, &mut len_buf).map_err(LD2410Error::Read)?;
        let len = u16::from_le_bytes(len_buf) as usize;
        if len > self.buf.len() {
            return Err(LD2410Error::TooLarge(len));
        }

        // Read the command data
        Self::read_exact(&mut self.uart, &mut self.buf[..len]).map_err(LD2410Error::Read)?;

        // Read the tail
        let mut tail = [0u8; 4];
        Self::read_exact(&mut self.uart, &mut tail).map_err(LD2410Error::Read)?;
        if tail != [0x04, 0x03, 0x02, 0x01] {
            return Err(LD2410Error::InvalidTail);
        }

        // Now parse the ACK frame
        // Example parsing (will need to be adjusted based on your exact needs):
        if len >= 4 {
            let ack_code = self.buf[0]; // Equivalent to latest_ack_ in C++
            let command_success = self.buf[2] == 0x00 && self.buf[3] == 0x00;

            if command_success {
                return Ok(Some(ack_code));
            }
        }

        Ok(None)
    }
}
