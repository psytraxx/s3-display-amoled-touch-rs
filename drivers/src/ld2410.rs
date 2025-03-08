use defmt::{debug, error, info, warn, Format};
use embedded_hal::delay::DelayNs;
use embedded_io::{Read, Write};

const LD2410_BUFFER_SIZE: usize = 256;

pub struct LD2410<UART, DELAY: DelayNs> {
    uart: UART,
    buf: [u8; LD2410_BUFFER_SIZE],
    delay: DELAY,
    config: PollingConfig,
}

pub struct PollingConfig {
    pub max_attempts: u8,
    pub delay_ms: u32,
    pub timeout_ms: u32,
}

impl Default for PollingConfig {
    fn default() -> Self {
        Self {
            max_attempts: 10,
            delay_ms: 10,
            timeout_ms: 100,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct FirmwareVersion {
    pub major: u8,
    pub minor: u8,
    pub bugfix: u32,
}

impl Format for FirmwareVersion {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "{}.{}.{}", self.major, self.minor, self.bugfix)
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Command {
    EnterConfigMode,
    ExitConfigMode,
    RequestFirmware,
    RequestRestart,
}

impl Command {
    fn payload(&self) -> &'static [u8] {
        match self {
            Command::EnterConfigMode => &[0x04, 0x00, 0xFF, 0x00, 0x01, 0x00],
            Command::ExitConfigMode => &[0x02, 0x00, 0xFE, 0x00],
            Command::RequestFirmware => &[0x02, 0x00, 0xA0, 0x00],
            Command::RequestRestart => &[0x02, 0x00, 0xA3, 0x00],
        }
    }

    fn expected_ack(&self) -> u8 {
        match self {
            Command::EnterConfigMode => 0xFF,
            Command::ExitConfigMode => 0xFE,
            Command::RequestFirmware => 0xA0,
            Command::RequestRestart => 0xA3,
        }
    }
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
    Write(E),
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
            buf: [0; LD2410_BUFFER_SIZE],
            delay,
            config: PollingConfig::default(),
        }
    }

    pub fn read_data_frame(&mut self) -> Result<Option<RadarData>, LD2410Error<UART::Error>> {
        let mut header = [0u8; 4];
        Self::read_exact(&mut self.uart, &mut header).map_err(LD2410Error::Read)?;
        if header != [0xF4, 0xF3, 0xF2, 0xF1] {
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
        if tail != [0xF8, 0xF7, 0xF6, 0xF5] {
            return Err(LD2410Error::InvalidTail);
        }

        Ok(self.decode_data_frame(&self.buf[..len]))
    }

    fn decode_data_frame(&self, buf: &[u8]) -> Option<RadarData> {
        // We no longer check the data type byte; just verify the magic bytes at expected positions.
        if buf.len() >= 13 && buf[1] == 0xAA && buf[11] == 0x55 && buf[12] == 0x00 {
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

    // Generic command execution method
    fn execute_command(&mut self, command: Command) -> Result<bool, LD2410Error<UART::Error>> {
        self.send_command_preamble()?;
        self.uart
            .write_all(command.payload())
            .map_err(LD2410Error::Write)?;
        self.send_command_postamble()?;

        let expected_ack = command.expected_ack();
        let mut elapsed_ms = 0;

        while elapsed_ms < self.config.timeout_ms {
            self.delay.delay_ms(self.config.delay_ms);
            elapsed_ms += self.config.delay_ms;

            if let Ok(Some(response)) = self.read_command_frame() {
                if response == expected_ack {
                    return Ok(true);
                }
            }
        }

        Ok(false) // Timeout without matching ACK
    }

    fn enter_configuration_mode(&mut self) -> Result<bool, LD2410Error<UART::Error>> {
        let result = self.execute_command(Command::EnterConfigMode)?;
        if result {
            debug!("Entered configuration mode");
        } else {
            warn!("Failed to enter configuration mode");
        }
        Ok(result)
    }

    fn leave_configuration_mode(&mut self) -> Result<bool, LD2410Error<UART::Error>> {
        let result = self.execute_command(Command::ExitConfigMode)?;
        if result {
            debug!("Left configuration mode");
        }
        Ok(result)
    }

    /// Requests firmware version from the sensor.
    /// Sends a firmware version request command and polls for a response.
    /// Returns Ok(Some(version_string)) if the expected ACK is received and version data is parsed, otherwise Ok(None).
    pub fn request_firmware_version(
        &mut self,
    ) -> Result<Option<FirmwareVersion>, LD2410Error<UART::Error>> {
        if let Ok(true) = self.enter_configuration_mode() {
            self.delay.delay_ms(50);

            let result = self.execute_command(Command::RequestFirmware)?;
            if result {
                self.delay.delay_ms(50);
                // Parse version information from the buffer
                if self.buf.len() >= 18 {
                    let firmware_major_version = self.buf[7];
                    let firmware_minor_version = self.buf[6];
                    let firmware_bugfix_version = self.buf[8] as u32
                        | ((self.buf[9] as u32) << 8)
                        | ((self.buf[10] as u32) << 16)
                        | ((self.buf[11] as u32) << 24);

                    let version = FirmwareVersion {
                        major: firmware_major_version,
                        minor: firmware_minor_version,
                        bugfix: firmware_bugfix_version,
                    };

                    self.leave_configuration_mode()?;
                    return Ok(Some(version));
                }
            }

            self.leave_configuration_mode()?;
        }

        warn!("Failed to request firmware version");
        Ok(None)
    }

    /// Requests the radar module to restart.
    /// Sends a restart request command and polls for a response.
    /// Returns Ok(true) if the expected ACK is received, otherwise Ok(false).
    pub fn request_restart(&mut self) -> Result<bool, LD2410Error<UART::Error>> {
        if let Ok(true) = self.enter_configuration_mode() {
            self.delay.delay_ms(50);

            let result = self.execute_command(Command::RequestRestart)?;
            if result {
                info!("Restart request successful");
                self.delay.delay_ms(50);
            }

            self.leave_configuration_mode()?;
            return Ok(result);
        }

        Ok(false)
    }

    /// Reads an ACK frame from the UART.
    /// This function expects the ACK frame header [0xFD, 0xFC, 0xFB, 0xFA] and tail [0x04, 0x03, 0x02, 0x01].
    /// If the frame is successfully read and the command was successful (bytes 2 and 3 are zero),
    /// the ACK code (byte 0 of the payload) is returned.
    fn read_command_frame(&mut self) -> Result<Option<u8>, LD2410Error<UART::Error>> {
        let mut header = [0u8; 4];
        Self::read_exact(&mut self.uart, &mut header).map_err(LD2410Error::Read)?;

        // ACK frame header
        if header != [0xFD, 0xFC, 0xFB, 0xFA] {
            return Err(LD2410Error::InvalidHeader);
        }

        let mut len_buf = [0u8; 2];
        Self::read_exact(&mut self.uart, &mut len_buf).map_err(LD2410Error::Read)?;
        let len = u16::from_le_bytes(len_buf) as usize;
        if len > self.buf.len() {
            return Err(LD2410Error::TooLarge(len));
        }

        // Read the ACK frame payload into self.buf
        Self::read_exact(&mut self.uart, &mut self.buf[..len]).map_err(LD2410Error::Read)?;

        let mut tail = [0u8; 4];
        Self::read_exact(&mut self.uart, &mut tail).map_err(LD2410Error::Read)?;
        if tail != [0x04, 0x03, 0x02, 0x01] {
            return Err(LD2410Error::InvalidTail);
        }

        // Parse the ACK frame. Here we assume a valid ACK frame has at least 4 bytes,
        // where byte 0 is the ACK code and bytes 2 and 3 indicate success.
        if len >= 4 {
            let ack_code = self.buf[0];
            let command_success = self.buf[2] == 0x00 && self.buf[3] == 0x00;
            if command_success {
                return Ok(Some(ack_code));
            }
        }
        Ok(None)
    }
}
