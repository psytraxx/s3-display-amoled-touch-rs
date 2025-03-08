use alloc::vec::Vec;
use core::convert::TryInto;
use defmt::{debug, error, info, warn, Format};
use embedded_hal::delay::DelayNs;
use embedded_io::{Read, Write};

const LD2410_BUFFER_SIZE: usize = 256;

const DATA_HEADER: [u8; 4] = [0xF4, 0xF3, 0xF2, 0xF1];
const DATA_TAIL: [u8; 4] = [0xF8, 0xF7, 0xF6, 0xF5];

const CMD_HEADER: [u8; 4] = [0xFD, 0xFC, 0xFB, 0xFA];
const CMD_TAIL: [u8; 4] = [0x04, 0x03, 0x02, 0x01];

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
pub struct RadarConfiguration {
    pub max_gate: u8,
    pub max_moving_gate: u8,
    pub max_stationary_gate: u8,
    pub motion_sensitivity: [u8; 9],
    pub stationary_sensitivity: [u8; 9],
    pub sensor_idle_time: u16,
}

impl Format for RadarConfiguration {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "RadarConfiguration {{ max_gate: {}, max_moving_gate: {}, max_stationary_gate: {}, idle_time: {}s }}",
            self.max_gate,
            self.max_moving_gate,
            self.max_stationary_gate,
            self.sensor_idle_time
        )
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Command {
    EnterConfigMode,
    ExitConfigMode,
    RequestFirmware,
    RequestRestart,
    RequestFactoryReset,
    RequestCurrentConfig,
}

impl Command {
    fn payload(&self) -> &'static [u8] {
        match self {
            Command::EnterConfigMode => &[0x04, 0x00, 0xFF, 0x00, 0x01, 0x00],
            Command::ExitConfigMode => &[0x02, 0x00, 0xFE, 0x00],
            Command::RequestFirmware => &[0x02, 0x00, 0xA0, 0x00],
            Command::RequestRestart => &[0x02, 0x00, 0xA3, 0x00],
            Command::RequestFactoryReset => &[0x02, 0x00, 0xA2, 0x00],
            Command::RequestCurrentConfig => &[0x02, 0x00, 0x61, 0x00],
        }
    }

    fn expected_ack(&self) -> u8 {
        match self {
            Command::EnterConfigMode => 0xFF,
            Command::ExitConfigMode => 0xFE,
            Command::RequestFirmware => 0xA0,
            Command::RequestRestart => 0xA3,
            Command::RequestFactoryReset => 0xA2,
            Command::RequestCurrentConfig => 0x61,
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

    /// Reads a data frame (header, length, payload, tail) and decodes it.
    pub fn read_data_frame(&mut self) -> Result<Option<RadarData>, LD2410Error<UART::Error>> {
        let mut header = [0u8; 4];
        Self::read_exact(&mut self.uart, &mut header).map_err(LD2410Error::Read)?;
        if header != DATA_HEADER {
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
        if tail != DATA_TAIL {
            return Err(LD2410Error::InvalidTail);
        }

        Ok(self.decode_data_frame(&self.buf[..len]))
    }

    fn decode_data_frame(&self, buf: &[u8]) -> Option<RadarData> {
        if buf.len() >= 13 && buf[1] == 0xAA && buf[11] == 0x55 && buf[12] == 0x00 {
            Some(RadarData {
                target_status: buf[2],
                movement_target_distance: u16::from_le_bytes(buf[3..5].try_into().unwrap()),
                stationary_target_distance: u16::from_le_bytes(buf[6..8].try_into().unwrap()),
                detection_distance: u16::from_le_bytes(buf[9..11].try_into().unwrap()),
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

    /// Reads a command ACK frame and returns a slice of the payload if the command succeeded.
    fn read_command_frame(
        &mut self,
        expected_ack: u8,
    ) -> Result<Option<&[u8]>, LD2410Error<UART::Error>> {
        let mut header = [0u8; 4];
        Self::read_exact(&mut self.uart, &mut header).map_err(LD2410Error::Read)?;
        if header != CMD_HEADER {
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

    fn send_command_preamble(&mut self) -> Result<(), LD2410Error<UART::Error>> {
        debug!("Sending command preamble");
        self.uart.write_all(&CMD_HEADER).map_err(LD2410Error::Read)
    }

    fn send_command_postamble(&mut self) -> Result<(), LD2410Error<UART::Error>> {
        debug!("Sending command postamble");
        self.uart.write_all(&CMD_TAIL).map_err(LD2410Error::Read)
    }

    /// Executes a command by sending the preamble, the command payload, the postamble,
    /// and then waiting for a matching ACK within the configured timeout.
    fn execute_command(
        &mut self,
        command: Command,
    ) -> Result<Option<Vec<u8>>, LD2410Error<UART::Error>> {
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
            if let Ok(Some(data)) = self.read_command_frame(expected_ack) {
                return Ok(Some(data.to_vec()));
            }
        }
        Ok(None)
    }

    fn enter_configuration_mode(&mut self) -> Result<bool, LD2410Error<UART::Error>> {
        let entered = self.execute_command(Command::EnterConfigMode)?.is_some();
        if entered {
            debug!("Entered configuration mode");
        } else {
            warn!("Failed to enter configuration mode");
        }
        Ok(entered)
    }

    fn leave_configuration_mode(&mut self) -> Result<bool, LD2410Error<UART::Error>> {
        let left = self.execute_command(Command::ExitConfigMode)?.is_some();
        if left {
            debug!("Left configuration mode");
        } else {
            warn!("Failed to leave configuration mode");
        }
        Ok(left)
    }

    /// Helper to wrap an operation inside configuration mode.
    fn with_configuration_mode<F, T>(
        &mut self,
        op: F,
    ) -> Result<Option<T>, LD2410Error<UART::Error>>
    where
        F: FnOnce(&mut Self) -> Result<Option<T>, LD2410Error<UART::Error>>,
    {
        if self.enter_configuration_mode()? {
            self.delay.delay_ms(50);
            let res = op(self)?;
            self.leave_configuration_mode()?;
            Ok(res)
        } else {
            warn!("Failed to enter configuration mode");
            Ok(None)
        }
    }

    pub fn request_firmware_version(
        &mut self,
    ) -> Result<Option<FirmwareVersion>, LD2410Error<UART::Error>> {
        self.with_configuration_mode(|this| this.execute_command(Command::RequestFirmware))
            .map(|opt_data| {
                opt_data.and_then(|data| {
                    if data.len() < 8 {
                        return None;
                    }
                    Some(FirmwareVersion {
                        major: data[3],
                        minor: data[2],
                        bugfix: u32::from_le_bytes(data[4..8].try_into().unwrap()),
                    })
                })
            })
    }

    pub fn request_restart(&mut self) -> Result<bool, LD2410Error<UART::Error>> {
        self.with_configuration_mode(|this| {
            let res = this.execute_command(Command::RequestRestart)?;
            if res.is_some() {
                info!("Restart request successful");
                this.delay.delay_ms(50);
            }
            Ok(res)
        })
        .map(|opt| opt.is_some())
    }

    pub fn request_factory_reset(&mut self) -> Result<bool, LD2410Error<UART::Error>> {
        self.with_configuration_mode(|this| {
            let res = this.execute_command(Command::RequestFactoryReset)?;
            if res.is_some() {
                info!("Factory reset request successful");
                this.delay.delay_ms(50);
            } else {
                warn!("Factory reset request failed");
            }
            Ok(res)
        })
        .map(|opt| opt.is_some())
    }

    pub fn request_current_configuration(
        &mut self,
    ) -> Result<Option<RadarConfiguration>, LD2410Error<UART::Error>> {
        self.with_configuration_mode(|this| this.execute_command(Command::RequestCurrentConfig))
            .map(|opt_data| {
                opt_data.map(|data| RadarConfiguration {
                    max_gate: data[1],
                    max_moving_gate: data[2],
                    max_stationary_gate: data[3],
                    motion_sensitivity: data[4..13].try_into().unwrap(),
                    stationary_sensitivity: data[13..22].try_into().unwrap(),
                    sensor_idle_time: u16::from_le_bytes(data[22..24].try_into().unwrap()),
                })
            })
    }
}
