use defmt::{error, info, warn};
use embedded_hal::delay::DelayNs;
use embedded_io::{Read, Write};

pub struct LD2410<UART> {
    uart: UART,
    buf: [u8; 256],
}

#[derive(Debug, Clone, Copy)]
pub struct RadarData {
    pub target_status: u8,
    pub movement_target_distance: u16,
    pub stationary_target_distance: u16,
    pub detection_distance: u16,
}

#[derive(Debug)]
pub enum LD2410Error<E> {
    Read(E),
    InvalidHeader,
    InvalidTail,
    TooLarge(usize),
    InvalidData,
}

impl<UART> LD2410<UART>
where
    UART: Read + Write,
{
    pub fn new(uart: UART) -> Self {
        Self {
            uart,
            buf: [0; 256],
        }
    }

    pub fn read_frame(&mut self) -> Result<Option<RadarData>, LD2410Error<UART::Error>> {
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

        Ok(self.decode(&self.buf[..len]))
    }

    fn decode(&self, buf: &[u8]) -> Option<RadarData> {
        if let [data_type, 0xAA, target_status, mt_l, mt_h, _mt_energy, st_l, st_h, _st_energy, det_l, det_h, 0x55, 0x00] =
            buf
        {
            match data_type {
                0x2 => {
                    let movement_target_distance = u16::from_le_bytes([*mt_l, *mt_h]);
                    let stationary_target_distance = u16::from_le_bytes([*st_l, *st_h]);
                    let detection_distance = u16::from_le_bytes([*det_l, *det_h]);

                    Some(RadarData {
                        target_status: *target_status,
                        movement_target_distance,
                        stationary_target_distance,
                        detection_distance,
                    })
                }
                0x1 => {
                    info!("Engineering data detected, ignoring");
                    None
                }
                _ => {
                    error!("Unknown data type: {:?}", data_type);
                    None
                }
            }
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
        self.uart
            .write_all(&[0xFD, 0xFC, 0xFB, 0xFA])
            .map_err(LD2410Error::Read)
    }

    fn send_command_postamble(&mut self) -> Result<(), LD2410Error<UART::Error>> {
        self.uart
            .write_all(&[0x04, 0x03, 0x02, 0x01])
            .map_err(LD2410Error::Read)
    }

    fn enter_configuration_mode(&mut self) -> Result<(), LD2410Error<UART::Error>> {
        // Example command to enter configuration mode:
        // Command: [0x04, 0x00, 0xFF, 0x00, 0x01, 0x00]
        self.send_command_preamble()?;
        self.uart
            .write_all(&[0x04, 0x00, 0xFF, 0x00, 0x01, 0x00])
            .map_err(LD2410Error::Read)?;
        self.send_command_postamble()?;
        // In a real implementation, you would wait for an acknowledgment.
        Ok(())
    }

    fn leave_configuration_mode(&mut self) -> Result<(), LD2410Error<UART::Error>> {
        // Example command to leave configuration mode:
        // Command: [0x02, 0x00, 0xFE, 0x00]
        self.send_command_preamble()?;
        self.uart
            .write_all(&[0x02, 0x00, 0xFE, 0x00])
            .map_err(LD2410Error::Read)?;
        self.send_command_postamble()?;
        Ok(())
    }

    /// Requests firmware version from the sensor.
    /// Sends a firmware request command and polls for a response.
    /// Returns Ok(true) if the expected ack is received, otherwise Ok(false).
    pub fn request_firmware_version<U: DelayNs>(
        &mut self,
        mut delay: U,
    ) -> Result<bool, LD2410Error<UART::Error>> {
        self.enter_configuration_mode()?;
        delay.delay_ms(50);
        // Send firmware version request command:
        // Command: [0x02, 0x00, 0xA0, 0x00]
        self.send_command_preamble()?;
        self.uart
            .write_all(&[0x02, 0x00, 0xA0, 0x00])
            .map_err(LD2410Error::Read)?;
        self.send_command_postamble()?;
        // Record command send time here if using a timeout mechanism
        // Poll for response (for simplicity, a fixed number of attempts are made)
        for _ in 0..100 {
            delay.delay_ms(10);
            // Try reading a frame which should contain the firmware ACK.
            if let Ok(Some(response)) = self.read_frame() {
                warn!(
                    "Firmware target_status response: {:?}",
                    response.target_status
                );
                // In this example, an ACK frame is assumed to have target_status == 0xA0
                if response.target_status == 0xA0 {
                    delay.delay_ms(50);
                    self.leave_configuration_mode()?;
                    return Ok(true);
                }
            }
        }
        self.leave_configuration_mode()?;
        Ok(false)
    }
}
