use defmt::{error, info};
use embedded_io::Read;

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
    UART: Read,
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
}
