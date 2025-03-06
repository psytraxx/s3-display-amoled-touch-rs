use embedded_io::{Error, Read, Write};

/// Driver for the LD2410 radar sensor.
/// Possible errors when communicating with the LD2410 sensor.
#[derive(Debug)]
pub enum SensorError<E: Error> {
    /// Communication error from the underlying UART
    Uart(E),
    /// Invalid header received
    InvalidHeader,
    /// Invalid footer received
    InvalidFooter,
    /// Invalid data format received
    InvalidData,
    /// Invalid response to a command
    InvalidResponse,
    /// Command timeout
    CommandTimeout,
    /// Device not in configuration mode
    NotInConfigMode,
    /// Underlying UART would block
    WouldBlock,
}

impl<E: embedded_io::Error> From<E> for SensorError<E> {
    fn from(error: E) -> Self {
        SensorError::Uart(error)
    }
}

/// Holds decoded radar measurement data.
#[derive(Debug)]
pub struct RadarData {
    pub target_status: u8,
    pub moving_distance: u16,
    pub stationary_distance: u16,
    pub detection_distance: u16,
}

/// Represents engineering mode data from the sensor
#[derive(Debug)]
pub struct EngineeringData {
    pub moving_gate_energy: [u8; 9],
    pub static_gate_energy: [u8; 9],
    pub light: u8,
    pub out_pin: u8,
}

/// Configuration parameters for the LD2410 sensor
#[derive(Debug)]
pub struct Ld2410Config {
    pub max_moving_distance_gate: u8,
    pub max_static_distance_gate: u8,
    pub moving_sensitivity: [u8; 9],
    pub static_sensitivity: [u8; 9],
    pub no_one_duration: u16,
}

impl Default for Ld2410Config {
    fn default() -> Self {
        Self {
            max_moving_distance_gate: 8,
            max_static_distance_gate: 8,
            moving_sensitivity: [50, 50, 40, 30, 20, 15, 15, 15, 15],
            static_sensitivity: [0, 0, 40, 40, 30, 30, 20, 20, 20],
            no_one_duration: 5,
        }
    }
}

/// Command types for the LD2410 sensor
#[derive(Debug, Clone, Copy)]
pub enum CommandType {
    StartConfiguration = 0xFF,
    EndConfiguration = 0xFE,
    SetDistance = 0x60,
    ReadParameters = 0x61,
    StartEngineering = 0x62,
    EndEngineering = 0x63,
    SetSensitivity = 0x64,
    GetFirmware = 0xA0,
    SetBaudrate = 0xA1,
    FactoryReset = 0xA2,
    Reboot = 0xA3,
    SetBluetooth = 0xA4,
    GetBluetoothMac = 0xA5,
}

/// A driver for the LD2410 sensor.
pub struct Ld2410Driver<UART> {
    uart: UART,
    config: Ld2410Config,
    in_config_mode: bool,
    engineering_mode: bool,
}

impl<UART> Ld2410Driver<UART> where
UART: Read + Write
{
    /// Create a new LD2410 driver from a UART instance.
    pub fn new(uart: UART) -> Self {
        Self { 
            uart, 
            config: Ld2410Config::default(),
            in_config_mode: false,
            engineering_mode: false,
        }
    }
    
    /// Get the currently configured parameters
    pub fn config(&self) -> &Ld2410Config {
        &self.config
    }

    /// Helper: Reads exactly `buf.len()` bytes into the provided buffer.
    fn read_bytes(&mut self, buf: &mut [u8]) -> Result<(), SensorError<UART::Error>> {
        self.uart.read(buf)?;
        Ok(())
    }

    /// Reads a complete packet from the sensor and decodes it.
    ///
    /// The protocol is expected to be:
    /// - Header: 4 bytes [0xF4, 0xF3, 0xF2, 0xF1]
    /// - Length: 2 bytes little endian (expected to be 13 for measurement packets)
    /// - Data: 13 bytes in the measurement format
    /// - Tail: 4 bytes [0xF8, 0xF7, 0xF6, 0xF5]
    ///
    /// Returns Ok(Some(RadarData)) if a valid measurement is received,
    /// Ok(None) if the packet is not a valid measurement packet,
    /// or an error.
    pub fn read_packet(&mut self) -> Result<Option<RadarData>, SensorError<UART::Error>> {
        let mut header = [0u8; 4];
        self.read_bytes(&mut header)?;
        if header != [0xF4, 0xF3, 0xF2, 0xF1] {
            return Err(SensorError::InvalidHeader);
        }

        let mut length_buf = [0u8; 2];
        self.read_bytes(&mut length_buf)?;
        let length = u16::from_le_bytes(length_buf) as usize;

        if length != 13 {
            let mut _dummy = [0u8; 13];
            let _ = self.read_bytes(&mut _dummy);
            let mut _tail = [0u8; 4];
            let _ = self.read_bytes(&mut _tail);
            return Ok(None);
        }

        let mut data_buf = [0u8; 13];
        self.read_bytes(&mut data_buf)?;

        let mut tail = [0u8; 4];
        self.read_bytes(&mut tail)?;
        if tail != [0xF8, 0xF7, 0xF6, 0xF5] {
            return Err(SensorError::InvalidFooter);
        }

        Ok(Self::decode_radar_data(&data_buf))
    }

    /// Decodes the raw measurement data.
    /// Returns Some(RadarData) if data_type is 0x2, otherwise None.
    fn decode_radar_data(buf: &[u8; 13]) -> Option<RadarData> {
        match buf {
            [
                data_type, 0xAA,
                target_status,
                mov_l, mov_h, _mov_energy,
                stat_l, stat_h, _stat_energy,
                det_l, det_h, 0x55, 0x00
            ] => {
                if *data_type == 0x2 {
                    let moving_distance = u16::from_le_bytes([*mov_l, *mov_h]);
                    let stationary_distance = u16::from_le_bytes([*stat_l, *stat_h]);
                    let detection_distance = u16::from_le_bytes([*det_l, *det_h]);
                    Some(RadarData {
                        target_status: *target_status,
                        moving_distance,
                        stationary_distance,
                        detection_distance,
                    })
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    /// Helper: Write one byte nonblocking.
    fn write_byte(&mut self, byte: u8) -> Result<(), SensorError<UART::Error>> {
        self.uart.write(&[byte])?;
        Ok(())
    }

    /// Send a command to the LD2410 sensor.
    fn send_command(&mut self, cmd: CommandType, data: &[u8]) -> Result<(), SensorError<UART::Error>> {
        let len = data.len() + 2;
        let header = [0xFD, 0xFC, 0xFB, 0xFA];
        for &byte in header.iter() {
            self.write_byte(byte)?;
        }

        self.write_byte(len as u8)?;
        self.write_byte(0x00)?;

        self.write_byte(cmd as u8)?;
        self.write_byte(0x00)?;

        for &byte in data.iter() {
            self.write_byte(byte)?;
        }

        let footer = [0x04, 0x03, 0x02, 0x01];
        for &byte in footer.iter() {
            self.write_byte(byte)?;
        }

        Ok(())
    }
    
    /// Enter configuration mode.
    pub fn enter_config_mode(&mut self) -> Result<(), SensorError<UART::Error>> {
        self.send_command(CommandType::StartConfiguration, &[0x01, 0x00])?;
        self.in_config_mode = true;
        Ok(())
    }
    
    /// Exit configuration mode.
    pub fn exit_config_mode(&mut self) -> Result<(), SensorError<UART::Error>> {
        self.send_command(CommandType::EndConfiguration, &[])?;
        self.in_config_mode = false;
        Ok(())
    }
    
    /// Set maximum distance ranges and no-one duration.
    pub fn set_distance_config(&mut self, max_moving_gate: u8, max_static_gate: u8, no_one_duration: u16) -> Result<(), SensorError<UART::Error>> {
        if !self.in_config_mode {
            return Err(SensorError::NotInConfigMode);
        }
        
        let mut data = [0u8; 18];
        data[2] = max_moving_gate;
        data[6] = 0x01;
        data[8] = max_static_gate;
        data[12] = 0x02;
        data[14] = (no_one_duration & 0xFF) as u8;
        data[15] = ((no_one_duration >> 8) & 0xFF) as u8;
        
        self.send_command(CommandType::SetDistance, &data)?;
        
        self.config.max_moving_distance_gate = max_moving_gate;
        self.config.max_static_distance_gate = max_static_gate;
        self.config.no_one_duration = no_one_duration;
        
        Ok(())
    }
    
    /// Set sensitivity for a specific gate.
    pub fn set_gate_sensitivity(&mut self, gate: u8, moving_sensitivity: u8, static_sensitivity: u8) -> Result<(), SensorError<UART::Error>> {
        if !self.in_config_mode {
            return Err(SensorError::NotInConfigMode);
        }
        if gate > 8 {
            return Err(SensorError::InvalidData);
        }
        
        let mut data = [0u8; 18];
        data[2] = gate;
        data[6] = 0x01;
        data[8] = moving_sensitivity;
        data[12] = 0x02;
        data[14] = static_sensitivity;
        
        self.send_command(CommandType::SetSensitivity, &data)?;
        
        self.config.moving_sensitivity[gate as usize] = moving_sensitivity;
        self.config.static_sensitivity[gate as usize] = static_sensitivity;
        
        Ok(())
    }
    
    /// Set all gate sensitivities to the same value.
    pub fn set_all_sensitivities(&mut self, moving_sensitivity: u8, static_sensitivity: u8) -> Result<(), SensorError<UART::Error>> {
        if !self.in_config_mode {
            return Err(SensorError::NotInConfigMode);
        }
        
        let mut data = [0u8; 18];
        data[2] = 0xFF;
        data[3] = 0xFF;
        data[6] = 0x01;
        data[8] = moving_sensitivity;
        data[12] = 0x02;
        data[14] = static_sensitivity;
        
        self.send_command(CommandType::SetSensitivity, &data)?;
        
        for i in 0..9 {
            self.config.moving_sensitivity[i] = moving_sensitivity;
            self.config.static_sensitivity[i] = static_sensitivity;
        }
        
        Ok(())
    }
    
    /// Start engineering mode.
    pub fn start_engineering_mode(&mut self) -> Result<(), SensorError<UART::Error>> {
        self.send_command(CommandType::StartEngineering, &[])?;
        self.engineering_mode = true;
        Ok(())
    }
    
    /// End engineering mode.
    pub fn end_engineering_mode(&mut self) -> Result<(), SensorError<UART::Error>> {
        self.send_command(CommandType::EndEngineering, &[])?;
        self.engineering_mode = false;
        Ok(())
    }
    
    /// Factory reset the device.
    pub fn factory_reset(&mut self) -> Result<(), SensorError<UART::Error>> {
        if !self.in_config_mode {
            return Err(SensorError::NotInConfigMode);
        }
        self.send_command(CommandType::FactoryReset, &[])?;
        self.config = Ld2410Config::default();
        Ok(())
    }
    
    /// Reboot the device.
    pub fn reboot(&mut self) -> Result<(), SensorError<UART::Error>> {
        if !self.in_config_mode {
            return Err(SensorError::NotInConfigMode);
        }
        self.send_command(CommandType::Reboot, &[])?;
        self.in_config_mode = false;
        self.engineering_mode = false;
        Ok(())
    }
    
    /// Get firmware version.
    pub fn get_firmware_version(&mut self) -> Result<(), SensorError<UART::Error>> {
        self.send_command(CommandType::GetFirmware, &[])?;
        Ok(())
    }
    
    /// Set baudrate (0-7).
    pub fn set_baudrate(&mut self, baud_idx: u8) -> Result<(), SensorError<UART::Error>> {
        if !self.in_config_mode {
            return Err(SensorError::NotInConfigMode);
        }
        if baud_idx > 7 {
            return Err(SensorError::InvalidData);
        }
        self.send_command(CommandType::SetBaudrate, &[baud_idx, 0x00])?;
        Ok(())
    }
    
    /// Read configuration parameters from the device.
    pub fn read_parameters(&mut self) -> Result<(), SensorError<UART::Error>> {
        if !self.in_config_mode {
            return Err(SensorError::NotInConfigMode);
        }
        self.send_command(CommandType::ReadParameters, &[])?;
        Ok(())
    }
}