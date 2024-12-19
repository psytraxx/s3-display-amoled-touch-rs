#![no_std]

use embedded_hal::spi::Operation;
use embedded_hal::spi::SpiDevice;
use esp_hal::{
    delay::Delay,
    gpio::{GpioPin, Level, Output},
};

// Define a structure for the LCD command
struct LcdCommand<'a> {
    addr: u8,                 // Command address
    params: &'a [u8],         // Command parameters
    delay_after: Option<u32>, // Delay in milliseconds after sending the command
}

// AMOLED initialization commands
const AMOLED_INIT_CMDS: &[LcdCommand] = &[
    LcdCommand {
        addr: 0xFE,
        params: &[0x04],
        delay_after: None,
    }, // SET APGE3
    LcdCommand {
        addr: 0x6A,
        params: &[0x00],
        delay_after: None,
    },
    LcdCommand {
        addr: 0xFE,
        params: &[0x05],
        delay_after: None,
    }, // SET APGE4
    LcdCommand {
        addr: 0xFE,
        params: &[0x07],
        delay_after: None,
    }, // SET APGE6
    LcdCommand {
        addr: 0x07,
        params: &[0x4F],
        delay_after: None,
    },
    LcdCommand {
        addr: 0xFE,
        params: &[0x01],
        delay_after: None,
    }, // SET APGE0
    LcdCommand {
        addr: 0x2A,
        params: &[0x02],
        delay_after: None,
    },
    LcdCommand {
        addr: 0x2B,
        params: &[0x73],
        delay_after: None,
    },
    LcdCommand {
        addr: 0xFE,
        params: &[0x0A],
        delay_after: None,
    }, // SET APGE9
    LcdCommand {
        addr: 0x29,
        params: &[0x10],
        delay_after: None,
    },
    LcdCommand {
        addr: 0xFE,
        params: &[0x00],
        delay_after: None,
    },
    LcdCommand {
        addr: 0x51,
        params: &[175],
        delay_after: None,
    }, // Set brightness (175 in decimal)
    LcdCommand {
        addr: 0x53,
        params: &[0x20],
        delay_after: None,
    },
    LcdCommand {
        addr: 0x35,
        params: &[0x00],
        delay_after: None,
    },
    LcdCommand {
        addr: 0x3A,
        params: &[0x75],
        delay_after: None,
    }, // Interface Pixel Format 16bit/pixel
    LcdCommand {
        addr: 0xC4,
        params: &[0x80],
        delay_after: None,
    },
    LcdCommand {
        addr: 0x11,
        params: &[0x00],
        delay_after: Some(120),
    }, // Sleep Out with 120ms delay
    LcdCommand {
        addr: 0x29,
        params: &[0x00],
        delay_after: Some(120),
    }, // Display ON with 120ms delay
];

pub struct DisplayDriver<SPI> {
    spi: SPI,
}

impl<SPI> DisplayDriver<SPI>
where
    SPI: SpiDevice,
{
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    pub fn init(&mut self, dc_pin: GpioPin<7>, rst_pin: GpioPin<17>, delay: &mut Delay)
    where
        SPI: SpiDevice,
    {
        let mut rst_pin = Output::new(rst_pin, Level::Low);
        let mut dc_pin = Output::new(dc_pin, Level::Low);

        // Reset the display
        rst_pin.set_high();
        delay.delay_millis(200);
        rst_pin.set_low();
        delay.delay_millis(300);
        rst_pin.set_high();
        delay.delay_millis(200);

        // Send initialization commands
        for cmd in AMOLED_INIT_CMDS {
            // Set DC low for command
            dc_pin.set_low();

            // Write the command address
            self.spi
                .transaction(&mut [Operation::Write(&[cmd.addr])])
                .unwrap();

            // Set DC high for data
            dc_pin.set_high();

            if !cmd.params.is_empty() {
                // Set DC low for parameter
                dc_pin.set_low();

                // Write the command parameters
                self.spi
                    .transaction(&mut [Operation::Write(cmd.params)])
                    .unwrap();
            }

            // Apply delay if specified
            if let Some(ms) = cmd.delay_after {
                delay.delay_millis(ms);
            }
        }
    }
}
