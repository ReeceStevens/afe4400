//! # AFE4400 - an `embedded-hal` compatible driver
//!
//! The [AFE4400][afe4400] is a pulse oximetry analog front-end chip by TI. It controls LED switching, has
//! ambient light cancellation, and other very nice features for pulse oximetry applications.
//!
//! The main communication interface for the AFE4400 is via SPI. However, there are also some
//! digital pins that can be configured for smoother operation; two are required and four are for
//! additional diagnostic value.
//!
//! ## Mandatory
//! - `adc_pdn`: The powerdown pin; should be held high (not floating!) for the duration of operation.
//! - `adc_rdy`: The "ADC Ready" pin, indicating a sample is ready for reading.
//! It is useful to use `adc_rdy` as the driver for an edge-triggered interrupt.
//!
//! ## Optional
//! - `daig_end`: Diagnostics complete
//! - `adc_done`: ADC conversion complete
//! - `led_err`: Connection issue with the LEDs
//! - `sensor_err`: Connection issue with the photodiodes
//!
//! [afe4400]: http://www.ti.com/product/afe4400
#![no_std]
#![allow(dead_code)]

extern crate embedded_hal as hal;
#[macro_use]
extern crate nb;

use hal::spi::FullDuplex;
use hal::digital::{InputPin, OutputPin};


/* Useful commands */
const SW_RST: u8 = 0x04;

/// Registers
///
/// See p.49 of docs for full register map.
mod registers {
    pub const CONTROL0: u8 = 0x00;
    pub const LED2STC: u8 = 0x01;
    pub const LED2ENDC: u8 = 0x02;
    pub const LED2LEDSTC: u8 = 0x03;
    pub const LED2LEDENDC: u8 = 0x04;
    pub const ALED2STC: u8 = 0x05;
    pub const ALED2ENDC: u8 = 0x06;

    pub const LED1STC: u8 = 0x07;
    pub const LED1ENDC: u8 = 0x08;
    pub const LED1LEDSTC: u8 = 0x09;
    pub const LED1LEDENDC: u8 = 0x0A;
    pub const ALED1STC: u8 = 0x0B;
    pub const ALED1ENDC: u8 = 0x0C;

    pub const LED2CONVST: u8 = 0x0D;
    pub const LED2CONVEND: u8 = 0x0E;
    pub const ALED2CONVST: u8 = 0x0F;
    pub const ALED2CONVEND: u8 = 0x10;

    pub const LED1CONVST: u8 = 0x11;
    pub const LED1CONVEND: u8 = 0x12;
    pub const ALED1CONVST: u8 = 0x13;
    pub const ALED1CONVEND: u8 = 0x14;

    pub const ADCRSTSTCT0: u8 = 0x15;
    pub const ADCRSTENDCT0: u8 = 0x16;
    pub const ADCRSTSTCT1: u8 = 0x17;
    pub const ADCRSTENDCT1: u8 = 0x18;
    pub const ADCRSTSTCT2: u8 = 0x19;
    pub const ADCRSTENDCT2: u8 = 0x1A;
    pub const ADCRSTSTCT3: u8 = 0x1B;
    pub const ADCRSTENDCT3: u8 = 0x1C;

    pub const PRPCOUNT: u8 = 0x1D;

    pub const CONTROL1: u8 = 0x1E;
    pub const TIAGAIN: u8 = 0x20;
    pub const TIA_AMB_GAIN: u8 = 0x21;
    pub const LEDCNTRL: u8 = 0x22;
    pub const CONTROL2: u8 = 0x23;

    pub const ALARM: u8 = 0x29;
    pub const LED2VAL: u8 = 0x2A;
    pub const ALED2VAL: u8 = 0x2B;
    pub const LED1VAL: u8 = 0x2C;
    pub const ALED1VAL: u8 = 0x2D;
    pub const LED2_ALED2VAL: u8 = 0x2E;
    pub const LED1_ALED1VAL: u8 = 0x2F;

    pub const DIAG: u8 = 0x30;
}

/// Controller in charge of communication with AFE4400
pub struct Afe4400<SPI: FullDuplex<u8>, IN: InputPin, OUT: OutputPin> {
    pub spi: SPI,
    pub diag_end: Option<IN>,
    pub adc_done: Option<IN>,
    pub led_err: Option<IN>,
    pub sensor_err: Option<IN>,
    pub afe_pdn: OUT,
    pub adc_rdy: IN
}

/// Possible AFE4400 errors.
///
/// Currently, this is simply failing the self-check or passing forward a communication error.
pub enum Error<E> {
    SelfCheckFail,
    Other(E),
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::Other(error)
    }
}

type SpiError<SPI> = <SPI as FullDuplex<u8>>::Error;
type AfeError<SPI> = Error<SpiError<SPI>>;

impl<SPI: FullDuplex<u8>, IN: InputPin, OUT: OutputPin> Afe4400<SPI, IN, OUT> {

    /// Send data to a specified register.
    ///
    /// `register` should be a constant defined in `afe4400::registers`.
    ///
    /// Can return an SPI error if communication failure occurs.
    pub fn writeData(&mut self, register: u8, data: u32) -> Result<(), AfeError<SPI>> {
        block!(self.spi.send(register))?;
        let first_transfer = ((data >> 16) & 0xFF) as u8;
        let second_transfer = ((data >> 8) & 0xFF) as u8;
        let third_transfer = (data & 0xFF) as u8;
        block!(self.spi.send(first_transfer))?;
        block!(self.spi.send(second_transfer))?;
        block!(self.spi.send(third_transfer))?;
        Ok(())
    }

    /// Read data from a specified register
    ///
    /// `register` should be a constant defined in `afe4400::registers`.
    ///
    /// Can return an SPI error if communication failure occurs.
    pub fn readData(&mut self, register: u8) -> Result<u32, AfeError<SPI>> {
        self.writeData(registers::CONTROL0, 0x01 as u32)?;
        block!(self.spi.send(register))?;
        let mut register_data: u32 = 0;
        for _ in 0..3 {
            register_data = register_data << 8;
            register_data |= block!(self.spi.read())? as u32;
        }
        Ok(register_data)
    }

    fn getLedData(&mut self, led_register: u8) -> Result<u32, AfeError<SPI>> {
        let mut value = self.readData(led_register)?;
        if value & 0x00200000 != 0 {
            value |= 0xFFD00000;
        }
        Ok(value)
    }

    /// Get data from LED1 (IR) after subtracting ambient background.
    pub fn getLed1Data(&mut self) -> Result<u32, AfeError<SPI>> {
        self.getLedData(registers::LED1_ALED1VAL)
    }

    /// Get data from LED2 (Red) after subtracting ambient background.
    pub fn getLed2Data(&mut self) -> Result<u32, AfeError<SPI>> {
        self.getLedData(registers::LED2_ALED2VAL)
    }

    /// Set the cancellation filter gain resistor `R_f` value
    ///
    /// LED current and cancellation filter gain are used to compensate for changing ambient light
    /// conditions.
    ///
    /// TIA_AMB_GAIN: RF_LED[2:0] set to 110
    /// (see p.64 in docs for all `R_f` options)
    pub fn setCancellationFilters(&mut self, value: u16) -> Result<(), AfeError<SPI>> {
        let mut tia_settings = self.readData(registers::TIA_AMB_GAIN)?;
        tia_settings &= !(0x07);
        tia_settings |= (value as u32) | 0x4400;
        self.writeData(registers::TIA_AMB_GAIN, tia_settings)?;
        Ok(())
    }

    /// Set current sent to LEDs.
    ///
    /// LED current and cancellation filter gain are used to compensate for changing ambient light
    /// conditions.
    ///
    /// LEDCNTRL (0x22)
    ///   LED1[15:8], LED2[7:0]
    ///  Formula:
    ///      LED_Register_value
    ///      ------------------  *  50 mA = current
    ///           256
    ///
    pub fn setLedCurrent(&mut self, value: u8) -> Result<u32, AfeError<SPI>> {
        let both_leds = ((value as u32) << 8) + value as u32;
        self.writeData(registers::LEDCNTRL, both_leds + 0x010000)?;
        self.readData(registers::LEDCNTRL)
    }

    /// Recommended default pulse timings according to the AFE4400 data sheet.
    pub fn defaultPulseTimings(&mut self) -> Result<(), AfeError<SPI>> {
        self.writeData(registers::CONTROL0, 0x000000)?;
        self.writeData(registers::CONTROL1, 0x0102)?; // Enable timers
        self.writeData(registers::CONTROL2, 0x020100)?;
        self.writeData(registers::LED2STC, 6050)?;
        self.writeData(registers::LED2ENDC, 7998)?;
        self.writeData(registers::LED2LEDSTC, 6000)?;
        self.writeData(registers::LED2LEDENDC, 7999)?;
        self.writeData(registers::ALED2STC, 50)?;
        self.writeData(registers::ALED2ENDC, 1998)?;
        self.writeData(registers::LED1STC, 2050)?;
        self.writeData(registers::LED1ENDC, 3998)?;
        self.writeData(registers::LED1LEDSTC, 2000)?;
        self.writeData(registers::LED1LEDENDC, 3999)?;
        self.writeData(registers::ALED1STC, 4050)?;
        self.writeData(registers::ALED1ENDC, 5998)?;
        self.writeData(registers::LED2CONVST, 4)?;
        self.writeData(registers::LED2CONVEND, 1999)?;
        self.writeData(registers::ALED2CONVST, 2004)?;
        self.writeData(registers::ALED2CONVEND, 3999)?;
        self.writeData(registers::LED1CONVST, 4004)?;
        self.writeData(registers::LED1CONVEND, 5999)?;
        self.writeData(registers::ALED1CONVST, 6004)?;
        self.writeData(registers::ALED1CONVEND, 7999)?;
        self.writeData(registers::ADCRSTSTCT0, 0)?;
        self.writeData(registers::ADCRSTENDCT0, 3)?;
        self.writeData(registers::ADCRSTSTCT1, 2000)?;
        self.writeData(registers::ADCRSTENDCT1, 2003)?;
        self.writeData(registers::ADCRSTSTCT2, 4000)?;
        self.writeData(registers::ADCRSTENDCT2, 4003)?;
        self.writeData(registers::ADCRSTSTCT3, 6000)?;
        self.writeData(registers::ADCRSTENDCT3, 6003)?;
        self.writeData(registers::PRPCOUNT, 7999)?;
        self.setCancellationFilters(0x06)?;
        self.setLedCurrent(0x1A)?;
        Ok(())
    }

    /// Perform a self check to verify connections and chip clock integrity.
    ///
    /// Returns Ok(()) if the self-check passes
    pub fn self_check(&mut self) -> Result<(), AfeError<SPI>> {
        let original_value = self.readData(registers::CONTROL1)?;
        for _ in 0..5 {
            if self.readData(registers::CONTROL1)? != original_value {
                return Err(Error::SelfCheckFail);
            }
        }
        if self.readData(registers::CONTROL1)? != 0 {
            Ok(())
        } else {
            Err(Error::SelfCheckFail)
        }
    }

}
