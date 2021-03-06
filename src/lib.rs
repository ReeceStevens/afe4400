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

use core::fmt::Debug;

use hal::spi::FullDuplex;
use hal::digital::v2::{InputPin, OutputPin};


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

pub enum LEDSource {
    LED,
    AMB,
    LEDAMB,
}

/// Possible AFE4400 errors.
///
/// Currently, this is simply failing the self-check or passing forward a communication error.
#[derive(Debug)]
pub enum Error<E: Debug> {
    SelfCheckFail,
    InvalidSetting,
    Other(E),
}

impl<E: Debug> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::Other(error)
    }
}

type SpiError<SPI> = <SPI as FullDuplex<u8>>::Error;
type AfeError<SPI> = Error<SpiError<SPI>>;

impl<SPI, IN, OUT> Afe4400<SPI, IN, OUT>
    where SPI: FullDuplex<u8>,
          IN: InputPin,
          OUT: OutputPin,
          <SPI as FullDuplex<u8>>::Error: Debug
{

    fn spi_send(&mut self, data: u8) -> Result<(), AfeError<SPI>> {
        block!(self.spi.send(data))?;
        block!(self.spi.read())?; // Dummy read, toss the result.
        Ok(())
    }

    fn spi_read(&mut self) -> Result<u8, AfeError<SPI>> {
        let dummy = 0_u8;
        block!(self.spi.send(dummy))?; // Dummy write for full duplex
        let result = block!(self.spi.read())?;
        Ok(result)
    }

    /// Send data to a specified register.
    ///
    /// `register` should be a constant defined in `afe4400::registers`.
    ///
    /// Can return an SPI error if communication failure occurs.
    pub fn write_data(&mut self, register: u8, data: u32) -> Result<(), AfeError<SPI>> {
        self.spi_send(register)?;
        let first_transfer = ((data >> 16) & 0xFF) as u8;
        let second_transfer = ((data >> 8) & 0xFF) as u8;
        let third_transfer = (data & 0xFF) as u8;
        self.spi_send(first_transfer)?;
        self.spi_send(second_transfer)?;
        self.spi_send(third_transfer)?;
        Ok(())
    }

    /// Read data from a specified register
    ///
    /// `register` should be a constant defined in `afe4400::registers`.
    ///
    /// Can return an SPI error if communication failure occurs.
    pub fn read_data(&mut self, register: u8) -> Result<u32, AfeError<SPI>> {
        self.write_data(registers::CONTROL0, 0x01 as u32)?;
        self.spi_send(register)?;
        let mut register_data: u32 = 0;
        for _ in 0..3 {
            register_data = register_data << 8;
            register_data |= self.spi_read()? as u32;
        }
        self.write_data(registers::CONTROL0, 0x00 as u32)?;
        Ok(register_data)
    }

    #[allow(overflowing_literals)]
    fn get_led_data(&mut self, led_register: u8) -> Result<i32, AfeError<SPI>> {
        let mut value = self.read_data(led_register)? as i32;
        if value & 0x00200000 != 0 {
            value |= 0xFFC00000;
        }
        Ok(value)
    }

    /// Get data from LED1 (IR)
    pub fn get_led1_data(&mut self, source: LEDSource) -> Result<i32, AfeError<SPI>> {
        match source {
            LEDSource::LED => self.get_led_data(registers::LED1VAL),
            LEDSource::AMB => self.get_led_data(registers::ALED1VAL),
            LEDSource::LEDAMB => self.get_led_data(registers::LED1_ALED1VAL),
        }
    }

    /// Get data from LED1 (IR)
    pub fn get_led2_data(&mut self, source: LEDSource) -> Result<i32, AfeError<SPI>> {
        match source {
            LEDSource::LED => self.get_led_data(registers::LED2VAL),
            LEDSource::AMB => self.get_led_data(registers::ALED2VAL),
            LEDSource::LEDAMB => self.get_led_data(registers::LED2_ALED2VAL),
        }
    }

    pub fn set_cancellation_current(&mut self, current: u8) -> Result<(), AfeError<SPI>> {
        if current > 0b1010 {
            Err(Error::InvalidSetting)
        } else {
            let mask = 0b1111 << 16;
            let mut tia_settings = self.read_data(registers::TIA_AMB_GAIN)?;
            tia_settings &= !mask;
            tia_settings |= (current as u32) << 16;
            self.write_data(registers::TIA_AMB_GAIN, tia_settings)?;
            Ok(())
        }
    }

    pub fn set_second_stage_gain(&mut self, gain: u8, enabled: bool) -> Result<(), AfeError<SPI>> {
        if gain > 0b0100 {
            Err(Error::InvalidSetting)
        } else {
            let mask = (0b01 << 14) & (0b0111 << 8);
            let mut tia_settings = self.read_data(registers::TIA_AMB_GAIN)?;
            tia_settings &= !mask;
            tia_settings |= (gain as u32) << 8;
            tia_settings |= if enabled { 0b01 << 14 } else { 0b00 << 14 };
            self.write_data(registers::TIA_AMB_GAIN, tia_settings)?;
            Ok(())
        }
    }

    pub fn set_first_stage_cap(&mut self, cap_value: u8) -> Result<(), AfeError<SPI>> {
        if cap_value > 0b11111 {
            Err(Error::InvalidSetting)
        } else {
            let mask = 0b011111 << 3;
            let mut tia_settings = self.read_data(registers::TIA_AMB_GAIN)?;
            tia_settings &= !mask;
            tia_settings |= (cap_value as u32) << 3;
            self.write_data(registers::TIA_AMB_GAIN, tia_settings)?;
            Ok(())
        }
    }

    pub fn set_first_stage_gain(&mut self, rf_value: u8) -> Result<(), AfeError<SPI>> {
        if rf_value > 0b111 {
            Err(Error::InvalidSetting)
        } else {
            let mask = 0b0111;
            let mut tia_settings = self.read_data(registers::TIA_AMB_GAIN)?;
            tia_settings &= !mask;
            tia_settings |= rf_value as u32;
            self.write_data(registers::TIA_AMB_GAIN, tia_settings)?;
            Ok(())
        }
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
    pub fn set_led_current(&mut self, value: u8) -> Result<u32, AfeError<SPI>> {
        let both_leds = ((value as u32) << 8) + value as u32;
        self.write_data(registers::LEDCNTRL, both_leds)?;
        self.read_data(registers::LEDCNTRL)
    }

    pub fn get_led_current(&mut self) -> Result<u8, AfeError<SPI>> {
        Ok((self.read_data(registers::LEDCNTRL)? & 0x00FF) as u8)
    }

    /// Recommended default pulse timings according to the AFE4400 data sheet.
    pub fn default_pulse_timings(&mut self) -> Result<(), AfeError<SPI>> {
        self.write_data(registers::CONTROL0, 0x000000)?;
        self.write_data(registers::CONTROL1, 0x0102)?; // Enable timers
        self.write_data(registers::CONTROL2, 0x000100)?;
        self.write_data(registers::LED2STC, 6050)?;
        self.write_data(registers::LED2ENDC, 7998)?;
        self.write_data(registers::LED2LEDSTC, 6000)?;
        self.write_data(registers::LED2LEDENDC, 7999)?;
        self.write_data(registers::ALED2STC, 50)?;
        self.write_data(registers::ALED2ENDC, 1998)?;
        self.write_data(registers::LED1STC, 2050)?;
        self.write_data(registers::LED1ENDC, 3998)?;
        self.write_data(registers::LED1LEDSTC, 2000)?;
        self.write_data(registers::LED1LEDENDC, 3999)?;
        self.write_data(registers::ALED1STC, 4050)?;
        self.write_data(registers::ALED1ENDC, 5998)?;
        self.write_data(registers::LED2CONVST, 4)?;
        self.write_data(registers::LED2CONVEND, 1999)?;
        self.write_data(registers::ALED2CONVST, 2004)?;
        self.write_data(registers::ALED2CONVEND, 3999)?;
        self.write_data(registers::LED1CONVST, 4004)?;
        self.write_data(registers::LED1CONVEND, 5999)?;
        self.write_data(registers::ALED1CONVST, 6004)?;
        self.write_data(registers::ALED1CONVEND, 7999)?;
        self.write_data(registers::ADCRSTSTCT0, 0)?;
        self.write_data(registers::ADCRSTENDCT0, 3)?;
        self.write_data(registers::ADCRSTSTCT1, 2000)?;
        self.write_data(registers::ADCRSTENDCT1, 2003)?;
        self.write_data(registers::ADCRSTSTCT2, 4000)?;
        self.write_data(registers::ADCRSTENDCT2, 4003)?;
        self.write_data(registers::ADCRSTSTCT3, 6000)?;
        self.write_data(registers::ADCRSTENDCT3, 6003)?;
        self.write_data(registers::PRPCOUNT, 7999)?;
        // self.set_cancellation_filters(0x06)?;
        // self.set_led_current(0x1A)?;
        Ok(())
    }

    /// Perform a self check to verify connections and chip clock integrity.
    ///
    /// Returns Ok(()) if the self-check passes
    pub fn self_check(&mut self) -> Result<(), AfeError<SPI>> {
        let original_value = self.read_data(registers::CONTROL1)?;
        for _ in 0..5 {
            if self.read_data(registers::CONTROL1)? != original_value {
                return Err(Error::SelfCheckFail);
            }
        }
        if self.read_data(registers::CONTROL1)? != 0 {
            Ok(())
        } else {
            Err(Error::SelfCheckFail)
        }
    }

}
