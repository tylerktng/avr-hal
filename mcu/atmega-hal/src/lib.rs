#![no_std]
#![feature(abi_avr_interrupt)]

//! `atmega-hal`
//! =============
//! Common HAL (hardware abstraction layer) for ATmega* microcontrollers.
//!
//! **Note**: This version of the documentation was built for
#![cfg_attr(feature = "atmega48p", doc = "**ATmega48P**.")]
#![cfg_attr(feature = "atmega168", doc = "**ATmega168**.")]
#![cfg_attr(feature = "atmega328p", doc = "**ATmega328P**.")]
#![cfg_attr(feature = "atmega328pb", doc = "**ATmega328PB**.")]
#![cfg_attr(feature = "atmega32u4", doc = "**ATmega32U4**.")]
#![cfg_attr(feature = "atmega2560", doc = "**ATmega2560**.")]
#![cfg_attr(feature = "atmega1280", doc = "**ATmega1280**.")]
#![cfg_attr(feature = "atmega128rfa1", doc = "**ATmega128rfa1**.")]
//! This means that only items which are available for this MCU are visible.  If you are using
//! a different chip, try building the documentation locally with:
//!
//! ```text
//! cargo doc --features <your-mcu> --open
//! ```

#[cfg(all(
    not(feature = "device-selected"),
    not(feature = "disable-device-selection-error")
))]
compile_error!(
    "This crate requires you to specify your target chip as a feature.

    Please select one of the following

    * atmega48p
    * atmega168
    * atmega328p
    * atmega328pb
    * atmega32u4
    * atmega1280
    * atmega2560
	* atmega128rfa1
    "
);

/// Reexport of `atmega1280` from `avr-device`
#[cfg(feature = "atmega1280")]
pub use avr_device::atmega1280 as pac;
/// Reexport of `atmega168` from `avr-device`
#[cfg(feature = "atmega168")]
pub use avr_device::atmega168 as pac;
/// Reexport of `atmega2560` from `avr-device`
#[cfg(feature = "atmega2560")]
pub use avr_device::atmega2560 as pac;
/// Reexport of `atmega328p` from `avr-device`
#[cfg(feature = "atmega328p")]
pub use avr_device::atmega328p as pac;
/// Reexport of `atmega328pb` from `avr-device`
#[cfg(feature = "atmega328pb")]
pub use avr_device::atmega328pb as pac;
/// Reexport of `atmega32u4` from `avr-device`
#[cfg(feature = "atmega32u4")]
pub use avr_device::atmega32u4 as pac;
/// Reexport of `atmega48p` from `avr-device`
#[cfg(feature = "atmega48p")]
pub use avr_device::atmega48p as pac;
#[cfg(feature = "atmega128rfa1")]
pub use avr_device::atmega128rfa1 as pac;

/// See [`avr_device::entry`](https://docs.rs/avr-device/latest/avr_device/attr.entry.html).
#[cfg(feature = "rt")]
pub use avr_device::entry;

#[cfg(feature = "device-selected")]
pub use pac::Peripherals;

pub use avr_hal_generic::clock;
pub use avr_hal_generic::delay;

#[cfg(feature = "device-selected")]
pub mod adc;
#[cfg(feature = "device-selected")]
pub use adc::Adc;

#[cfg(feature = "device-selected")]
pub mod i2c;
#[cfg(feature = "device-selected")]
pub use i2c::I2c;

#[cfg(feature = "device-selected")]
pub mod spi;
#[cfg(feature = "device-selected")]
pub use spi::Spi;

#[cfg(feature = "device-selected")]
pub mod port;
#[cfg(feature = "device-selected")]
pub use port::Pins;

#[cfg(feature = "device-selected")]
pub mod usart;
#[cfg(feature = "device-selected")]
pub use usart::Usart;

#[cfg(feature = "device-selected")]
pub mod wdt;
#[cfg(feature = "device-selected")]
pub use wdt::Wdt;

pub struct Atmega;

#[cfg(any(feature = "atmega48p", feature = "atmega168", feature = "atmega328p"))]
#[macro_export]
macro_rules! pins {
    ($p:expr) => {
        $crate::Pins::new($p.PORTB, $p.PORTC, $p.PORTD)
    };
}
#[cfg(feature = "atmega328pb")]
#[macro_export]
macro_rules! pins {
    ($p:expr) => {
        $crate::Pins::new($p.PORTB, $p.PORTC, $p.PORTD, $p.PORTE)
    };
}
#[cfg(feature = "atmega32u4")]
#[macro_export]
macro_rules! pins {
    ($p:expr) => {
        $crate::Pins::new($p.PORTB, $p.PORTC, $p.PORTD, $p.PORTE, $p.PORTF)
    };
}
#[cfg(any(feature = "atmega1280", feature = "atmega2560"))]
#[macro_export]
macro_rules! pins {
    ($p:expr) => {
        $crate::Pins::new(
            $p.PORTA, $p.PORTB, $p.PORTC, $p.PORTD, $p.PORTE, $p.PORTF, $p.PORTG, $p.PORTH,
            $p.PORTJ, $p.PORTK, $p.PORTL,
        )
    };
}

#[cfg(feature = "atmega128rfa1")]
#[macro_export]
macro_rules! pins {
    ($p:expr) => {
        $crate::Pins::new($p.PORTA, $p.PORTB, $p.PORTC, $p.PORTD, $p.PORTE, $p.PORTF, $p.PORTG)
    };
}

#[cfg(feature = "atmega128rfa1")]
pub mod trx24; //RF registers, pins, etc
