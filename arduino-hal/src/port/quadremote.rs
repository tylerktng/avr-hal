pub use atmega_hal::port::mode;
pub use atmega_hal::port::Pin;

avr_hal_generic::renamed_pins! {
    type Pin = Pin;

    /// Pins of the **NVSL Quadcopter Remote**.
    ///
    /// This struct is best initialized via the [`arduino_hal::pins!()`][pins] macro.
    pub struct Pins from atmega_hal::Pins {
        /// `D0` / `USART_RX`
        ///
        /// * RXD (USART input pin)
        pub d0: atmega_hal::port::PE0 = pe0,
        /// `D1` / `TX`
        ///
        /// * TXD (USART output pin)
        pub d1: atmega_hal::port::PE1 = pe1,
        /// `D2`
        ///
        pub d2: atmega_hal::port::PE2 = pe2,
        /// `D3`
        ///
        /// * **PWM**: [atmega168_hal::timer::Timer3Pwm]
        /// * INT1 (external interrupt 1 input)
        /// * OC2B (Timer/Counter2 output compare match B output)
        /// * PCINT19 (pin change interrupt 19)
        pub d3: atmega_hal::port::PE3 = pe3,
        /// `D4`
        /// * **PWM**: [atmega168_hal::timer::Timer3Pwm]
        /// * XCK (USART external clock input/output)
        /// * T0 (Timer/Counter 0 external counter input)
        /// * PCINT20 (pin change interrupt 20)
        pub d4: atmega_hal::port::PE4 = pe4,
        /// `D5`
        ///
        /// * **PWM**: [atmega168_hal::timer::Timer3Pwm]
        /// * T1 (Timer/Counter 1 external counter input)
        /// * OC0B (Timer/Counter0 output compare match B output)
        /// * PCINT21 (pin change interrupt 21)
        pub d5: atmega_hal::port::PE5 = pe5,
        /// `D6`
        ///
        /// * **PWM**: [atmega168_hal::timer::Timer3Pwm]
        /// * AIN0 (analog comparator positive input)
        /// * OC0A (Timer/Counter0 output compare match A output)
        /// * PCINT22 (pin change interrupt 22)
        pub d6: atmega_hal::port::PE6 = pe6,
        /// `D7`
        ///
        /// * AIN1 (analog comparator negative input)
        /// * PCINT23 (pin change interrupt 23)
        pub d7: atmega_hal::port::PE7 = pe7,
        /// `D8`
        ///
        /// * **PWM**: [atmega168_hal::timer::Timer3Pwm]
        /// * ICP1 (Timer/Counter1 input capture input)
        /// * CLKO (divided system clock output)
        /// * PCINT0 (pin change interrupt 0)
        pub d8: atmega_hal::port::PB5 = pb5,
        /// `D9`
        ///
        /// * **PWM**: [atmega168_hal::timer::Timer3Pwm]
        /// * OC1A (Timer/Counter1 output compare match A output)
        /// * PCINT1 (pin change interrupt 1)
        pub d9: atmega_hal::port::PB4 = pb4,
        /// `D10`
        ///
        /// * **PWM**: [atmega168_hal::timer::Timer3Pwm]
        /// * SPI_SSN (SPI bus master slave select)
        pub d10: atmega_hal::port::PB0 = pb0,
        /// `D11`
        ///
        /// * **PWM**: [atmega168_hal::timer::Timer3Pwm]
        /// * SPI_MOSI (SPI bus master/slave input)
        pub d11: atmega_hal::port::PB2 = pb2,
        /// `D12`
        ///
        /// * SPI_MISO (SPI bus master input/slave output)
        pub d12: atmega_hal::port::PB3 = pb3,
        /// `D13`
        ///
        /// * SPI_SCK (SPI bus master clock input)
        pub d13: atmega_hal::port::PB1 = pb1,

        /// `D14`
        ///
		/// I2C_SDA
		pub d14: atmega_hal::port::PD1 = pd1,

        /// `D15`
        ///
		/// I2C_SCL
		pub d15: atmega_hal::port::PD0 = pd0,

		/// `D16`
		///
		/// LED1 / LED
		pub d16: atmega_hal::port::PG0 = pg0,

		/// `D17
		pub d17: atmega_hal::port::PG1 = pg1,

		pub d18: atmega_hal::port::PG2 = pg2,

		pub d19: atmega_hal::port::PG5 = pg5,

		pub d20: atmega_hal::port::PD2 = pd2,

		pub d21: atmega_hal::port::PD3 = pd3,
		pub d22: atmega_hal::port::PD4 = pd4,
		pub d23: atmega_hal::port::PD5 = pd5,
		pub d24: atmega_hal::port::PD6 = pd6,
		pub d25: atmega_hal::port::PD7 = pd7,

		//Is this A0-A7, or D26-D33? Or both?
		pub d26: atmega_hal::port::PF0 = pf0,
		pub d27: atmega_hal::port::PF1 = pf1,
		pub d28: atmega_hal::port::PF2 = pf2,
		pub d29: atmega_hal::port::PF3 = pf3,
		pub d30: atmega_hal::port::PF4 = pf4,
		pub d31: atmega_hal::port::PF5 = pf5,
		pub d32: atmega_hal::port::PF6 = pf6,
		pub d33: atmega_hal::port::PF7 = pf7,

		pub d34: atmega_hal::port::PB6 = pb6,
		pub d35: atmega_hal::port::PB7 = pb7,
		pub d36: atmega_hal::port::PG3 = pg3,
		pub d37: atmega_hal::port::PG4 = pg4,
		// //A0-A7
		// pub a0: atmega_hal::port::PF0 = pf0,
		// pub a1: atmega_hal::port::PF1 = pf1,
		// pub a2: atmega_hal::port::PF2 = pf2,
		// pub a3: atmega_hal::port::PF3 = pf3,
		// pub a4: atmega_hal::port::PF4 = pf4,
		// pub a5: atmega_hal::port::PF5 = pf5,
		// pub a6: atmega_hal::port::PF6 = pf6,
		// pub a7: atmega_hal::port::PF7 = pf7,
    }
}
