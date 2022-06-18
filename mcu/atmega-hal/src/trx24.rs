// 2.4 GHz Trasmitter

use crate::pac::{TRX24, trx24 as rf24};
use crate::pac::pwrctrl as pc;
// pub struct IEEEMacAddress {
// 	reg0: IEEE_ADDR_0,
// 	reg1: IEEE_ADDR_1,
// 	reg2: IEEE_ADDR_2,
// 	reg3: IEEE_ADDR_3,
// 	reg4: IEEE_ADDR_4,
// 	reg5: IEEE_ADDR_5,
// 	reg6: IEEE_ADDR_6,
// 	reg7: IEEE_ADDR_7,
// }

// impl IEEEMacAddress {

// }

pub enum RfChannel {
	Mhz2405 = 11,
	Mhz2410 = 12,
	Mhz2415 = 13,
	Mhz2420 = 14,
	Mhz2425 = 15,
	Mhz2430 = 16,
	Mhz2435 = 17,
	Mhz2440 = 18,
	Mhz2445 = 19,
	Mhz2450 = 20,
	Mhz2455 = 21,
	Mhz2460 = 22,
	Mhz2465 = 23,
	Mhz2470 = 24,
	Mhz2475 = 25,
	Mhz2480 = 26,
}

impl Default for RfChannel {
	fn default() -> Self {
		Self::Mhz2405
	}
}

pub struct RfBuf {
	buf: [u8; 256],
	r_idx: usize,
	w_idx: usize,
}

impl RfBuf {
	pub fn new() -> Self {
		let buf: [u8; 256] = [0; 256];
		Self {
			buf,
			r_idx: 0,
			w_idx: 0,
		}
	}

	pub fn read(&mut self) -> &[u8] {
		let len = self.buf[self.r_idx];
		let idx = self.r_idx;
		self.r_idx = (self.r_idx + 128) % 256;
		if len > 0 {
			&self.buf[idx..idx + 128]
		} else {
			&[]
		}
	}

	pub fn write(&mut self, data: &[u8]) {
		let len = data[0] as usize;
		self.buf[self.w_idx] = len as u8;
		for i in 1..len+1 {
			self.buf[self.w_idx + i] = data[i];
		}
	}
}

pub struct AesModule {
	aes_ctrl: rf24::AES_CTRL,
	aes_status: rf24::AES_STATUS,
	aes_key: rf24::AES_KEY,
	aes_state: rf24::AES_STATE,
}

// AES Security Module
// To use:
// 1. Setup the encryption/decryption key by using `write_key()`.
// 2. Configure AES
// 3. Write data (128 bits)
// 4. Start operation using `start()`
// 5. Wait	for AES to finish (AES_READY IRQ, poll `is_done()`, or wait 24 us)
// 6. Read data (128 bits)

// Security Notes: 
// ECB Mode is insecure and is vulnerable to attacks.
// If at all possible, configure the module to use CBC.
impl AesModule {
	pub fn new(
		aes_ctrl: rf24::AES_CTRL,
		aes_status: rf24::AES_STATUS,
		aes_key: rf24::AES_KEY,
		aes_state: rf24::AES_STATE,
	) -> Self {
		Self {
			aes_ctrl,
			aes_status,
			aes_key,
			aes_state
		}
	}
/* Status */
	fn is_done(&self) -> bool {
		self.aes_status.read().aes_done().bit()
	}

	fn is_error(&self) -> bool {
		self.aes_status.read().aes_er().bit()
	}
/* Run */
	fn start(&self) {
		self.aes_ctrl.write(|w| unsafe { w.aes_request().set_bit()});
	}
/* Configuration */
	fn set_mode(&self, mode: rf24::aes_ctrl::AES_MODE_A) {
		self.aes_ctrl.write(|w| unsafe {w.aes_mode().variant(mode)});
	}

	fn set_dir(&self,dir: rf24::aes_ctrl::AES_DIR_A) {
		self.aes_ctrl.write(|w| unsafe {w.aes_dir().variant(dir)});
	}

	fn en_interrupt(&self, en: bool) {
		self.aes_ctrl.write(|w| unsafe {w.aes_im().bit(en)})
	}
/* Key */
	fn read_key(&self) -> [u8; 16] {
		let mut key: [u8; 16] = [0; 16];
		for i in 0..16 {
			key[i] = self.aes_key.read().bits();
		}
		key
	}

	fn write_key(&self, key: &[u8; 16]) {
		for i in 0..16 {
			self.aes_key.write(|w| unsafe {w.bits(key[i])});
		}
	}
/* Data */
	fn read_data(&self) -> [u8; 16] {
		let mut data: [u8; 16] = [0; 16];
		for i in 0..16 {
			data[i] = self.aes_state.read().bits();
		}
		data
	}

	fn write_data(&self, data: &[u8; 16]) {
		for i in 0..16 {
			self.aes_state.write(|w| unsafe {w.bits(data[i])});
		}
	}

}

// Radio 2.4 GHz
// To use:
// 0. Initialize by using `rfBegin()` (only need to be called once)
// 1. To listen for signals, use `rx_on()`.
pub struct Trx24 {
	/// Peripheral 
	p: crate::pac::TRX24,
	/// TRX Power management reg
	trx_pr: pc::TRXPR,

	//Mac address regs	
	// mac: MacAddress,
	//Tranceiver controller
	// trx_ctrl: rf24::TRX_CTRL_1,
	// //Tranceiver state
	// trx_state: rf24::TRX_STATE,
	// //Tranceiver status
	// trx_status: rf24::TRX_STATUS,
	// //Tranceiver pin register
	// trx_pr: pc::TRXPR,
	// //Frame buffer address start
	// trx_fb_st: rf24::TRXFBST,
	// //Frame buffer address end
	// trx_fb_end: rf24::TRXFBEND,
	// // ??
	// phy_cc_cca: rf24::PHY_CC_CCA,
	// //Interrupt
	// irq_mask: rf24::IRQ_MASK,
	// //Security
	// aes: AesModule,
	// //Channel. Should be between 11 and 26 (2405 MHz - 2480 MHz)
	// rf_channel: u8,

	// data_buf: RingBuf<u8>
}


impl Trx24 {
	/// Initializes the Trx24 module.
	/// 
	pub fn new(peripheral: TRX24, trx_pr: pc::TRXPR) -> Self {
		//TODO
		let new = Self {
			p: peripheral,
			trx_pr,
		};
		new.rfBegin(RfChannel::default());
		new
	}

	// Convenience method for configuring the tranceiver. Ported from radio.cpp
	// Original by Jim Lindblom, Sparkfun Electronics (License: Beerware)
	fn rfBegin(&self, rf_channel: RfChannel) {
		self.trx_pr.write(|w| unsafe {w.trxrst().set_bit()}); //reset
		self.p.irq_mask.write(|w| unsafe {w.bits(0)}); //clear interrupts
		self.p.trx_state.write(|w| unsafe {w.trx_cmd().cmd_trx_off()}); //turn off
		// arduino_hal::delay_ms(1); //wait for transmission to finish
		//Enable auto crc calculations
		self.p.trx_ctrl_1.write(|w| unsafe {w.tx_auto_crc_on().set_bit()});
		//Enable interrupts
		self.p.irq_mask.write(|w| unsafe {
			w.rx_start_en().set_bit()
			 .rx_end_en().set_bit()
			 .tx_end_en().set_bit()
		});
		//Configure channel (See datasheet: 9.12.12)
		let channel = rf_channel as u8;
		if channel < 11 {
			self.p.phy_cc_cca.write(|w| unsafe {w.channel().bits(11)});
		} else if channel > 26 {
			self.p.phy_cc_cca.write(|w| unsafe {w.channel().bits(26)});
		} else {
			self.p.phy_cc_cca.write(|w| unsafe {w.channel().bits(channel)});
		}
		//Default state is RX_ON to listen for incoming messages.
		self.p.trx_state.write(|w| unsafe {w.trx_cmd().cmd_rx_on()});
		unsafe {RFBUFFER = Some(RfBuf::new());}
	}

	/// Puts the tranceiver into the state TRX_OFF from SLEEP. If the state is
	/// not in SLEEP, this function returns false.
	pub fn rfWake(&self, force: bool) -> bool {
		if self.p.trx_status.read().trx_status().is_sleep() {
			self.trx_pr.write(|w| unsafe {w.slptr().clear_bit()});
			true
		} else {
			false
		}
	}

	/// Puts the tranceiver into the state SLEEP from TRX_OFF. If `force` is
	/// true, forcibly turns off the tranceiver off before setting the state.
	/// Otherwise, sets the state to off after the tranceiver is no longer busy
	/// before putting the tranceiver to SLEEP.
	pub fn rfSleep(&self, force: bool) {
		if !force {
			let status = self.p.trx_status.read().trx_status();
			// Busy wait for the status. Perhaps we can use interrupts here
			// instead?
			while status.is_busy_rx() || 
			      status.is_busy_tx() ||
			      status.is_busy_rx_aack() ||
			      status.is_busy_tx_aret() ||
			      status.is_state_transition_in_progress() {}
			self.p.trx_state.write(|w| unsafe {w.trx_cmd().cmd_trx_off()});
		} else {
			self.p.trx_state.write(|w| unsafe {w.trx_cmd().cmd_force_trx_off()});
		}
		while self.p.trx_status.read().trx_status().is_state_transition_in_progress() {}
		self.trx_pr.write(|w| unsafe {w.slptr().set_bit()});
	}

	// Write up to 128 bits of data to the frame buffer.
	pub fn rfWrite(&self, data: &[u8], len: u8) {
		self.p.trx_state.write(|w| unsafe {w.trx_cmd().cmd_pll_on()});
		//Spin until PLL is locked
		while self.p.trx_status.read().trx_status().is_pll_on() {} 
		//Frame buffer address
		unsafe {
			let fb_ptr = self.p.trxfbst.as_ptr();
			*fb_ptr = len + 2;
			for i in 0..len {
				*(fb_ptr.add((i + 1) as usize)) = data[i as usize];
			}
		}
	}

	/// Returns the number of bytes stored in the RF buffer.
	pub fn rfAvailable(&self) -> usize {
		0
	}

	pub fn rfRead(&self) {

	}
}

// Registers for callbacks. Initialize when the TRX Peripheral is initialized.
// static mut PHYRSSI: Option<rf24::PHY_RSSI> = None;
// static mut FRAMEBUF: Option<rf24::TRXFBST> = None;
static mut RFBUFFER: Option<RfBuf> = None;


/// When receiver is done, call this interrupt 
#[avr_device::interrupt(atmega128rfa1)]
fn TRX24_RX_END() {
		let validrx = unsafe {(*TRX24::ptr()).phy_rssi.read().rx_crc_valid().is_crc_valid()};
		if validrx {
			//length is contained in the first byte of the frame buffer.
			let ptr = unsafe {(*TRX24::ptr()).trxfbst.as_ptr()};
			let mut temp: [u8; 128] = [0; 128];
			//copy over 
			temp[0] = unsafe {*ptr};
			//copy over to the main mem buffer
			for i in 1..128 {
				temp[i] = unsafe{*ptr.add(i)};
			}
			unsafe {
				if RFBUFFER.is_some() {
					RFBUFFER.as_mut().map(|buf| buf.write(&temp[0..128]));
				}
			}
			


		}
}


// Move to AVR_HAL?
// macro_rules! impl_txr24 {
// 	(
// 		hal: $HAL:ty,
// 		peripheral: $TRX24:ty,
// 		mac: {
// 			$($mac_idx:expr, $mac_reg:ty),+
// 		},
// 		control: {
// 			$($control_name:ident, $control_reg:ty),+
// 		},
// 		state: $reg:ty, 
// 		aes: $($aes_ctl:ty, $aes_status:ty, $aes_state:ty, $aes_key:ty),


// 	) => {

// 		use mod 

// 		pub struct MacAddress {
// 			$(pub $mac_reg)
// 		}


// 		pub struct AESModule {
// 			controller: 
// 		}
// 	}
// }
