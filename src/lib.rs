/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;

mod interface;
pub use interface::{DeviceInterface, SerialInterface};

use hal::blocking::delay::{DelayMs, DelayUs};

mod messages;
use messages::*;

// #[cfg(debug_assertions)]
use cortex_m_semihosting::hprintln;

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE> {
    /// Sensor communication error
    Comm(CommE),

    /// Sensor is not responding
    Unresponsive,

}

pub fn new_serial_driver<UART, CommE>(
    uart: UART,
) -> UbxDriver<SerialInterface<UART>>
where
    UART: hal::serial::Read<u8, Error = CommE>,
    CommE: core::fmt::Debug,
{
    let iface = interface::SerialInterface::new(uart);
    UbxDriver::new_with_interface(iface)
}

const READ_BUF_LEN: usize = 256;

pub struct UbxDriver<DI> {
    /// the device interface
    di: DI,
    read_buf: [u8; READ_BUF_LEN],

    /// The last received NAV_PVT solution from the device, if any
    last_nav_pvt: Option<NavPosVelTimeM8>,
    last_mon_hw: Option<MonHardwareM8>,
    last_nav_dop: Option<NavDopM8>,
}

impl<DI, CommE> UbxDriver<DI>
where
    DI: DeviceInterface<InterfaceError = Error<CommE>>,
    CommE: core::fmt::Debug,
{
    pub(crate) fn new_with_interface(device_interface: DI) -> Self {
        Self {
            di: device_interface,
            read_buf: [0; READ_BUF_LEN],
            last_nav_pvt: None,
            last_mon_hw: None,
            last_nav_dop: None,
        }
    }

    pub fn setup(
        &mut self,
        _delay_source: &mut impl DelayUs<u32>,
    ) -> Result<(), DI::InterfaceError> {
        //TODO configure ublox sensor? or assume it's preconfigured?
        //let _ = self.handle_one_message(&mut delay_source);
        Ok(())
    }

    pub fn take_last_nav_pvt(&mut self) -> Option<NavPosVelTimeM8> {
        self.last_nav_pvt.take()
    }

    pub fn take_last_nav_dop(&mut self) -> Option<NavDopM8> {
        self.last_nav_dop.take()
    }

    pub fn take_last_mon_hw(&mut self) -> Option<MonHardwareM8> {
        self.last_mon_hw.take()
    }

    /// generate a 16 bit checksum for a payload
    fn checksum_for_payload(payload: &[u8], _dump_ck: bool) -> [u8; UBX_CKSUM_LEN] {
        let mut checksum = [0u8; UBX_CKSUM_LEN];
        for word in payload {
            checksum[0] = checksum[0].wrapping_add(*word);
            checksum[1] = checksum[1].wrapping_add(checksum[0]);
            //if dump_ck { hprintln!("{} : {} {}", *word, checksum[0], checksum[1]).unwrap(); }
        }
        checksum
    }

    /// Read our interface for a message of known size
    ///
    fn read_ubx_message(&mut self, msg_len: usize, dump_ck: bool) -> Result<(bool, usize), DI::InterfaceError>  {
        // The length sent in the header is defined as being that of the payload only.
        // It does not include the Preamble, Message Class, Message ID, Length, or CRC fields.
        // The number format of the length field is a Little-Endian unsigned 16-bit integer.

        let max_pay_idx = UBX_HEADER_LEN + msg_len;
        let max_idx = max_pay_idx + UBX_CKSUM_LEN;
        let desired_count = max_idx - UBX_HEADER_LEN;
        self.read_buf[max_idx] = 0;
        let read_count = self.di.read_many(&mut self.read_buf[UBX_HEADER_LEN..max_idx])?;
        if read_count < desired_count {
            hprintln!(">>> {} < {}", read_count, desired_count).unwrap();
            return Ok((false, 0));
        }
        let calc_ck = Self::checksum_for_payload(&self.read_buf[..max_pay_idx], dump_ck);
        let recvd_ck = &self.read_buf[(max_idx-UBX_CKSUM_LEN)..max_idx];
        let matches = calc_ck[0] == recvd_ck[0] && calc_ck[1] == recvd_ck[1];
        if matches {
            Ok((true, max_pay_idx))
        }
        else {
            if dump_ck {
                hprintln!(">>> ckf  {:x?} != {:x?} for {:x?}",calc_ck, recvd_ck, &self.read_buf[..max_idx]).unwrap();
            }
            else {
                let msg_unique_id: u16 =
                    (self.read_buf[0] as u16) << 8 | (self.read_buf[1] as u16);
                hprintln!(">>> ckf 0x{:x}",msg_unique_id);
                //hprintln!(">>> ckf 0x{:x} {:x?} != {:x?}", msg_unique_id,calc_ck, recvd_ck).unwrap();
            }
            Ok((false, 0))
        }
    }

    /// Read a UBX-NAV-PVT message from the device
    fn handle_msg_nav_pvt(&mut self) -> Result<(), DI::InterfaceError> {
        let (ck_ok, max_pay_idx) = self.read_ubx_message(UBX_MSG_LEN_NAV_PVT, false)?;
        if ck_ok {
            self.last_nav_pvt =
                messages::nav_pvt_from_bytes(&self.read_buf[UBX_HEADER_LEN..max_pay_idx]);
        }
        else {
        }
        Ok(())
    }

    /// Read a UBX-NAV-DOP message from the device
    fn handle_msg_nav_dop(&mut self) -> Result<(), DI::InterfaceError> {
        let (ck_ok, max_pay_idx) = self.read_ubx_message(UBX_MSG_LEN_NAV_DOP, true)?;
        if ck_ok {
            self.last_nav_dop =
                messages::nav_dop_from_bytes(&self.read_buf[UBX_HEADER_LEN..max_pay_idx]);
        }
        Ok(())
    }

    /// Read a UBX-MON-HW message from the device
    fn handle_msg_mon_hw(&mut self) -> Result<(), DI::InterfaceError> {
        let (ck_ok, max_pay_idx) = self.read_ubx_message(UBX_MSG_LEN_MON_HW, false)?;
        if ck_ok {
            self.last_mon_hw =
                messages::mon_hw_from_bytes(&self.read_buf[UBX_HEADER_LEN..max_pay_idx]);
        }
        Ok(())
    }

    pub fn handle_all_messages(
        &mut self,
        delay_source: &mut impl DelayUs<u32>
    ) -> Result<usize, DI::InterfaceError> {
        let mut msg_count = 0;
        loop {
            let handled_count = self.handle_one_message(delay_source)?;
            if handled_count > 0 {
                msg_count += handled_count;
            } else {
                break;
            }
        }
        return Ok(msg_count);
    }

    /// return 1 if we handled a message?
    pub fn handle_one_message(&mut self, delay_source: &mut impl DelayUs<u32>) -> Result<usize, DI::InterfaceError> {
        let mut msg_idx = 0;
        // fill our incoming message buffer to avoid overruns
        let available = self.di.fill(delay_source);
        if available < 12 { //TODO somewhat arbitrary
            return Ok(0);
        }
        // hprintln!(">>> fill: {} ", available).unwrap();

        loop {
            if msg_idx < 2 {
                let byte = self.di.read()?;
                if byte == UBX_PRELUDE_BYTES[msg_idx] {
                    msg_idx += 1;
                }
                else {
                    // reset: the byte doesn't match the prelude sequence
                    //hprintln!(">>> jnk {:x} ", byte).unwrap();
                    msg_idx = 0;
                }
            }
            else {
                let rc = self.di.read_many(&mut self.read_buf[..UBX_HEADER_LEN]);
                if let Ok(read_count) = rc {
                    if read_count < UBX_HEADER_LEN {
                        hprintln!(">>> header trunc {} ", read_count).unwrap();
                        return Ok(0);
                    }
                }
                else {
                    hprintln!(">>> read_many fail {:?} ", rc).unwrap();
                    return Ok(0);
                }

                let msg_unique_id: u16 =
                    (self.read_buf[0] as u16) << 8 | (self.read_buf[1] as u16);
                return match msg_unique_id {
                    UBX_MSG_ID_NAV_PVT => {
                        self.handle_msg_nav_pvt()?;
                        Ok(1)
                    }
                    UBX_MSG_ID_NAV_DOP => {
                        self.handle_msg_nav_dop()?;
                        Ok(1)
                    }
                    UBX_MSG_ID_MON_HW => {
                        self.handle_msg_mon_hw()?;
                        Ok(1)
                    }
                    _ => {
                        // unhandled message type...skip to next message
                        hprintln!(">>> unh 0x{:x}", msg_unique_id).unwrap();
                        Ok(1)
                    }
                }

            }
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
