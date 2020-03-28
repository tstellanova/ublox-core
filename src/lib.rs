/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;

mod interface;
pub use interface::{DeviceInterface, SerialInterface};

use crate::messages::{MonHardwareM8, NavDopM8, NavPosVelTimeM8, mon_hw_from_bytes};
use hal::blocking::delay::DelayMs;

mod messages;

#[cfg(debug_assertions)]
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
        _delay_source: &mut impl DelayMs<u8>,
    ) -> Result<(), DI::InterfaceError> {
        //TODO configure ublox sensor? or assume it's preconfigured?

        Ok(())
    }

    pub fn get_last_nav_pvt(&self) -> Option<NavPosVelTimeM8> {
        return self.last_nav_pvt.clone();
    }

    /// generate a 16 bit checksum for a payload
    fn checksum_for_payload(payload: &[u8]) -> [u8; 2] {
        let mut checksum = [0u8; 2];
        for b in payload {
            checksum[0] = checksum[0].wrapping_add(*b);
            checksum[1] = checksum[1].wrapping_add(checksum[0]);
        }
        checksum
    }

    fn checksum_ok(&self, msg_len: usize) -> bool {
        let ck = Self::checksum_for_payload(&self.read_buf[..msg_len]);
        let recvd_ck = &self.read_buf[msg_len..msg_len+2];
        ck[0] == recvd_ck[0] && ck[1] == recvd_ck[1]
    }

    /// Read a UBX-NAV-PVT message from the device
    fn handle_msg_nav_pvt(&mut self) -> Result<(), DI::InterfaceError> {
        const UBX_MSG_LEN_NAV_PVT: usize = 94;
        self.di
            .read_many(&mut self.read_buf[2..UBX_MSG_LEN_NAV_PVT+4])?;
        if self.checksum_ok(UBX_MSG_LEN_NAV_PVT) {
            self.last_nav_pvt =
                messages::nav_pvt_from_bytes(&self.read_buf.as_ref());
        }
        Ok(())
    }

    /// Read a UBX-NAV-DOP message from the device
    fn handle_msg_nav_dop(&mut self) -> Result<(), DI::InterfaceError> {
        const UBX_MSG_LEN_NAV_DOP: usize = 18;
        self.di
            .read_many(&mut self.read_buf[2..UBX_MSG_LEN_NAV_DOP+4])?;
        if self.checksum_ok(UBX_MSG_LEN_NAV_DOP) {
            self.last_nav_dop =
                messages::nav_dop_from_bytes(&self.read_buf.as_ref());
        }
        Ok(())
    }

    /// Read a UBX-MON-HW message from the device
    fn handle_msg_mon_hw(&mut self) -> Result<(), DI::InterfaceError> {
        const UBX_MSG_LEN_MON_HW: usize = 60;
        self.di
            .read_many(&mut self.read_buf[2..UBX_MSG_LEN_MON_HW+4])?;
        if self.checksum_ok(UBX_MSG_LEN_MON_HW) {
            self.last_mon_hw = mon_hw_from_bytes(&self.read_buf.as_ref());
        }
        Ok(())
    }

    pub fn handle_all_messages(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<usize, DI::InterfaceError> {
        let mut msg_count = 0;
        loop {
            let handled_count = self.handle_one_message()?;
            if handled_count > 0 {
                msg_count += handled_count;
                delay_source.delay_ms(1);
            } else {
                break;
            }
        }
        return Ok(msg_count);
    }

    /// return 1 if we handled a message?
    pub fn handle_one_message(&mut self) -> Result<usize, DI::InterfaceError> {
        const UBX_PRELUDE_BYTES: [u8; 2] = [0xB5, 0x62];
        const UBX_MSG_CLASS_NAV: u8 = 0x01;
        const UBX_MSG_CLASS_MON: u8 = 0x0A;

        const UBX_MSG_ID_NAV_PVT: u16 = (UBX_MSG_CLASS_NAV as u16) << 8 | 0x07;
        const UBX_MSG_ID_NAV_DOP: u16 = (UBX_MSG_CLASS_NAV as u16) << 8 | 0x04;
        const UBX_MSG_ID_MON_HW: u16 = (UBX_MSG_CLASS_MON as u16) << 8 | 0x09;

        let mut msg_idx = 0;
        let mut msg_class_id: u8 = 0;
        loop {
            let byte = self.di.read()?;
            match msg_idx {
                0 | 1 => {
                    // look for the beginning of a UBX header
                    if byte == UBX_PRELUDE_BYTES[msg_idx] {
                        msg_idx += 1;
                    } else {
                        // reset: the byte doesn't match the prelude sequence
                        msg_idx = 0;
                    }
                }
                2 => {
                    // next comes msg class
                    msg_class_id = byte;
                    msg_idx += 1;
                }
                3 => {
                    // next comes msg ID
                    let msg_sub_id = byte;
                    let msg_unique_id: u16 =
                        (msg_class_id as u16) << 8 | (msg_sub_id as u16);
                    self.read_buf[0] = msg_class_id;
                    self.read_buf[1] = msg_sub_id;
                    match msg_unique_id {
                        UBX_MSG_ID_NAV_PVT => {
                            self.handle_msg_nav_pvt()?;
                            hprintln!(">>> nav_pvt").unwrap();
                            return Ok(1);
                        }
                        UBX_MSG_ID_NAV_DOP => {
                            self.handle_msg_nav_dop()?;
                            hprintln!(">>> nav_dop").unwrap();
                            return Ok(1);
                        }
                        UBX_MSG_ID_MON_HW => {
                            self.handle_msg_mon_hw()?;
                            hprintln!(">>> mon_hw").unwrap();
                            return Ok(1);
                        }
                        _ => {
                            // unhandled message type...skip to next message
                            hprintln!(">>> unh 0x{:x}", msg_unique_id).unwrap();
                            return Ok(1);
                        }
                    }
                }
                _ => {
                    msg_idx = 0; //skip to next message
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
