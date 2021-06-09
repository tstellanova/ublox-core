/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;

mod interface;
pub use interface::{DeviceInterface, SerialInterface};

use hal::blocking::delay::DelayUs;

mod messages;
use messages::*;

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

/// Read buffer size based on maximum UBX message size we support
const READ_BUF_LEN: usize = 128;

pub struct UbxDriver<DI> {
    /// the device interface
    di: DI,
    read_buf: [u8; READ_BUF_LEN],

    /// The last received UBX-NAV-PVT from the device, if any
    last_nav_pvt: Option<NavPosVelTimeM8>,
    /// The last received UBX-MON-HW from the device, if any
    last_mon_hw: Option<MonHardwareM8>,
    /// The last received UBX-NAV-DOP from the device, if any
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
        //TODO configure ublox sensor using CFG message
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
    fn checksum_for_payload(
        payload: &[u8],
        _dump_ck: bool,
    ) -> [u8; UBX_CKSUM_LEN] {
        let mut checksum = [0u8; UBX_CKSUM_LEN];
        for word in payload {
            checksum[0] = checksum[0].wrapping_add(*word);
            checksum[1] = checksum[1].wrapping_add(checksum[0]);
        }
        checksum
    }

    /// Read our interface for a message of known size
    ///
    fn read_ubx_message(
        &mut self,
        msg_len: usize,
        dump_ck: bool,
    ) -> Result<(bool, usize), DI::InterfaceError> {
        // The length sent in the header is defined as being that of the payload only.
        // It does not include the Preamble, Message Class, Message ID, Length, or CRC fields.
        // The number format of the length field is a Little-Endian unsigned 16-bit integer.

        let max_pay_idx = UBX_HEADER_LEN + msg_len;
        let max_msg_idx = max_pay_idx + UBX_CKSUM_LEN;
        let desired_count = max_msg_idx - UBX_HEADER_LEN;
        self.read_buf[max_msg_idx] = 0;
        let read_count = self
            .di
            .read_many(&mut self.read_buf[UBX_HEADER_LEN..max_msg_idx])?;
        if read_count < desired_count {
            // unable to read enough bytes to fill the message struct
            return Ok((false, 0));
        }
        let calc_ck =
            Self::checksum_for_payload(&self.read_buf[..max_pay_idx], dump_ck);
        let recvd_ck =
            &self.read_buf[(max_msg_idx - UBX_CKSUM_LEN)..max_msg_idx];
        let matches = calc_ck[0] == recvd_ck[0] && calc_ck[1] == recvd_ck[1];
        if matches {
            Ok((true, max_pay_idx))
        } else {
            Ok((false, 0))
        }
    }

    /// Read a UBX-NAV-PVT message from the device
    fn handle_msg_nav_pvt(&mut self) -> Result<(), DI::InterfaceError> {
        let (ck_ok, max_pay_idx) =
            self.read_ubx_message(UBX_MSG_LEN_NAV_PVT, false)?;
        if ck_ok {
            self.last_nav_pvt = messages::nav_pvt_from_bytes(
                &self.read_buf[UBX_HEADER_LEN..max_pay_idx],
            );
        }
        Ok(())
    }

    /// Read a UBX-NAV-DOP message from the device
    fn handle_msg_nav_dop(&mut self) -> Result<(), DI::InterfaceError> {
        let (ck_ok, max_pay_idx) =
            self.read_ubx_message(UBX_MSG_LEN_NAV_DOP, true)?;
        if ck_ok {
            self.last_nav_dop = messages::nav_dop_from_bytes(
                &self.read_buf[UBX_HEADER_LEN..max_pay_idx],
            );
        }
        Ok(())
    }

    /// Read a UBX-MON-HW message from the device
    fn handle_msg_mon_hw(&mut self) -> Result<(), DI::InterfaceError> {
        let (ck_ok, max_pay_idx) =
            self.read_ubx_message(UBX_MSG_LEN_MON_HW, false)?;
        if ck_ok {
            self.last_mon_hw = messages::mon_hw_from_bytes(
                &self.read_buf[UBX_HEADER_LEN..max_pay_idx],
            );
        }
        Ok(())
    }

    /// Handle a message we don't recognize, by reading past it
    fn skip_unhandled_msg(&mut self) -> Result<(), DI::InterfaceError> {
        // The length sent in the header is defined as being that of the payload only.
        // It does not include the Preamble, Message Class, Message ID, Length, or CRC fields.
        // The number format of the length field is a Little-Endian unsigned 16-bit integer.
        let msg_len = ((self.read_buf[2] as u16)
            + ((self.read_buf[3] as u16) << 8)) as usize;
        let max_pay_idx = UBX_HEADER_LEN + msg_len;
        let max_msg_idx = (max_pay_idx + UBX_CKSUM_LEN).min(READ_BUF_LEN);
        self.di
            .read_many(&mut self.read_buf[UBX_HEADER_LEN..max_msg_idx])?;

        Ok(())
    }

    pub fn handle_all_messages(
        &mut self,
        delay_source: &mut impl DelayUs<u32>,
    ) -> Result<usize, DI::InterfaceError> {
        let mut msg_count = 0;
        loop {
            let handled_count = self.handle_one_message()?;
            if handled_count > 0 {
                msg_count += handled_count;
            } else {
                break;
            }
            delay_source.delay_us(1000);
        }
        Ok(msg_count)
    }

    /// return 1 if we handled a message?
    pub fn handle_one_message(&mut self) -> Result<usize, DI::InterfaceError> {
        let mut msg_idx = 0;
        // fill our incoming message buffer to avoid overruns
        let available = self.di.fill();
        if available < UBX_MIN_MSG_LEN {
            return Ok(0);
        }

        loop {
            if msg_idx < 2 {
                let byte = self.di.read()?;
                if byte == UBX_PRELUDE_BYTES[msg_idx] {
                    msg_idx += 1;
                } else {
                    // reset: the byte doesn't match the prelude sequence
                    msg_idx = 0;
                }
            } else {
                let rc =
                    self.di.read_many(&mut self.read_buf[..UBX_HEADER_LEN]);
                let header_fail = match rc {
                    Ok(read_count) => read_count != UBX_HEADER_LEN,
                    _ => true,
                };
                if header_fail {
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
                        self.skip_unhandled_msg()?;
                        Ok(1)
                    }
                };
            }
        }
    }
}
