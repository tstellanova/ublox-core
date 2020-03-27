/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;

mod interface;
pub use interface::{DeviceInterface, SerialInterface};

use crate::messages::NavPosVelTimeM8;
use hal::blocking::delay::DelayMs;

mod messages;

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

pub struct UbxDriver<DI> {
    /// the device interface
    di: DI,
    /// The last received NAV_PVT solution from the device, if any
    last_nav_pvt: Option<NavPosVelTimeM8>,
}

impl<DI, CommE> UbxDriver<DI>
where
    DI: DeviceInterface<InterfaceError = Error<CommE>>,
    CommE: core::fmt::Debug,
{
    pub(crate) fn new_with_interface(device_interface: DI) -> Self {
        Self {
            di: device_interface,
            last_nav_pvt: None,
        }
    }

    pub fn setup(
        &mut self,
        _delay_source: &mut impl DelayMs<u8>,
    ) -> Result<(), DI::InterfaceError> {
        //TODO configure ublox sensor? or assume it's preconfigured?

        let msg_count = self.handle_one_message().unwrap();
        if msg_count > 0 {
            //yippee, we're already able to receive messages
        } else {
            //TODO the uart setup could be fallible
            //TODO reset the baud rate and retry
            //self.di.setup(&mut delay_source);
        }

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

    /// Read a NAV_PVT message from the device
    fn handle_nav_pvt_msg(&mut self) -> Result<(), DI::InterfaceError> {
        const UBX_MSG_LEN_NAV_PVT: usize = 96;
        let read_buf = [0u8; UBX_MSG_LEN_NAV_PVT];
        let mut cksum_buf = [0u8; 2];
        self.di.read_many(&mut cksum_buf)?;
        //TODO verify checksum

        let msg = messages::read_nav_pvt(&read_buf.as_ref());
        self.last_nav_pvt = Some(msg);
        Ok(())
    }

    pub fn handle_all_messages(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<usize, DI::InterfaceError> {
        let mut msg_count = 0;
        loop {
            if let Ok(handled_count) = self.handle_one_message() {
                if handled_count > 0 {
                    msg_count += handled_count;
                    delay_source.delay_ms(1);
                } else {
                    break;
                }
            } else {
                break;
            }
        }
        return Ok(msg_count);
    }

    /// return 1 if we handled a message?
    fn handle_one_message(&mut self) -> Result<usize, DI::InterfaceError> {
        const UBX_PRELUDE_BYTES: [u8; 2] = [0xB5, 0x62];
        const UBX_MSG_CLASS_NAV: u8 = 0x01;
        const UBX_MSG_ID_NAV_PVT: u8 = 0x07;

        let mut msg_idx = 0;
        let mut msg_class_id: u8 = 0;
        let mut msg_sub_id: u8;
        loop {
            if let Ok(byte) = self.di.read() {
                match msg_idx {
                    0 | 1 => {
                        // look for the beginning of a UBX header
                        if byte == UBX_PRELUDE_BYTES[msg_idx] {
                            msg_idx += 1;
                            continue;
                        } else {
                            // reset: the byte doesn't match the prelude sequence
                            msg_idx = 0;
                            continue;
                        }
                    },
                    2 => {
                        // next comes msg class
                        msg_class_id = byte;
                        msg_idx += 1;
                        continue;
                    },
                    3 => {
                        // next comes msg ID
                        msg_sub_id = byte;
                        msg_idx += 1;

                        match msg_class_id  {
                            UBX_MSG_CLASS_NAV => {
                                match msg_sub_id {
                                    UBX_MSG_ID_NAV_PVT => {
                                        self.handle_nav_pvt_msg()?;
                                        return Ok(1);
                                    },
                                    _ => {}
                                }
                            },
                            _ => {}
                        }
                        // unhandled message type...skip to next message
                        msg_idx = 0;
                        continue;
                    },
                    _ => {}
                }
                return Ok(0);
            } else {
                //read failed so we're unable to handle any messages
                break;
            }
        }
        Ok(0)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
