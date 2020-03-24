/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;



mod interface;
pub use interface::{DeviceInterface, SerialInterface};

use hal::blocking::{delay::DelayMs};

mod messages;

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE> {
    /// Sensor communication error
    Comm(CommE),

    /// Sensor is not responding
    Unresponsive,
}


pub fn new_serial_driver<UART, CommE>( uart: UART) -> UbxDriver<SerialInterface<UART>>
    where
        UART: hal::serial::Read<u8, Error = CommE>,
        CommE: core::fmt::Debug,
{
    let iface = interface::SerialInterface::new(uart);
    UbxDriver::new_with_interface(iface)
}


pub struct UbxDriver<DI>
{
    /// the device interface
    di: DI,
}

impl<DI, CommE> UbxDriver<DI>
    where
        DI: DeviceInterface<InterfaceError = Error<CommE>>,
        CommE: core::fmt::Debug,
{
    pub(crate) fn new_with_interface(device_interface: DI) -> Self {
        Self {
            di: device_interface,
        }
    }

    pub fn setup(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), DI::InterfaceError> {
        //TODO configure ublox sensor? or assume it's preconfigured?
        delay_source.delay_ms(100);
        Ok(())
    }

    /// generate a 16 bit checksum for a payload
    fn checksum_for_payload(payload: &[u8]) -> [u8; 2] {
        let mut checksum = [0u8; 2];
        for b in payload {
            checksum[0].wrapping_add(*b);
            checksum[1].wrapping_add(checksum[0]);
        }
        checksum
    }

    /// Read a NAV_PVT message from the device
    fn handle_nav_pvt_msg(&mut self) -> Result<(), DI::InterfaceError> {
        const UBX_MSG_LEN_NAV_PVT:usize = 96;
        let mut read_buf = [0u8; UBX_MSG_LEN_NAV_PVT];
        let mut cksum_buf = [0u8; 2];
        self.di.read_many(&mut cksum_buf)?;
        //TODO verify checksum

        let msg = messages::read_nav_pvt(&read_buf.as_ref());
        Ok(())
    }

    /// return 1 if we handled a message?
    fn handle_one_message(&mut self)  -> Result<usize, DI::InterfaceError> {
        const UBX_PRELUDE_BYTES: [u8;2] = [0xB5, 0x62];
        const UBX_MSG_CLASS_NAV:u8 = 0x01;
        const UBX_MSG_ID_NAV_PVT:u8 = 0x07;
        const UBX_MSG_CLASSID_NAV_PVT: u16 = (UBX_MSG_ID_NAV_PVT as u16) << 8 | (UBX_MSG_CLASS_NAV as u16);


        let mut msg_idx = 0;
        let mut msg_class_id: u16 = 0;
        let mut msg_len: usize = 0;
        loop {
            let byte = self.di.read()?;
            match msg_idx {
                0|1 => {
                    // look for the beginning of a UBX header
                    if byte == UBX_PRELUDE_BYTES[msg_idx] {
                        msg_idx += 1;
                    }
                    else {
                        // reset: the byte doesn't match the prelude sequence
                        msg_idx = 0;
                        continue;
                    }
                },
                2 => {
                    //first comes the message class
                    msg_class_id = byte as u16;
                    msg_idx += 1;
                },
                3 => {
                    // next comes the message ID
                    msg_class_id |= (byte as u16) << 8;
                    msg_idx += 1;
                    //TODO verify the msg_class is in a recognized set
                    if msg_class_id == UBX_MSG_CLASSID_NAV_PVT {
                        //TODO read the rest of this recognized packet
                        self.handle_nav_pvt_msg();
                    }

                    //skip to the next packet header
                    msg_idx = 0;
                },
                _ => {
                    // start a new packet
                    msg_idx = 0;
                }
            }

            return Ok(0)
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

