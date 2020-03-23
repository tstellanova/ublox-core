#![no_std]

use embedded_hal as hal;

mod interface;
pub use interface::{DeviceInterface, SerialInterface};

use serde_derive::{Serialize, Deserialize};

use hal::blocking::{delay::DelayMs};


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


pub struct UbxDriver<DI>
{
    /// the device interface
    di: DI,
    /// buffer for blocking reads
    read_buf: [u8; 128],
}

impl<DI, CommE> UbxDriver<DI>
    where
        DI: DeviceInterface<InterfaceError = Error<CommE>>,
        CommE: core::fmt::Debug,
{
    pub(crate) fn new_with_interface(device_interface: DI) -> Self {
        Self {
            di: device_interface,
            read_buf: [0; 128]
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

    fn handle_one_message(&mut self)  -> Result<usize, DI::InterfaceError> {
        const UBX_PRELUDE_BYTES: [u8;2] = [0xB5, 0x62];
        const UBX_MSG_CLASS_NAV:u8 = 0x01;
        const UBX_MSG_ID_NAV_PVT:u8 = 0x07;
        const UBX_MSG_CLASSID_NAV_PVT: u16 = (UBX_MSG_ID_NAV_PVT as u16) << 8 | (UBX_MSG_CLASS_NAV as u16);
        const UBX_MSG_LEN_NAV_PVT:usize = 96;


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
                    }
                    else {
                        //skip to the next packet header
                        msg_idx = 0;
                    }
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

/// Support UBX-NAV-PVT message
/// Navigation Position Velocity Time Solution
#[derive(Serialize, Deserialize, Debug)]
pub struct NavPosVelTimeM8 {
    /// GPS time of week of the navigation epoch. (ms)
    pub itow: u32,
    /// Year (UTC)
    pub year: u16,
    /// Month, range 1..12 (UTC)
    pub month: u8,
    /// Day of month, range 1..31 (UTC)
    pub day: u8,
    /// Hour of day, range 0..23 (UTC)
    pub hour: u8,
    /// Minute of hour, range 0..59 (UTC)
    pub min: u8,
    /// Seconds of minute, range 0..60 (UTC)
    pub sec: u8,
    /// Validity flags
    pub validity_flags: u8,
    /// Time accuracy estimate (ns UTC)
    pub time_accuracy: u32,
    /// Fraction of second, range -1e9 .. 1e9 (ns UTC)
    pub nanosecond: i32,
    /// GNSS fix type:
    /// 0 no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix,
    /// 4: GNSS + dead reckoning combined 5: time only fix
    pub fix_type: u8,
    /// Fix status flags
    pub flags: u8,
    /// Additional flags
    pub flags2: u8,
    /// Number of satellites used in Nav Solution
    pub num_satellites: u8,
    /// Longitude (1e-7 degrees)
    pub lon: i32,
    /// Latitude (1e-7 degrees)
    pub lat: i32,
    /// Height above ellipsoid (mm)
    pub height: i32,
    /// Height above mean sea level (AMSL, mm)
    pub height_msl: i32,
    /// Horizontal accuracy estimate (mm)
    pub h_accuracy: u32,
    /// Vertical accuracy estimate (mm)
    pub v_accuracy: u32,
    /// NED north velocity (mm/s)
    pub vel_north: i32,
    /// NED east velocity (mm/s)
    pub vel_east: i32,
    /// NED down velocity (mm/s)
    pub vel_down: i32,
    /// Ground Speed  (mm/s)
    pub ground_speed: i32,
    /// 2D Heading of motion (1e-5 degrees)
    pub heading_motion: i32,
    /// Speed accuracy estimate (mm/s)
    pub speed_accuracy: u32,
    /// Heading accuracy estimate for both motion and vehicle (degrees)
    pub heading_accuracy: u32,
    /// Position Dilution of Precision
    pub pos_dop: u16,
    pub reserved1_a: u16,
    pub reserved1_b: u32,
    /// Heading of vehicle (1e-5 degrees)
    pub heading_vehicle: i32,
    /// Magnetic declination (1e-2 degrees)
    pub mag_dec: i16,
    /// Magnetic declination accuracy (1e-2 degrees)
    pub mag_accuracy: u16,
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}

