//used for converting byte slices to structs
use genio::Read;

/// Support UBX-NAV-PVT message
/// Navigation Position Velocity Time Solution
#[repr(C)]
#[derive(Copy, Clone, Debug)]
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

/// Read a NavPosVelTimeM8 from a slice
pub fn read_nav_pvt(buf: &[u8]) -> NavPosVelTimeM8 {
    let struct_size = core::mem::size_of::<NavPosVelTimeM8>();
    unsafe {
        let mut msg: NavPosVelTimeM8 = core::mem::zeroed();
        let msg_as_slice = core::slice::from_raw_parts_mut(
            &mut msg as *mut _ as *mut u8,
            struct_size,
        );
        (&buf[..]).read_exact(msg_as_slice).unwrap();
        msg
    }
}
