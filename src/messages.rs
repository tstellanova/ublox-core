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
pub fn nav_pvt_from_bytes(buf: &[u8]) -> Option<NavPosVelTimeM8> {
    ubx_struct_from_bytes(buf)
}

/// UBX-MON-HW message: Hardware Status
/// See 32.16.4 UBX-MON-HW (0x0A 0x09)
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct MonHardwareM8 {
    pub pin_sel: u32,  //pinSel - Mask of Pins Set as Peripheral/PIO
    pub pin_bank: u32, //pinBank - Mask of Pins Set as Bank A/B
    pub pin_direction: u32, //pinDir - Mask of Pins Set as Input/Output
    pub pin_values: u32, //pinVal - Mask of Pins Value Low/High
    pub noise_per_ms: u16, //noisePerMS - Noise Level as measured by the GPS Core
    pub agc_count: u16, //agcCnt - AGC Monitor (counts SIGHI xor SIGLO, range 0 to 8191)
    pub ant_status: u8, //aStatus - Status of the Antenna Supervisor State Machine (0=INIT, 1=DONTKNOW, 2=OK, 3=SHORT, 4=OPEN)
    pub ant_power: u8, //aPower - Current PowerStatus of Antenna (0=OFF, 1=ON, 2=DONTKNOW)
    pub flags: u8,     //flags - Flags (see graphic below)
    pub reserved: u8,  //reserved1 - Reserved
    pub used_mask: u32, //usedMask - Mask of Pins that are used by the Virtual Pin Manager
    pub pin_maps: [u8; 17], //U1[17] - VP - Array of Pin Mappings for each of the 17 Physical Pins
    pub jam_ind: u8, //45 U1 - jamInd - CW Jamming indicator, scaled (0 = no CW jamming, 255 = strong CW jamming)
    pub reserved2: [u8; 2], //46 U1[2] - reserved2 - Reserved
    pub pin_irq: u32, //48 X4 - pinIrq - Mask of Pins Value using the PIO Irq
    pub pull_high: u32, //52 X4 - pullH - Mask of Pins Value using the PIO Pull High Resistor
    pub pull_low: u32, //56 X4 - pullL - Mask of Pins Value using the PIO Pull Low Resistor
}
pub fn mon_hw_from_bytes(buf: &[u8]) -> Option<MonHardwareM8> {
    ubx_struct_from_bytes(buf)
}

/// UBX-NAV-DOP message: Dilution of precision
/// See 32.17.5 UBX-NAV-DOP (0x01 0x04)
#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct NavDopM8 {
    pub itow: u32, //0  ms GPS time of week of the navigation epoch. See the description of iTOW for details.
    pub g_dop: u16, //4 gDOP Geometric DOP 0.01
    pub p_dop: u16, //6 pDOP Position DOP 0.01
    pub t_dop: u16, //8 tDOP Time DOP 0.01
    pub v_dop: u16, //10 vDOP Vertical DOP 0.01
    pub h_dop: u16, //12 Horizontal DOP 0.01
    pub n_dop: u16, //14 Northing DOP 0.01
    pub e_dop: u16, //16 Easting DOP 0.01
}
pub fn nav_dop_from_bytes(buf: &[u8]) -> Option<NavDopM8> {
    ubx_struct_from_bytes(buf)
}

//UBX-CFG-MSG

/// Read a UBX packed message type from bytes
pub fn ubx_struct_from_bytes<T>(buf: &[u8]) -> Option<T> {
    let struct_size = core::mem::size_of::<T>();
    unsafe {
        let mut msg: T = core::mem::zeroed();
        let msg_as_slice = core::slice::from_raw_parts_mut(
            &mut msg as *mut _ as *mut u8,
            struct_size,
        );
        if (&buf[..]).read_exact(msg_as_slice).is_ok() {
            Some(msg)
        } else {
            None
        }
    }
}
