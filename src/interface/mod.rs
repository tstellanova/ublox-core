pub mod serial;

pub use self::serial::SerialInterface;

use embedded_hal as hal;
use hal::blocking::delay::DelayUs;

/// A method of communicating with the device
pub trait DeviceInterface {
    /// Interface associated error type
    type InterfaceError;

    /// Fill up our buffer with unsolicited / periodic UBX messages.
    /// This function should be called before attempting to read.
    /// Returns the number of available bytes.
    fn fill(&mut self, delay_source: &mut impl DelayUs<u32>) -> usize;

    /// Read a single buffered byte (see the `fill` function)
    fn read(&mut self) -> Result<u8, Self::InterfaceError>;

    /// Read multiple buffered bytes (see the `fille` function)
    fn read_many(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<usize, Self::InterfaceError>;
}
