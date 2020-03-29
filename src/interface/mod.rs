pub mod serial;

pub use self::serial::SerialInterface;

use embedded_hal as hal;
use hal::blocking::delay::DelayUs;


/// A method of communicating with the device
pub trait DeviceInterface {
    /// Interface associated error type
    type InterfaceError;

    /// fill up our buffer
    /// return the number of available bytes
    fn fill(&mut self, delay_source: &mut impl DelayUs<u32>) -> usize;

    /// Read a single byte from the device
    fn read(&mut self) -> Result<u8, Self::InterfaceError>;

    /// Read multiple bytes from the device
    fn read_many(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<usize, Self::InterfaceError>;
}
