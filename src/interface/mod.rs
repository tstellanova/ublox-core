pub mod serial;
pub use self::serial::SerialInterface;

pub mod spi;
pub use self::spi::SpiInterface;

/// A method of communicating with the device
pub trait DeviceInterface {
    /// Interface associated error type
    type InterfaceError;

    /// Fill up our buffer with unsolicited / periodic UBX messages.
    /// This function should be called before attempting to read.
    /// Returns the number of available bytes.
    fn fill(&mut self) -> usize;

    /// Read a single buffered byte.
    /// Call `fill` before calling this.
    fn read(&mut self) -> Result<u8, Self::InterfaceError>;

    /// Read multiple buffered bytes.
    /// Call `fill` before calling this.
    fn read_many(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<usize, Self::InterfaceError>;
}
