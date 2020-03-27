pub mod serial;

pub use self::serial::SerialInterface;

/// A method of communicating with the device
pub trait DeviceInterface {
    /// Interface associated error type
    type InterfaceError;

    /// Read a single byte from the device
    fn read(&mut self) -> Result<u8, Self::InterfaceError>;

    /// Read multiple bytes from the device
    fn read_many(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<(), Self::InterfaceError>;
}
