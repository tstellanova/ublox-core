
pub mod serial;

pub use self::serial::SerialInterface;

/// A method of communicating with the device
pub trait DeviceInterface {
    /// Interface associated error type
    type InterfaceError;

    fn send_command(&mut self, cmd: &[u8]) -> Result<(), Self::InterfaceError>;

}
