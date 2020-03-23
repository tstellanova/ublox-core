use embedded_hal as hal;
use super::DeviceInterface;
use crate::Error;

/// This encapsulates the Serial UART peripheral
/// and associated pins such as
/// - DRDY: Data Ready: Sensor uses this to indicate it had data available for read
pub struct SerialInterface<SER> {
    /// the serial port to use when communicating
    serial: SER,
}

impl<SER, CommE> SerialInterface<SER>
    where
        SER: hal::blocking::serial::Write<u8>
        + hal::serial::Read<u8, Error = CommE>,
{
    pub fn new(serial_port: SER) -> Self {
        Self { serial: serial_port }
    }
}

impl<SER, CommE> DeviceInterface for SerialInterface<SER>
    where
        SER: hal::blocking::serial::Write<u8>
        + hal::serial::Read<u8, Error = CommE>,
{
    type InterfaceError = Error<CommE>;

    fn send_command(&mut self, cmd: &[u8]) -> Result<(), Self::InterfaceError> {
        self.serial.bwrite_all(cmd);
        Ok(())
    }
}