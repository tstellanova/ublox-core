use embedded_hal as hal;
use super::DeviceInterface;
use crate::Error;

/// This encapsulates the Serial UART peripheral
/// and associated pins such as
/// - DRDY: Data Ready: Sensor uses this to indicate it had data available for read
pub struct SerialInterface<UART> {
    /// the UART port to use when communicating
    uart: UART,

}

impl<UART, CommE> SerialInterface<UART>
    where
        UART: hal::blocking::serial::Write<u8, Error = CommE>
        + hal::serial::Read<u8, Error = CommE>,
{
    pub fn new(uart: UART) -> Self {
        Self { uart }
    }
}

impl<UART, CommE> DeviceInterface for SerialInterface<UART>
    where
        UART: hal::blocking::serial::Write<u8, Error = CommE>
        + hal::serial::Read<u8, Error = CommE>,
{
    type InterfaceError = Error<CommE>;

    fn send_command(&mut self, cmd: &[u8]) -> Result<u8, Self::InterfaceError> {
        self.uart.bwrite_all(cmd).map_err(Error::Comm)?;
        unimplemented!()
    }
}