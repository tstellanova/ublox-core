use embedded_hal as hal;
use super::DeviceInterface;
use crate::Error;
use nb::{block};


/// This encapsulates the Serial UART peripheral
/// and associated pins such as
/// - DRDY: Data Ready: Sensor uses this to indicate it had data available for read
pub struct SerialInterface<SER> {
    /// the serial port to use when communicating
    serial: SER,
}

impl<SER, CommE> SerialInterface<SER>
    where
        SER: hal::serial::Read<u8, Error = CommE>,
{
    pub fn new(serial_port: SER) -> Self {
        Self { serial: serial_port }
    }
}

impl<SER, CommE> DeviceInterface for SerialInterface<SER>
    where
        SER: hal::serial::Read<u8, Error = CommE>,
{
    type InterfaceError = Error<CommE>;


    fn read(&mut self) -> Result<u8, Self::InterfaceError> {
        let byte = block!(self.serial.read()).map_err(Error::Comm)?;
        Ok(byte)
    }

    fn read_many(&mut self, buffer: &mut [u8]) -> Result<(), Self::InterfaceError> {
        for word in buffer {
            *word = block!(self.serial.read()).map_err(Error::Comm)?;
        }

        Ok(())
    }


}