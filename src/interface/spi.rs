
use embedded_hal as hal;
use hal::digital::v2::OutputPin;

use super::DeviceInterface;
use crate::Error;
use shufflebuf::ShuffleBuf;

/// This encapsulates the SPI peripheral and associated pins such as:
/// - CSN: The chip select pin
pub struct SpiInterface<SPI,CSN> {
    /// the serial port to use when communicating
    _spi: SPI,
    /// the Chip Select pin (GPIO output) to use when communicating
    _csn: CSN,
    _shuffler: ShuffleBuf,
}

impl<SPI, CSN, CommE, PinE>DeviceInterface for SpiInterface<SPI, CSN>
    where
        SPI: hal::blocking::spi::Write<u8, Error = CommE>
        + hal::blocking::spi::Transfer<u8, Error = CommE>,
        CSN: OutputPin<Error = PinE>,
{
    type InterfaceError = Error<CommE>;

    fn fill(&mut self) -> usize {
        // See: 11.6.3 Back-To-Back Read and Write Access
        unimplemented!()
    }

    fn read(&mut self) -> Result<u8, Self::InterfaceError> {
        unimplemented!()
    }

    fn read_many(
        &mut self,
        _buffer: &mut [u8],
    ) -> Result<usize, Self::InterfaceError> {
        unimplemented!()
    }
}