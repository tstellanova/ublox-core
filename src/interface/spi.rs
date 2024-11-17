use embedded_hal as hal;

use super::DeviceInterface;
use shufflebuf::ShuffleBuf;

/// This encapsulates the SPI peripheral and associated pins such as:
/// - CSN: The chip select pin
pub struct SpiInterface<SPI> {
    /// the serial port to use when communicating
    _spi: SPI,
    /// the Chip Select pin (GPIO output) to use when communicating
    _shuffler: ShuffleBuf<256>,
}

impl<SPI> DeviceInterface for SpiInterface<SPI>
where
    SPI: embedded_hal::spi::SpiDevice,
{
    type InterfaceError = SPI::Error;

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
