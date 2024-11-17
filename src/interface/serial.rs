use super::DeviceInterface;

use shufflebuf::ShuffleBuf;

/// This encapsulates the Serial UART peripheral
/// and associated pins such as
/// - DRDY: Data Ready: Sensor uses this to indicate it had data available for read
pub struct SerialInterface<SER> {
    /// the serial port to use when communicating
    serial: SER,
    shuffler: ShuffleBuf<256>,
}

impl<SER> SerialInterface<SER>
where
    SER: embedded_io::Read,
{
    pub fn new(serial_port: SER) -> Self {
        Self {
            serial: serial_port,
            shuffler: ShuffleBuf::default(),
        }
    }
}

impl<SER> DeviceInterface for SerialInterface<SER>
where
    SER: embedded_io::Read,
{
    type InterfaceError = SER::Error;

    fn read(&mut self) -> Result<u8, Self::InterfaceError> {
        let (count, byte) = self.shuffler.read_one();
        if count > 0 {
            Ok(byte)
        } else {
            let mut block_byte = [0u8; 1];
            //TODO in practice this hasn't failed yet, but we should handle the error
            self.read_many(&mut block_byte)?;
            Ok(block_byte[0])
        }
    }

    fn fill(&mut self) -> usize {
        let mut fetch_count = self.shuffler.vacant();
        let mut err_count = 0;

        let byte = 0u8;
        while fetch_count > 0 {
            let rc = self.serial.read_exact(&mut [byte]);
            match rc {
                Ok(()) => {
                    err_count = 0; //reset
                    self.shuffler.push_one(byte);
                    fetch_count -= 1;
                }
                Err(_e) => {
                    // in practice this is returning Overrun a ton on stm32h7
                    err_count += 1;
                    if err_count > 100 {
                        break;
                    }
                }
            }
        }
        self.shuffler.available()
    }

    fn read_many(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<usize, Self::InterfaceError> {
        let avail = self.shuffler.available();
        if avail >= buffer.len() {
            let final_read_count = self.shuffler.read_many(buffer);
            return Ok(final_read_count);
        }

        Ok(0)
    }
}
