use super::DeviceInterface;
use crate::Error;
use embedded_hal as hal;

use arraydeque::ArrayDeque;
use crate::Error::NoData;

/// This encapsulates the Serial UART peripheral
/// and associated pins such as
/// - DRDY: Data Ready: Sensor uses this to indicate it had data available for read
pub struct SerialInterface<SER> {
    /// the serial port to use when communicating
    serial: SER,
    inner_buf: ArrayDeque<[u8; 256]>,
}

impl<SER, CommE> SerialInterface<SER>
where
    SER: hal::serial::Read<u8, Error = CommE>,
{
    pub fn new(serial_port: SER) -> Self {
        Self {
            serial: serial_port,
            inner_buf: ArrayDeque::new(),
        }
    }
}

impl<SER, CommE> DeviceInterface for SerialInterface<SER>
where
    SER: hal::serial::Read<u8, Error = CommE>,
{
    type InterfaceError = Error<CommE>;

    fn read(&mut self) -> Result<u8, Self::InterfaceError> {
        if self.inner_buf.len() > 0 {
            let byte = self.inner_buf.pop_front().unwrap();
            return Ok(byte);
        }
        Err(NoData)
    }


    fn fill(&mut self) -> usize {
        let mut err_count = 0;

        while !self.inner_buf.is_full()    {
            let rc = self.serial.read();
            match rc {
                Ok(byte) => {
                    err_count = 0; //reset
                    //fail hard if pushing fails
                    self.inner_buf.push_back(byte).unwrap();
                }
                Err(nb::Error::WouldBlock) => {}
                Err(nb::Error::Other(_)) => {
                    // in practice this is returning Overrun a ton on stm32h7
                    err_count += 1;
                    if err_count > 100 {
                        break;
                    }
                }
            }
        }
        self.inner_buf.len()
    }

    fn read_many(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<usize, Self::InterfaceError> {
        let mut write_idx: usize = 0;
        let desired = buffer.len();
        let avail = self.inner_buf.len();
        if avail >= desired {
            while write_idx < desired {
                buffer[write_idx] = self.inner_buf.pop_front().unwrap();
                write_idx += 1;
            }
        }

        return Ok(write_idx);
    }
}
