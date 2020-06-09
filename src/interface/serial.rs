use super::DeviceInterface;
use crate::Error;
use embedded_hal as hal;



use bbqueue::{BBBuffer, ConstBBBuffer};
use embedded_hal::blocking::spi::Write;
use core::sync::atomic::{AtomicUsize, Ordering::SeqCst};
use core::ptr::NonNull;

type InnerBuffSize = bbqueue::consts::U256;

/// used to limit how much we read at a time
const MAX_FILL_SIZE: usize = 128;

/// This encapsulates the Serial UART peripheral
/// and associated pins such as
/// - DRDY: Data Ready: Sensor uses this to indicate it had data available for read
pub struct SerialInterface<'a, SER> {
    /// the serial port to use when communicating
    serial: SER,
    inner_buf:BBBuffer<InnerBuffSize>,
    prod: NonNull<bbqueue::Producer<'a, InnerBuffSize>>,
    cons: NonNull<bbqueue::Consumer<'a, InnerBuffSize>>,
    /// The number of bytes available immediately to read in the inner buffer
    buf_avail: AtomicUsize
}

impl<SER, CommE> SerialInterface<'_, SER>
where
    SER: hal::serial::Read<u8, Error = CommE>,
{
    pub fn new(serial_port: SER) -> Self {
        //TODO  yes this a self-referential struct: DANGER
        let mut inst = Self {
            serial: serial_port,
            inner_buf: BBBuffer::new(),
            buf_avail: AtomicUsize::new(0),
            prod: NonNull::dangling(),
            cons: NonNull::dangling()
        };

        // NonNull::new_unchecked(self as *const _ as *mut _)
        let ( mut prod, mut cons) = inst.inner_buf.try_split().unwrap();
        inst.prod = unsafe { NonNull::new_unchecked(&mut prod) };
        inst.cons = unsafe { NonNull::new_unchecked(&mut cons) };

        inst
    }
}

impl<SER, CommE> DeviceInterface for SerialInterface<'_, SER>
where
    SER: hal::serial::Read<u8, Error = CommE>,
{
    type InterfaceError = Error<CommE>;

    fn read(&mut self) -> Result<u8, Self::InterfaceError> {
        let mut buf = [0u8];
        let count = self.read_many(&mut buf)?;
        //TODO ^ could be zero count return?
        return Ok(buf[0]);
    }

    fn fill(&mut self) -> usize {
        let mut wg = self.prod.as_ptr().as_ref().unwrap().grant_max_remaining(MAX_FILL_SIZE).unwrap();
        let mut fetch_count = wg.len();
        let mut err_count = 0;

        let mut write_idx:usize = 0;
        while write_idx < fetch_count {
            let rc = self.serial.read();
            match rc {
                Ok(byte) => {
                    err_count = 0; //reset
                    wg[write_idx] = byte;
                    write_idx += 1;
                    fetch_count -= 1;
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
        // commit the bytes we actually wrote
        wg.commit(write_idx);
        self.buf_avail.fetch_add(write_idx, SeqCst);
        self.buf_avail.load(SeqCst)
    }

    fn read_many(
        &mut self,
        buffer: &mut [u8],
    ) -> Result<usize, Self::InterfaceError> {
        if let Ok(rg) = self.cons.as_ptr().as_ref().unwrap().read() {
            let available = rg.len();
            let consumed = if buffer.len() > available { available } else {buffer.len()};
            // let sliceo = rg.buf();
            buffer.copy_from_slice(&rg.buf()[..consumed]);
            self.buf_avail.fetch_sub(consumed, SeqCst);
            rg.release(consumed);
            return Ok(consumed);
        }

        return Ok(0);
    }
}
