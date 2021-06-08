#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

// pick a panicking behavior
#[cfg(not(debug_assertions))]
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
#[cfg(debug_assertions)]
extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use cortex_m;
use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use p_hal::stm32;
use stm32h7xx_hal as p_hal;
use stm32h7xx_hal::{pac, prelude::*};

use stm32::UART7;
use stm32::USART1;

use core::convert::TryInto;
use core::fmt::Write;
use p_hal::serial::Error;

type Usart1PortType = p_hal::serial::Serial<
    USART1,
>;

/// This isn't strictly an example: it's a tool for
/// copying the input on the input port and writing it to the output port.
///
/// Run example using:
/// `cargo run --example serial_dump --target thumbv7em-none-eabihf`
///

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    // Constrain and Freeze power
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    // Constrain and Freeze clock
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;
    let mut delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);

    // Acquire the GPIOC peripheral. This also enables the clock for
    // GPIOC in the RCC register.
    // let gpioc = dp.GPIOC.split(&mut ccdr.ahb4);

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);

    //UART7 is debug (dronecode port): `(PF6, PE8)`
    let uart7_port = {
        let config =
            p_hal::serial::config::Config::default().baudrate(57_600_u32.bps());
        let rx = gpiof.pf6.into_alternate_af7();
        let tx = gpioe.pe8.into_alternate_af7();
        dp.UART7.serial((tx, rx), config, ccdr.peripheral.UART7, &ccdr.clocks).unwrap()
    };

    const BAUD_SEQ: [u32; 6] = [115200, 38400, 57600, 9600, 115200, 230400];
    let baud_idx = 0;
    let baud = BAUD_SEQ[baud_idx];

    // GPS1 port USART1:
    let mut usart1_port = {
        let config =
            p_hal::serial::config::Config::default().baudrate(baud.bps());
        let rx = gpiob.pb7.into_alternate_af7();
        let tx = gpiob.pb6.into_alternate_af7();
        dp.USART1.serial((tx, rx), config, ccdr.peripheral.USART1, &ccdr.clocks).unwrap()
    };

    delay_source.delay_ms(1u8);

    let (mut dtx, mut _drx) = uart7_port.split();

    fn read_many(
        port: &mut Usart1PortType,
        buffer: &mut [u8],
        dbg_port: &mut p_hal::serial::Tx<UART7>,
    ) -> Result<usize, u32> {
        let mut read_count = 0;
        while read_count < buffer.len() {
            let rc = port.read();
            {
                match rc {
                    Ok(byte) => {
                        buffer[read_count] = byte;
                        read_count += 1;
                    }
                    Err(nb::Error::WouldBlock) => {}
                    Err(nb::Error::Other(Error::Overrun)) => {
                        write!(dbg_port, ".").unwrap();
                    }
                    _ => {
                        break;
                    }
                }
            }
        }

        Ok(read_count)
    }

    const UBX_SYNC1: u8 = 0xb5;
    const UBX_SYNC2: u8 = 0x62;
    let mut error_count = 0;
    let mut last_byte: u8 = 0;
    let mut read_buf: [u8; 256] = [0; 256];
    loop {
        let result = usart1_port.read();
        if let Ok(byte) = result {
            error_count = 0;

            if byte == UBX_SYNC2 && last_byte == UBX_SYNC1 {
                //for i in 0..read_buf.len() { read_buf[i] = 0;}

                if let Ok(header_read) = read_many(
                    &mut usart1_port,
                    read_buf[..4].as_mut(),
                    &mut dtx,
                ) {
                    //writeln!(dtx, "\r\n{:?}\r", read_buf[0..4].as_ref()).unwrap();
                    if header_read != 4 {
                        writeln!(dtx, "header trunc! {} < 4 \r", header_read)
                            .unwrap();
                        last_byte = read_buf[header_read];
                        continue;
                    }
                    let msg_class = read_buf[0];
                    let msg_id = read_buf[1];
                    let payload_len: usize =
                        u16::from_le_bytes(read_buf[2..4].try_into().unwrap())
                            as usize;
                    writeln!(
                        dtx,
                        "[{}] 0x{:x} 0x{:x} \r",
                        payload_len, msg_class, msg_id,
                    )
                    .unwrap();
                    let body_len = payload_len + 2; //include checksum bytes
                    if body_len < 255 {
                        if let Ok(body_read) = read_many(
                            &mut usart1_port,
                            read_buf[4..body_len + 4].as_mut(),
                            &mut dtx,
                        ) {
                            if body_read != body_len {
                                writeln!(
                                    dtx,
                                    "body trunc! {} < {} \r",
                                    body_read, body_len
                                )
                                .unwrap();
                            }
                            writeln!(
                                dtx,
                                "{:x?}\r",
                                read_buf[..4 + body_read].as_ref()
                            )
                            .unwrap();
                            last_byte = read_buf[4 + body_len];
                        }
                    } else {
                        last_byte = read_buf[header_read];
                    }
                } else {
                    writeln!(dtx, "header fail \r").unwrap(); //" {} != {} \r",header_read, 4).unwrap();
                }
            } else {
                last_byte = byte;
            }
        } else {
            match result {
                Err(nb::Error::WouldBlock) => {}
                Err(nb::Error::Other(Error::Overrun)) => {}
                // Err(nb::Error::Other(Error::Framing)) => {
                //     error_count += 1;
                // },
                _ => {
                    writeln!(
                        dtx,
                        "\r\n{} {:?} {}\r",
                        baud, result, error_count
                    )
                    .unwrap();
                    error_count += 1;
                }
            }
        }
    }
}
