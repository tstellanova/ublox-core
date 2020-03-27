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
use cortex_m_rt as rt;
use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use stm32h7xx_hal as p_hal;
use stm32h7xx_hal::{pac, prelude::*};

use core::fmt::Write;
use p_hal::serial::Error;

use ublox_core as ublox;

/// This example was tested on the Durandal stm32h743-based
/// flight controller board, which provides a USART1 ("GPS1") port that
/// connects to a number of ublox-m8 based GPS modules using a
/// "dronecode" standard connector.
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
    let mut ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;
    let mut delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);

    // Acquire the gpio peripherals.
    // This also enables the clock for GPIO in the RCC register.
    let gpiob = dp.GPIOB.split(&mut ccdr.ahb4);
    let gpioe = dp.GPIOE.split(&mut ccdr.ahb4);
    let gpiof = dp.GPIOF.split(&mut ccdr.ahb4);

    //UART7 is debug (dronecode debug port): `(PF6, PE8)`
    let uart7_port = {
        let config =
            p_hal::serial::config::Config::default().baudrate(57_600_u32.bps());
        let rx = gpiof.pf6.into_alternate_af7();
        let tx = gpioe.pe8.into_alternate_af7();
        dp.UART7.usart((tx, rx), config, &mut ccdr).unwrap()
    };

    // GPS1 port USART1:
    let mut usart1_port = {
        let config =
            p_hal::serial::config::Config::default().baudrate(115200.bps());
        let rx = gpiob.pb7.into_alternate_af7();
        let tx = gpiob.pb6.into_alternate_af7();
        dp.USART1.usart((tx, rx), config, &mut ccdr).unwrap()
    };

    delay_source.delay_ms(1u8);

    let (mut dtx, mut _drx) = uart7_port.split();
    let driver = ublox::new_serial_driver(usart1_port);
    monitor_serial_ubx(&mut dtx, &mut driver);
}

fn monitor_serial_ubx(console: &mut u32, driver: &mut u32) {
    const UBX_SYNC1: u8 = 0xb5;
    const UBX_SYNC2: u8 = 0x62;
    let mut packet_buf = [0u8; 128];
    let mut recv_count = 0;
    let mut error_count = 0;
    // let mut framing_error_count = 0;
    let mut header_found = false;
    let mut last_byte: u8 = 0;
    loop {
        let result = usart1_port.read();
        if let Ok(byte) = result {
            error_count = 0;

            if byte == UBX_SYNC2 && last_byte == UBX_SYNC1 {
                if header_found && recv_count > 2 {
                    //close out the last packet
                    let slc = &packet_buf[..recv_count - 1];
                    write!(dtx, "[{}] ", recv_count).unwrap();
                    for b in slc {
                        write!(dtx, "{:x} ", b).unwrap();
                    }
                    writeln!(dtx, "\r").unwrap();
                    //write!(dtx, "{:?} ", packet_buf).unwrap();
                }
                header_found = true;
                recv_count = 0;
                // writeln!(dtx, "\r\n>>> baud: {} {} \r", baud, recv_count).unwrap();
                packet_buf[0] = UBX_SYNC1;
                packet_buf[1] = UBX_SYNC2;
            }
            if header_found && recv_count > 2 {
                packet_buf[recv_count] = byte;
                //write!(dtx, "{:x} ", byte).unwrap();
            }
            recv_count += 1;
            if recv_count > 200 {
                error_count = 100;
                header_found = false; //reset
            }
            last_byte = byte;
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
