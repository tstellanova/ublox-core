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
// use cortex_m_rt as rt;
use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use p_hal::{pac, prelude::*};
use stm32h7xx_hal as p_hal;

use arrayvec::ArrayString;
use core::fmt;
use core::fmt::{Arguments, Write};

use ublox_core as ublox;

/// This example was tested on the Durandal stm32h743-based
/// flight controller board, which provides a USART1 ("GPS1") port that
/// connects to a number of ublox-m8 based GPS modules using a
/// "dronecode" standard connector.
/// In addition the Durandal provides a "debug" port that consists
/// of both SWD as well as a serial UART.
///
/// This sample assumes that the ublox M8 compatible sensor is
/// attached to the GPS port and that a serial terminal emulator is
/// attached to the debug port.
///
/// run with eg `cargo run --example monitor --target thumbv7em-none-eabihf`
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
    let usart1_port = {
        let config =
            p_hal::serial::config::Config::default().baudrate(115200.bps());
        let rx = gpiob.pb7.into_alternate_af7();
        let tx = gpiob.pb6.into_alternate_af7();
        dp.USART1.usart((tx, rx), config, &mut ccdr).unwrap()
    };
    delay_source.delay_ms(1u8);

    let (mut console_tx, mut _console_rx) = uart7_port.split();
    let mut driver = ublox::new_serial_driver(usart1_port);
    driver.setup(&mut delay_source).unwrap();

    loop {
        let rc = driver.handle_one_message();
        if let Ok(msg_count) = rc {
            if msg_count > 0 {
                if let Some(nav_pvt) = driver.take_last_nav_pvt() {
                    console_print(
                        &mut console_tx,
                        format_args!(
                            ">>> nav_pvt {} lat, lon: {}, {} \r\n",
                            nav_pvt.itow, nav_pvt.lat, nav_pvt.lon,
                        ),
                    );
                }
                if let Some(nav_dop) = driver.take_last_nav_dop() {
                    console_print(
                        &mut console_tx,
                        format_args!(">>> nav_dop {} \r\n", nav_dop.itow),
                    );
                }
                if let Some(mon_hw) = driver.take_last_mon_hw() {
                    console_print(
                        &mut console_tx,
                        format_args!(">>> mon_hw jam: {} \r\n", mon_hw.jam_ind),
                    );
                }
            }
        } else {
            console_print(&mut console_tx, format_args!(">>> {:?} \r\n", rc));
        }
        delay_source.delay_ms(1u8);
    }
}

fn console_print(
    out: &mut (impl Write + embedded_hal::serial::Write<u8>),
    args: Arguments<'_>,
) {
    let mut format_buf = ArrayString::<[u8; 64]>::new();
    format_buf.clear();
    if fmt::write(&mut format_buf, args).is_ok() {
        //write on console out
        let _ = out.write_str(format_buf.as_str());
        let _ = out.flush();
    }
}
