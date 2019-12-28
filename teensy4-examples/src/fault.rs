//! Demonstrates a custom fault handler that
//! logs the exception frame

#![no_std]
#![no_main]

use bsp::rt::{exception, ExceptionFrame};
use embedded_hal::digital::v2::OutputPin;
use teensy4_bsp as bsp;

extern crate panic_halt;

static mut LED: Option<bsp::LED> = None;

#[exception]
unsafe fn HardFault(frame: &ExceptionFrame) -> ! {
    LED.as_mut().unwrap().set_high().unwrap();
    log::error!("HARD FAULT\n{:?}", frame);
    loop {
        core::sync::atomic::spin_loop_hint();
    }
}

#[bsp::rt::entry]
fn main() -> ! {
    let p = bsp::Peripherals::take().unwrap();
    unsafe { LED = Some(p.led) };
    p.log.init(Default::default());

    bsp::delay(5000);
    log::info!("Writing to read-only memory in 3...");
    bsp::delay(1000);
    log::info!("2...");
    bsp::delay(1000);
    log::info!("1...");
    bsp::delay(1000);
    unsafe {
        const LPI2C1_VERID: *mut u32 = 0x403F_0000 as *mut u32;
        LPI2C1_VERID.write(5);
        log::info!("You won't see this read: {}", LPI2C1_VERID.read());
    }
    loop {
        core::sync::atomic::spin_loop_hint();
    }
}
