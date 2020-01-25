//! An example of a UART interrupt

#![no_std]
#![no_main]

extern crate panic_halt;

use bsp::interrupt;
use bsp::rt;
use teensy4_bsp as bsp;

use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::serial::{Read, Write};

use core::sync::atomic::{AtomicUsize, Ordering};

struct UnsafeStackPtr<T>(*mut T);
impl<T> UnsafeStackPtr<T> {
    const fn null() -> Self {
        UnsafeStackPtr(core::ptr::null_mut())
    }

    fn new(obj: &mut T) -> Self {
        UnsafeStackPtr(obj)
    }
}
unsafe impl<T> Send for UnsafeStackPtr<T> {}
unsafe impl<T> Sync for UnsafeStackPtr<T> {}
static mut UART3: UnsafeStackPtr<bsp::hal::uart::UART<bsp::hal::uart::module::_3>> =
    UnsafeStackPtr::null();

/// Change the TX FIFO sizes to see how the FIFO affects the number
/// of `WouldBlock`s that we would see. Setting this to zero disables
/// the FIFO.
const TX_FIFO_SIZE: u8 = 4;
/// Change me to affect the partity bit generation
const PARITY: Option<bsp::hal::uart::Parity> = Some(bsp::hal::uart::Parity::Odd);

/// Writes `bytes` to the provided `uart`. The function will count the
/// number of blocks that we hit, and will log how many blocks we
/// required to transmit `bytes`.
fn write<W: Write<u8>>(uart: &mut W, bytes: &[u8]) -> Result<(), W::Error> {
    let mut blocks = 0;
    for byte in bytes {
        loop {
            match uart.write(*byte) {
                Ok(()) => break,
                Err(nb::Error::WouldBlock) => blocks += 1,
                Err(nb::Error::Other(err)) => return Err(err),
            }
        }
    }
    log::info!("{} blocks to transmit {:?}", blocks, bytes);
    Ok(())
}

static DATA_AVAILBLE: AtomicUsize = AtomicUsize::new(0);
static mut BUFFER: [u8; 256] = [0; 256];

#[rt::interrupt]
unsafe fn LPUART3() {
    let uart = &mut *(UART3.0);
    if let Ok(byte) = uart.read() {
        let idx = DATA_AVAILBLE.load(Ordering::SeqCst);
        BUFFER[idx] = byte;
        DATA_AVAILBLE.fetch_add(1, Ordering::SeqCst);
    }
}

#[rt::entry]
fn main() -> ! {
    let mut peripherals = bsp::Peripherals::take().unwrap();
    peripherals.log.init(Default::default());
    bsp::delay(5_000);
    let uarts = peripherals.uart.clock(
        &mut peripherals.ccm.handle,
        bsp::hal::ccm::uart::ClockSelect::OSC,
        bsp::hal::ccm::uart::PrescalarSelect::DIVIDE_1,
    );
    let mut uart3 = uarts
        .uart3
        .init(
            peripherals.pins.p17.alt2(),
            peripherals.pins.p16.alt2(),
            115_200,
        )
        .unwrap();
    let fifo_size = uart3.set_tx_fifo(core::num::NonZeroU8::new(TX_FIFO_SIZE));
    log::info!("Setting TX FIFO to {}", fifo_size);
    // If this is disabled, we won't receive the four bytes from the transfer!
    unsafe {
        UART3 = UnsafeStackPtr::new(&mut uart3);
    }
    uart3.set_parity(PARITY);
    uart3.set_receiver_interrupt(Some(0));
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::LPUART3);
    }
    loop {
        bsp::delay(1_000);
        peripherals.led.toggle().unwrap();
        write(&mut uart3, &[0xDE, 0xAD, 0xBE, 0xEF]).unwrap();
        while DATA_AVAILBLE.load(Ordering::SeqCst) < 4 {
            core::sync::atomic::spin_loop_hint();
        }
        unsafe {
            log::info!("Read data {:?}", &BUFFER[..4]);
        }
        DATA_AVAILBLE.store(0, Ordering::SeqCst);
    }
}
