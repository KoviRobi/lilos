/*!
 * To use `cargo run -r -F pa-1core` or `cargo run -r -F pa-crit`
 */
#![no_std]
#![no_main]

use core::arch::asm;
use core::convert::Infallible;
use core::mem::MaybeUninit;
use core::pin::pin;

use embedded_hal::digital::v2::OutputPin as _;

use panic_halt as _;

use defmt_rtt as _;

use rp_pico as bsp;

use portable_atomic::{AtomicU32, Ordering};

use bsp::{hal, pac};
use cortex_m::peripheral::syst::SystClkSource;
use hal::gpio::{FunctionSioOutput, Pin as GpioPin, PullDown};
use hal::usb::UsbBus as UsbBusImpl;
use pac::interrupt;

type OutputPin<I, P> = GpioPin<I, FunctionSioOutput, P>;

use usb_device::class_prelude::*;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

use lilos::exec::{run_tasks_with_idle, Notify, ALL_TASKS};
use lilos::spsc::{Popper, Pusher, Queue};

static MEASURE_EVT: Notify = Notify::new();
static USB_EVT: Notify = Notify::new();

const SERIAL_READ_BUF: usize = 128;
const SERIAL_WRITE_BUF: usize = 256;
struct SerialPortWrite<'a>(
    SerialPort<'a, UsbBusImpl, [u8; SERIAL_READ_BUF], [u8; SERIAL_WRITE_BUF]>,
);

use core::fmt::Write as _;
impl<'a> core::fmt::Write for SerialPortWrite<'a> {
    fn write_str(&mut self, s: &str) -> Result<(), core::fmt::Error> {
        match self.0.write(s.as_bytes()) {
            Ok(_) => Ok(()),
            Err(_) => Err(core::fmt::Error),
        }
    }
}

#[cfg(not(feature = "flash"))]
extern "C" {
    #[link_name = "__vector_table"]
    static VECTOR_TABLE: u32;
}

#[cortex_m_rt_macros::entry]
fn main() -> ! {
    unsafe {
        const SIO_BASE: u32 = 0xd0000000;
        const SPINLOCK0_PTR: *mut u32 = (SIO_BASE + 0x100) as *mut u32;
        const SPINLOCK_COUNT: usize = 32;
        for i in 0..SPINLOCK_COUNT {
            SPINLOCK0_PTR.wrapping_add(i).write_volatile(1);
        }
    }

    let mut core = pac::CorePeripherals::take().unwrap();
    let mut pac = pac::Peripherals::take().unwrap();
    let sio = hal::Sio::new(pac.SIO);

    // Usually the vector table is at the start, but the way copy-to-RAM works
    // is the entry point has to be at the start
    #[cfg(not(feature = "flash"))]
    unsafe {
        core.SCB.vtor.write(&VECTOR_TABLE as *const u32 as u32)
    };

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let gpios = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut sched_gpio = gpios
        .gpio2
        .into_push_pull_output_in_state(hal::gpio::PinState::High);

    let mut usb_gpio = gpios
        .gpio3
        .into_push_pull_output_in_state(hal::gpio::PinState::High);

    let mut meas_gpio = gpios
        .gpio4
        .into_push_pull_output_in_state(hal::gpio::PinState::High);

    let usb_alloc = UsbBusAllocator::new(UsbBusImpl::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    static mut USB_ALLOC: Option<UsbBusAllocator<UsbBusImpl>> = None;
    unsafe { USB_ALLOC = Some(usb_alloc) };
    let usb_ref = unsafe { USB_ALLOC.as_mut().unwrap() };

    let mut serial = SerialPortWrite(SerialPort::new_with_store(
        usb_ref,
        [0; SERIAL_READ_BUF],
        [0; SERIAL_WRITE_BUF],
    ));

    let mut buf = [MaybeUninit::<u32>::uninit(); 512];
    let mut serial_q = Queue::new(&mut buf);
    let (push, pop) = serial_q.split();

    let usb_dev = UsbDeviceBuilder::new(usb_ref, UsbVidPid(0x1234, 0xabcd))
        .manufacturer("KoviRobi")
        .serial_number("1234ABCD")
        .product("Lilos profile")
        .build();
    static mut USB_DEV: Option<UsbDevice<'_, UsbBusImpl>> = None;
    unsafe { USB_DEV = Some(usb_dev) };
    let usb_dev = unsafe { USB_DEV.as_mut().unwrap() };

    let reload = (1 << 24) - 1;
    core.SYST.set_reload(reload);
    core.SYST.clear_current();
    core.SYST.set_clock_source(SystClkSource::Core);
    core.SYST.disable_interrupt();
    core.SYST.enable_counter();

    run_tasks_with_idle(
        &mut [
            pin!(measure(push, &core.SYST, &mut meas_gpio)),
            pin!(usb_task(pop, &mut serial, usb_dev, &mut usb_gpio)),
            // A couple of never tasks to exercise the task walking bit of lilos
            pin!(never_task()),
            pin!(never_task()),
            pin!(never_task()),
            pin!(never_task()),
            pin!(never_task()),
        ],
        ALL_TASKS,
        || {
            let _ = sched_gpio.set_high();
            MEASURE_EVT.notify();
            let _ = sched_gpio.set_low();
        },
    );
}

async fn measure(
    mut pusher: Pusher<'_, u32>,
    syst: &pac::SYST,
    gpio: &mut OutputPin<hal::gpio::bank0::Gpio4, PullDown>,
) -> Infallible {
    let mut prev = syst.cvr.read();
    loop {
        let now = syst.cvr.read();
        pusher.reserve().await.push(prev - now);
        prev = syst.cvr.read();
        let _ = gpio.set_low();
        MEASURE_EVT.until_next().await; // Let the scheduler go to idle
        let _ = gpio.set_high();
    }
}

async fn never_task() -> Infallible {
    let never = Notify::new();
    loop {
        never.until(|| false).await;
    }
}

async fn usb_task(
    mut popper: Popper<'_, u32>,
    serial: &mut SerialPortWrite<'_>,
    usb_dev: &mut UsbDevice<'_, UsbBusImpl>,
    gpio: &mut OutputPin<hal::gpio::bank0::Gpio3, PullDown>,
) -> Infallible {
    let mut prev_pop = 0;
    let atomic: AtomicU32 = AtomicU32::new(0);

    loop {
        let _ = gpio.set_low();
        unsafe { pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ) };
        USB_EVT
            .until(|| pac::NVIC::is_pending(pac::Interrupt::USBCTRL_IRQ))
            .await;
        pac::NVIC::unpend(pac::Interrupt::USBCTRL_IRQ);
        let _ = gpio.set_high();

        if usb_dev.poll(&mut [&mut serial.0]) {
            let mut buf = [0; 64];
            match serial.0.read(&mut buf) {
                Ok(0) => {}
                Err(_) => {}
                Ok(n) => {
                    for b in buf.into_iter().take(n) {
                        if b == b'\x08' || b == b'\x7f' {
                            let _ = serial.0.write(b"\x08 \x08");
                        } else if b == b'b' {
                            hal::rom_data::reset_to_usb_boot(0, 0);
                        } else if b == b'a' {
                            let mut a: u32;
                            let mut b: u32;
                            unsafe {
                                asm!(
                                    "ldr {a}, [{SYST}, 8]",
                                    SYST = in(reg) cortex_m::peripheral::SYST::PTR,
                                    a = out(reg) a,
                                );
                            }
                            let x = atomic.load(Ordering::SeqCst);
                            unsafe {
                                asm!(
                                    "ldr {b}, [{SYST}, 8]",
                                    SYST = in(reg) cortex_m::peripheral::SYST::PTR,
                                    b = out(reg) b,
                                );
                            }
                            let _ = write!(
                                serial,
                                "atomic load {} took {}\r\n",
                                x,
                                a - b
                            );
                        } else if b == b'+' {
                            let mut a: u32;
                            let mut b: u32;
                            unsafe {
                                asm!(
                                    "ldr {a}, [{SYST}, 8]",
                                    "adds {r}, {r}, #0",
                                    "ldr {b}, [{SYST}, 8]",
                                    SYST = in(reg) cortex_m::peripheral::SYST::PTR,
                                    a = out(reg) a,
                                    b = out(reg) b,
                                    r = in(reg) 0,
                                );
                            }
                            let _ = write!(serial, "add took {}\r\n", a - b);
                        } else if b == b'*' {
                            let mut a: u32;
                            let mut b: u32;
                            unsafe {
                                asm!(
                                    "ldr {a}, [{SYST}, 8]",
                                    "muls {r}, {r}, {one}",
                                    "ldr {b}, [{SYST}, 8]",
                                    SYST = in(reg) cortex_m::peripheral::SYST::PTR,
                                    a = out(reg) a,
                                    b = out(reg) b,
                                    r = in(reg) 0,
                                    one = in(reg) 1,
                                );
                            }
                            let _ = write!(serial, "mul took {}\r\n", a - b);
                        } else if b == b'n' {
                            let mut a: u32;
                            let mut b: u32;
                            unsafe {
                                asm!(
                                    "ldr {a}, [{SYST}, 8]",
                                    "ldr {b}, [{SYST}, 8]",
                                    SYST = in(reg) cortex_m::peripheral::SYST::PTR,
                                    a = out(reg) a,
                                    b = out(reg) b,
                                );
                            }
                            let _ = write!(serial, "noop took {}\r\n", a - b);
                        } else if b == b'i' {
                            let mut a: u32;
                            let mut b: u32;
                            unsafe {
                                asm!(
                                    "ldr {a}, [{SYST}, 8]",
                                    "isb",
                                    "ldr {b}, [{SYST}, 8]",
                                    SYST = in(reg) cortex_m::peripheral::SYST::PTR,
                                    a = out(reg) a,
                                    b = out(reg) b,
                                );
                            }
                            let _ = write!(serial, "isb took {}\r\n", a - b);
                        } else if b == b'p' {
                            let _ = write!(serial, "profiling\r\n");
                            let mut i = 0;
                            while let Some(w) = popper.try_pop() {
                                if w != prev_pop || i & 0xFF == 0 {
                                    let _ = write!(
                                        serial,
                                        "{:03}. took {}\r\n",
                                        i, w
                                    );
                                    prev_pop = w;
                                }
                                i += 1;
                            }
                        } else {
                            let _ = write!(
                                serial,
                                "Can time: + * i(sb) n(oop) a(tomic load)\r\n"
                            );
                            let _ = write!(
                                serial,
                                "p for profile task switch, print some times\r\n"
                            );
                            let _ = write!(
                                serial,
                                "b for reset to USB bootloader\r\n"
                            );
                        }
                    }
                }
            }
        }
    }
}

#[interrupt]
fn USBCTRL_IRQ() {
    USB_EVT.notify();
    pac::NVIC::mask(pac::Interrupt::USBCTRL_IRQ);
}
