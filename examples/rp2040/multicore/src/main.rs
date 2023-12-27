//! Multi-core example of using `lilos` to blink an LED at varying intervals on
//! the Raspberry Pi Pico board.
//!
//! This starts a task on each core, one which computes a delay, and sends it to
//! the other core via the FIFO, which then uses that delay to blink the LED.
//!
//! It is an adaptation of the `multicore_fifo_blink` example in `rp2040-hal.
//!
//! This demonstrates
//!
//! 1. How to start the `lilos` executor and configure timekeeping.
//! 2. How to perform periodic actions and delays.
//! 3. How to share data between cores using the multicor FIFO
//! 4. How to use a custom lilos timer implementation instead of the default
//!    single-core systick implementation

// We won't be using the standard library.
#![no_std]
// We don't have a conventional `main` (`cortex_m_rt::entry` is different).
#![no_main]

// Pull in a panic handling crate. We have to `extern crate` this explicitly
// because it isn't otherwise referenced in code!
extern crate panic_halt;

pub mod fifo;
use fifo::AsyncFifo;

use rp_pico as bsp;

use bsp::{hal, hal::pac};
use hal::multicore::{Multicore, Stack};
use hal::Clock;

use embedded_hal::digital::v2::ToggleableOutputPin;

// For RP2040, we need to include a bootloader. The general Cargo build process
// doesn't have great support for this, so we included it as a binary constant.
#[link_section = ".boot_loader"]
#[used]
static BOOT: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

static mut CORE1_STACK: Stack<4096> = Stack::new();

#[export_name = "lilos::exec::cpu_core_id"]
fn cpu_core_id() -> u16 {
    hal::Sio::core() as u16
}

#[bsp::entry]
fn main() -> ! {
    // Check out peripherals from the runtime.
    let mut core = pac::CorePeripherals::take().unwrap();
    let mut pac = pac::Peripherals::take().unwrap();
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
    let sys_clk = clocks.system_clock.freq();

    // Make it so that `wfe` waits for masked interrupts as well as events --
    // the problem is that the idle-task is called with interrupts disabled (to
    // not have an interrupt fire before we call the idle task but after we
    // check that we should sleep -- for `wfi` it would just wake up).
    // See
    // https://www.embedded.com/the-definitive-guide-to-arm-cortex-m0-m0-wake-up-operation/
    const SEVONPEND: u32 = 1 << 4;
    unsafe {
        core.SCB.scr.modify(|scr| scr | SEVONPEND);
    }

    let mut sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led = pins.gpio25.into_push_pull_output();

    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    let _task = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        // Because both core's peripherals are mapped to the same address, this
        // is not necessary, but serves as a reminder that core 1 has its own
        // core peripherals
        // See also https://github.com/rust-embedded/cortex-m/issues/149
        let mut core = unsafe { pac::CorePeripherals::steal() };
        let pac = unsafe { pac::Peripherals::steal() };
        let mut sio = hal::Sio::new(pac.SIO);

        // Make it so that `wfe` waits for masked interrupts as well as events --
        // the problem is that the idle-task is called with interrupts disabled (to
        // not have an interrupt fire before we call the idle task but after we
        // check that we should sleep -- for `wfi` it would just wake up).
        // See
        // https://www.embedded.com/the-definitive-guide-to-arm-cortex-m0-m0-wake-up-operation/
        const SEVONPEND: u32 = 1 << 4;
        unsafe {
            core.SCB.scr.modify(|scr| scr | SEVONPEND);
        }

        // And so we need to initialize sys-tick on core 1 too
        lilos::time::initialize_sys_tick(&mut core.SYST, sys_clk.to_Hz());

        fifo::reset_read_fifo(&mut sio.fifo);

        // Create a task to blink the LED. You could also write this as an `async
        // fn` but we've inlined it as an `async` block for simplicity.
        let blink = core::pin::pin!(async {
            // Loop forever, blinking things. Note that this borrows the device
            // peripherals `p` from the enclosing stack frame.
            loop {
                let delay = sio.fifo.read_async().await;
                lilos::time::sleep_for(lilos::time::Millis(delay as u64)).await;
                led.toggle().unwrap();
            }
        });

        lilos::exec::run_tasks_with_idle(
            &mut [blink],           // <-- array of tasks
            lilos::exec::ALL_TASKS, // <-- which to start initially
            // We use `SEV` to signal from the other core that we can send more
            // data. See also the comment above on SEVONPEND
            cortex_m::asm::wfe,
        )
    });

    lilos::time::initialize_sys_tick(&mut core.SYST, sys_clk.to_Hz());

    let compute_delay = core::pin::pin!(async {
        /// How much we adjust the LED period every cycle
        const INC: i32 = 2;
        /// The minimum LED toggle interval we allow for.
        const MIN: i32 = 0;
        /// The maximum LED toggle interval period we allow for. Keep it reasonably short so it's easy to see.
        const MAX: i32 = 100;
        loop {
            for period in (MIN..MAX).step_by(INC as usize) {
                sio.fifo.write_async(period as u32).await;
            }
            for period in (MIN..MAX).step_by(INC as usize).rev() {
                sio.fifo.write_async(period as u32).await;
            }
        }
    });

    // Set up and run the scheduler with a single task.
    lilos::exec::run_tasks_with_idle(
        &mut [compute_delay],   // <-- array of tasks
        lilos::exec::ALL_TASKS, // <-- which to start initially
        // We use `SEV` to signal from the other core that we can send more
        // data. See also the comment above on SEVONPEND
        cortex_m::asm::wfe,
    )
}
