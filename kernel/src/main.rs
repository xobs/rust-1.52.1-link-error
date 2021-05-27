// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

#![cfg_attr(baremetal, no_main)]
#![cfg_attr(baremetal, no_std)]

#[cfg(baremetal)]
#[macro_use]
extern crate bitflags;

#[macro_use]
mod debug;

mod arch;

#[macro_use]
mod args;
mod irq;
mod macros;
mod mem;
mod server;
mod services;
mod syscall;

use services::SystemServices;
use xous_kernel::*;

#[cfg(baremetal)]
use core::panic::PanicInfo;
#[cfg(baremetal)]
#[panic_handler]
fn handle_panic(_arg: &PanicInfo) -> ! {
    println!("PANIC in PID {}: {}", crate::arch::current_pid(), _arg);
    loop {
        arch::idle();
    }
}

#[cfg(baremetal)]
#[no_mangle]
/// This function is called from baremetal startup code to initialize various kernel structures
/// based on arguments passed by the bootloader. It is unused when running under an operating system.
pub extern "C" fn init(arg_offset: *const u32, init_offset: *const u32, rpt_offset: *mut u32) {
    // Either map memory using a syscall, or if we're debugging the syscall
    // handler then directly map it.
    #[cfg(any(feature = "debug-print", feature = "print-panics"))]
    {
        xous_kernel::claim_interrupt(4, debug::irq, 0 as *mut usize)
            .expect("Couldn't claim debug interrupt");
    }
}

/// Common main function for baremetal and hosted environments.
#[no_mangle]
pub extern "C" fn kmain() {
    loop {}
}
