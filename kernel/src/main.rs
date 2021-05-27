// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

#![cfg_attr(baremetal, no_main)]
#![cfg_attr(baremetal, no_std)]

use core::num::NonZeroUsize;

#[cfg(baremetal)]
#[macro_use]
extern crate bitflags;

#[macro_use]
mod debug;

#[cfg(baremetal)]
use core::panic::PanicInfo;
#[cfg(baremetal)]
#[panic_handler]
fn handle_panic(_arg: &PanicInfo) -> ! {
    loop {}
}

#[no_mangle]
pub extern "C" fn kmain() -> ! {
    loop {}
}

#[no_mangle]
pub extern "C" fn _enable_all_irqs() {}

#[export_name = "_start_trap_rust"]
pub extern "C" fn trap_handler(
    a0: usize,
    a1: usize,
    a2: usize,
    a3: usize,
    a4: usize,
    a5: usize,
    a6: usize,
    a7: usize,
) -> ! {
    loop {}
}

#[cfg(baremetal)]
#[no_mangle]
/// This function is called from baremetal startup code to initialize various kernel structures
/// based on arguments passed by the bootloader. It is unused when running under an operating system.
pub extern "C" fn init(arg_offset: *const u32, init_offset: *const u32, rpt_offset: *mut u32) {
    // Either map memory using a syscall, or if we're debugging the syscall
    // handler then directly map it.
    claim_interrupt(0, debug::irq).expect("Couldn't claim debug interrupt");
}

/// Claim a hardware interrupt for this process.
pub fn claim_interrupt(
    irq_no: usize,
    callback: fn(irq_no: usize, arg: *mut usize),
) -> core::result::Result<(), xous_kernel::Error> {
    let result = rsyscall(xous_kernel::SysCall::ClaimInterrupt(
        irq_no,
        NonZeroUsize::new(callback as *mut usize as usize).unwrap(),
        NonZeroUsize::new(4),
    ));
    Err(xous_kernel::Error::InternalError)
}

/// Perform a raw syscall and return the result. This will transform
/// `xous::Result::Error(e)` into an `Err(e)`.
pub fn rsyscall(call: xous_kernel::SysCall) {
    let mut ret = xous_kernel::Result::Ok;
    unsafe { _xous_syscall(0, 1, 2, 3, 4, 5, 6, 7, &mut ret) };
}

extern "Rust" {
    fn _xous_syscall(
        nr: usize,
        a1: usize,
        a2: usize,
        a3: usize,
        a4: usize,
        a5: usize,
        a6: usize,
        a7: usize,
        ret: &mut xous_kernel::Result,
    );
}
