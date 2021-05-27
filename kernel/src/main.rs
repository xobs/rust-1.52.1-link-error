// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

#![cfg_attr(target_os = "none", no_main)]
#![cfg_attr(target_os = "none", no_std)]

use core::num::NonZeroUsize;

#[macro_use]
mod debug;

use core::panic::PanicInfo;
#[panic_handler]
fn handle_panic(_arg: &PanicInfo) -> ! {
    loop {}
}

#[no_mangle]
/// This function is called from baremetal startup code to initialize various kernel structures
/// based on arguments passed by the bootloader. It is unused when running under an operating system.
pub extern "C" fn _start() -> !{
    // Either map memory using a syscall, or if we're debugging the syscall
    // handler then directly map it.
    rsyscall(NonZeroUsize::new(debug::irq as *mut usize as usize).unwrap());
    loop {}
}

#[repr(C)]
pub enum TestResult {
    Pass,
    Fail,
}

/// Perform a raw syscall and return the result. This will transform
/// `xous::Result::Error(e)` into an `Err(e)`.
pub fn rsyscall(
    fpntr: NonZeroUsize, /* function pointer */
) {
    let mut ret = TestResult::Pass;
}
