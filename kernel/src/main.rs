// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

#![no_main]
#![no_std]

use core::num::NonZeroUsize;

mod debug;

use core::panic::PanicInfo;
#[panic_handler]
fn handle_panic(_arg: &PanicInfo) -> ! {
    loop {}
}

#[no_mangle]
pub extern "C" fn _start() -> ! {
    rsyscall(NonZeroUsize::new(debug::irq as *mut usize as usize).unwrap());
    loop {}
}

pub fn rsyscall(fpntr: NonZeroUsize /* function pointer */) {
}
