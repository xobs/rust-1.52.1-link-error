// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

use xous_kernel::{SysCall, PID, TID};

/// Disable external interrupts
pub fn disable_all_irqs() {
}

/// Enable external interrupts
#[export_name = "_enable_all_irqs"]
pub extern "C" fn enable_all_irqs() {
}

pub fn enable_irq(irq_no: usize) {
}

pub fn disable_irq(irq_no: usize) -> Result<(), xous_kernel::Error> {
    Ok(())
}

pub unsafe fn set_isr_return_pair(pid: PID, tid: TID) {
}

/// Trap entry point rust (_start_trap_rust)
///
/// scause is read to determine the cause of the trap. The top bit indicates if
/// it's an interrupt or an exception. The result is converted to an element of
/// the Interrupt or Exception enum and passed to handle_interrupt or
/// handle_exception.
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
