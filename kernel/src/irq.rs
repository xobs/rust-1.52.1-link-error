// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

use crate::arch;
use xous_kernel::{MemoryAddress, PID};

static mut IRQ_HANDLERS: [Option<(PID, MemoryAddress, Option<MemoryAddress>)>; 32] = [None; 32];


pub fn interrupt_claim(
    irq: usize,
    pid: PID,
    f: MemoryAddress,
    arg: Option<MemoryAddress>,
) -> Result<(), xous_kernel::Error> {
    Ok(())
}
