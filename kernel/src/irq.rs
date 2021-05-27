// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

pub fn interrupt_claim(
    irq: usize,
    pid: xous_kernel::PID,
    f: xous_kernel::MemoryAddress,
    arg: Option<xous_kernel::MemoryAddress>,
) -> Result<(), xous_kernel::Error> {
    Ok(())
}
