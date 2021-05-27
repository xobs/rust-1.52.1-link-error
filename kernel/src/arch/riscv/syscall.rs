// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

use crate::services::Thread;

pub fn invoke(
    thread: &mut Thread,
    supervisor: bool,
    pc: usize,
    sp: usize,
    ret_addr: usize,
    args: &[usize],
) {
}

pub fn resume(supervisor: bool, thread: &Thread) -> ! {
    loop {}
}
