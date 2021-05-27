// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

use crate::mem::MemoryManager;
use xous_kernel::{MemoryFlags, MemoryType, PID};

pub const TRNG_KERNEL: Trng = Trng {
    // the HW device mapping is done in xous-rs/src/lib.rs/init()
    // the manually chosen virtual address has to be in the top 4MiB as it is the only page shared among all processes
    base: 0xffce_0000 as *mut usize, // see https://github.com/betrusted-io/xous-core/blob/master/docs/memory.md
};

pub struct Trng {
    pub base: *mut usize,
}

pub fn init() {
}

pub fn get_u32() -> u32 {
    4
}
