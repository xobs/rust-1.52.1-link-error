// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

#[cfg(any(windows, unix))]
mod hosted;
#[cfg(any(windows, unix))]
pub use hosted::*;

#[cfg(target_arch = "riscv32")]
mod riscv;
#[cfg(target_arch = "riscv32")]
pub use crate::arch::riscv::*;

#[cfg(all(target_arch = "riscv64", not(target_os = "xous")))]
mod riscv;
#[cfg(all(target_arch = "riscv64", not(target_os = "xous")))]
pub use riscv::*;

#[cfg(all(target_arch = "x86_64", not(any(windows, unix))))]
mod x86_64;
#[cfg(all(target_arch = "x86_64", not(any(windows, unix))))]
pub use x86_64::*;
