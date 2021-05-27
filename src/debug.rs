// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

pub struct Uart {}

#[cfg(feature = "gdbserver")]
mod gdb_server {
    use gdbstub::GdbStubStateMachine;
    pub static mut GDB_SERVER: Option<GdbStubStateMachine> = None;
}

pub fn irq(_irq_number: usize, _arg: *mut usize) {
    let b = b'4' as char;

    #[cfg(feature = "gdbserver")]
    unsafe {
        use crate::debug::gdb_server::GDB_SERVER;
        if let Some(gdb) = &mut GDB_SERVER.as_mut() {
            gdb.pump(b as u8).unwrap();
        }
    }
}
