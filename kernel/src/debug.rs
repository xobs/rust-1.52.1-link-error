// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

pub struct Uart {}

#[cfg(feature = "gdbserver")]
mod gdb_server {
    use gdbstub::arch::Arch;

    pub struct MyTarget {}

    #[derive(Debug, Clone, PartialEq, Default)]
    pub struct MyReg {}

    #[derive(Debug, Clone, PartialEq)]
    pub struct MyRegId {}

    impl gdbstub::arch::RegId for MyRegId {
        fn from_raw_id(id: usize) -> Option<(Self, usize)> {
            None
        }
    }

    impl gdbstub::arch::Registers for MyReg
    {
        type ProgramCounter = u32;

        fn pc(&self) -> Self::ProgramCounter {
            0
        }

        fn gdb_serialize(&self, mut write_byte: impl FnMut(Option<u8>)) {}
        fn gdb_deserialize(&mut self, bytes: &[u8]) -> Result<(), ()> {
            Ok(())
        }
    }

    impl gdbstub::arch::Arch for MyTarget {
        type Usize = u32;
        type Registers = MyReg;
        type RegId = MyRegId;
        type BreakpointKind = usize;
        fn target_description_xml() -> Option<&'static str> {
            Some(r#"<target version="1.0"><architecture>riscv</architecture></target>"#)
        }
    }
    use gdbstub::common::Tid;
    use gdbstub::target::ext::base::multithread::{
        GdbInterrupt, MultiThreadOps, ResumeAction, ThreadStopReason,
    };

    use gdbstub::target::ext::base::BaseOps;
    use gdbstub::target::{Target, TargetResult};
    use gdbstub::GdbStubStateMachine;

    pub struct XousTarget {}

    pub static mut GDB_SERVER: Option<(GdbStubStateMachine<XousTarget, super::Uart>, XousTarget)> =
        None;

    impl Target for XousTarget {
        type Arch = MyTarget;
        type Error = &'static str;
        fn base_ops(&mut self) -> BaseOps<Self::Arch, Self::Error> {
            BaseOps::MultiThread(self)
        }
    }

    impl MultiThreadOps for XousTarget {
        #[inline(never)]
        fn resume(
            &mut self,
            default_resume_action: ResumeAction,
            gdb_interrupt: GdbInterrupt<'_>,
        ) -> Result<ThreadStopReason<u32>, Self::Error> {
            Ok(ThreadStopReason::GdbInterrupt)
        }

        #[inline(never)]
        fn clear_resume_actions(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }

        #[inline(never)]
        fn set_resume_action(
            &mut self,
            _tid: Tid,
            _action: ResumeAction,
        ) -> Result<(), Self::Error> {
            Ok(())
        }

        #[inline(never)]
        fn read_registers(
            &mut self,
            _regs: &mut MyReg,
            _tid: Tid,
        ) -> TargetResult<(), Self> {
            Ok(())
        }

        #[inline(never)]
        fn write_registers(
            &mut self,
            _regs: &MyReg,
            _tid: Tid,
        ) -> TargetResult<(), Self> {
            Ok(())
        }

        #[inline(never)]
        fn read_addrs(
            &mut self,
            _start_addr: u32,
            data: &mut [u8],
            _tid: Tid, // same address space for each core
        ) -> TargetResult<(), Self> {
            Ok(())
        }

        #[inline(never)]
        fn write_addrs(
            &mut self,
            _start_addr: u32,
            _data: &[u8],
            _tid: Tid, // same address space for each core
        ) -> TargetResult<(), Self> {
            Ok(())
        }

        #[inline(never)]
        fn list_active_threads(
            &mut self,
            register_thread: &mut dyn FnMut(Tid),
        ) -> Result<(), Self::Error> {
            Ok(())
        }
    }
}

#[cfg(feature = "gdbserver")]
impl gdbstub::Connection for Uart {
    type Error = ();

    fn read(&mut self) -> Result<u8, Self::Error> {
        Err(())
    }
    fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
        Ok(())
    }
    fn peek(&mut self) -> Result<Option<u8>, Self::Error> {
        Ok(None)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub fn irq(_irq_number: usize, _arg: *mut usize) {
    let b = b'4' as char;

    #[cfg(feature = "gdbserver")]
    unsafe {
        use crate::debug::gdb_server::GDB_SERVER;
        if let Some((gdb, target)) = &mut GDB_SERVER.as_mut() {
            gdb.pump(target, b as u8).unwrap();
            return;
        }
    }
}
