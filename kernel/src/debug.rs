// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

#[cfg(baremetal)]
use core::fmt::{Error, Write};

#[macro_use]
#[cfg(all(
    not(test),
    baremetal,
    any(feature = "debug-print", feature = "print-panics")
))]
pub mod debug_print_hardware {
    // the HW device mapping is done in main.rs/init(); the virtual address has to be in the top 4MiB as it is the only page shared among all processes
    pub const SUPERVISOR_UART_ADDR: *mut usize = 0xffcf_0000 as *mut usize; // see https://github.com/betrusted-io/xous-core/blob/master/docs/memory.md

    #[macro_export]
    macro_rules! print
    {
        ($($args:tt)+) => ({
                use core::fmt::Write;
                let _ = write!(crate::debug::Uart {}, $($args)+);
        });
    }
}
#[cfg(all(
    not(test),
    baremetal,
    any(feature = "debug-print", feature = "print-panics")
))]
pub use crate::debug::debug_print_hardware::SUPERVISOR_UART_ADDR;

#[cfg(all(
    not(test),
    baremetal,
    not(any(feature = "debug-print", feature = "print-panics"))
))]
#[macro_export]
macro_rules! print {
    ($($args:tt)+) => {{
        ()
    }};
}

#[cfg(baremetal)]
#[macro_export]
macro_rules! println
{
	() => ({
		print!("\r\n")
	});
	($fmt:expr) => ({
		print!(concat!($fmt, "\r\n"))
	});
	($fmt:expr, $($args:tt)+) => ({
		print!(concat!($fmt, "\r\n"), $($args)+)
	});
}

#[cfg(baremetal)]
pub struct Uart {}
#[cfg(baremetal)]
static mut INITIALIZED: bool = false;

#[cfg(all(baremetal, feature = "wrap-print"))]
static mut CHAR_COUNT: usize = 0;

#[cfg(baremetal)]
impl Uart {
    #[allow(dead_code)]
    pub fn init(self) {
        unsafe { INITIALIZED = true };
    }

    pub fn putc(&self, c: u8) {
        if unsafe { INITIALIZED != true } {
            return;
        }
    }

    #[allow(dead_code)]
    pub fn getc(&self) -> Option<u8> {
        None
    }
}

#[cfg(all(feature = "gdbserver", baremetal))]
mod gdb_server {
    use gdbstub::common::Tid;
    use gdbstub::target::ext::base::multithread::{
        GdbInterrupt, MultiThreadOps, ResumeAction, ThreadStopReason,
    };

    use gdbstub::target::ext::base::BaseOps;
    use gdbstub::target::{Target, TargetResult};
    use gdbstub::GdbStubStateMachine;

    pub struct XousTarget {
        pid: Option<xous_kernel::PID>,
        tid: Option<xous_kernel::TID>,
        thread_mask: usize,
    }

    pub static mut GDB_SERVER: Option<(GdbStubStateMachine<XousTarget, super::Uart>, XousTarget)> =
        None;
    pub static mut GDB_BUFFER: [u8; 4096] = [0u8; 4096];

    impl XousTarget {
        pub fn new() -> XousTarget {
            XousTarget {
                pid: None,
                tid: None,
                thread_mask: 0,
            }
        }
    }

    impl Target for XousTarget {
        type Arch = gdbstub_arch::riscv::Riscv32;
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
            match default_resume_action {
                ResumeAction::Step | ResumeAction::StepWithSignal(_) => {
                    Err("single-stepping not supported")?
                }
                _ => (),
            }

            crate::services::SystemServices::with(|system_services| {
                let current_pid = system_services.current_pid();

                for process in &system_services.processes {
                    if !process.free() {
                        println!(
                            "PID {}:",
                            process.pid,
                        );
                        process.activate().unwrap();
                        crate::arch::mem::MemoryMapping::current().print_map();
                        println!();
                    }
                }
                system_services
                    .get_process(current_pid)
                    .unwrap()
                    .activate()
                    .unwrap();
            });
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
            _regs: &mut gdbstub_arch::riscv::reg::RiscvCoreRegs<u32>,
            _tid: Tid,
        ) -> TargetResult<(), Self> {
            Ok(())
        }

        #[inline(never)]
        fn write_registers(
            &mut self,
            _regs: &gdbstub_arch::riscv::reg::RiscvCoreRegs<u32>,
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
            data.iter_mut().for_each(|b| *b = 0x55);
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

#[cfg(all(feature = "gdbserver", baremetal))]
impl gdbstub::Connection for Uart {
    type Error = ();

    fn read(&mut self) -> Result<u8, Self::Error> {
        Err(())
    }
    fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
        if unsafe { INITIALIZED != true } {
            Err(())?;
        }
        Ok(())
    }
    fn peek(&mut self) -> Result<Option<u8>, Self::Error> {
        Ok(None)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[cfg(all(
    not(test),
    baremetal,
    any(feature = "debug-print", feature = "print-panics")
))]
pub fn irq(_irq_number: usize, _arg: *mut usize) {
    let b = Uart {}
        .getc()
        .expect("no character queued despite interrupt") as char;

    #[cfg(feature = "gdbserver")]
    unsafe {
        use crate::debug::gdb_server::GDB_SERVER;
        if let Some((gdb, target)) = &mut GDB_SERVER.as_mut() {
            gdb.pump(target, b as u8).unwrap();
            return;
        }
    }

    match b {
        #[cfg(feature = "gdbserver")]
        'g' => {
            use gdb_server::{XousTarget, GDB_BUFFER, GDB_SERVER};
            println!("Starting GDB server -- attach your debugger now");
            let xous_target = XousTarget::new();
            match gdbstub::GdbStubBuilder::new(Uart {})
                .with_packet_buffer(unsafe { &mut GDB_BUFFER })
                .build()
            {
                Ok(gdb) => match gdb.run_state_machine() {
                    Ok(gdb_state_machine) => unsafe {
                        GDB_SERVER = Some((gdb_state_machine, xous_target))
                    },
                    Err(e) => println!("Unable to start GDB state machine: {}", e),
                },
                Err(e) => println!("Unable to start GDB server: {}", e),
            }
        }
        'h' => {
            println!("Xous Kernel Debug");
            println!("key | command");
            println!("--- + -----------------------");
            #[cfg(feature = "gdbserver")]
            println!(" g  | enter the gdb server");
        }
        _ => {}
    }
}

#[cfg(baremetal)]
impl Write for Uart {
    fn write_str(&mut self, s: &str) -> Result<(), Error> {
        for c in s.bytes() {
            self.putc(c);
        }
        Ok(())
    }
}

#[cfg(feature = "debug-print")]
#[macro_export]
macro_rules! klog
{
	() => ({
		print!(" [{}:{}]", file!(), line!())
	});
	($fmt:expr) => ({
        print!(concat!(" [{}:{} ", $fmt, "]"), file!(), line!())
	});
	($fmt:expr, $($args:tt)+) => ({
		print!(concat!(" [{}:{} ", $fmt, "]"), file!(), line!(), $($args)+)
	});
}

#[cfg(not(feature = "debug-print"))]
#[macro_export]
macro_rules! klog {
    ($($args:tt)+) => {{
        ()
    }};
}
