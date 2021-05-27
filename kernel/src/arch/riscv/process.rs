// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

use core::mem;
static mut PROCESS: *mut ProcessImpl = 0xff80_1000 as *mut ProcessImpl;
pub const MAX_THREAD: TID = 31;
pub const INITIAL_TID: TID = 1;
pub const IRQ_TID: TID = 0;
use crate::services::ProcessInner;
use xous_kernel::{ProcessInit, ThreadInit, PID, TID};

// use crate::args::KernelArguments;
pub const DEFAULT_STACK_SIZE: usize = 131072;
pub const MAX_PROCESS_COUNT: usize = 64;
// pub use crate::arch::mem::DEFAULT_STACK_TOP;

/// This is the address a program will jump to in order to return from an ISR.
pub const RETURN_FROM_ISR: usize = 0xff80_2000;

/// This is the address a thread will return to when it exits.
pub const EXIT_THREAD: usize = 0xff80_3000;

// Thread IDs have three possible meaning:
// Logical Thread ID: What the user sees
// Thread Context Index: An index into the thread slice
// Hardware Thread ID: The index that the ISR uses
//
// The Hardware Thread ID is always equal to the Thread Context
// Index, minus one. For example, the default thread ID is
// Hardware Thread ID 1 is Thread Context Index 0.
// The Logical Thread ID is equal to the Hardware Thread ID
// plus one again. This is because the ISR context is Thread
// Context Index 0.
// Therefore, the first Logical Thread ID is 1, which maps
// to Hardware Thread ID 2, which is Thread Context Index 1.
//
// +-----------------+-----------------+-----------------+
// |    Thread ID    |  Context Index  | Hardware Thread |
// +=================+=================+=================+
// |   ISR Context   |        0        |        1        |
// |        1        |        1        |        2        |
// |        2        |        2        |        3        |

// ProcessImpl occupies a multiple of pages mapped to virtual address `0xff80_1000`.
// Each thread is 128 bytes (32 4-byte registers). The first "thread" does not exist,
// and instead is any bookkeeping information related to the process.
#[derive(Debug, Copy, Clone)]
#[repr(C)]
struct ProcessImpl {
    /// Used by the interrupt handler to calculate offsets
    scratch: usize,

    /// The currently-active thread for this process. This must
    /// be the 2nd item, because the ISR directly writes this value.
    hardware_thread: usize,

    /// Global parameters used by the operating system
    pub inner: ProcessInner,

    /// The last thread ID that was allocated
    last_tid_allocated: u8,

    /// Pad everything to 128 bytes, so the Thread slice starts at
    /// offset 128.
    _padding: [u32; 13],

    /// This enables the kernel to keep track of threads in the
    /// target process, and know which threads are ready to
    /// receive messages.
    threads: [Thread; MAX_THREAD],
}

/// Singleton process table. Each process in the system gets allocated from this table.
struct ProcessTable {
    /// The process upon which the current syscall is operating
    current: PID,

    /// The actual table contents. `true` if a process is allocated,
    /// `false` if it is free.
    table: [bool; MAX_PROCESS_COUNT],
}

static mut PROCESS_TABLE: ProcessTable = ProcessTable {
    current: unsafe { PID::new_unchecked(1) },
    table: [false; MAX_PROCESS_COUNT],
};

#[repr(C)]
#[cfg(baremetal)]
/// The stage1 bootloader sets up some initial processes.  These are reported
/// to us as (satp, entrypoint, sp) tuples, which can be turned into a structure.
/// The first element is always the kernel.
pub struct InitialProcess {
    /// The RISC-V SATP value, which includes the offset of the root page
    /// table plus the process ID.
    pub satp: usize,

    /// Where execution begins
    pub entrypoint: usize,

    /// Address of the top of the stack
    pub sp: usize,
}

#[repr(C)]
#[derive(Debug)]
pub struct Process {
    pid: PID,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Default)]
/// Everything required to keep track of a single thread of execution.
pub struct Thread {
    /// Storage for all RISC-V registers, minus $zero
    pub registers: [usize; 31],

    /// The return address.  Note that if this thread was created because of an
    /// `ecall` instruction, you will need to add `4` to this before returning,
    /// to prevent that instruction from getting executed again. If this is 0,
    /// then this thread is not valid.
    pub sepc: usize,
}

impl Process {
    pub fn current() -> Process {
        let pid = unsafe { PROCESS_TABLE.current };
        let hardware_pid = (riscv::register::satp::read().bits() >> 22) & ((1 << 9) - 1);
        assert!((pid.get() as usize) == hardware_pid);
        Process { pid }
    }

    /// Mark this process as running on the current core
    pub fn activate(&mut self) -> Result<(), xous_kernel::Error> {
        Ok(())
    }

    /// Calls the provided function with the current inner process state.
    pub fn with_inner<F, R>(f: F) -> R
    where
        F: FnOnce(&ProcessInner) -> R,
    {
        let process = unsafe { &*PROCESS };
        f(&process.inner)
    }

    /// Calls the provided function with the current inner process state.
    pub fn with_current<F, R>(f: F) -> R
    where
        F: FnOnce(&Process) -> R,
    {
        let process = Self::current();
        f(&process)
    }

    /// Calls the provided function with the current inner process state.
    pub fn with_current_mut<F, R>(f: F) -> R
    where
        F: FnOnce(&mut Process) -> R,
    {
        let mut process = Self::current();
        f(&mut process)
    }

    pub fn with_inner_mut<F, R>(f: F) -> R
    where
        F: FnOnce(&mut ProcessInner) -> R,
    {
        let process = unsafe { &mut *PROCESS };
        f(&mut process.inner)
    }

    pub fn current_thread_mut(&mut self) -> &mut Thread {
        let process = unsafe { &mut *PROCESS };
        assert!(process.hardware_thread != 0, "thread number was 0");
        &mut process.threads[process.hardware_thread - 1]
    }

    pub fn current_thread(&self) -> &Thread {
        let process = unsafe { &mut *PROCESS };
        &mut process.threads[process.hardware_thread - 1]
        // self.thread(process.hardware_thread - 1)
    }

    pub fn current_tid(&self) -> TID {
        let process = unsafe { &*PROCESS };
        process.hardware_thread - 1
    }

    pub fn thread_exists(&self, tid: TID) -> bool {
        self.thread(tid).sepc != 0
    }

    /// Set the current thread number.
    pub fn set_thread(&mut self, thread: TID) -> Result<(), xous_kernel::Error> {
        let mut process = unsafe { &mut *PROCESS };
        // println!("KERNEL({}:{}): Switching to thread {}", self.pid, process.hardware_thread - 1, thread);
        assert!(
            thread <= process.threads.len(),
            "attempt to switch to an invalid thread {}",
            thread
        );
        process.hardware_thread = thread + 1;
        Ok(())
    }

    pub fn thread_mut(&mut self, thread: TID) -> &mut Thread {
        let process = unsafe { &mut *PROCESS };
        assert!(
            thread <= process.threads.len(),
            "attempt to retrieve an invalid thread {}",
            thread
        );
        &mut process.threads[thread]
    }

    pub fn thread(&self, thread: TID) -> &Thread {
        let process = unsafe { &mut *PROCESS };
        assert!(
            thread <= process.threads.len(),
            "attempt to retrieve an invalid thread {}",
            thread
        );
        &process.threads[thread]
    }

    pub fn find_free_thread(&self) -> Option<TID> {
        let process = unsafe { &mut *PROCESS };
        let start_tid = process.last_tid_allocated as usize;
        let a = &process.threads[start_tid..process.threads.len()];
        let b = &process.threads[0..start_tid];
        for (index, thread) in a.iter().chain(b.iter()).enumerate() {
            let mut tid = index + start_tid;
            if tid >= process.threads.len() {
                tid -= process.threads.len()
            }

            if tid != IRQ_TID && thread.sepc == 0 {
                process.last_tid_allocated = tid as _;
                return Some(tid as TID);
            }
        }
        None
    }

    pub fn set_thread_result(&mut self, thread_nr: TID, result: xous_kernel::Result) {
        let vals = unsafe { mem::transmute::<_, [usize; 8]>(result) };
        let thread = self.thread_mut(thread_nr);
        for (idx, reg) in vals.iter().enumerate() {
            thread.registers[9 + idx] = *reg;
        }
    }

    pub fn retry_instruction(&mut self, tid: TID) -> Result<(), xous_kernel::Error> {
        let process = unsafe { &mut *PROCESS };
        let mut thread = &mut process.threads[tid];
        if thread.sepc >= 4 {
            thread.sepc -= 4;
        }
        Ok(())
    }


    pub fn find_thread<F>(&self, op: F) -> Option<(TID, &mut Thread)>
    where
        F: Fn(TID, &Thread) -> bool,
    {
        let process = unsafe { &mut *PROCESS };
        for (idx, thread) in process.threads.iter_mut().enumerate() {
            if thread.sepc == 0 {
                continue;
            }
            if op(idx, thread) {
                return Some((idx, thread));
            }
        }
        None
    }
}

impl Thread {
    /// The current stack pointer for this thread
    pub fn stack_pointer(&self) -> usize {
        self.registers[1]
    }

    pub fn a0(&self) -> usize {
        self.registers[9]
    }

    pub fn a1(&self) -> usize {
        self.registers[10]
    }
}

pub fn set_current_pid(pid: PID) {
    let pid_idx = (pid.get() - 1) as usize;
    unsafe {
        let mut pt = &mut PROCESS_TABLE;

        match pt.table.get(pid_idx) {
            None | Some(false) => panic!("PID {} does not exist", pid),
            _ => (),
        }
        pt.current = pid;
    }
}

pub fn current_pid() -> PID {
    unsafe { PROCESS_TABLE.current }
}

pub fn current_tid() -> TID {
    unsafe { ((*PROCESS).hardware_thread) - 1 }
}
