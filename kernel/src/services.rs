// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

use xous_kernel::MemoryRange;

use core::num::NonZeroU8;

use crate::filled_array;
// use core::mem;
use xous_kernel::{
    pid_from_usize, Error, MemoryAddress, Message, ProcessInit, ThreadInit, CID, PID, SID, TID,
};

/// A big unifying struct containing all of the system state.
/// This is inherited from the stage 1 bootloader.
pub struct SystemServices {
    /// A table of all processes in the system
    pub processes: [Process; 4],
}

#[derive(Copy, Clone, PartialEq)]
pub enum ProcessState {
    /// This is an unallocated, free process
    Free,

    /// This process has been allocated, but has no threads yet
    Allocated,

    /// This is a brand-new process that hasn't been run yet, and needs its
    /// initial context set up.
    #[allow(dead_code)]
    Setup(ThreadInit),

    /// This process is able to be run.  The context bitmask describes contexts
    /// that are ready.
    Ready(usize /* context bitmask */),

    /// This is the current active process.  The context bitmask describes
    /// contexts that are ready, excluding the currently-executing context.
    Running(usize /* context bitmask */),

    /// This process is waiting for an event, such as as message or an
    /// interrupt.  There are no contexts that can be run. This is
    /// functionally equivalent to the invalid `Ready(0)` state.
    Sleeping,

    /// The process is currently being debugged. When it is resumed,
    /// this will turn into `Ready(usize)`
    Debug(usize),
}

impl core::fmt::Debug for ProcessState {
    fn fmt(&self, fmt: &mut core::fmt::Formatter) -> core::result::Result<(), core::fmt::Error> {
        use ProcessState::*;
        match *self {
            Free => write!(fmt, "Free"),
            Allocated => write!(fmt, "Allocated"),
            Setup(ti) => write!(fmt, "Setup({:?})", ti),
            Ready(rt) => write!(fmt, "Ready({:b})", rt),
            Running(rt) => write!(fmt, "Running({:b})", rt),
            Debug(rt) => write!(fmt, "Debug({:b})", rt),
            Sleeping => write!(fmt, "Sleeping"),
        }
    }
}

impl Default for ProcessState {
    fn default() -> ProcessState {
        ProcessState::Free
    }
}

#[derive(Copy, Clone, PartialEq)]
pub struct Process {
    /// Where this process is in terms of lifecycle
    state: ProcessState,

    /// This process' PID. This should match up with the index in the process table.
    pub pid: PID,

    /// The process that created this process, which tells who is allowed to
    /// manipulate this process.
    pub ppid: PID,

    /// The current thread ID
    current_thread: TID,

    /// The context number that was active before this process was switched
    /// away.
    previous_thread: TID,
}

impl Default for Process {
    fn default() -> Self {
        Process {
            ppid: unsafe { PID::new_unchecked(1) },
            ..Default::default()
        }
    }
}

/// This is per-process data.  The arch-specific definitions will instantiate
/// this struct in order to avoid the need to statically-allocate this for
/// all possible processes.
/// Note that this data is only available when the current process is active.
#[repr(C)]
#[derive(Debug, PartialEq, Copy, Clone)]
/// Default virtual address when MapMemory is called with no `virt`
pub struct ProcessInner {
    /// A copy of this process' ID
    pub pid: PID,

    /// Some reserved data to pad this out to a multiple of 32 bytes.
    pub _reserved: [u8; 1],
}

impl Default for ProcessInner {
    fn default() -> Self {
        ProcessInner {
            pid: unsafe { PID::new_unchecked(1) },
            _reserved: [0; 1],
        }
    }
}

impl Process {
    /// This process has at least one context that may be run
    pub fn runnable(&self) -> bool {
        match self.state {
            ProcessState::Setup(_) | ProcessState::Ready(_) => true,
            _ => false,
        }
    }

    /// This process slot is unallocated and may be turn into a process
    pub fn free(&self) -> bool {
        match self.state {
            ProcessState::Free => true,
            _ => false,
        }
    }

    pub fn activate(&self) -> Result<(), xous_kernel::Error> {
        Ok(())
    }

    pub fn terminate(&mut self) -> Result<(), xous_kernel::Error> {
        Ok(())
    }
}

#[cfg(not(baremetal))]
std::thread_local!(static SYSTEM_SERVICES: core::cell::RefCell<SystemServices> = core::cell::RefCell::new(SystemServices {
    processes: [Process {
        state: ProcessState::Free,
        ppid: unsafe { PID::new_unchecked(1) },
        pid: unsafe { PID::new_unchecked(1) },
        current_thread: 0 as TID,
        previous_thread: INITIAL_TID as TID,
    }; MAX_PROCESS_COUNT],
}));

#[cfg(baremetal)]
static mut SYSTEM_SERVICES: SystemServices = SystemServices {
    processes: [Process {
        state: ProcessState::Free,
        ppid: unsafe { PID::new_unchecked(1) },
        pid: unsafe { PID::new_unchecked(1) },
        current_thread: 0 as TID,
        previous_thread: 1 as TID,
    }; 4],
};

impl core::fmt::Debug for Process {
    fn fmt(&self, fmt: &mut core::fmt::Formatter) -> core::result::Result<(), core::fmt::Error> {
        write!(
            fmt,
            "Process {} state: {:?}",
            self.pid.get(),
            self.state,
        )
    }
}

impl SystemServices {
    /// Calls the provided function with the current inner process state.
    pub fn with<F, R>(f: F) -> R
    where
        F: FnOnce(&SystemServices) -> R,
    {
        #[cfg(baremetal)]
        unsafe {
            f(&SYSTEM_SERVICES)
        }
        #[cfg(not(baremetal))]
        SYSTEM_SERVICES.with(|ss| f(&ss.borrow()))
    }

    pub fn with_mut<F, R>(f: F) -> R
    where
        F: FnOnce(&mut SystemServices) -> R,
    {
        #[cfg(baremetal)]
        unsafe {
            f(&mut SYSTEM_SERVICES)
        }

        #[cfg(not(baremetal))]
        SYSTEM_SERVICES.with(|ss| f(&mut ss.borrow_mut()))
    }

    pub fn get_process(&self, pid: PID) -> Result<&Process, xous_kernel::Error> {
        // PID0 doesn't exist -- process IDs are offset by 1.
        let pid_idx = pid.get() as usize - 1;
        Ok(&self.processes[pid_idx])
    }

    pub fn current_pid(&self) -> PID {
        PID::new(4).unwrap()
    }
}
