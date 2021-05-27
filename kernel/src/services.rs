// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

use crate::arch;
pub use crate::arch::process::Process as ArchProcess;
pub use crate::arch::process::Thread;
use xous_kernel::MemoryRange;

use core::num::NonZeroU8;

use crate::filled_array;
// use core::mem;
use xous_kernel::{
    pid_from_usize, Error, MemoryAddress, Message, ProcessInit, ThreadInit, CID, PID, SID, TID,
};

pub use crate::arch::process::{INITIAL_TID, MAX_PROCESS_COUNT};

/// A big unifying struct containing all of the system state.
/// This is inherited from the stage 1 bootloader.
pub struct SystemServices {
    /// A table of all processes in the system
    pub processes: [Process; MAX_PROCESS_COUNT],
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
    pub mem_default_base: usize,

    /// The last address allocated from
    pub mem_default_last: usize,

    /// Address where messages are passed into
    pub mem_message_base: usize,

    /// The last address that was allocated from
    pub mem_message_last: usize,

    /// Base address of the heap
    pub mem_heap_base: usize,

    /// Current size of the heap
    pub mem_heap_size: usize,

    /// Maximum size of the heap
    pub mem_heap_max: usize,

    /// A mapping of connection IDs to server indexes
    pub connection_map: [Option<NonZeroU8>; 32],

    /// A copy of this process' ID
    pub pid: PID,

    /// Some reserved data to pad this out to a multiple of 32 bytes.
    pub _reserved: [u8; 1],
}

impl Default for ProcessInner {
    fn default() -> Self {
        ProcessInner {
            mem_default_base: arch::mem::DEFAULT_BASE,
            mem_default_last: arch::mem::DEFAULT_BASE,
            mem_message_base: arch::mem::DEFAULT_MESSAGE_BASE,
            mem_message_last: arch::mem::DEFAULT_MESSAGE_BASE,
            mem_heap_base: arch::mem::DEFAULT_HEAP_BASE,
            mem_heap_size: 0,
            mem_heap_max: 524_288,
            connection_map: [None; 32],
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
        crate::arch::process::set_current_pid(self.pid);
        let mut current_process = crate::arch::process::Process::current();
        current_process.activate()
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
        previous_thread: INITIAL_TID as TID,
    }; MAX_PROCESS_COUNT],
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

    /// Add a new entry to the process table. This results in a new address space
    /// and a new PID, though the process is in the state `Setup()`.
    pub fn create_process(&mut self, init_process: ProcessInit) -> Result<PID, xous_kernel::Error> {
        for (idx, mut entry) in self.processes.iter_mut().enumerate() {
            if entry.state != ProcessState::Free {
                continue;
            }
            let new_pid = pid_from_usize(idx + 1)?;
            arch::process::Process::create(new_pid, init_process);
            let ppid = crate::arch::process::current_pid();
            // println!("Creating new process for PID {} with PPID {}", new_pid, ppid);
            entry.state = ProcessState::Allocated;
            entry.ppid = ppid;
            entry.pid = new_pid;
            return Ok(new_pid);
        }
        Err(xous_kernel::Error::ProcessNotFound)
    }

    pub fn get_process(&self, pid: PID) -> Result<&Process, xous_kernel::Error> {
        // PID0 doesn't exist -- process IDs are offset by 1.
        let pid_idx = pid.get() as usize - 1;
        Ok(&self.processes[pid_idx])
    }

    pub fn get_process_mut(&mut self, pid: PID) -> Result<&mut Process, xous_kernel::Error> {
        // PID0 doesn't exist -- process IDs are offset by 1.
        let pid_idx = pid.get() as usize - 1;

        Ok(&mut self.processes[pid_idx])
    }

    // pub fn current_thread(&self, pid: PID) -> usize {
    //     self.processes[pid.get() as usize - 1].current_thread as usize
    // }

    pub fn current_pid(&self) -> PID {
        arch::process::current_pid()
    }

    /// Create a stack frame in the specified process and jump to it.
    /// 1. Pause the current process and switch to the new one
    /// 2. Save the process state, if it hasn't already been saved
    /// 3. Run the new process, returning to an illegal instruction
    #[cfg(baremetal)]
    pub fn finish_callback_and_resume(
        &mut self,
        pid: PID,
        tid: TID,
    ) -> Result<(), xous_kernel::Error> {
        // Get the current process (which was the interrupt handler) and mark it
        // as Ready.  Note that the new PID may very well be the same PID.
        {
            let current_pid = self.current_pid();
            let mut current = self
                .get_process_mut(current_pid)
                .expect("couldn't get current PID");
            // println!("Finishing callback in PID {}", current_pid);
            current.state = match current.state {
                ProcessState::Running(0) => ProcessState::Sleeping,
                ProcessState::Running(x) => ProcessState::Ready(x),
                y => panic!("current process was {:?}, not 'Running(_)'", y),
            };
            // current.current_thread = current.previous_context;
        }

        // Get the new process, and ensure that it is in a state where it's fit
        // to run.  Again, if the new process isn't fit to run, then the system
        // is in a very bad state.
        {
            let mut process = self.get_process_mut(pid)?;
            // Ensure the new context is available to be run
            let available_contexts = match process.state {
                ProcessState::Ready(x) if x & 1 << tid != 0 => x & !(1 << tid),
                other => panic!(
                    "process {} was in an invalid state {:?} -- thread {} not available to run",
                    pid, other, tid
                ),
            };
            process.state = ProcessState::Running(available_contexts);
            // process.current_thread = tid as u8;
            process.activate()?;

            // Activate the current context
            let mut arch_process = crate::arch::process::Process::current();
            arch_process.set_thread(tid)?;
            process.current_thread = tid;
        }
        // self.pid = pid;
        Ok(())
    }

    // #[cfg(not(baremetal))]
    // pub fn make_callback_to(
    //     &mut self,
    //     _pid: PID,
    //     _pc: *const usize,
    //     _irq_no: usize,
    //     _arg: *mut usize,
    // ) -> Result<(), xous_kernel::Error> {
    //     Err(xous_kernel::Error::UnhandledSyscall)
    // }

    /// Create a stack frame in the specified process and jump to it.
    /// 1. Pause the current process and switch to the new one
    /// 2. Save the process state, if it hasn't already been saved
    /// 3. Run the new process, returning to an illegal instruction
    #[cfg(baremetal)]
    pub fn make_callback_to(
        &mut self,
        pid: PID,
        pc: *const usize,
        irq_no: usize,
        arg: *mut usize,
    ) -> Result<(), xous_kernel::Error> {
        // Get the current process (which was just interrupted) and mark it as
        // "ready to run".  If this function is called when the current process
        // isn't running, that means the system has gotten into an invalid
        // state.
        {
            let current_pid = self.current_pid();
            let mut current = self
                .get_process_mut(current_pid)
                .expect("couldn't get current PID");
            current.state = match current.state {
                ProcessState::Running(x) => {
                    ProcessState::Ready(x | (1 << arch::process::current_tid()))
                }
                y => panic!("current process was {:?}, not 'Running(_)'", y),
            };
            // println!("Making PID {} state {:?}", current_pid, current.state);
        }

        // Get the new process, and ensure that it is in a state where it's fit
        // to run.  Again, if the new process isn't fit to run, then the system
        // is in a very bad state.
        {
            let mut process = self.get_process_mut(pid)?;
            let available_threads = match process.state {
                ProcessState::Ready(x) | ProcessState::Running(x) => x,
                ProcessState::Sleeping | ProcessState::Debug(_) => 0,
                ProcessState::Free => panic!("process was not allocated"),
                ProcessState::Setup(_) | ProcessState::Allocated => {
                    panic!("process hasn't been set up yet")
                }
            };
            process.state = ProcessState::Running(available_threads);
            process.previous_thread = process.current_thread;
            process.current_thread = arch::process::IRQ_TID;
            process.activate()?;
        }

        // Switch to new process memory space, allowing us to save the context
        // if necessary.
        // self.pid = pid;

        // Invoke the syscall, but use the current stack pointer.  When this
        // function returns, it will jump to the RETURN_FROM_ISR address,
        // causing an instruction fault and exiting the interrupt.
        ArchProcess::with_current_mut(|arch_process| {
            let sp = if pid.get() == 1 {
                arch::mem::EXCEPTION_STACK_TOP
            } else {
                arch_process.current_thread().stack_pointer()
            };

            // Activate the current context
            arch_process.set_thread(arch::process::IRQ_TID).unwrap();

            // Construct the new frame
            arch::syscall::invoke(
                arch_process.current_thread_mut(),
                pid.get() == 1,
                pc as usize,
                sp,
                arch::process::RETURN_FROM_ISR,
                &[irq_no, arg as usize],
            );
        });
        Ok(())
    }

    /// Mark the specified context as ready to run. If the thread is Sleeping, mark
    /// it as Ready.
    pub fn ready_thread(&mut self, pid: PID, tid: TID) -> Result<(), xous_kernel::Error> {
        let process = self.get_process_mut(pid)?;
        process.state = match process.state {
            ProcessState::Free => {
                panic!("PID {} was not running, so cannot wake thread {}", pid, tid)
            }
            ProcessState::Running(x) if x & (1 << tid) == 0 => {
                ProcessState::Running(x | (1 << tid))
            }
            ProcessState::Ready(x) if x & (1 << tid) == 0 => ProcessState::Ready(x | (1 << tid)),
            ProcessState::Sleeping => ProcessState::Ready(1 << tid),
            other => panic!(
                "PID {} was not in a state to wake thread {}: {:?}",
                pid, tid, other
            ),
        };
        klog!("Readying ({}:{}) -> {:?}", pid, tid, process.state);
        Ok(())
    }

    /// Mark the current process as "Ready to run".
    ///
    /// # Panics
    ///
    /// If the current process is not running, or if it's "Running" but has no free contexts
    pub fn switch_to_thread(
        &mut self,
        pid: PID,
        tid: Option<TID>,
    ) -> Result<(), xous_kernel::Error> {
        let process = self.get_process_mut(pid)?;
        // klog!(
        //     "switch_to_thread({}:{:?}): Old state was {:?}",
        //     pid, tid, process.state
        // );

        // Determine which thread to switch to
        process.state = match process.state {
            ProcessState::Free => return Err(xous_kernel::Error::ProcessNotFound),
            ProcessState::Sleeping => return Err(xous_kernel::Error::ProcessNotFound),
            ProcessState::Allocated => return Err(xous_kernel::Error::ProcessNotFound),
            ProcessState::Debug(_) => panic!("foo"),
            ProcessState::Setup(setup) => {
                // Activate the process, which enables its memory mapping
                process.activate()?;

                // If a context is specified for a Setup task to switch to,
                // ensure it's the INITIAL_TID. Otherwise it's not valid.
                if let Some(tid) = tid {
                    if tid != INITIAL_TID {
                        panic!("switched to an incorrect thread");
                    }
                }

                let mut p = crate::arch::process::Process::current();
                p.setup_thread(INITIAL_TID, setup)?;
                p.set_thread(INITIAL_TID)?;
                ArchProcess::with_inner_mut(|process_inner| process_inner.pid = pid);
                process.current_thread = INITIAL_TID as _;

                // Mark the current proces state as "running, and no waiting contexts"
                ProcessState::Running(0)
            }
            ProcessState::Ready(0) => {
                panic!("ProcessState was `Ready(0)`, which is invalid!");
            }
            ProcessState::Ready(x) => {
                let new_thread = match tid {
                    None => {
                        let mut new_context = 0;

                        while x & (1 << new_context) == 0 {
                            new_context += 1;
                            if new_context > arch::process::MAX_THREAD {
                                new_context = 0;
                            }
                        }
                        new_context
                    }
                    Some(ctx) => {
                        // Ensure the specified context is ready to run
                        if x & (1 << ctx) == 0 {
                            return Err(xous_kernel::Error::InvalidThread);
                        }
                        ctx
                    }
                };

                process.activate()?;
                let mut p = crate::arch::process::Process::current();
                // FIXME: What happens if this fails? We're currently in the new process
                // but without a context to switch to.
                p.set_thread(new_thread)?;
                process.current_thread = new_thread as _;

                // Remove the new context from the available context list
                ProcessState::Running(x & !(1 << new_thread))
            }
            ProcessState::Running(0) => {
                // TODO: If `context` is not `None`, what do we do here?

                // This process is already running, and there aren't any new available
                // contexts, so keep on going.
                ProcessState::Running(0)
            }
            ProcessState::Running(ready_threads) => {
                let mut p = crate::arch::process::Process::current();
                // let current_thread = p.current_thread();
                let new_thread = match tid {
                    None => {
                        let mut new_thread = 0;

                        while ready_threads & (1 << new_thread) == 0 {
                            new_thread += 1;
                            if new_thread > arch::process::MAX_THREAD {
                                new_thread = 0;
                            }
                        }
                        new_thread
                    }
                    Some(tid) => {
                        // Ensure the specified context is ready to run, or is
                        // currently running.
                        if ready_threads & (1 << tid) == 0
                        /*&& ctx != current_thread*/
                        {
                            return Err(xous_kernel::Error::InvalidThread);
                        }
                        tid
                    }
                };

                // Remove the new thread ID from the list of thread IDs
                let new_mask = ready_threads & !(1 << new_thread);

                // Activate this process on this CPU
                process.activate()?;
                p.set_thread(new_thread)?;
                process.current_thread = new_thread as _;
                ProcessState::Running(new_mask)
            }
        };

        // println!(
        //     "switch_to_thread({}:{:?}): New state is {:?} Thread is ",
        //     pid, tid, process.state
        // );
        // ArchProcess::with_current(|current| current.print_thread());

        Ok(())
    }

    /// Switches away from the specified process ID.
    /// If `can_resume` is `true`, then the current thread ID will be placed
    /// in the list of available thread IDs.
    /// If no thread IDs are available, the process will enter a `Sleeping` state.
    ///
    /// # Panics
    ///
    /// If the current process is not running.
    pub fn switch_from_thread(&mut self, pid: PID, tid: TID) -> Result<(), xous_kernel::Error> {
        let process = self.get_process_mut(pid)?;
        // klog!(
        //     "switch_from_thread({}:{}): Old state was {:?}",
        //     pid, tid, process.state
        // );
        // ArchProcess::with_current(|current| current.print_thread());

        process.state = match process.state {
            ProcessState::Running(x) if x & (1 << tid) != 0 => panic!(
                "PID {} thread {} was already queued for running when `switch_from_thread()` was called",
                pid, tid
            ),
            ProcessState::Running(0) => {
                if cfg!(baremetal) {
                    ProcessState::Sleeping
                } else {
                    ProcessState::Running(0)
                }
            }
            ProcessState::Running(x) => {
                if cfg!(baremetal) {
                    ProcessState::Ready(x)
                } else {
                    ProcessState::Running(x)
                }
            }
            other => {
                // ::debug_here::debug_here!();
                panic!(
                    "PID {} TID {} was not in a state to be switched from: {:?}",
                    pid, tid, other
                );
            },
        };
        // klog!(
        //     "switch_from_thread({}:{}): New state is {:?}",
        //     pid, tid, process.state
        // );
        Ok(())
    }

    pub fn thread_is_running(&self, pid: PID, tid: TID) -> bool {
        let process = self.get_process(pid).unwrap();
        if let ProcessState::Running(thread_ids) = process.state {
            if thread_ids & (1 << tid) == 0 {
                return true;
            }
        }
        panic!("PID {} TID {} not running: {:?}", pid, tid, process.state);
        // match &process.state {
        //     &ProcessState::Sleeping => false,
        //     &ProcessState::Ready(_x) => false,
        //     &ProcessState::Free => false,
        //     &ProcessState::Running(x) if x & (1 << tid) != 0 => false,
        //     &ProcessState::Setup(_) => false,
        //     &ProcessState::Running(_) => true,
        // }
    }

    pub fn set_thread_result(
        &mut self,
        pid: PID,
        tid: TID,
        result: xous_kernel::Result,
    ) -> Result<(), xous_kernel::Error> {
        // Temporarily switch into the target process memory space
        // in order to pass the return value.
        let current_pid = self.current_pid();
        {
            let target_process = self.get_process(pid)?;
            target_process.activate()?;
            let mut arch_process = crate::arch::process::Process::current();
            arch_process.set_thread_result(tid, result);
        }

        // Return to the original memory space.
        let current_process = self
            .get_process(current_pid)
            .expect("couldn't switch back after setting context result");
        current_process.activate()?;
        Ok(())
    }

    /// Resume the given process, picking up exactly where it left off. If the
    /// process is in the Setup state, set it up and then resume.
    pub fn activate_process_thread(
        &mut self,
        previous_tid: TID,
        new_pid: PID,
        mut new_tid: TID,
        can_resume: bool,
    ) -> Result<TID, xous_kernel::Error> {
        let previous_pid = self.current_pid();

        if new_tid != 0 {
            klog!("Activating process {} thread {}", new_pid, new_tid);
        } else {
            klog!("Activating process {} thread ANY", new_pid);
        }
        //ArchProcess::with_current(|current| current.print_thread());

        // Save state if the PID has changed.  This will activate the new memory
        // space.
        if new_pid != previous_pid {
            let new = self.get_process_mut(new_pid)?;
            // println!("New state: {:?}", new.state);

            // Ensure the new process can be run.
            match new.state {
                ProcessState::Free => {
                    println!("PID {} was free", new_pid);
                    return Err(xous_kernel::Error::ProcessNotFound);
                }
                ProcessState::Setup(_) | ProcessState::Allocated => new_tid = INITIAL_TID,
                ProcessState::Running(x) | ProcessState::Ready(x) => {
                    // If no new context is specified, take the previous
                    // context.  If that is not runnable, do a round-robin
                    // search for the next available context.
                    assert!(
                        x != 0,
                        "process was {:?} but had no free contexts",
                        new.state
                    );
                    if new_tid == 0 {
                        new_tid = (new.current_thread + 1) as usize;
                        while x & (1 << new_tid) == 0 {
                            new_tid += 1;
                            if new_tid > arch::process::MAX_THREAD {
                                new_tid = 0;
                            } else if new_tid == (new.current_thread + 1) as usize {
                                // If we've looped around, return an error.
                                // println!("Looked through all contexts and couldn't find one that was ready");
                                return Err(xous_kernel::Error::ProcessNotFound);
                            }
                        }
                        new.current_thread = new_tid as _;
                        klog!("picked thread ID {}", new_tid);
                    } else if x & (1 << new_tid) == 0 {
                        // println!(
                        //     "thread is {:?}, which is not valid for new thread {}",
                        //     new.state, new_tid
                        // );
                        return Err(xous_kernel::Error::ProcessNotFound);
                    } else {
                        new.current_thread = new_tid as _;
                    }
                }
                ProcessState::Sleeping | ProcessState::Debug(_) => {
                    // println!("PID {} was sleeping or being debugged", new_pid);
                    Err(xous_kernel::Error::ProcessNotFound)?;
                }
            }


            // Set up the new process, if necessary.  Remove the new thread from
            // the list of ready threads.
            new.state = match new.state {
                ProcessState::Setup(thread_init) => {
                    // println!("Setting up new process...");
                    ArchProcess::setup_process(new_pid, thread_init)
                        .expect("couldn't set up new process");
                    ArchProcess::with_inner_mut(|process_inner| process_inner.pid = new_pid);

                    ProcessState::Running(0)
                }
                ProcessState::Allocated => {
                    ArchProcess::with_inner_mut(|process_inner| process_inner.pid = new_pid);
                    ProcessState::Running(0)
                }
                ProcessState::Free => panic!("process was suddenly Free"),
                ProcessState::Ready(x) | ProcessState::Running(x) => {
                    ProcessState::Running(x & !(1 << new_tid))
                }
                ProcessState::Sleeping => ProcessState::Running(0),
                ProcessState::Debug(_) => panic!("Process was being debugged"),
            };
            new.activate()?;

            // Mark the previous process as ready to run, since we just switched
            // away
            let previous = self
                .get_process_mut(previous_pid)
                .expect("couldn't get previous pid");
            let _oldstate = previous.state; // for tracking state in the debug print after the following closure
            previous.state = match previous.state {
                // If the previous process had exactly one thread that can be
                // run, then the Running thread list will be 0.  In that case,
                // we will either need to Sleep this process, or mark it as
                // being Ready to run.
                ProcessState::Running(x) if x == 0 => {
                    if can_resume
                    /*|| advance_thread*/
                    {
                        ProcessState::Ready(1 << previous_tid)
                    } else {
                        ProcessState::Sleeping
                    }
                }
                // Otherwise, there are additional threads that can be run.
                // Convert the previous process into "Ready", and include the
                // current context number only if `can_resume` is `true`.
                ProcessState::Running(x) => {
                    if can_resume {
                        ProcessState::Ready(x | (1 << previous_tid))
                    } else {
                        ProcessState::Ready(x)
                    }
                }
                other => panic!(
                    "previous process PID {} was in an invalid state (not Running): {:?}",
                    previous_pid, other
                ),
            };
            klog!(
                "PID {:?} state change from {:?} -> {:?}",
                previous_pid,
                _oldstate,
                previous.state
            );
            // klog!(
            //     "Set previous process PID {} state to {:?} (with can_resume = {})",
            //     previous_pid,
            //     previous.state,
            //     can_resume
            // );
        } else {
            let new = self.get_process_mut(new_pid)?;

            // If we wanted to switch to a "new" thread, and it's the same
            // as the one we just switched from, do nothing.
            if previous_tid == new_tid {
                if !can_resume {
                    panic!("tried to switch to our own thread without resume (current_thread: {}  previous_tid: {}  new_tid: {})",
                            new.current_thread, previous_tid, new_tid);
                }
                let mut process = crate::arch::process::Process::current();
                process.set_thread(new_tid).unwrap();
                new.current_thread = new_tid;
                return Ok(new_tid);
            }

            // Transition to the new state.
            new.state = if let ProcessState::Running(x) = new.state {
                let previous_tid = new.current_thread;
                // If no new thread is specified, take the previous
                // thread.  If that is not runnable, do a round-robin
                // search for the next available thread.
                if new_tid == 0 {
                    new_tid = (new.current_thread + 1) as usize;
                    while x & (1 << new_tid) == 0 {
                        new_tid += 1;
                        if new_tid > arch::process::MAX_THREAD {
                            new_tid = 0;
                        } else if new_tid == (new.current_thread + 1) as usize {
                            // If we've looped around, return an error.
                            return Err(xous_kernel::Error::ProcessNotFound);
                        }
                    }
                    new.current_thread = new_tid as _;
                } else if x & (1 << new_tid) == 0 {
                    return Err(xous_kernel::Error::ProcessNotFound);
                }
                // Mark the previous TID as being runnable, and remove the new TID
                // from the list of threads that can be run.
                ProcessState::Running(
                    x & !(1 << new_tid) | if can_resume { 1 << previous_tid } else { 0 },
                )
            } else {
                panic!(
                    "PID {} invalid process state (not Running): {:?}",
                    previous_pid, new.state
                )
            };
        }

        let mut process = crate::arch::process::Process::current();

        // Restore the previous thread, if one exists.
        process.set_thread(new_tid)?;

        Ok(new_tid)
    }

    /// Create a new thread in the current process.  Execution begins at
    /// `entrypoint`, with the stack pointer set to `stack_pointer`.  A single
    /// argument will be passed to the new function.
    ///
    /// The return address of this thread will be `EXIT_THREAD`, which the
    /// kernel can trap on to indicate a thread exited.
    ///
    /// # Errors
    ///
    /// * **ThreadNotAvailable**: The process has used all of its context
    ///   slots.
    pub fn create_thread(
        &mut self,
        pid: PID,
        thread_init: ThreadInit,
    ) -> Result<TID, xous_kernel::Error> {
        let mut process = self.get_process_mut(pid)?;
        process.activate()?;

        let mut arch_process = crate::arch::process::Process::current();
        let new_tid = arch_process
            .find_free_thread()
            .ok_or(xous_kernel::Error::ThreadNotAvailable)?;

        arch_process.setup_thread(new_tid, thread_init)?;

        // println!("KERNEL({}): Created new thread {}", pid, new_tid);

        // Queue the thread to run
        process.state = match process.state {
            ProcessState::Running(x) => ProcessState::Running(x | (1 << new_tid)),

            // This is the initial thread in this process -- schedule it to be run.
            ProcessState::Allocated => {
                ArchProcess::with_inner_mut(|process_inner| process_inner.pid = pid);
                ProcessState::Ready(1 << new_tid)
            }

            other => panic!(
                "error spawning thread: process was in an invalid state {:?}",
                other
            ),
        };

        Ok(new_tid)
    }

    /// Destroy the given thread. Returns `true` if the PID has been updated.
    /// # Errors
    ///
    /// * **ThreadNotAvailable**: The thread does not exist in this process
    #[cfg(baremetal)]
    pub fn destroy_thread(&mut self, pid: PID, tid: TID) -> Result<bool, xous_kernel::Error> {
        let current_pid = self.current_pid();
        assert_eq!(pid, current_pid);

        let mut waiting_threads = match self.get_process_mut(pid)?.state {
            ProcessState::Running(x) => x,
            state => panic!("Process was in an invalid state: {:?}", state),
        };

        // Destroy the thread at a hardware level
        let mut arch_process = crate::arch::process::Process::current();
        let return_value = arch_process.destroy_thread(tid).unwrap_or_default();

        // If there's another thread waiting on the return value of this thread,
        // wake it up and set its return value.
        if let Some((waiting_tid, _thread)) = arch_process.find_thread(|waiting_tid, thr| {
            (waiting_threads & (1 << waiting_tid)) == 0 // Thread is waiting (i.e. not ready to run)
                && thr.a0() == (xous_kernel::SysCallNumber::JoinThread as usize) // Thread called `JoinThread`
                && thr.a1() == (tid as usize) // It is waiting on our thread
        }) {
            // Wake up the thread
            self.set_thread_result(pid, waiting_tid, xous_kernel::Result::Scalar1(return_value))?;
            waiting_threads |= 1 << waiting_tid;
        }

        // Mark this process as `Ready` if there are waiting threads, or `Sleeping` if
        // there are no waiting threads.
        let mut new_pid = pid;
        {
            let process = self.get_process_mut(pid)?;
            process.state = if waiting_threads == 0 {
                new_pid = process.ppid;
                ProcessState::Sleeping
            } else {
                ProcessState::Ready(waiting_threads)
            };
        }

        // Switch to the next available TID. This moves the process back to a `Running` state.
        self.switch_to_thread(new_pid, None)?;

        Ok(new_pid != pid)
    }

    /// Park this thread if the target thread is currently running. Otherwise,
    /// return the value of the given thread.
    pub fn join_thread(
        &mut self,
        pid: PID,
        tid: TID,
        join_tid: TID,
    ) -> Result<xous_kernel::Result, xous_kernel::Error> {
        let current_pid = self.current_pid();
        assert_eq!(pid, current_pid);

        // We cannot wait on ourselves.
        if tid == join_tid {
            Err(xous_kernel::Error::ThreadNotAvailable)?
        }

        // If the target thread exists, put this thread to sleep.
        let arch_process = crate::arch::process::Process::current();
        if arch_process.thread_exists(join_tid) {
            // The target thread exists -- put this thread to sleep
            let ppid = self.get_process(pid).unwrap().ppid;
            self.activate_process_thread(tid, ppid, 0, false)
                .map(|_| Ok(xous_kernel::Result::ResumeProcess))
                .unwrap_or(Err(xous_kernel::Error::ProcessNotFound))
        } else {
            // The thread does not exist -- continue execution
            // Err(xous_kernel::Error::ThreadNotAvailable)
            Ok(xous_kernel::Result::Scalar1(0))
        }
    }
}
