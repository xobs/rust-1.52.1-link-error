// SPDX-FileCopyrightText: 2020 Sean Cross <sean@xobs.io>
// SPDX-License-Identifier: Apache-2.0

use crate::arch;
use crate::arch::process::Process as ArchProcess;
use crate::irq::interrupt_claim;
use crate::mem::{MemoryManager, PAGE_SIZE};
use crate::server::{SenderID, WaitingMessage};
use crate::services::SystemServices;
use core::mem;
use xous_kernel::*;

/* Quoth Xobs:
 The idea behind SWITCHTO_CALLER was that you'd have a process act as a scheduler,
 where it would know all of its children processes. It would call SwitchTo(pid, tid)
 on its children, which would call Yield or WaitEvent as necessary that would then
 cause execution to return to the parent process.

 If the timer hit, it would call ReturnToParent() which would also return to the caller.

 Currently (as of Mar 2021) this functionality isn't being used, it's just returning
 back to the kernel, e.g. (PID,TID) = (1,1)
*/
/// This is the PID/TID of the last person that called SwitchTo
static mut SWITCHTO_CALLER: Option<(PID, TID)> = None;

#[derive(PartialEq)]
enum ExecutionType {
    Blocking,
    NonBlocking,
}

pub fn reset_switchto_caller() {
    unsafe { SWITCHTO_CALLER = None };
}

fn retry_syscall(pid: PID, tid: TID) -> SysCallResult {
    if cfg!(baremetal) {
        arch::process::Process::with_current_mut(|p| p.retry_instruction(tid))?;
        do_yield(pid, tid)
    } else {
        Ok(xous_kernel::Result::WouldBlock)
    }
}

fn do_yield(_pid: PID, tid: TID) -> SysCallResult {
    // If we're not running on bare metal, treat this as a no-op.
    if !cfg!(baremetal) {
        return Ok(xous_kernel::Result::Ok);
    }

    let (parent_pid, parent_ctx) = unsafe {
        SWITCHTO_CALLER
            .take()
            .expect("yielded when no parent context was present")
    };
    //println!("\n\r ***YIELD CALLED***");
    SystemServices::with_mut(|ss| {
        // TODO: Advance thread
        ss.activate_process_thread(tid, parent_pid, parent_ctx, true)
            .map(|_| Ok(xous_kernel::Result::ResumeProcess))
            .unwrap_or(Err(xous_kernel::Error::ProcessNotFound))
    })
}

fn send_message(pid: PID, thread: TID, cid: CID, message: Message) -> SysCallResult {
    SystemServices::with_mut(|ss| {
        let sidx = ss
            .sidx_from_cid(cid)
            .ok_or(xous_kernel::Error::ServerNotFound)?;

        let server_pid = ss
            .server_from_sidx(sidx)
            .expect("server couldn't be located")
            .pid;

        // Remember the address the message came from, in case we need to
        // return it after the borrow is through.
        let client_address = match &message {
            Message::Scalar(_) | Message::BlockingScalar(_) => None,
            Message::Move(msg) | Message::MutableBorrow(msg) | Message::Borrow(msg) => {
                Some(msg.buf.addr)
            }
        };

        // Translate memory messages from the client process to the server
        // process. Additionally, determine whether the call is blocking. If
        // so, switch to the server context right away.
        let blocking = message.is_blocking();
        let message = match message {
            Message::Scalar(_) | Message::BlockingScalar(_) => message,
            Message::Move(msg) => {
                let new_virt = ss.send_memory(
                    msg.buf.as_mut_ptr() as *mut usize,
                    server_pid,
                    core::ptr::null_mut(),
                    msg.buf.len(),
                )?;
                Message::Move(MemoryMessage {
                    id: msg.id,
                    buf: MemoryRange::new(new_virt as usize, msg.buf.len())?,
                    offset: msg.offset,
                    valid: msg.valid,
                })
            }
            Message::MutableBorrow(msg) => {
                let new_virt = ss.lend_memory(
                    msg.buf.as_mut_ptr() as *mut usize,
                    server_pid,
                    core::ptr::null_mut(),
                    msg.buf.len(),
                    true,
                )?;
                Message::MutableBorrow(MemoryMessage {
                    id: msg.id,
                    buf: MemoryRange::new(new_virt as usize, msg.buf.len())?,
                    offset: msg.offset,
                    valid: msg.valid,
                })
            }
            Message::Borrow(msg) => {
                let new_virt = ss.lend_memory(
                    msg.buf.as_mut_ptr() as *mut usize,
                    server_pid,
                    core::ptr::null_mut(),
                    msg.buf.len(),
                    false,
                )?;
                // println!(
                //     "Lending {} bytes from {:08x} in PID {} to {:08x} in PID {}",
                //     msg.buf.len(),
                //     msg.buf.as_mut_ptr() as usize,
                //     pid,
                //     new_virt as usize,
                //     server_pid,
                // );
                Message::Borrow(MemoryMessage {
                    id: msg.id,
                    buf: MemoryRange::new(new_virt as usize, msg.buf.len())?,
                    offset: msg.offset,
                    valid: msg.valid,
                })
            }
        };

        // If the server has an available thread to receive the message,
        // transfer it right away.
        if let Some(server_tid) = ss
            .server_from_sidx_mut(sidx)
            .expect("server couldn't be located")
            .take_available_thread()
        {
            // klog!(
            //     "there are threads available in PID {} to handle this message -- marking as Ready",
            //     server_pid
            // );
            let sender_idx = if message.is_blocking() {
                ss.remember_server_message(sidx, pid, thread, &message, client_address)
                    .map_err(|e| {
                        klog!("error remembering server message: {:?}", e);
                        ss.server_from_sidx_mut(sidx)
                            .expect("server couldn't be located")
                            .return_available_thread(server_tid);
                        e
                    })?
            } else {
                0
            };
            let sender = SenderID::new(sidx, sender_idx, Some(pid));
            klog!(
                "server connection data: sidx: {}, idx: {}, server pid: {}",
                sidx,
                sender_idx,
                server_pid
            );
            let envelope = MessageEnvelope {
                sender: sender.into(),
                body: message,
            };

            // Mark the server's context as "Ready". If this fails, return the context
            // to the blocking list.
            ss.ready_thread(server_pid, server_tid).map_err(|e| {
                ss.server_from_sidx_mut(sidx)
                    .expect("server couldn't be located")
                    .return_available_thread(server_tid);
                e
            })?;

            if blocking && cfg!(baremetal) {
                klog!("Activating Server context and switching away from Client");
                ss.activate_process_thread(thread, server_pid, server_tid, false)
                    .map(|_| Ok(xous_kernel::Result::Message(envelope)))
                    .unwrap_or(Err(xous_kernel::Error::ProcessNotFound))
            } else if blocking && !cfg!(baremetal) {
                klog!("Blocking client, since it sent a blocking message");
                ss.switch_from_thread(pid, thread)?;
                ss.switch_to_thread(server_pid, Some(server_tid))?;
                ss.set_thread_result(
                    server_pid,
                    server_tid,
                    xous_kernel::Result::Message(envelope),
                )
                .map(|_| xous_kernel::Result::BlockedProcess)
            } else if cfg!(baremetal) {
                klog!("Setting the return value of the Server ({}:{}) to {:?} and returning to Client",
                    server_pid, server_tid, envelope);
                ss.set_thread_result(
                    server_pid,
                    server_tid,
                    xous_kernel::Result::Message(envelope),
                )
                .map(|_| xous_kernel::Result::Ok)
            } else {
                klog!(
                    "setting the return value of the Server to {:?} and returning to Client",
                    envelope
                );
                // "Switch to" the server PID when not running on bare metal. This ensures
                // that it's "Running".
                ss.switch_to_thread(server_pid, Some(server_tid))?;
                ss.set_thread_result(
                    server_pid,
                    server_tid,
                    xous_kernel::Result::Message(envelope),
                )
                .map(|_| xous_kernel::Result::Ok)
            }
        } else {
            klog!(
                "no threads available in PID {} to handle this message, so queueing",
                server_pid
            );
            // Add this message to the queue.  If the queue is full, this
            // returns an error.
            let _queue_idx = ss.queue_server_message(sidx, pid, thread, message, client_address)?;
            klog!("queued into index {:x}", _queue_idx);

            // Park this context if it's blocking.  This is roughly
            // equivalent to a "Yield".
            if blocking {
                if cfg!(baremetal) {
                    // println!("Returning to parent");
                    let process = ss.get_process(pid).expect("Can't get current process");
                    let ppid = process.ppid;
                    unsafe { SWITCHTO_CALLER = None };
                    ss.activate_process_thread(thread, ppid, 0, false)
                        .map(|_| Ok(xous_kernel::Result::ResumeProcess))
                        .unwrap_or(Err(xous_kernel::Error::ProcessNotFound))
                } else {
                    ss.switch_from_thread(pid, thread)?;
                    Ok(xous_kernel::Result::BlockedProcess)
                }
            } else {
                // println!("Returning to Client with Ok result");
                Ok(xous_kernel::Result::Ok)
            }
        }
    })
}

fn return_memory(
    server_pid: PID,
    server_tid: TID,
    in_irq: bool,
    sender: MessageSender,
    buf: MemoryRange,
    offset: Option<MemorySize>,
    valid: Option<MemorySize>,
) -> SysCallResult {
    SystemServices::with_mut(|ss| {
        let sender = SenderID::from(sender);

        let server = ss
            .server_from_sidx_mut(sender.sidx)
            .ok_or(xous_kernel::Error::ServerNotFound)?;
        if server.pid != server_pid {
            return Err(xous_kernel::Error::ServerNotFound);
        }
        let result = server.take_waiting_message(sender.idx, Some(&buf))?;
        klog!("waiting message was: {:?}", result);
        let (client_pid, client_tid, _server_addr, client_addr, len) = match result {
            WaitingMessage::BorrowedMemory(
                client_pid,
                client_ctx,
                server_addr,
                client_addr,
                len,
            ) => (client_pid, client_ctx, server_addr, client_addr, len),
            WaitingMessage::MovedMemory => {
                return Ok(xous_kernel::Result::Ok);
            }
            WaitingMessage::ForgetMemory(range) => {
                return MemoryManager::with_mut(|mm| {
                    let mut result = Ok(xous_kernel::Result::Ok);
                    let virt = range.addr.get();
                    let size = range.size.get();
                    if cfg!(baremetal) && virt & 0xfff != 0 {
                        klog!("VIRT NOT DIVISIBLE BY 4: {:08x}", virt);
                        return Err(xous_kernel::Error::BadAlignment);
                    }
                    for addr in (virt..(virt + size)).step_by(PAGE_SIZE) {
                        if let Err(e) = mm.unmap_page(addr as *mut usize) {
                            if result.is_ok() {
                                result = Err(e);
                            }
                        }
                    }
                    result
                })
            }
            WaitingMessage::ScalarMessage(_pid, _tid) => {
                println!("WARNING: Tried to wait on a message that was a scalar");
                return Err(xous_kernel::Error::InternalError);
            }
            WaitingMessage::None => {
                println!("WARNING: Tried to wait on a message that didn't exist");
                return Err(xous_kernel::Error::ProcessNotFound);
            }
        };
        // println!(
        //     "KERNEL({}): Returning {} bytes from {:08x} in PID {} to {:08x} in PID {} in context {}",
        //     pid,
        //     len,
        //     _server_addr.get(),
        //     pid,
        //     client_addr.get(),
        //     client_pid,
        //     client_tid
        // );
        #[cfg(baremetal)]
        let src_virt = _server_addr.get() as _;
        #[cfg(not(baremetal))]
        let src_virt = buf.addr.get() as _;

        // Return the memory to the calling process
        ss.return_memory(
            src_virt,
            client_pid,
            client_tid,
            client_addr.get() as _,
            len.get(),
        )?;

        // Unblock the client context to allow it to continue.
        if !cfg!(baremetal) || in_irq {
            // Send a message to the client, in order to wake it up
            // print!(" [waking up PID {}:{}]", client_pid, client_tid);
            ss.ready_thread(client_pid, client_tid)?;
            ss.switch_to_thread(client_pid, Some(client_tid))?;
            ss.set_thread_result(
                client_pid,
                client_tid,
                xous_kernel::Result::MemoryReturned(offset, valid),
            )?;

            // Return success to the server
            Ok(xous_kernel::Result::Ok)
        } else {
            // Switch away from the server, but leave it as Runnable
            ss.switch_from_thread(server_pid, server_tid)?;
            ss.ready_thread(server_pid, server_tid)?;
            ss.set_thread_result(server_pid, server_tid, xous_kernel::Result::Ok)?;

            // Switch to the client
            ss.ready_thread(client_pid, client_tid)?;
            ss.switch_to_thread(client_pid, Some(client_tid))?;
            Ok(xous_kernel::Result::MemoryReturned(offset, valid))
        }
    })
}

fn return_scalar(
    server_pid: PID,
    server_tid: TID,
    in_irq: bool,
    sender: MessageSender,
    arg: usize,
) -> SysCallResult {
    SystemServices::with_mut(|ss| {
        let sender = SenderID::from(sender);

        let server = ss
            .server_from_sidx_mut(sender.sidx)
            .ok_or(xous_kernel::Error::ServerNotFound)?;
        if server.pid != server_pid {
            return Err(xous_kernel::Error::ServerNotFound);
        }
        let result = server.take_waiting_message(sender.idx, None)?;
        let (client_pid, client_tid) = match result {
            WaitingMessage::ScalarMessage(pid, tid) => (pid, tid),
            WaitingMessage::ForgetMemory(_) => {
                println!(
                    "WARNING: Tried to wait on a scalar message that was actually forgettingmemory"
                );
                return Err(xous_kernel::Error::ProcessNotFound);
            }
            WaitingMessage::BorrowedMemory(_, _, _, _, _) => {
                println!(
                    "WARNING: Tried to wait on a scalar message that was actually borrowed memory"
                );
                return Err(xous_kernel::Error::ProcessNotFound);
            }
            WaitingMessage::MovedMemory => {
                println!(
                    "WARNING: Tried to wait on a scalar message that was actually moved memory"
                );
                return Err(xous_kernel::Error::ProcessNotFound);
            }
            WaitingMessage::None => {
                println!("WARNING: Tried to wait on a message that didn't exist");
                return Err(xous_kernel::Error::ProcessNotFound);
            }
        };

        if !cfg!(baremetal) || in_irq {
            // In a hosted environment, `switch_to_thread()` doesn't continue
            // execution from the new thread. Instead it continues in the old
            // thread. Therefore, we need to instruct the client to resume, and
            // return to the server.
            // In a baremetal environment, the opposite is true -- we instruct
            // the server to resume and return to the client.
            ss.set_thread_result(client_pid, client_tid, xous_kernel::Result::Scalar1(arg))?;
            if in_irq {
                ss.ready_thread(client_pid, client_tid)?;
            }
            Ok(xous_kernel::Result::Ok)
        } else {
            // Switch away from the server, but leave it as Runnable
            ss.switch_from_thread(server_pid, server_tid)?;
            ss.ready_thread(server_pid, server_tid)?;
            ss.set_thread_result(server_pid, server_tid, xous_kernel::Result::Ok)?;

            // Switch to the client
            ss.ready_thread(client_pid, client_tid)?;
            ss.switch_to_thread(client_pid, Some(client_tid))?;
            Ok(xous_kernel::Result::Scalar1(arg))
        }
    })
}

fn return_scalar2(
    server_pid: PID,
    server_tid: TID,
    in_irq: bool,
    sender: MessageSender,
    arg1: usize,
    arg2: usize,
) -> SysCallResult {
    SystemServices::with_mut(|ss| {
        let sender = SenderID::from(sender);

        let server = ss
            .server_from_sidx_mut(sender.sidx)
            .ok_or(xous_kernel::Error::ServerNotFound)?;
        // .expect("Couldn't get server from SIDX");
        if server.pid != server_pid {
            return Err(xous_kernel::Error::ServerNotFound);
        }
        let result = server.take_waiting_message(sender.idx, None)?;
        let (client_pid, client_tid) = match result {
            WaitingMessage::ScalarMessage(pid, tid) => (pid, tid),
            WaitingMessage::ForgetMemory(_) => {
                println!("WARNING: Tried to wait on a scalar message that was actually forgetting memory");
                return Err(xous_kernel::Error::ProcessNotFound);
            }
            WaitingMessage::BorrowedMemory(_, _, _, _, _) => {
                println!(
                    "WARNING: Tried to wait on a scalar message that was actually borrowed memory"
                );
                return Err(xous_kernel::Error::ProcessNotFound);
            }
            WaitingMessage::MovedMemory => {
                println!(
                    "WARNING: Tried to wait on a scalar message that was actually moved memory"
                );
                return Err(xous_kernel::Error::ProcessNotFound);
            }
            WaitingMessage::None => {
                println!("WARNING: Tried to wait on a message that didn't exist");
                return Err(xous_kernel::Error::ProcessNotFound);
            }
        };

        if !cfg!(baremetal) || in_irq {
            // In a hosted environment, `switch_to_thread()` doesn't continue
            // execution from the new thread. Instead it continues in the old
            // thread. Therefore, we need to instruct the client to resume, and
            // return to the server.
            // In a baremetal environment, the opposite is true -- we instruct
            // the server to resume and return to the client.
            ss.set_thread_result(
                client_pid,
                client_tid,
                xous_kernel::Result::Scalar2(arg1, arg2),
            )?;
            if in_irq {
                ss.ready_thread(client_pid, client_tid)?;
            }
            Ok(xous_kernel::Result::Ok)
        } else {
            // Switch away from the server, but leave it as Runnable
            ss.switch_from_thread(server_pid, server_tid)?;
            ss.ready_thread(server_pid, server_tid)?;
            ss.set_thread_result(server_pid, server_tid, xous_kernel::Result::Ok)?;

            // Switch to the client
            ss.ready_thread(client_pid, client_tid)?;
            ss.switch_to_thread(client_pid, Some(client_tid))?;
            Ok(xous_kernel::Result::Scalar2(arg1, arg2))
        }
    })
}

fn receive_message(pid: PID, tid: TID, sid: SID, blocking: ExecutionType) -> SysCallResult {
    SystemServices::with_mut(|ss| {
        assert!(
            ss.thread_is_running(pid, tid),
            "current thread is not running"
        );
        // See if there is a pending message.  If so, return immediately.
        let sidx = ss
            .sidx_from_sid(sid, pid)
            .ok_or(xous_kernel::Error::ServerNotFound)?;
        let server = ss
            .server_from_sidx_mut(sidx)
            .ok_or(xous_kernel::Error::ServerNotFound)?;
        // server.print_queue();

        // Ensure the server is for this PID
        if server.pid != pid {
            return Err(xous_kernel::Error::ServerNotFound);
        }

        // If there is a pending message, return it immediately.
        if let Some(msg) = server.take_next_message(sidx) {
            klog!("waiting messages found -- returning {:x?}", msg);
            return Ok(xous_kernel::Result::Message(msg));
        }

        if blocking == ExecutionType::NonBlocking {
            klog!("nonblocking message -- returning None");
            return Ok(xous_kernel::Result::None);
        }

        // There is no pending message, so return control to the parent
        // process and mark ourselves as awaiting an event.  When a message
        // arrives, our return value will already be set to the
        // MessageEnvelope of the incoming message.
        klog!(
            "did not have any waiting messages -- parking thread {}",
            tid
        );
        server.park_thread(tid);

        // For baremetal targets, switch away from this process.
        if cfg!(baremetal) {
            unsafe { SWITCHTO_CALLER = None };
            let ppid = ss.get_process(pid).expect("Can't get current process").ppid;
            // TODO: Advance thread
            ss.activate_process_thread(tid, ppid, 0, false)
                .map(|_| Ok(xous_kernel::Result::ResumeProcess))
                .unwrap_or(Err(xous_kernel::Error::ProcessNotFound))
        }
        // For hosted targets, simply return `BlockedProcess` indicating we'll make
        // a callback to their socket at a later time.
        else {
            ss.switch_from_thread(pid, tid)
                .map(|_| xous_kernel::Result::BlockedProcess)
        }
    })
}

pub fn handle(pid: PID, tid: TID, in_irq: bool, call: SysCall) -> SysCallResult {
    #[cfg(feature = "debug-print")]
    print!("KERNEL({}:{}): Syscall {:x?}", pid, tid, call);

    let result = if in_irq && !call.can_call_from_interrupt() {
        Err(xous_kernel::Error::InvalidSyscall)
    } else {
        handle_inner(pid, tid, in_irq, call)
    };

    #[cfg(feature = "debug-print")]
    println!(
        " -> ({}:{}) {:x?}",
        crate::arch::current_pid(),
        crate::arch::process::Process::current().current_tid(),
        result
    );
    result
}

pub fn handle_inner(pid: PID, tid: TID, in_irq: bool, call: SysCall) -> SysCallResult {
    match call {
        SysCall::MapMemory(phys, virt, size, req_flags) => {
            MemoryManager::with_mut(|mm| {
                let phys_ptr = phys
                    .map(|x| x.get() as *mut u8)
                    .unwrap_or(core::ptr::null_mut());
                let virt_ptr = virt
                    .map(|x| x.get() as *mut u8)
                    .unwrap_or(core::ptr::null_mut());

                // Don't let the address exceed the user area (unless it's PID 1)
                if pid.get() != 1
                    && virt
                        .map(|x| x.get() >= arch::mem::USER_AREA_END)
                        .unwrap_or(false)
                {
                    println!("Exceeded user area");
                    return Err(xous_kernel::Error::BadAddress);

                // Don't allow mapping non-page values
                } else if size.get() & (PAGE_SIZE - 1) != 0 {
                    // println!("map: bad alignment of size {:08x}", size);
                    return Err(xous_kernel::Error::BadAlignment);
                }
                // println!(
                //     "Mapping {:08x} -> {:08x} ({} bytes, flags: {:?})",
                //     phys_ptr as u32, virt_ptr as u32, size, req_flags
                // );
                let range = mm.map_range(
                    phys_ptr,
                    virt_ptr,
                    size.get(),
                    pid,
                    req_flags,
                    MemoryType::Default,
                )?;

                // If we're handing back an address in main RAM, zero it out. If
                // phys is 0, then the page will be lazily allocated, so we
                // don't need to do this.
                if phys.is_some() {
                    if mm.is_main_memory(phys_ptr) {
                        println!(
                            "Going to zero out {} bytes @ {:08x}",
                            range.size.get(),
                            range.addr.get()
                        );
                        unsafe {
                            range
                                .as_mut_ptr()
                                .write_bytes(0, range.size.get() / mem::size_of::<usize>())
                        };
                        // println!("Done zeroing out");
                    }
                    for offset in
                        (range.addr.get()..(range.addr.get() + range.size.get())).step_by(PAGE_SIZE)
                    {
                        // println!("Handing page to user");
                        crate::arch::mem::hand_page_to_user(offset as *mut u8)
                            .expect("couldn't hand page to user");
                    }
                }

                Ok(xous_kernel::Result::MemoryRange(range))
            })
        }
        SysCall::UnmapMemory(range) => MemoryManager::with_mut(|mm| {
            let mut result = Ok(xous_kernel::Result::Ok);
            let virt = range.as_ptr() as usize;
            let size = range.len();
            if cfg!(baremetal) {
                if virt & 0xfff != 0 {
                    return Err(xous_kernel::Error::BadAlignment);
                }
            }
            for addr in (virt..(virt + size)).step_by(PAGE_SIZE) {
                if let Err(e) = mm.unmap_page(addr as *mut usize) {
                    if result.is_ok() {
                        result = Err(e);
                    }
                }
            }
            result
        }),
        SysCall::IncreaseHeap(delta, flags) => {
            if delta & 0xfff != 0 {
                return Err(xous_kernel::Error::BadAlignment);
            }
            let start = {
                ArchProcess::with_inner_mut(|process_inner| {
                    if process_inner.mem_heap_size + delta > process_inner.mem_heap_max {
                        return Err(xous_kernel::Error::OutOfMemory);
                    }

                    let start = process_inner.mem_heap_base + process_inner.mem_heap_size;
                    process_inner.mem_heap_size += delta;
                    Ok(start as *mut u8)
                })?
            };
            MemoryManager::with_mut(|mm| {
                Ok(xous_kernel::Result::MemoryRange(
                    mm.reserve_range(start, delta, flags)?,
                ))
            })
        }
        SysCall::DecreaseHeap(delta) => {
            if delta & 0xfff != 0 {
                return Err(xous_kernel::Error::BadAlignment);
            }
            let start = ArchProcess::with_inner_mut(|process_inner| {
                if process_inner.mem_heap_size + delta > process_inner.mem_heap_max {
                    return Err(xous_kernel::Error::OutOfMemory);
                }

                let start = process_inner.mem_heap_base + process_inner.mem_heap_size;
                process_inner.mem_heap_size -= delta;
                Ok(start)
            })?;
            MemoryManager::with_mut(|mm| {
                for page in ((start - delta)..start).step_by(crate::arch::mem::PAGE_SIZE) {
                    mm.unmap_page(page as *mut usize)
                        .expect("unable to unmap page");
                }
            });
            Ok(xous_kernel::Result::Ok)
        }
        SysCall::SwitchTo(new_pid, new_context) => SystemServices::with_mut(|ss| {
            unsafe {
                assert!(
                    SWITCHTO_CALLER.is_none(),
                    "SWITCHTO_CALLER was {:?} and not None, indicating SwitchTo was called twice",
                    SWITCHTO_CALLER,
                );
                SWITCHTO_CALLER = Some((pid, tid));
            }
            // println!(
            //     "Activating process thread {} in pid {} coming from pid {} thread {}",
            //     new_context, new_pid, pid, tid
            // );
            let result = ss
                .activate_process_thread(tid, new_pid, new_context, true)
                .map(|_ctx| xous_kernel::Result::ResumeProcess);
            // println!("Done activating process thread: {:?}", result);
            result
        }),
        SysCall::ClaimInterrupt(no, callback, arg) => {
            interrupt_claim(no, pid as definitions::PID, callback, arg)
                .map(|_| xous_kernel::Result::Ok)
        }
        SysCall::Yield => do_yield(pid, tid),
        SysCall::ReturnToParent(_pid, _cpuid) => {
            unsafe {
                SWITCHTO_CALLER.take().map(|(parent_pid, parent_ctx)| {
                    crate::arch::irq::set_isr_return_pair(parent_pid, parent_ctx)
                });
            };
            Ok(xous_kernel::Result::ResumeProcess)
        }
        SysCall::ReceiveMessage(sid) => receive_message(pid, tid, sid, ExecutionType::Blocking),
        SysCall::TryReceiveMessage(sid) => {
            receive_message(pid, tid, sid, ExecutionType::NonBlocking)
        }
        SysCall::WaitEvent => SystemServices::with_mut(|ss| {
            let process = ss.get_process(pid).expect("Can't get current process");
            let ppid = process.ppid;
            unsafe { SWITCHTO_CALLER = None };
            // TODO: Advance thread
            if cfg!(baremetal) {
                ss.activate_process_thread(tid, ppid, 0, false)
                    .map(|_| Ok(xous_kernel::Result::ResumeProcess))
                    .unwrap_or(Err(xous_kernel::Error::ProcessNotFound))
            } else {
                Ok(xous_kernel::Result::Ok)
            }
        }),
        SysCall::CreateThread(thread_init) => SystemServices::with_mut(|ss| {
            ss.create_thread(pid, thread_init).map(|new_tid| {
                if !cfg!(baremetal) {
                    ss.switch_to_thread(pid, Some(new_tid))
                        .expect("couldn't activate new thread");
                }
                xous_kernel::Result::ThreadID(new_tid)
            })
        }),
        SysCall::CreateProcess(process_init) => SystemServices::with_mut(|ss| {
            ss.create_process(process_init)
                .map(xous_kernel::Result::ProcessID)
        }),
        SysCall::CreateServerWithAddress(name) => SystemServices::with_mut(|ss| {
            ss.create_server_with_address(pid, name)
                .map(|(sid, cid)| xous_kernel::Result::NewServerID(sid, cid))
        }),
        SysCall::CreateServer => SystemServices::with_mut(|ss| {
            ss.create_server(pid)
                .map(|(sid, cid)| xous_kernel::Result::NewServerID(sid, cid))
        }),
        SysCall::CreateServerId => SystemServices::with_mut(|ss| {
            ss.create_server_id()
                .map(|sid| xous_kernel::Result::ServerID(sid))
        }),
        SysCall::TryConnect(sid) => SystemServices::with_mut(|ss| {
            ss.connect_to_server(sid)
                .map(xous_kernel::Result::ConnectionID)
        }),
        SysCall::ReturnMemory(sender, buf, offset, valid) => {
            return_memory(pid, tid, in_irq, sender, buf, offset, valid)
        }
        SysCall::ReturnScalar1(sender, arg) => return_scalar(pid, tid, in_irq, sender, arg),
        SysCall::ReturnScalar2(sender, arg1, arg2) => {
            return_scalar2(pid, tid, in_irq, sender, arg1, arg2)
        }
        SysCall::TrySendMessage(cid, message) => send_message(pid, tid, cid, message),
        SysCall::TerminateProcess(_ret) => SystemServices::with_mut(|ss| {
            ss.switch_from_thread(pid, tid)?;
            ss.terminate_process(pid)?;
            // Clear out `SWITCHTO_CALLER` since we're resuming the parent process.
            unsafe { SWITCHTO_CALLER = None };
            Ok(xous_kernel::Result::ResumeProcess)
        }),
        SysCall::Shutdown => {
            SystemServices::with_mut(|ss| ss.shutdown().map(|_| xous_kernel::Result::Ok))
        }
        SysCall::GetProcessId => Ok(xous_kernel::Result::ProcessID(pid)),
        SysCall::GetThreadId => Ok(xous_kernel::Result::ThreadID(tid)),

        SysCall::Connect(sid) => {
            let result = SystemServices::with_mut(|ss| {
                ss.connect_to_server(sid)
                    .map(xous_kernel::Result::ConnectionID)
            });
            match result {
                Ok(o) => Ok(o),
                Err(xous_kernel::Error::ServerNotFound) => retry_syscall(pid, tid),
                Err(e) => Err(e),
            }
        }
        SysCall::ConnectForProcess(pid, sid) => {
            let result = SystemServices::with_mut(|ss| {
                ss.connect_process_to_server(pid, sid)
                    .map(xous_kernel::Result::ConnectionID)
            });
            match result {
                Ok(o) => Ok(o),
                Err(xous_kernel::Error::ServerNotFound) => retry_syscall(pid, tid),
                Err(e) => Err(e),
            }
        }
        SysCall::SendMessage(cid, message) => {
            let result = send_message(pid, tid, cid, message);
            match result {
                Ok(o) => Ok(o),
                Err(xous_kernel::Error::ServerQueueFull) => retry_syscall(pid, tid),
                Err(e) => Err(e),
            }
        }
        SysCall::Disconnect(cid) => SystemServices::with_mut(|ss| {
            ss.disconnect_from_server(cid)
                .and(Ok(xous_kernel::Result::Ok))
        }),
        SysCall::DestroyServer(sid) => SystemServices::with_mut(|ss| {
            ss.destroy_server(pid, sid).and(Ok(xous_kernel::Result::Ok))
        }),
        SysCall::JoinThread(other_tid) => {
            SystemServices::with_mut(|ss| ss.join_thread(pid, tid, other_tid)).map(|ret| {
                unsafe { SWITCHTO_CALLER = None };
                ret
            })
        }
        _ => Err(xous_kernel::Error::UnhandledSyscall),
    }
}
