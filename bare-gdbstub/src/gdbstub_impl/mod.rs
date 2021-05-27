use crate::protocol::recv_packet::RecvPacketStateMachine;
use managed::ManagedSlice;

/// Describes why the GDB session ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DisconnectReason {
    /// Target exited with given status code
    TargetExited(u8),
    /// Target terminated with given signal
    TargetTerminated(u8),
    /// GDB issued a disconnect command
    Disconnect,
    /// GDB issued a kill command
    Kill,
    /// GDB should wait for a `Resume` command
    WaitForResume,
}

/// A variant of [`GdbStub`] which parses incoming packets using an asynchronous
/// state machine.
///
/// TODO: more docs
pub struct GdbStubStateMachine<'a> {
    packet_buffer: ManagedSlice<'a, u8>,
    recv_packet: RecvPacketStateMachine,
}

impl<'a> GdbStubStateMachine<'a> {
    /// Pass a byte to the `gdbstub` packet parser.
    ///
    /// Returns a `Some(DisconnectReason)` if the GDB client
    pub fn pump(&mut self, byte: u8) -> Option<DisconnectReason> {
        self.recv_packet.pump(&mut self.packet_buffer, byte).ok();

        None
    }
}
