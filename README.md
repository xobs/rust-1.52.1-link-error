# Demonstration of lld link error

Build the target using Rust 1.52.1 (stable) using the command:

```
rustup target add riscv32imac-unknown-none-elf
cargo build --target riscv32imac-unknown-none-elf
```

This will produce an error about "note: rust-lld: error: undefined symbol: __atomic_load_4"

Then, do one of the following:

* Comment out the syscall in `src/main.rs`
* Remove the rustflags from `.cargo/config`
* Build with `riscv32i-unknown-none-elf`
* Build with a previous version of Rust
* Modify `Cargo.toml` to use the `bare-gdbstub` repo instead of `gdbstub`

Any of these will resolve the error.
