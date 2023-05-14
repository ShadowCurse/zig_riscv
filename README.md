# Simple RISC-V emulator in Zig

Emulator implements riscv32imac instruction set.

## Bluild

```zig
$ zig build
```      

## Build example
Example program is a simple gcd algorithm.
You will need riscv toolchain to compile it.
```bash
$ riscv64-unknown-linux-gnu-as -march=rv32im -misa-spec=2.2 ./gcd.s -o gcd
$ riscv64-unknown-linux-gnu-objcopy -O binary -j .text gcd gcd_a
```

## Run example
To run binary provide path and base address
```zig
$ zig build run -- example/gcd_a 0
```
