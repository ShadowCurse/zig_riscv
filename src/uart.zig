const std = @import("std");

const Uart = struct {
    const UART8250_TX_REG_ADDR: u32 = 0x10000000;

    const REGS_SIZE: u32 = 16;
    const FIFO_SIZE: u32 = 16;

    const REG_RX_TX_DIV_LATCH_LO: u32 = 0;
    const REG_IER_LATCH_HI: u32 = 1;
    // READ
    const REG_IIR: u32 = 2;
    // WRITE
    const REG_FCR: u32 = 2;
    // READ/WRITE
    const REG_LCR: u32 = 3;
    // READ/WRITE
    const REG_MCR: u32 = 4;
    // READ
    const REG_LSR: u32 = 5;
    // READ
    const REG_MSR: u32 = 6;
    // READ/WRITE
    const REG_SCRATCH: u32 = 7;

    mutex: std.Thread.Mutex,
    rx_fifo: std.fifo.LinearFifo(u8, std.fifo.LinearFifoBufferType.Static(FIFO_SIZE)),
    tx_fifo: std.fifo.LinearFifo(u8, std.fifo.LinearFifoBufferType.Static(FIFO_SIZE)),
    regs: [REGS_SIZE]u8,

    ux_therad: std.Thread,

    const Self = @This();

    fn deinit(self: *Self) void {
        self.ux_thread.join();
    }

    fn create_ux_thread(self: *Self) !void {
        self.ux_therad = try std.Thread.spawn(.{}, Self.ux_thread, .{self});
    }

    fn ux_thread(self: *Self) void {
        _ = self;
    }

    fn bus_write(self: *Self, addr: u32, value: u32) void {
        self.mutex.lock();
        defer self.mutex.unlock();

        const byte = @truncate(u8, value);
        switch (addr) {
            REG_RX_TX_DIV_LATCH_LO => {
                if (!self.dlab) {
                    try self.tx_fifo.write(byte);

                    if ((!self.fifo_enabled) or (byte == '\n')) {
                        self.tx_needs_flush = 1;
                    }

                    if (self.regs[REG_IIR] == 2) {
                        self.tx_stop_triggering = 0;
                    }
                } else {
                    // No DLAB support
                    std.log.info("DLAB write access: addr: {x}, value: {x}", .{ addr, value });
                }
            },
            REG_IER_LATCH_HI => {
                if (!self.dlab) {
                    self.irq_enabled_rx_data_available = value & 1 << 0;
                    self.irq_enabled_tx_holding_reg_empty = value & 1 << 1;
                    self.irq_enabled_rlsr_change = value & 1 << 2;
                    self.irq_enabled_msr_change = value & 1 << 3;
                    self.irq_enabled_sleep = value & 1 << 4;
                    self.irq_enabled_low_power = value & 1 << 5;

                    self.regs[REG_IER_LATCH_HI] = value;
                } else {
                    // No DLAB support
                    std.log.info("DLAB write access: addr: {x}, value: {x}", .{ addr, value });
                }
            },
            REG_FCR => {
                // DMA mode
                if (byte & 1 << 3) {
                    std.log.info("DMA mode not supported");
                    std.process.exit(1);
                }

                // Clear RX fifo
                if (byte & 1 << 1) {
                    self.rx_fifo.discard(self.rx_fifo.readableLength());
                }

                // Clear TX fifo
                if (byte & 1 << 2) {
                    self.tx_fifo.discard(self.tx_fifo.readableLength());
                }

                self.fifo_enabled = byte & 1;

                self.rx_irq_fifo_level = switch (@truncate(u2, ((byte & (3 << 6)) >> 6))) {
                    0b00 => {
                        1;
                    },
                    0b01 => {
                        4;
                    },
                    0b10 => {
                        8;
                    },
                    0b11 => {
                        14;
                    },
                };
            },
            REG_LCR => {
                self.regs[REG_LCR] = byte;
                self.dlab = byte & 1 < 7;

                if (self.dlab) {
                    std.log.info("DLAB activated", .{});
                } else {
                    std.log.info("DLAB dectivated", .{});
                }
            },
            REG_MCR => {
                self.regs[REG_MCR] = byte;
            },
            else => {
                std.log.info("UART: Attempt to write to addr: {x} value: {x}. Not supported", .{ addr, value });
            },
        }
    }

    fn bus_read(self: *Self, addr: u32) !u32 {
        switch (addr) {
            REG_RX_TX_DIV_LATCH_LO => {
                if (!self.dlab) {
                    var byte: u8 = undefined;
                    try self.rx_fifo.read(&byte);
                    return byte;
                } else {
                    // No DLAB support
                    std.log.info("DLAB read access: addr: {x}", .{addr});
                    return 0;
                }
            },
            REG_IER_LATCH_HI => {
                if (!self.dlab) {
                    return ((self.irq_enabled_rx_data_available << 0) |
                        (self.irq_enabled_tx_holding_reg_empty << 1) |
                        (self.irq_enabled_rlsr_change << 2) |
                        (self.irq_enabled_msr_change << 3) |
                        (self.irq_enabled_sleep << 4) |
                        (self.irq_enabled_low_power << 5));
                } else {
                    // No DLAB support
                    std.log.info("DLAB read access: addr: {x}", .{addr});
                    return 0;
                }
            },
            REG_IIR => {
                // self.wait_for_iir_read = 0;
                if (self.regs[REG_IIR] == 2) {
                    self.curr_iir_id = 1;
                    self.tx_stop_triggering = 1;
                }
                // 1 means no interrupt pending
                return self.regs[REG_IIR];
            },
            REG_LSR => {
                {
                    // THR empty and line idle is always true here in our emulation
                    const data_avail = @boolToInt(self.rx_fifo.readableLength() != 0);
                    const overrun_err = 0;
                    const parity_err = 0;
                    const framing_err = 0;
                    const brk_sig = 0;
                    const thr_empty = @boolToInt(self.tx_fifo.readableLength() != 0);
                    const thr_empty_and_idle = thr_empty;
                    const err_data_fifo = 0;

                    if (self.lsr_change) {
                        self.lsr_change = 0;
                    }

                    return (data_avail << 0 |
                        overrun_err << 1 |
                        parity_err << 2 |
                        framing_err << 3 |
                        brk_sig << 4 |
                        thr_empty << 5 |
                        thr_empty_and_idle << 6 |
                        err_data_fifo << 7);
                }
            },
            REG_LCR => {
                return self.regs[REG_LCR];
            },
            REG_MSR => {
                // Not supported currently
                return 0xb0;
            },
            REG_MCR => {
                // Not supported currently
                return 0x8;
            },
            else => {
                std.log.info("UART: Attempt to read from addr: {x}. Not supported", .{addr});
            },
        }
    }
};
