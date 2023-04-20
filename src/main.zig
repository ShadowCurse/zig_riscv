const std = @import("std");

const Encodings = @import("encodings.zig");

const Ram = struct {
    base_addr: u32,
    mem: [1024 * 8]u8,

    const Self = @This();

    fn read(self: *Self, comptime t: type, addr: u32) t {
        const index = addr - self.base_addr;
        switch (t) {
            u32 => {
                return @as(u32, self.mem[index]) |
                    @as(u32, self.mem[index + 1]) << 8 |
                    @as(u32, self.mem[index + 2]) << 16 |
                    @as(u32, self.mem[index + 3]) << 24;
            },
            u16 => {
                return self.mem[index] | self.mem[index + 1] << 8;
            },
            u8 => {
                return self.mem[index];
            },
            else => unreachable,
        }
    }

    fn write(self: *Self, comptime t: type, addr: u32, value: t) void {
        const index = addr - self.base_addr;
        switch (t) {
            u32 => {
                self.mem[index] = value & 0xff;
                self.mem[index + 1] = (value >> 8) & 0xff;
                self.mem[index + 2] = (value >> 16) & 0xff;
                self.mem[index + 3] = (value >> 24) & 0xff;
            },
            u16 => {
                self.mem[index] = value & 0xff;
                self.mem[index + 1] = (value >> 8) & 0xff;
            },
            u8 => {
                self.mem[index] = value;
            },
            else => unreachable,
        }
    }
};

const Cpu = struct {
    regs: [32]u32,
    pc: u32,
    ram: Ram,

    const Self = @This();

    fn fetch_next_intruction(self: *Self) u32 {
        return self.ram.read(u32, self.pc);
    }

    fn run_instruction(self: *Self) void {
        const instruction = self.fetch_next_intruction();
        self.pc += 4;

        var opcode = instruction & 0b1111111;
        std.log.info("opcode: {b}", .{opcode});
        switch (opcode) {
            // RV32I Base Instruction Set
            //LUI
            0b0110111 => {
                const u_type = @bitCast(Encodings.UType, instruction);
                std.log.info("LUI: {any}", .{u_type});
                self.regs[u_type.rd] = @intCast(u32, u_type.get_imm());
            },
            //AUIPC
            0b0010111 => {},
            //JAL
            0b1101111 => {
                const j_type = @bitCast(Encodings.JType, instruction);
                std.log.info("JAL: {any}", .{j_type});
                std.log.info("JAL: {}", .{j_type.get_imm()});
                self.regs[j_type.rd] = self.pc + 4;
                self.pc = @intCast(u32, @intCast(i32, self.pc) +% j_type.get_imm());
            },
            //JALR
            0b1100111 => {
                const i_type = @bitCast(Encodings.IType, instruction);
                std.log.info("JALR: {any}", .{i_type});
                self.regs[i_type.rd] = self.pc + 4;
                self.pc = @intCast(u32, @intCast(i32, self.regs[i_type.rs1]) +% i_type.get_imm()) & 0xfffffffe;
            },
            0b1100011 => {
                const b_type = @bitCast(Encodings.BType, instruction);
                const a = self.regs[b_type.rs1];
                const b = self.regs[b_type.rs2];
                switch (b_type.func3) {
                    //BEQ
                    0b000 => {
                        std.log.info("BEQ: {any}", .{b_type});
                        if (a == b) {
                            self.pc = self.pc +% @intCast(u32, b_type.get_imm());
                        }
                    },
                    //BNE
                    0b001 => {
                        std.log.info("BNE: {any}", .{b_type});
                        if (a != b) {
                            self.pc = self.pc +% @intCast(u32, b_type.get_imm());
                        }
                    },
                    //BLT
                    0b100 => {
                        std.log.info("BLT: {any}", .{b_type});
                        if (@intCast(i32, a) < @intCast(i32, b)) {
                            self.pc = self.pc +% @intCast(u32, b_type.get_imm());
                        }
                    },
                    //BGE
                    0b101 => {
                        std.log.info("BGE: {any}", .{b_type});
                        if (@intCast(i32, a) >= @intCast(i32, b)) {
                            self.pc = self.pc +% @intCast(u32, b_type.get_imm());
                        }
                    },
                    //BLTU
                    0b110 => {
                        std.log.info("BLTU: {any}", .{b_type});
                        if (a < b) {
                            self.pc = self.pc +% @intCast(u32, b_type.get_imm());
                        }
                    },
                    //BGEU
                    0b111 => {
                        std.log.info("BGEU: {any}", .{b_type});
                        if (a >= b) {
                            self.pc = self.pc +% @intCast(u32, b_type.get_imm());
                        }
                    },
                    else => unreachable,
                }
            },

            0b0000011 => {
                const i_type = @bitCast(Encodings.IType, instruction);
                switch (i_type.func3) {
                    //LB
                    0b000 => {},
                    //LH
                    0b001 => {},
                    //LW
                    0b010 => {},
                    //LBU
                    0b100 => {},
                    //LHU
                    0b101 => {},
                    else => unreachable,
                }
            },
            0b0100011 => {
                const s_type = @bitCast(Encodings.SType, instruction);
                switch (s_type.func3) {
                    //SB
                    0b000 => {},
                    //SH
                    0b001 => {},
                    //SW
                    0b010 => {},
                    else => unreachable,
                }
            },
            0b0010011 => {
                const i_type = @bitCast(Encodings.IType, instruction);
                switch (i_type.func3) {
                    //ADDI
                    0b000 => {
                        std.log.info("ADDI: {any}", .{i_type});
                        // Also NOP
                        // NOP = ADDI x0, x0, 0.
                        self.regs[i_type.rd] = @bitCast(u32, @intCast(i32, self.regs[i_type.rs1]) +% i_type.get_imm());
                    },
                    //SLTI
                    0b010 => {
                        std.log.info("SLTI: {any}", .{i_type});
                        if (@intCast(i32, self.regs[i_type.rs1]) < i_type.get_imm()) {
                            self.regs[i_type.rd] = 1;
                        } else {
                            self.regs[i_type.rd] = 0;
                        }
                    },
                    //SLTIU
                    0b011 => {
                        std.log.info("SLTIU: {any}", .{i_type});
                        if (@as(u32, self.regs[i_type.rs1]) < @intCast(u32, i_type.get_imm())) {
                            self.regs[i_type.rd] = 1;
                        } else {
                            self.regs[i_type.rd] = 0;
                        }
                    },
                    //XORI
                    0b100 => {
                        std.log.info("XORI: {any}", .{i_type});
                        self.regs[i_type.rd] = self.regs[i_type.rs1] ^ @intCast(u32, i_type.get_imm());
                    },
                    //ORI
                    0b110 => {
                        std.log.info("ORI: {any}", .{i_type});
                        self.regs[i_type.rd] = self.regs[i_type.rs1] | @intCast(u32, i_type.get_imm());
                    },
                    //ANDI
                    0b111 => {
                        std.log.info("ANDI: {any}", .{i_type});
                        self.regs[i_type.rd] = self.regs[i_type.rs1] & @intCast(u32, i_type.get_imm());
                    },
                    //SLLI
                    0b001 => {
                        std.log.info("SLLI: {any}", .{i_type});
                        const shift = i_type.get_imm() & 0b11111;
                        self.regs[i_type.rd] = std.math.shl(u32, self.regs[i_type.rs1], shift);
                    },
                    0b101 => {
                        const r_type = @bitCast(Encodings.RType, instruction);
                        switch (r_type.func7) {
                            //SRLI
                            0b0000000 => {
                                std.log.info("SRLI: {any}", .{i_type});
                                const shift = i_type.get_imm() & 0b11111;
                                self.regs[i_type.rd] = std.math.shr(u32, self.regs[i_type.rs1], shift);
                            },
                            //SRAI
                            0b0100000 => {
                                std.log.info("SRAI: {any}", .{i_type});
                                const shift = i_type.get_imm() & 0b11111;
                                self.regs[i_type.rd] = std.math.shr(u32, self.regs[i_type.rs1], shift);
                            },
                            else => unreachable,
                        }
                    },
                }
            },
            0b0110011 => {
                const r_type = @bitCast(Encodings.RType, instruction);
                switch (r_type.func7) {
                    // Base set
                    0b0000000 => {
                        switch (r_type.func3) {
                            //ADD
                            0b000 => {
                                std.log.info("ADD: {any}", .{r_type});
                                self.regs[r_type.rd] = self.regs[r_type.rs1] +% self.regs[r_type.rs2];
                            },
                            //SLL
                            0b001 => {
                                std.log.info("SLL: {any}", .{r_type});
                                const shift = self.regs[r_type.rs2] & 0b11111;
                                self.regs[r_type.rd] = std.math.shl(u32, self.regs[r_type.rs1], shift);
                            },
                            //SLT
                            0b010 => {
                                std.log.info("SLT: {any}", .{r_type});
                                if (@intCast(i32, self.regs[r_type.rs1]) < @intCast(i32, self.regs[r_type.rs2])) {
                                    self.regs[r_type.rd] = 1;
                                } else {
                                    self.regs[r_type.rd] = 0;
                                }
                            },
                            //SLTU
                            0b011 => {
                                std.log.info("SLTU: {any}", .{r_type});
                                if (self.regs[r_type.rs1] < self.regs[r_type.rs2]) {
                                    self.regs[r_type.rd] = 1;
                                } else {
                                    self.regs[r_type.rd] = 0;
                                }
                            },
                            //XOR
                            0b100 => {
                                std.log.info("XOR: {any}", .{r_type});
                                self.regs[r_type.rd] = self.regs[r_type.rs1] ^ self.regs[r_type.rs2];
                            },
                            //OR
                            0b110 => {
                                std.log.info("OR: {any}", .{r_type});
                                self.regs[r_type.rd] = self.regs[r_type.rs1] | self.regs[r_type.rs2];
                            },
                            //AND
                            0b111 => {
                                std.log.info("AND: {any}", .{r_type});
                                self.regs[r_type.rd] = self.regs[r_type.rs1] & self.regs[r_type.rs2];
                            },
                            //SRL
                            0b101 => {
                                std.log.info("SRL: {any}", .{r_type});
                                const shift = self.regs[r_type.rs2] & 0b11111;
                                self.regs[r_type.rd] = std.math.shr(u32, self.regs[r_type.rs1], shift);
                            },
                        }
                    },
                    0b0100000 => {
                        switch (r_type.func3) {
                            // SUB
                            0b000 => {
                                std.log.info("SUB: {any}", .{r_type});
                                const op = @subWithOverflow(self.regs[r_type.rs1], self.regs[r_type.rs2]);
                                self.regs[r_type.rd] = op.@"0";
                            },
                            // SRA
                            0b101 => {
                                std.log.info("SRA: {any}", .{r_type});
                                self.regs[r_type.rd] = self.regs[r_type.rs1] >> r_type.rs2 & 0b11111;
                            },
                            else => unreachable,
                        }
                    },
                    // RV32M Standard Extension
                    0b0000001 => {
                        switch (r_type.func3) {
                            //MUL
                            0b000 => {},
                            //MULH
                            0b001 => {},
                            //MULHSU
                            0b010 => {},
                            //MULHU
                            0b011 => {},
                            //DIV
                            0b100 => {},
                            //DIVU
                            0b101 => {},
                            //REM
                            0b110 => {},
                            //REMU
                            0b111 => {},
                        }
                    },
                    else => unreachable,
                }
            },
            //FENCE
            0b0001111 => {},
            0b1110011 => {
                const i_type = @bitCast(Encodings.IType, instruction);
                switch (i_type.get_imm()) {
                    //ECALL
                    0b000000000000 => {},
                    //EBREAK
                    0b000000000001 => {},
                    else => unreachable,
                }
            },
            else => unreachable,
        }
    }
};

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    if (args.len != 2) {
        std.log.info("No file provided", .{});
        std.process.exit(1);
    }

    const file = try std.fs.cwd().openFile(args[1], .{});
    defer file.close();

    var code = try file.reader().readAllAlloc(allocator, 99999);
    defer allocator.free(code);

    var ram: Ram = .{ .base_addr = 0x110b4, .mem = undefined };
    std.mem.copy(u8, &ram.mem, code);

    var cpu = Cpu{
        .regs = undefined,
        .pc = 0x110bc,
        .ram = ram,
    };
    std.mem.set(u32, &cpu.regs, 0);

    for (0..5) |_| {
        const intsruction = cpu.fetch_next_intruction();
        std.log.info("{x}", .{intsruction});
        cpu.run_instruction();
    }
}
