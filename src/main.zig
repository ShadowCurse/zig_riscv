const std = @import("std");

const Encodings = @import("encodings.zig");

const Ram = struct {
    base_addr: u32,
    mem: [1024 * 8]u8,

    const Self = @This();

    fn read(self: *Self, comptime t: type, addr: u32) t {
        std.log.info("mem read addr: {x}", .{addr});
        const index = addr - self.base_addr;
        switch (t) {
            u32 => {
                return @as(u32, self.mem[index]) |
                    @as(u32, self.mem[index + 1]) << 8 |
                    @as(u32, self.mem[index + 2]) << 16 |
                    @as(u32, self.mem[index + 3]) << 24;
            },
            u16 => {
                return @as(u16, self.mem[index]) | @as(u16, self.mem[index + 1]) << 8;
            },
            u8 => {
                return self.mem[index];
            },
            else => unreachable,
        }
    }

    fn write(self: *Self, comptime t: type, addr: u32, value: t) void {
        std.log.info("mem write addr: {x}", .{addr});
        const index = addr - self.base_addr;
        switch (t) {
            u32 => {
                self.mem[index] = @truncate(u8, value);
                self.mem[index + 1] = @truncate(u8, (value >> 8));
                self.mem[index + 2] = @truncate(u8, (value >> 16));
                self.mem[index + 3] = @truncate(u8, (value >> 24));
            },
            u16 => {
                self.mem[index] = @truncate(u8, value & 0xff);
                self.mem[index + 1] = @truncate(u8, (value >> 8) & 0xff);
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

    fn print_regs(self: *Self) void {
        for (self.regs, 0..) |reg, i| {
            std.log.info("reg[{}]: u32({}), i32({})", .{ i, reg, @bitCast(i32, reg) });
        }
        std.log.info("pc: {x}", .{self.pc});
    }

    fn fetch_next_intruction(self: *Self) u32 {
        return self.ram.read(u32, self.pc);
    }

    fn run_instruction(self: *Self) bool {
        const instruction = self.fetch_next_intruction();
        const stop = switch (instruction & 0b11) {
            0b00 => self.run_compressed_00(instruction),
            0b01 => self.run_compressed_01(instruction),
            0b10 => self.run_compressed_10(instruction),
            0b11 => self.run_full(instruction),
            else => unreachable(),
        };
        return stop;
    }

    fn run_compressed_00(self: *Self, instruction: u32) bool {
        std.log.info("run_compressed_00", .{});
        switch ((instruction & 0b1110000000000000) >> 13) {
            // C.ADDI4SPN or ILLEGAL
            0b000 => {
                const ciw_type = @bitCast(Encodings.CIWType, @truncate(u16, instruction));
                const p_5_4 = @as(u9, ciw_type.imm & 0b11000000) >> 2;
                const p_9_6 = @as(u9, ciw_type.imm & 0b00111100) << 4;
                const p_2 = @as(u9, ciw_type.imm & 0b00000010) << 1;
                const p_3 = @as(u9, ciw_type.imm & 0b00000001) << 3;
                const imm = @as(u32, p_9_6 | p_5_4 | p_3 | p_2);
                std.log.info("ADDI4SPN: {any}", .{ciw_type});
                self.regs[ciw_type.rd] = self.regs[2] + imm;
            },
            // C.FLD (RV32/RV64) only for D extension
            0b001 => unreachable(),
            // C.LW
            0b010 => {
                const cl_type = @bitCast(Encodings.CLType, @truncate(u16, instruction));
                const p_2 = @as(u9, cl_type.imm1 & 0b10) << 1;
                const p_6 = @as(u9, cl_type.imm1 & 0b01) << 6;
                const p_5_3 = @as(u9, cl_type.imm2) << 3;
                const imm = @as(u32, p_2 | p_5_3 | p_6);
                std.log.info("LW: {any}", .{cl_type});
                self.regs[cl_type.rd] = self.ram.read(u32, self.regs[cl_type.rs1] + imm);
            },
            // C.FLW (RV32)
            0b011 => unreachable(),
            // Reserved
            0b100 => {},
            // C.FSD (RV32/64)
            0b101 => unreachable(),
            // C.SW
            0b110 => {
                const cs_type = @bitCast(Encodings.CSType, @truncate(u16, instruction));
                const p_2 = @as(u9, cs_type.imm1 & 0b10) << 1;
                const p_6 = @as(u9, cs_type.imm1 & 0b01) << 6;
                const p_5_3 = @as(u9, cs_type.imm2) << 3;
                const imm = @as(u32, p_2 | p_5_3 | p_6);
                std.log.info("SW: {any}", .{cs_type});
                self.ram.write(u32, self.regs[cs_type.rs1] + imm, self.regs[cs_type.rs2]);
            },
            // C.FSW (RV32)
            0b111 => unreachable(),
            else => unreachable(),
        }
        return true;
    }
    fn run_compressed_01(self: *Self, original_instruction: u32) bool {
        const instruction = @truncate(u16, original_instruction);
        std.log.info("run_compressed_01: {x}", .{instruction});
        switch ((instruction & 0b1110000000000000) >> 13) {
            // C.NOP (HINT, nzimm != 0)
            // C.ADDI (HINT, nzimm = 0)
            0b000 => {
                const ci_type = @bitCast(Encodings.CIType, instruction);
                const imm = @as(i64, @as(i6, ci_type.imm1) | @as(i6, ci_type.imm2) << 5);
                std.log.info("C.ADDI: {any}", .{ci_type});
                self.regs[ci_type.rd] = @intCast(u32, @as(i64, self.regs[ci_type.rd]) + imm);
            },
            // C.JAL (RV32)
            0b001 => {
                const cj_type = @bitCast(Encodings.CJType, instruction);

                // offset[11|4|9:8|10|6|7|3:1|5]
                const sign = @as(u16, instruction & 0b1000000000000);
                const p_11 = @as(u16, cj_type.imm & 0b10000000000) << 1;
                const p_4 = @as(u16, cj_type.imm & 0b01000000000) >> 5;
                const p_9_8 = @as(u16, cj_type.imm & 0b00110000000) << 1;
                const p_10 = @as(u16, cj_type.imm & 0b00001000000) << 4;
                const p_6 = @as(u16, cj_type.imm & 0b00000100000) << 1;
                const p_7 = @as(u16, cj_type.imm & 0b00000010000) << 3;
                const p_3_1 = @as(u16, cj_type.imm & 0b00000001110);
                const p_5 = @as(u16, cj_type.imm & 0b00000000001) << 5;
                const imm = @as(i64, @bitCast(i16, sign | p_11 | p_10 | p_9_8 | p_7 | p_6 | p_5 | p_4 | p_3_1));

                std.log.info("C.JAL: {any}", .{cj_type});
                self.regs[1] = self.pc + 2;
                self.pc = @intCast(u32, @as(i64, self.pc) + imm);
            },
            // C.LI (HINT, rd=0)
            0b010 => {
                const ci_type = @bitCast(Encodings.CIType, instruction);
                const imm = @as(i32, @as(i6, ci_type.imm1) | @as(i6, ci_type.imm2) << 5);
                std.log.info("C.LI: {any}", .{ci_type});
                self.regs[ci_type.rd] = @bitCast(u32, imm);
            },
            // C.ADDI16SP (RES, nzimm=0)
            // C.LUI (RES, nzimm=0; HINT, rd=0)
            0b011 => {
                const ci_type = @bitCast(Encodings.CIType, instruction);
                if (ci_type.rd == 2) {
                    const sign = @bitCast(i12, @truncate(u12, instruction & 0b1000000000000));
                    const p_9 = @as(i12, ci_type.imm2);
                    const p_4 = @as(i12, ci_type.imm1 & 0b10000);
                    const p_6 = @as(i12, ci_type.imm1 & 0b01000) << 2;
                    const p_8_7 = @as(i12, ci_type.imm1 & 0b00110) << 5;
                    const p_5 = @as(i12, ci_type.imm1 & 0b00001) << 5;
                    const imm = @as(i64, sign | p_9 | p_8_7 | p_6 | p_5 | p_4);
                    std.log.info("C.ADDI16SP: {any}", .{ci_type});
                    self.regs[2] = @intCast(u32, @as(i64, self.regs[2]) + imm);
                } else {
                    const imm = @as(u32, @as(u6, ci_type.imm1) | @as(u6, ci_type.imm2) << 5) << 12;
                    const val = (0 -% ci_type.imm2) & 0x000 & imm;
                    std.log.info("C.LUI: {any}", .{ci_type});
                    self.regs[ci_type.rd] = val;
                }
            },
            // C.SRLI (RV32 NSE, nzuimm[5]=1)
            // C.SRAI (RV32 NSE, nzuimm[5]=1)
            // C.ANDI
            // C.SUB
            // C.XOR
            // C.OR
            // C.AND
            // Reserved
            // Reserved
            0b100 => {
                std.log.info("C.SRLI", .{});
            },
            // C.J
            0b101 => {
                const cj_type = @bitCast(Encodings.CJType, instruction);

                // offset[11|4|9:8|10|6|7|3:1|5]
                const sign = @truncate(u13, instruction & 0b1000000000000);
                const p_11 = @as(u13, cj_type.imm & 0b10000000000) << 1;
                const p_4 = @as(u13, cj_type.imm & 0b01000000000) >> 5;
                const p_9_8 = @as(u13, cj_type.imm & 0b00110000000) << 1;
                const p_10 = @as(u13, cj_type.imm & 0b00001000000) << 4;
                const p_6 = @as(u13, cj_type.imm & 0b00000100000) << 1;
                const p_7 = @as(u13, cj_type.imm & 0b00000010000) << 3;
                const p_3_1 = @as(u13, cj_type.imm & 0b00000001110);
                const p_5 = @as(u13, cj_type.imm & 0b00000000001) << 5;
                const imm = @as(i64, @bitCast(i13, sign | p_11 | p_10 | p_9_8 | p_7 | p_6 | p_5 | p_4 | p_3_1));

                std.log.info("C.J: {any}", .{cj_type});
                self.pc = @intCast(u32, @as(i64, self.pc) + imm);
            },
            // C.BEQZ
            0b110 => {
                std.log.info("C.BEQZ", .{});
            },
            // C.BNEZ
            0b111 => {
                std.log.info("C.BNEZ", .{});
            },
            else => return true,
        }
        self.pc += 2;
        return false;
    }
    fn run_compressed_10(self: *Self, instruction: u32) bool {
        std.log.info("run_compressed_10", .{});
        switch ((instruction & 0b1110000000000000) >> 13) {
            // C.SLLI (HINT, rd=0; RV32 NSE, nzuimm[5]=1)
            0b000 => {},
            // C.FLDSP (RV32/64)
            0b001 => {},
            // C.LWSP (RES, rd=0
            0b010 => {},
            // C.FLWSP (RV32)
            0b011 => {},
            // C.JR (RES, rs1=0)
            // C.MV (HINT, rd=0)
            // C.EBREAK
            // C.JALR
            // C.ADD (HINT, rd=0)
            0b100 => {},
            // C.FSDSP (RV32/64)
            0b101 => {},
            // C.SWSP
            0b110 => {},
            // C.FSWSP (RV32)
            0b111 => {},
            else => unreachable(),
        }
        _ = self;
        return true;
    }

    fn run_full(self: *Self, instruction: u32) bool {
        var next_pc = self.pc + 4;

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
            0b0010111 => {
                const u_type = @bitCast(Encodings.UType, instruction);
                std.log.info("AUIPC: {any}", .{u_type});
                self.regs[u_type.rd] = @intCast(u32, @as(i64, self.pc) +% @as(i64, u_type.get_imm()));
            },
            //JAL
            0b1101111 => {
                const j_type = @bitCast(Encodings.JType, instruction);
                std.log.info("JAL: {any}", .{j_type});
                next_pc = @intCast(u32, @as(i64, self.pc) +% @as(i64, j_type.get_imm()));
                self.regs[j_type.rd] = self.pc + 4;
            },
            //JALR
            0b1100111 => {
                const i_type = @bitCast(Encodings.IType, instruction);
                std.log.info("JALR: {any}", .{i_type});
                // std.log.info("rs1: {}", .{self.regs[i_type.rs1]});
                // std.log.info("imm: {}", .{i_type.get_imm()});
                std.log.info("JARL before: {}", .{next_pc});
                next_pc = @intCast(u32, @intCast(i64, self.regs[i_type.rs1]) +% @as(i64, i_type.get_imm())) & 0xfffffffe;
                self.regs[i_type.rd] = self.pc + 4;
                std.log.info("JARL next_pc: {}", .{next_pc});
            },
            0b1100011 => {
                const b_type = @bitCast(Encodings.BType, instruction);
                const a = self.regs[b_type.rs1];
                const b = self.regs[b_type.rs2];
                switch (b_type.func3) {
                    //BEQ
                    0b000 => {
                        std.log.info("BEQ: {any}", .{b_type});
                        std.log.info("BEQ: {}", .{b_type.get_imm()});
                        if (a == b) {
                            next_pc = @intCast(u32, @as(i64, self.pc) +% @as(i64, b_type.get_imm()));
                        }
                    },
                    //BNE
                    0b001 => {
                        std.log.info("BNE: {any}", .{b_type});
                        if (a != b) {
                            next_pc = @intCast(u32, @as(i64, self.pc) +% @as(i64, b_type.get_imm()));
                        }
                    },
                    //BLT
                    0b100 => {
                        std.log.info("BLT: {any}", .{b_type});
                        if (@intCast(i64, a) < @intCast(i64, b)) {
                            next_pc = @intCast(u32, @as(i64, self.pc) +% @as(i64, b_type.get_imm()));
                        }
                    },
                    //BGE
                    0b101 => {
                        std.log.info("BGE: {any}", .{b_type});
                        if (@intCast(i64, a) >= @intCast(i64, b)) {
                            next_pc = @intCast(u32, @as(i64, self.pc) +% @as(i64, b_type.get_imm()));
                        }
                    },
                    //BLTU
                    0b110 => {
                        std.log.info("BLTU: {any}", .{b_type});
                        if (a < b) {
                            next_pc = @intCast(u32, @as(i64, self.pc) +% @as(i64, b_type.get_imm()));
                        }
                    },
                    //BGEU
                    0b111 => {
                        std.log.info("BGEU: {any}", .{b_type});
                        if (a >= b) {
                            next_pc = @intCast(u32, @as(i64, self.pc) +% @as(i64, b_type.get_imm()));
                        }
                    },
                    else => unreachable,
                }
            },
            0b0000011 => {
                const i_type = @bitCast(Encodings.IType, instruction);
                const addr = self.regs[i_type.rs1] + @bitCast(u32, i_type.get_imm());
                switch (i_type.func3) {
                    //LB
                    0b000 => {
                        std.log.info("LB: {any}", .{i_type});
                        self.regs[i_type.rd] = @bitCast(u32, @as(i32, @bitCast(i8, self.ram.read(u8, addr))));
                    },
                    //LH
                    0b001 => {
                        std.log.info("LH: {any}", .{i_type});
                        self.regs[i_type.rd] = @bitCast(u32, @as(i32, @bitCast(i16, self.ram.read(u16, addr))));
                    },
                    //LW
                    0b010 => {
                        std.log.info("LW: {any}", .{i_type});
                        self.regs[i_type.rd] = self.ram.read(u32, addr);
                    },
                    //LBU
                    0b100 => {
                        std.log.info("LBU: {any}", .{i_type});
                        self.regs[i_type.rd] = @as(u32, self.ram.read(u8, addr));
                    },
                    //LHU
                    0b101 => {
                        std.log.info("LHU: {any}", .{i_type});
                        self.regs[i_type.rd] = @as(u32, self.ram.read(u16, addr));
                    },
                    else => unreachable,
                }
            },
            0b0100011 => {
                const s_type = @bitCast(Encodings.SType, instruction);
                const addr = @intCast(u32, @intCast(i64, self.regs[s_type.rs1]) + s_type.get_imm());
                switch (s_type.func3) {
                    //SB
                    0b000 => {
                        std.log.info("SB: {any}", .{s_type});
                        const data = self.regs[s_type.rs2];
                        self.ram.write(u8, addr, @truncate(u8, data));
                    },
                    //SH
                    0b001 => {
                        std.log.info("SH: {any}", .{s_type});
                        const data = self.regs[s_type.rs2];
                        self.ram.write(u16, addr, @truncate(u16, data));
                    },
                    //SW
                    0b010 => {
                        std.log.info("SW: {any}", .{s_type});
                        const data = self.regs[s_type.rs2];
                        self.ram.write(u32, addr, data);
                    },
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
                        self.regs[i_type.rd] = @bitCast(u32, @bitCast(i32, self.regs[i_type.rs1]) +% i_type.get_imm());
                    },
                    //SLTI
                    0b010 => {
                        std.log.info("SLTI: {any}", .{i_type});
                        if (@as(i64, self.regs[i_type.rs1]) < @as(i64, i_type.get_imm())) {
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
                        self.regs[i_type.rd] = self.regs[i_type.rs1] ^ @bitCast(u32, i_type.get_imm());
                    },
                    //ORI
                    0b110 => {
                        std.log.info("ORI: {any}", .{i_type});
                        self.regs[i_type.rd] = self.regs[i_type.rs1] | @bitCast(u32, i_type.get_imm());
                    },
                    //ANDI
                    0b111 => {
                        std.log.info("ANDI: {any}", .{i_type});
                        self.regs[i_type.rd] = self.regs[i_type.rs1] & @bitCast(u32, i_type.get_imm());
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
                            0b000 => {
                                std.log.info("MUL: {any}", .{r_type});
                                self.regs[r_type.rd] = self.regs[r_type.rs1] *% self.regs[r_type.rs2];
                            },
                            //MULH
                            0b001 => {
                                std.log.info("MULH: {any}", .{r_type});
                                const a = @as(i64, @bitCast(i32, self.regs[r_type.rs1]));
                                const b = @as(i64, @bitCast(i32, self.regs[r_type.rs2]));
                                const res = @bitCast(u64, a *% b);
                                self.regs[r_type.rd] = @truncate(u32, res >> 32);
                            },
                            //MULHSU
                            0b010 => {
                                std.log.info("MULHSU: {any}", .{r_type});
                                const a = @as(i64, @bitCast(i32, self.regs[r_type.rs1]));
                                const b = @as(i64, @bitCast(i32, self.regs[r_type.rs2]));
                                const res = @bitCast(u64, a *% b);
                                self.regs[r_type.rd] = @truncate(u32, res >> 32);
                            },
                            //MULHU
                            0b011 => {
                                std.log.info("MULHU: {any}", .{r_type});
                                const a = @as(u64, self.regs[r_type.rs1]);
                                const b = @as(u64, self.regs[r_type.rs2]);
                                const res = @bitCast(u64, a *% b);
                                self.regs[r_type.rd] = @truncate(u32, res >> 32);
                            },
                            //DIV
                            0b100 => {
                                std.log.info("DIV: {any}", .{r_type});
                                const a = @bitCast(i32, self.regs[r_type.rs1]);
                                const b = @bitCast(i32, self.regs[r_type.rs2]);
                                if (b == 0) {
                                    self.regs[r_type.rd] = std.math.maxInt(u32);
                                } else {
                                    const res = @bitCast(u32, @divTrunc(a, b));
                                    self.regs[r_type.rd] = res;
                                }
                            },
                            //DIVU
                            0b101 => {
                                std.log.info("DIVU: {any}", .{r_type});
                                const a = self.regs[r_type.rs1];
                                const b = self.regs[r_type.rs2];
                                if (b == 0) {
                                    self.regs[r_type.rd] = std.math.maxInt(u32);
                                } else {
                                    self.regs[r_type.rd] = a / b;
                                }
                            },
                            //REM
                            0b110 => {
                                std.log.info("REM: {any}", .{r_type});
                                const a = @bitCast(i32, self.regs[r_type.rs1]);
                                const b = @bitCast(i32, self.regs[r_type.rs2]);
                                if (b == 0) {
                                    self.regs[r_type.rd] = @bitCast(u32, a);
                                } else {
                                    const res = @bitCast(u32, @rem(a, b));
                                    self.regs[r_type.rd] = res;
                                }
                            },
                            //REMU
                            0b111 => {
                                std.log.info("REMU: {any}", .{r_type});
                                const a = self.regs[r_type.rs1];
                                const b = self.regs[r_type.rs2];
                                if (b == 0) {
                                    self.regs[r_type.rd] = a;
                                } else {
                                    self.regs[r_type.rd] = a % b;
                                }
                            },
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
            else => return true,
        }
        self.pc = next_pc;
        return false;
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
        .pc = 0x110b4,
        .ram = ram,
    };
    std.mem.set(u32, &cpu.regs, 0);

    cpu.regs[2] = 0x110fc + @intCast(u32, code.len + 1000);

    var stop: bool = false;
    while (!stop) {
        const intsruction = cpu.fetch_next_intruction();
        std.log.info("instruction: {x}", .{intsruction});
        cpu.print_regs();
        stop = cpu.run_instruction();
    }
}
