const std = @import("std");

const Encodings = @import("encodings.zig");

const Cpu = struct {
    regs: [32]u32,
    pc: usize,
};

fn run_instruction(cpu: *Cpu, instruction: u32) void {
    var opcode = instruction & 0b1111111;
    std.log.info("opcode: {b}", .{opcode});
    switch (opcode) {
        // RV32I Base Instruction Set
        //LUI
        0b0110111 => {
            const u_type = @bitCast(Encodings.UType, instruction);
            cpu.regs[u_type.rd] = @intCast(u32, u_type.get_imm());
        },
        //AUIPC
        0b0010111 => {},
        //JAL
        0b1101111 => {
            const j_type = @bitCast(Encodings.JType, instruction);
            cpu.regs[j_type.rd] = cpu.pc + 4;
            cpu.pc = @intCast(u32, @intCast(i32, cpu.pc) +% j_type.get_imm());
        },
        //JALR
        0b1100111 => {
            const i_type = @bitCast(Encodings.IType, instruction);
            cpu.regs[i_type.rd] = cpu.pc + 4;
            cpu.pc = @intCast(u32, @intCast(i32, cpu.regs[i_type.rs1]) +% i_type.get_imm()) & 0xfffffffe;
        },
        0b1100011 => {
            const b_type = @bitCast(Encodings.BType, instruction);
            const a = cpu.regs[b_type.rs1];
            const b = cpu.regs[b_type.rs2];
            switch (b_type.func3) {
                //BEQ
                0b000 => {
                    if (a == b) {
                        cpu.pc = cpu.pc +% @intCast(u32, b_type.get_imm());
                    }
                },
                //BNE
                0b001 => {
                    if (a != b) {
                        cpu.pc = cpu.pc +% @intCast(u32, b_type.get_imm());
                    }
                },
                //BLT
                0b100 => {
                    if (@intCast(i32, a) < @intCast(i32, b)) {
                        cpu.pc = cpu.pc +% @intCast(u32, b_type.get_imm());
                    }
                },
                //BGE
                0b101 => {
                    if (@intCast(i32, a) >= @intCast(i32, b)) {
                        cpu.pc = cpu.pc +% @intCast(u32, b_type.get_imm());
                    }
                },
                //BLTU
                0b110 => {
                    if (a < b) {
                        cpu.pc = cpu.pc +% @intCast(u32, b_type.get_imm());
                    }
                },
                //BGEU
                0b111 => {
                    if (a >= b) {
                        cpu.pc = cpu.pc +% @intCast(u32, b_type.get_imm());
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
                    // Also NOP
                    // NOP = ADDI x0, x0, 0.
                    cpu.regs[i_type.rd] = cpu.regs[i_type.rs1] +% @intCast(u32, i_type.get_imm());
                },
                //SLTI
                0b010 => {
                    if (@intCast(i32, cpu.regs[i_type.rs1]) < i_type.get_imm()) {
                        cpu.regs[i_type.rd] = 1;
                    } else {
                        cpu.regs[i_type.rd] = 0;
                    }
                },
                //SLTIU
                0b011 => {
                    if (@as(u32, cpu.regs[i_type.rs1]) < @intCast(u32, i_type.get_imm())) {
                        cpu.regs[i_type.rd] = 1;
                    } else {
                        cpu.regs[i_type.rd] = 0;
                    }
                },
                //XORI
                0b100 => {
                    cpu.regs[i_type.rd] = cpu.regs[i_type.rs1] ^ @intCast(u32, i_type.get_imm());
                },
                //ORI
                0b110 => {
                    cpu.regs[i_type.rd] = cpu.regs[i_type.rs1] | @intCast(u32, i_type.get_imm());
                },
                //ANDI
                0b111 => {
                    cpu.regs[i_type.rd] = cpu.regs[i_type.rs1] & @intCast(u32, i_type.get_imm());
                },
                //SLLI
                0b001 => {
                    const shift = i_type.get_imm() & 0b11111;
                    cpu.regs[i_type.rd] = std.math.shl(u32, cpu.regs[i_type.rs1], shift);
                },
                0b101 => {
                    const r_type = @bitCast(Encodings.RType, instruction);
                    switch (r_type.func7) {
                        //SRLI
                        0b0000000 => {
                            const shift = i_type.get_imm() & 0b11111;
                            cpu.regs[i_type.rd] = std.math.shr(u32, cpu.regs[i_type.rs1], shift);
                        },
                        //SRAI
                        0b0100000 => {
                            const shift = i_type.get_imm() & 0b11111;
                            cpu.regs[i_type.rd] = std.math.shr(u32, cpu.regs[i_type.rs1], shift);
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
                            cpu.regs[r_type.rd] = cpu.regs[r_type.rs1] +% cpu.regs[r_type.rs2];
                        },
                        //SLL
                        0b001 => {
                            const shift = cpu.regs[r_type.rs2] & 0b11111;
                            cpu.regs[r_type.rd] = std.math.shl(u32, cpu.regs[r_type.rs1], shift);
                        },
                        //SLT
                        0b010 => {
                            if (@intCast(i32, cpu.regs[r_type.rs1]) < @intCast(i32, cpu.regs[r_type.rs2])) {
                                cpu.regs[r_type.rd] = 1;
                            } else {
                                cpu.regs[r_type.rd] = 0;
                            }
                        },
                        //SLTU
                        0b011 => {
                            if (cpu.regs[r_type.rs1] < cpu.regs[r_type.rs2]) {
                                cpu.regs[r_type.rd] = 1;
                            } else {
                                cpu.regs[r_type.rd] = 0;
                            }
                        },
                        //XOR
                        0b100 => {
                            cpu.regs[r_type.rd] = cpu.regs[r_type.rs1] ^ cpu.regs[r_type.rs2];
                        },
                        //OR
                        0b110 => {
                            cpu.regs[r_type.rd] = cpu.regs[r_type.rs1] | cpu.regs[r_type.rs2];
                        },
                        //AND
                        0b111 => {
                            cpu.regs[r_type.rd] = cpu.regs[r_type.rs1] & cpu.regs[r_type.rs2];
                        },
                        //SRL
                        0b101 => {
                            const shift = cpu.regs[r_type.rs2] & 0b11111;
                            cpu.regs[r_type.rd] = std.math.shr(u32, cpu.regs[r_type.rs1], shift);
                        },
                    }
                },
                0b0100000 => {
                    switch (r_type.func3) {
                        // SUB
                        0b000 => {
                            const op = @subWithOverflow(cpu.regs[r_type.rs1], cpu.regs[r_type.rs2]);
                            cpu.regs[r_type.rd] = op.@"0";
                        },
                        // SRA
                        0b101 => {
                            cpu.regs[r_type.rd] = cpu.regs[r_type.rs1] >> r_type.rs2 & 0b11111;
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

pub fn main() !void {
    var cpu = Cpu{
        .regs = undefined,
        .pc = 0,
    };

    std.mem.set(u32, &cpu.regs, 0);

    cpu.regs[0] = 1;
    cpu.regs[1] = 10;

    std.log.info("before regs[0]: {}", .{cpu.regs[0]});

    // const instruction: u32 = 0b00000000000000000011_00000_0110111;
    const instruction: u32 = 0b0000000_00000_00001_000_00000_0110011;

    run_instruction(&cpu, instruction);

    std.log.info("after regs[0]: {}", .{cpu.regs[0]});
}
