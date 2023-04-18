const std = @import("std");

const Cpu = struct {
    regs: [32]u32,
    pc: usize,
};

const RType = packed struct {
    opcode: u7,
    rd: u5,
    func3: u3,
    rs1: u5,
    rs2: u5,
    func7: u7,
};
const IType = packed struct {
    opcode: u7,
    rd: u5,
    func3: u3,
    rs1: u5,
    imm: i12,

    const Self = @This();
    fn get_imm(self: *const Self) i32 {
        return self.imm;
    }
};
const SType = packed struct {
    opcode: u7,
    imm1: u5,
    func3: u3,
    rs1: u5,
    rs2: u5,
    imm2: u7,

    const Self = @This();
    fn get_imm(self: *const Self) i32 {
        const sign = @as(i12, self.imm2 & 0b1000000) << 5;
        const imm2 = @as(i12, self.imm2 & 0b111111) << 5;
        const imm1 = @as(i12, self.imm1);
        return @as(i32, sign | imm2 | imm1);
    }
};
const BType = packed struct {
    opcode: u7,
    imm1: u5,
    func3: u3,
    rs1: u5,
    rs2: u5,
    imm2: u7,

    const Self = @This();
    fn get_imm(self: *const Self) i32 {
        const sign = @as(i13, self.imm2 & 0b1000000) << 6;
        const p_10_5 = @as(i13, self.imm2 & 0b111111) << 5;
        const p_4_1 = @as(i13, self.imm1 & 0b11110);
        const p_11 = @as(i13, self.imm1 & 1) << 11;
        return @as(i32, sign | p_11 | p_10_5 | p_4_1);
    }
};
const UType = packed struct {
    opcode: u7,
    rd: u5,
    imm: i20,

    const Self = @This();
    fn get_imm(self: *const Self) i32 {
        return @as(i32, self.imm) << 12;
    }
};
const JType = packed struct {
    opcode: u7,
    rd: u5,
    imm: u20,

    const Self = @This();
    fn get_imm(self: *const Self) i32 {
        const sign = @as(i21, self.imm & 0b10000000000000000000);
        const p_20 = @as(i21, self.imm & 0b01000000000000000000);
        const p_10_1 = @as(i21, self.imm & 0b00111111111100000000);
        const p_11 = @as(i21, self.imm & 0b00000000000010000000);
        const p_19_12 = @as(i21, self.imm & 0b00000000000001111111);

        std.log.info("{b:0>25} : sign", .{sign});
        std.log.info("{b:0>25} : p_20", .{p_20});
        std.log.info("{b:0>25} : p_10_1", .{p_10_1});
        std.log.info("{b:0>25} : p_11", .{p_11});
        std.log.info("{b:0>25} : p_19_12", .{p_19_12});

        return @as(i32, (sign | p_20 | p_19_12 | p_11 | p_10_1) << 1);
    }
};

fn run_instruction(cpu: *Cpu, instruction: u32) void {
    var opcode = instruction & 0b1111111;
    std.log.info("opcode: {b}", .{opcode});
    switch (opcode) {
        // RV32I Base Instruction Set
        //LUI
        0b0110111 => {
            const u_type = @bitCast(UType, instruction);
            std.log.info("imm: {b}", .{u_type.imm});
            std.log.info("var: {b}", .{u_type.get_imm()});
            cpu.regs[u_type.rd] = @bitCast(u32, u_type.get_imm());
        },
        //AUIPC
        0b0010111 => {},
        //JAL
        0b1101111 => {},
        //JALR
        0b1100111 => {},
        0b1100011 => {
            const b_type = @bitCast(BType, instruction);
            switch (b_type.func3) {
                //BEQ
                0b000 => {},
                //BNE
                0b001 => {},
                //BLT
                0b100 => {},
                //BGE
                0b101 => {},
                //BLTU
                0b110 => {},
                //BGEU
                0b111 => {},
                else => unreachable,
            }
        },

        0b0000011 => {
            const i_type = @bitCast(IType, instruction);
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
            const s_type = @bitCast(SType, instruction);
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
            const i_type = @bitCast(IType, instruction);
            switch (i_type.func3) {
                //ADDI
                0b000 => {
                    // Also NOP
                    // NOP = ADDI x0, x0, 0.
                    const op = @addWithOverflow(cpu.regs[i_type.rs1], @intCast(u32, i_type.get_imm()));
                    cpu.regs[i_type.rd] += op.@"0";
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
                    cpu.regs[i_type.rd] = 1; //cpu.regs[i_type.rs1] << i_type.get_imm();//@as(u32, @intCast(u32, i_type.get_imm()) & @as(u32, 0b11111));
                },
                0b101 => {
                    const r_type = @bitCast(RType, instruction);
                    switch (r_type.func7) {
                        //SRLI
                        0b0000000 => {
                            const shift = i_type.get_imm() & 0b11111;
                            cpu.regs[i_type.rd] = std.math.shl(u32, cpu.regs[i_type.rs1], shift);
                        },
                        //SRAI
                        0b0100000 => {
                            const shift = i_type.get_imm() & 0b11111;
                            cpu.regs[i_type.rd] = std.math.shl(u32, cpu.regs[i_type.rs1], shift);
                        },
                        else => unreachable,
                    }
                },
                // else => unreachable,
            }
        },
        0b0110011 => {
            const r_type = @bitCast(RType, instruction);
            switch (r_type.func7) {
                // Base set
                0b0000000 => {
                    switch (r_type.func3) {
                        //ADD
                        0b000 => {
                            const op = @addWithOverflow(cpu.regs[r_type.rs1], cpu.regs[r_type.rs2]);
                            cpu.regs[r_type.rd] = op.@"0";
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
            const i_type = @bitCast(IType, instruction);
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

test "IType" {
    const expect = std.testing.expect;

    {
        const i_instruction: u32 = 0b111111111101_00000_010_11111_1010101;
        const i_type = @bitCast(IType, i_instruction);
        try expect(i_type.opcode == 0b1010101);
        try expect(i_type.rd == 0b11111);
        try expect(i_type.func3 == 0b010);
        try expect(i_type.rs1 == 0b00000);
        try expect(i_type.imm == -3);
    }

    {
        const i_instruction: u32 = 0xffef8b93;
        const i_type = @bitCast(IType, i_instruction);
        try expect(i_type.opcode == 0b0010011);
        try expect(i_type.rd == 23);
        try expect(i_type.func3 == 0);
        try expect(i_type.rs1 == 31);
        try expect(i_type.imm == -2);
    }

    {
        const i_instruction: u32 = 0x7fff8b93;
        const i_type = @bitCast(IType, i_instruction);
        try expect(i_type.opcode == 0b0010011);
        try expect(i_type.rd == 23);
        try expect(i_type.func3 == 0);
        try expect(i_type.rs1 == 31);
        try expect(i_type.imm == 2047);
    }

    {
        const i_instruction: u32 = 0x8003e693;
        const i_type = @bitCast(IType, i_instruction);
        try expect(i_type.opcode == 0b0010011);
        try expect(i_type.rd == 13);
        try expect(i_type.func3 == 0b110);
        try expect(i_type.rs1 == 7);
        try expect(i_type.imm == -2048);
    }
}

test "SType" {
    const expect = std.testing.expect;

    {
        const s_instruction: u32 = 0x81f78023;
        const s_type = @bitCast(SType, s_instruction);
        try expect(s_type.opcode == 0b0100011);
        try expect(s_type.func3 == 0);
        try expect(s_type.rs1 == 15);
        try expect(s_type.rs2 == 31);
        try expect(s_type.get_imm() == -2048);
    }

    {
        const s_instruction: u32 = 0x7f219fa3;
        const s_type = @bitCast(SType, s_instruction);
        try expect(s_type.opcode == 0b0100011);
        try expect(s_type.func3 == 1);
        try expect(s_type.rs1 == 3);
        try expect(s_type.rs2 == 18);
        try expect(s_type.get_imm() == 2047);
    }

    {
        const s_instruction: u32 = 0x008ba0a3;
        const s_type = @bitCast(SType, s_instruction);
        try expect(s_type.opcode == 0b0100011);
        try expect(s_type.func3 == 2);
        try expect(s_type.rs1 == 23);
        try expect(s_type.rs2 == 8);
        try expect(s_type.get_imm() == 1);
    }

    {
        const s_instruction: u32 = 0xfe5cafa3;
        const s_type = @bitCast(SType, s_instruction);
        try expect(s_type.opcode == 0b0100011);
        try expect(s_type.func3 == 2);
        try expect(s_type.rs1 == 25);
        try expect(s_type.rs2 == 5);
        try expect(s_type.get_imm() == -1);
    }
}

test "BType" {
    const expect = std.testing.expect;

    {
        const b_instruction: u32 = 0x80e50063;
        const b_type = @bitCast(BType, b_instruction);
        try expect(b_type.opcode == 0b1100011);
        try expect(b_type.func3 == 0);
        try expect(b_type.rs1 == 10);
        try expect(b_type.rs2 == 14);
        try expect(b_type.get_imm() == -4096);
    }

    {
        const b_instruction: u32 = 0x7f51cfe3;
        const b_type = @bitCast(BType, b_instruction);
        try expect(b_type.opcode == 0b1100011);
        try expect(b_type.func3 == 0b100);
        try expect(b_type.rs1 == 3);
        try expect(b_type.rs2 == 21);
        try expect(b_type.get_imm() == 4094);
    }

    {
        const b_instruction: u32 = 0xfe095fe3;
        const b_type = @bitCast(BType, b_instruction);
        try expect(b_type.opcode == 0b1100011);
        try expect(b_type.func3 == 0b101);
        try expect(b_type.rs1 == 18);
        try expect(b_type.rs2 == 0);
        try expect(b_type.get_imm() == -2);
    }
}

test "UType" {
    const expect = std.testing.expect;

    {
        const u_instruction: u32 = 0x00000fb7;
        const u_type = @bitCast(UType, u_instruction);
        try expect(u_type.opcode == 0b0110111);
        try expect(u_type.rd == 31);
        try expect(u_type.get_imm() == 0);
    }

    {
        const u_instruction: u32 = 0x123ab8b7;
        const u_type = @bitCast(UType, u_instruction);
        try expect(u_type.opcode == 0b0110111);
        try expect(u_type.rd == 17);
        try expect(u_type.get_imm() == 0x123ab000);
    }
}

test "JType" {
    const expect = std.testing.expect;

    {
        const j_instruction: u32 = 0x7ffff06f;
        const j_type = @bitCast(JType, j_instruction);
        try expect(j_type.opcode == 0b1101111);
        try expect(j_type.rd == 0);
        try expect(j_type.get_imm() == 0xffffe);
    }

    {
        const j_instruction: u32 = 0x80000fef;
        const j_type = @bitCast(JType, j_instruction);
        try expect(j_type.opcode == 0b1101111);
        try expect(j_type.rd == 31);
        try expect(j_type.get_imm() == -0x100000);
    }

    {
        const j_instruction: u32 = 0xfffff6ef;
        const j_type = @bitCast(JType, j_instruction);
        try expect(j_type.opcode == 0b1101111);
        try expect(j_type.rd == 13);
        try expect(j_type.get_imm() == -2);
    }
}
