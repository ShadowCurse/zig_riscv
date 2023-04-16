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

fn run_instruction(cpu: *Cpu, intruction: u32) void {
    var opcode = intruction & 0b1111111;
    std.log.info("opcode: {b}", .{opcode});
    switch (opcode) {
        // RV32I Base Instruction Set
        //LUI
        0b0110111 => {
            const u_type = @bitCast(UType, intruction);
            cpu.regs[u_type.rd] = @as(u32, u_type.get_imm());
        },
        else => {},
        //
        // 0b0010111 => {}, //AUIPC
        //
        // 0b1101111 => {}, //JAL
        //
        // 0b1100111 => {}, //JALR
        //
        // 0b1100011 => {}, //BEQ
        // 0b1100011 => {}, //BNE
        // 0b1100011 => {}, //BLT
        // 0b1100011 => {}, //BGE
        // 0b1100011 => {}, //BLTU
        // 0b1100011 => {}, //BGEU
        //
        // 0b0000011 => {}, //LB
        // 0b0000011 => {}, //LH
        // 0b0000011 => {}, //LW
        // 0b0000011 => {}, //LBU
        // 0b0000011 => {}, //LHU
        //
        // 0b0100011 => {}, //SB
        // 0b0100011 => {}, //SH
        // 0b0100011 => {}, //SW
        //
        // 0b0010011 => {}, //ADDI
        // 0b0010011 => {}, //SLTI
        // 0b0010011 => {}, //SLTIU
        // 0b0010011 => {}, //XORI
        // 0b0010011 => {}, //ORI
        // 0b0010011 => {}, //ANDI
        // 0b0010011 => {}, //SLLI
        // 0b0010011 => {}, //SRLI
        // 0b0010011 => {}, //SRAI
        //
        // 0b0110011 => {}, //ADD
        // 0b0110011 => {}, //SUB
        // 0b0110011 => {}, //SLL
        // 0b0110011 => {}, //SLT
        // 0b0110011 => {}, //SLTU
        // 0b0110011 => {}, //XOR
        // 0b0110011 => {}, //SRL
        // 0b0110011 => {}, //SRA
        // 0b0110011 => {}, //OR
        // 0b0110011 => {}, //AND
        //
        // 0b0001111 => {}, //FENCE
        //
        // 0b1110011 => {}, //ECALL
        // 0b1110011 => {}, //EBREAK
        //
        // // RV32M Standard Extension
        // 0b0110011 => {}, //MUL
        // 0b0110011 => {}, //MULH
        // 0b0110011 => {}, //MULHSU
        // 0b0110011 => {}, //MULHU
        // 0b0110011 => {}, //DIV
        // 0b0110011 => {}, //DIVU
        // 0b0110011 => {}, //REM
        // 0b0110011 => {}, //REMU
    }
}

pub fn main() !void {
    var cpu = Cpu{
        .regs = undefined,
        .pc = 0,
    };

    std.mem.set(u32, &cpu.regs, 0);

    std.log.info("before regs[0]: {}", .{cpu.regs[0]});

    const instruction: u32 = 0b00000000000000000011000000110111;

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
