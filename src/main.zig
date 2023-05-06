const std = @import("std");

const Soc = @import("soc.zig");

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();
    const allocator = gpa.allocator();

    const args = try std.process.argsAlloc(allocator);
    defer std.process.argsFree(allocator, args);

    if (args.len != 3) {
        std.log.info("Invalid arguments: expect [executable_path] [load_address]", .{});
        std.process.exit(1);
    }

    const load_address = try std.fmt.parseInt(u32, args[2], 16);

    const file = try std.fs.cwd().openFile(args[1], .{});
    defer file.close();

    var code = try file.reader().readAllAlloc(allocator, 99999);
    defer allocator.free(code);

    var ram: Soc.Ram = .{ .base_addr = load_address, .mem = undefined };
    std.mem.copy(u8, &ram.mem, code);

    var cpu = Soc.Cpu{
        .csr = undefined,
        .regs = undefined,
        .pc = load_address,
        .ram = ram,
    };
    std.mem.set(u32, &cpu.regs, 0);

    cpu.regs[2] = 0x110fc + @intCast(u32, code.len + 1000);

    while (cpu.run_instruction()) {
        cpu.print_regs();
    } else |err| {
        std.log.info("error: {}", .{err});
    }
}
