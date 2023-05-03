const std = @import("std");

const Soc = @import("soc.zig");

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

    var ram: Soc.Ram = .{ .base_addr = 0x110b4, .mem = undefined };
    std.mem.copy(u8, &ram.mem, code);

    var cpu = Soc.Cpu{
        .regs = undefined,
        .pc = 0x110b4,
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
