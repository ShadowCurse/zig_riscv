const std = @import("std");

const Soc = @import("soc.zig");
const Uart = @import("uart.zig");

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

    var code = try file.reader().readAllAlloc(allocator, std.math.maxInt(usize));
    defer allocator.free(code);

    var cpu = try Soc.Cpu.new(load_address, allocator);
    defer cpu.deinit(allocator);

    std.mem.copy(u8, cpu.ram.mem, code);

    cpu.regs[2] = load_address + @intCast(u32, code.len);

    while (cpu.run_instruction()) {
        cpu.print_regs();
    } else |err| {
        std.log.info("error: {}", .{err});
    }
}
