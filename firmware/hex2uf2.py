"""hex2uf2.py <in.hex> <out.uf2> — Intel HEX → UF2 для nRF52840 (XIAO): скопировать на диск XIAO-SENSE."""
import struct, sys
FAM = 0xADA52840
src, dst = sys.argv[1], sys.argv[2]
mem, upper = {}, 0
for line in open(src):
    line = line.strip()
    if not line.startswith(':'): continue
    b = bytes.fromhex(line[1:]); n, addr, typ = b[0], (b[1] << 8) | b[2], b[3]; data = b[4:4+n]
    if typ == 4: upper = ((data[0] << 8) | data[1]) << 16
    elif typ == 2: upper = ((data[0] << 8) | data[1]) << 4
    elif typ == 0:
        for i, d in enumerate(data): mem[upper + addr + i] = d
    elif typ == 1: break
addrs = sorted(mem); lo = addrs[0] & ~0xff; hi = addrs[-1]
blocks = []
a = lo
while a <= hi:
    if any((a + i) in mem for i in range(256)):
        blocks.append((a, bytes(mem.get(a + i, 0xff) for i in range(256))))
    a += 256
out = bytearray()
for i, (a, chunk) in enumerate(blocks):
    out += struct.pack('<IIIIIIII', 0x0A324655, 0x9E5D5157, 0x2000, a, 256, i, len(blocks), FAM)
    out += chunk + b'\x00' * 220 + struct.pack('<I', 0x0AB16F30)
open(dst, 'wb').write(out)
print(f"0x{lo:X}-0x{hi:X}: {len(blocks)} blocks, {len(out)} bytes -> {dst}")
