"""Write a pattern VIDEO.BIN for the pattern checker in basys3_top.v.

byte i = i[7:0] ^ i[15:8] ^ i[23:16]   (every value 00-FF appears, incl. 00/FF)

    python make_pattern.py [output_path] [size]
"""
import functools
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "VIDEO.BIN"
size = int(sys.argv[2]) if len(sys.argv) > 2 else 100000

data = bytes(((i & 0xFF) ^ ((i >> 8) & 0xFF) ^ ((i >> 16) & 0xFF)) for i in range(size))
with open(path, "wb") as f:
    f.write(data)
x = functools.reduce(lambda a, b: a ^ b, data, 0)
print("%s: %d bytes  SW2/SW3 = %04X%s  SW1 = 00%02X  SW10 should be 0000"
      % (path, size, size & 0xFFFF, "." if size > 0xFFFF else "", x))
