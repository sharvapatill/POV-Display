"""Build a small sparse FAT32 disk image for tb_basys3_top.v.

Only the blocks that are not all-zero are written, as two $readmemh files:
  lbas.hex    one 32-bit LBA per line
  blocks.hex  512 bytes per LBA, one byte per line, same order
plus
  file.hex    the expected VIDEO.BIN contents, one byte per line
  expect.hex  part_start, first_cluster, file_size, xor checksum

Layout (deliberately awkward to exercise fat32_locate / fat32_reader):
  - MBR at LBA 0, partition 1 (type 0x0C) at LBA 2048
  - 512 B/sector, 2 sectors/cluster, 32 reserved sectors, 2 FATs x 2000 sectors
  - root directory is fragmented: cluster 2 -> cluster 5
  - cluster 2 is full of decoys (volume label, a DIRECTORY named VIDEO.BIN,
    a deleted VIDEO.BIN, LFN entries, other files); the real VIDEO.BIN is
    in cluster 5
  - VIDEO.BIN first cluster is 70000 (needs the high 16-bit word), and its
    chain jumps around across FAT sectors: 70000 -> 70001 -> 10 -> 200 -> 201 -> 130
  - file size is not a multiple of 512, and the slack after it is 0xEE so
    any slack byte leaking into the stream changes the checksum
"""
import os
import random
import struct
import sys

out = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))

PART_START = 2048
SPC = 2
RSVD = 32
NFATS = 2
FATSZ = 2000
ROOT = 2
FAT_START = PART_START + RSVD
DATA_START = FAT_START + NFATS * FATSZ
EOC = 0x0FFFFFFF

blocks = {}


def put(lba, data):
    assert len(data) == 512
    blocks[lba] = bytearray(data)


def clus_lba(c):
    return DATA_START + (c - 2) * SPC


def dirent(name11, attr, clus, size, first=None):
    assert len(name11) == 11, name11
    e = bytearray(32)
    e[0:11] = name11.encode()
    if first is not None:
        e[0] = first
    e[11] = attr
    struct.pack_into("<H", e, 20, clus >> 16)
    struct.pack_into("<H", e, 26, clus & 0xFFFF)
    struct.pack_into("<I", e, 28, size)
    return e


def lfn(seq):
    e = bytearray(b"\xFF" * 32)
    e[0] = seq
    e[11] = 0x0F
    e[12] = 0
    e[26:28] = b"\x00\x00"
    return e


# ---------------- MBR ----------------
mbr = bytearray(512)
mbr[0:2] = b"\x33\xC0"                   # typical MBR code start (not EB/E9)
mbr[0x1BE + 4] = 0x0C
struct.pack_into("<I", mbr, 0x1BE + 8, PART_START)
struct.pack_into("<I", mbr, 0x1BE + 12, 4_000_000)
mbr[510:512] = b"\x55\xAA"
put(0, mbr)

# ---------------- VBR ----------------
vbr = bytearray(512)
vbr[0:3] = b"\xEB\x58\x90"
vbr[3:11] = b"MSWIN4.1"
struct.pack_into("<H", vbr, 11, 512)
vbr[13] = SPC
struct.pack_into("<H", vbr, 14, RSVD)
vbr[16] = NFATS
vbr[21] = 0xF8
struct.pack_into("<I", vbr, 32, 4_000_000)
struct.pack_into("<I", vbr, 36, FATSZ)
struct.pack_into("<I", vbr, 44, ROOT)
vbr[82:90] = b"FAT32   "
vbr[510:512] = b"\x55\xAA"
put(PART_START, vbr)

# ---------------- file contents ----------------
random.seed(3337)
FILE_SIZE = 5 * 1024 + 300
data = bytes(random.randrange(256) for _ in range(FILE_SIZE))
chain = [70000, 70001, 10, 200, 201, 130]
assert len(chain) == -(-FILE_SIZE // (SPC * 512))

# ---------------- FAT (both copies) ----------------
fat = {0: 0x0FFFFFF8, 1: EOC, ROOT: 5, 5: EOC}
for a, b in zip(chain, chain[1:]):
    fat[a] = b
fat[chain[-1]] = EOC
fat[300] = 301                            # unrelated file, just noise
fat[301] = EOC
for copy in range(NFATS):
    base = FAT_START + copy * FATSZ
    for sec in sorted({c // 128 for c in fat}):
        b = bytearray(512)
        for c, v in fat.items():
            if c // 128 == sec:
                struct.pack_into("<I", b, (c % 128) * 4, v)
        put(base + sec, b)

# ---------------- root directory ----------------
root1 = bytearray()
root1 += dirent("POVDISK    ", 0x08, 0, 0)            # volume label
root1 += lfn(0x41)                                      # LFN fragment
root1 += dirent("VIDEO   BIN", 0x10, 300, 0)           # a DIRECTORY with the same name
root1 += dirent("VIDEO   BIN", 0x20, 300, 999, first=0xE5)   # deleted file
root1 += dirent("VIDEO   BIX", 0x20, 300, 999)         # near miss
root1 += dirent("XIDEO   BIN", 0x20, 300, 999)         # near miss
i = 0
while len(root1) < SPC * 512:
    root1 += dirent("FILE%04dTXT" % i, 0x20, 300, 100)
    i += 1
for s in range(SPC):
    put(clus_lba(ROOT) + s, root1[s * 512:(s + 1) * 512])
assert len(root1) == SPC * 512

root2 = bytearray()
root2 += lfn(0x42)
root2 += lfn(0x01)
root2 += dirent("VIDEO   BIN", 0x20, chain[0], FILE_SIZE)
root2 += bytes(32)                                       # end of directory
root2 += dirent("NEVERSEEBIN", 0x20, 300, 1)            # after the end marker
root2 += bytes(SPC * 512 - len(root2))
for s in range(SPC):
    put(clus_lba(5) + s, root2[s * 512:(s + 1) * 512])

# ---------------- file data clusters ----------------
padded = data + b"\xEE" * (len(chain) * SPC * 512 - FILE_SIZE)
for n, c in enumerate(chain):
    for s in range(SPC):
        off = (n * SPC + s) * 512
        put(clus_lba(c) + s, padded[off:off + 512])

# ---------------- write hex files ----------------
xor = 0
for b in data:
    xor ^= b
lbas = sorted(blocks)
MAXB = 64                                   # must match tb_basys3_top.v
assert len(lbas) <= MAXB
# padded to the testbench array sizes; LBA ffffffff marks the end
with open(os.path.join(out, "lbas.hex"), "w") as f:
    f.writelines("%08x\n" % l for l in lbas)
    f.writelines("ffffffff\n" for _ in range(MAXB - len(lbas)))
with open(os.path.join(out, "blocks.hex"), "w") as f:
    for l in lbas:
        f.writelines("%02x\n" % b for b in blocks[l])
    f.writelines("00\n" for _ in range((MAXB - len(lbas)) * 512))
with open(os.path.join(out, "file.hex"), "w") as f:
    f.writelines("%02x\n" % b for b in data)
    f.writelines("00\n" for _ in range(65536 - len(data)))
with open(os.path.join(out, "expect.hex"), "w") as f:
    f.write("%08x\n%08x\n%08x\n%08x\n" % (PART_START, chain[0], FILE_SIZE, xor))
print("blocks=%d part_start=%d first_cluster=%d file_size=%d xor=%02x data_start=%d"
      % (len(lbas), PART_START, chain[0], FILE_SIZE, xor, DATA_START))
