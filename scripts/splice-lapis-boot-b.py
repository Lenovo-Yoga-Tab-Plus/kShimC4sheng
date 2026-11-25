#!/usr/bin/env python3
"""Install a packed lapis menu into stock boot_b without moving its signature.

boot_b is a 96 MiB partition. The signed vbmeta starts at the footer's
original-image size, and the AVB footer is the last 64 bytes. The packed
kernel is larger than that prefix, so this copies only the patched kernel
header into the signed region and places the shim, Linux payload and
manifest after the vbmeta. The vbmeta blob and the footer stay byte-identical
to the stock image.

The packed kernel must have been built with its shim slot at --shim-offset
(the default matches a stock boot_b whose vbmeta begins at 0x21d0000). Raise
the stock kernel header image_size to that offset before running make-boot.sh
so the packer emits the shim there.
"""
import argparse
import struct
import sys

PAGE = 4096
VBMETA_OFF = 0x21D0000
VBMETA_SIZE = 0x9C0
SHIM_OFFSET = 0x21D1000


def die(msg):
    sys.exit("splice-lapis-boot-b: " + msg)


def be64(buf, off):
    return int.from_bytes(buf[off:off + 8], "big")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stock-boot", required=True)
    ap.add_argument("--packed", required=True, help="packed-kernel.bin from make-boot.sh")
    ap.add_argument("--out", required=True)
    ap.add_argument("--shim-offset", type=lambda s: int(s, 0), default=SHIM_OFFSET)
    args = ap.parse_args()

    boot = bytearray(open(args.stock_boot, "rb").read())
    packed = open(args.packed, "rb").read()
    if boot[:8] != b"ANDROID!" or len(boot) < PAGE + 64:
        die("stock boot is not an Android boot image")
    footer = boot[-64:]
    if footer[:4] != b"AVBf":
        die("stock boot has no AVB footer")
    orig = be64(footer, 12)
    voff = be64(footer, 20)
    vsz = be64(footer, 28)
    if orig != VBMETA_OFF or voff != VBMETA_OFF or vsz != VBMETA_SIZE:
        die("unexpected footer: image %#x vbmeta %#x size %#x" % (orig, voff, vsz))
    if boot[voff:voff + 4] != b"AVB0":
        die("vbmeta magic is missing at %#x" % voff)

    shim_off = struct.unpack_from("<Q", packed, 0x30)[0]
    if shim_off != args.shim_offset:
        die("packed shim is at %#x, want %#x" % (shim_off, args.shim_offset))
    file_shim = PAGE + shim_off
    if file_shim < voff + vsz:
        die("shim overlaps the vbmeta")
    if PAGE + len(packed) > len(boot) - 64:
        die("packed kernel reaches the footer")

    out = bytearray(boot)
    out[PAGE:PAGE + 0x40] = packed[:0x40]
    out[file_shim:file_shim + (len(packed) - shim_off)] = packed[shim_off:]
    struct.pack_into("<I", out, 8, len(packed))

    allowed = set(range(8, 12)) | set(range(PAGE, PAGE + 0x40))
    for i in range(orig):
        if out[i] != boot[i] and i not in allowed:
            die("signed prefix changed at %#x" % i)
    if out[voff:voff + vsz] != boot[voff:voff + vsz] or out[-64:] != boot[-64:]:
        die("vbmeta or footer changed")

    word = struct.unpack_from("<I", out, PAGE)[0]
    if (word & 0xFC000000) != 0x94000000:
        die("kernel entry is not a BL")
    imm = word & 0x03FFFFFF
    if imm & 0x02000000:
        imm -= 0x04000000
    if imm * 4 != shim_off:
        die("BL target %#x does not match the shim" % (imm * 4))

    open(args.out, "wb").write(out)
    print("wrote %s (%d bytes), shim at %#x" % (args.out, len(out), shim_off))


if __name__ == "__main__":
    main()
