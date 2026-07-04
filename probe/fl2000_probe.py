#!/usr/bin/env python3
"""Userspace probe for the Fresco Logic FL2000DX (1d5c:2000).

Talks to the chip with vendor control transfers on EP0 — no kernel driver
needed. Protocol extracted from the official GPL reference driver; see
notes/PROTOCOL.md.

Usage:  sudo python3 fl2000_probe.py [dump|detect|edid|all]
"""

import sys
import time

import usb.core

VID, PID = 0x1D5C, 0x2000

REQ_REG_READ = 64
REQ_REG_WRITE = 65
RT_READ = 0xC0   # DIR_IN | TYPE_VENDOR | RECIP_DEVICE
RT_WRITE = 0x40  # DIR_OUT | TYPE_VENDOR | RECIP_DEVICE

REG_I2C_CTRL = 0x8020
REG_I2C_RDATA = 0x8024
REG_I2C_WDATA = 0x8028

I2C_ADDR_ITE = 0x4C   # IT66121 HDMI transmitter on HDMI dongles
I2C_ADDR_DDC = 0x50   # monitor EDID (VGA dongles wire DDC straight through)

ITE_VENDOR_ID = 0x4954  # "IT"
ITE_DEVICE_ID = 0x612

# Registers the reference driver touches, for the dump command.
VGA_REGS = [0x0070, 0x0078] + [
    off for off in range(0x8000, 0x808C, 4) if off not in (0x8060, 0x8068, 0x806C, 0x8080, 0x8084)
]


class FL2000:
    def __init__(self):
        self.dev = usb.core.find(idVendor=VID, idProduct=PID)
        if self.dev is None:
            sys.exit("FL2000 (1d5c:2000) not found — is it plugged in?")

    def reg_read(self, offset: int) -> int:
        data = self.dev.ctrl_transfer(RT_READ, REQ_REG_READ, 0, offset, 4, timeout=2000)
        return int.from_bytes(bytes(data), "little")

    def reg_write(self, offset: int, value: int) -> None:
        self.dev.ctrl_transfer(
            RT_WRITE, REQ_REG_WRITE, 0, offset, value.to_bytes(4, "little"), timeout=2000
        )

    def i2c_op(self, addr: int, offset: int, read: bool = True, data: int | None = None):
        """One 4-byte I2C transaction. Returns (data_or_None, per_byte_status)."""
        if not read:
            assert data is not None
            self.reg_write(REG_I2C_WDATA, data)

        ctrl = self.reg_read(REG_I2C_CTRL)
        ctrl |= 0x10000000        # bit 28 reads as 0 but must be written 1 (hw quirk)
        ctrl &= ~0x8003FFFF       # clear OpStatus, SpiErase, IsSpi, offset, RW, Addr
        ctrl |= (addr & 0x7F) | ((1 if read else 0) << 7) | ((offset & 0xFF) << 8)
        self.reg_write(REG_I2C_CTRL, ctrl)

        time.sleep(0.003)
        for _ in range(10):
            status = self.reg_read(REG_I2C_CTRL)
            if status & 0x80000000:
                byte_status = (status >> 24) & 0xF
                value = self.reg_read(REG_I2C_RDATA) if read else None
                return value, byte_status
            time.sleep(0.01)
        raise TimeoutError(f"i2c op addr=0x{addr:02X} offset=0x{offset:02X} never completed")

    def i2c_read32(self, addr: int, offset: int):
        return self.i2c_op(addr, offset, read=True)


class IT66121:
    """ITE IT66121 HDMI transmitter behind the FL2000 I2C master.

    ITE registers are byte-wide; the FL2000 I2C engine moves 4 bytes per
    transaction, so byte access = read-modify-write on the aligned dword.
    """

    RESET_DELAY = 0.3

    def __init__(self, fl: FL2000):
        self.fl = fl

    def read_byte(self, off: int) -> int:
        rem, al = off % 4, off & ~3
        dw, st = self.fl.i2c_read32(I2C_ADDR_ITE, al)
        if st:
            raise IOError(f"ite read 0x{off:02X}: byte_status=0x{st:X}")
        return (dw >> (rem * 8)) & 0xFF

    def write_byte(self, off: int, val: int) -> None:
        rem, al = off % 4, off & ~3
        dw, st = self.fl.i2c_read32(I2C_ADDR_ITE, al)
        if st:
            raise IOError(f"ite rmw-read 0x{off:02X}: byte_status=0x{st:X}")
        dw = (dw & ~(0xFF << (rem * 8))) | ((val & 0xFF) << (rem * 8))
        _, st = self.fl.i2c_op(I2C_ADDR_ITE, al, read=False, data=dw)
        if st:
            raise IOError(f"ite write 0x{off:02X}: byte_status=0x{st:X}")

    def write_dword(self, off: int, val: int) -> None:
        _, st = self.fl.i2c_op(I2C_ADDR_ITE, off, read=False, data=val)
        if st:
            raise IOError(f"ite write32 0x{off:02X}: byte_status=0x{st:X}")

    def write_masked(self, off: int, mask: int, val: int) -> None:
        cur = self.read_byte(off)
        self.write_byte(off, (cur & ~mask) | (val & mask))

    def bit_set(self, off: int, bit: int) -> None:
        self.write_byte(off, self.read_byte(off) | (1 << bit))

    def reset(self) -> None:
        self.bit_set(0x04, 5)          # SW_RST
        time.sleep(self.RESET_DELAY)

    # {reg, mask, value} table from fl2000_hdmi_power_up()
    POWER_UP = [
        (0x0F, 0x78, 0x38),  # PwrOn GRCLK
        (0x05, 0x01, 0x00),  # PwrOn PCLK
        (0x61, 0x20, 0x00),  # PwrOn DRV
        (0x62, 0x44, 0x00),  # PwrOn XPLL
        (0x64, 0x40, 0x00),  # PwrOn IPLL
        (0x61, 0x10, 0x00),  # DRV_RST off
        (0x62, 0x08, 0x08),  # XP_RESETB
        (0x64, 0x04, 0x04),  # IP_RESETB
        (0x6A, 0xFF, 0x70),
        (0x66, 0xFF, 0x1F),
        (0x63, 0xFF, 0x38),
        (0x0F, 0x78, 0x08),  # PwrOn IACLK
    ]

    def power_up(self) -> None:
        for reg, mask, val in self.POWER_UP:
            if mask == 0xFF:
                self.write_byte(reg, val)
            else:
                self.write_masked(reg, mask, val)

    def switch_bank(self, bank: int) -> None:
        self.write_byte(0x0F, bank & 1)

    def hpd(self) -> tuple[bool, bool]:
        st = self.read_byte(0x0E)      # SYS_STATUS
        return bool(st & 0x40), bool(st & 0x20)  # HPD, RxSense

    def clear_ddc_fifo(self) -> None:
        self.write_byte(0x10, 0x01)    # DDC master = host
        self.write_byte(0x15, 0x09)    # CMD_FIFO_CLR

    def abort_ddc(self) -> None:
        sw_rst = self.read_byte(0x04)
        cp = self.read_byte(0x20) & ~0x01          # clear CPDesire
        self.write_byte(0x20, cp)
        self.write_byte(0x04, sw_rst | 0x01)       # HDCP reset
        self.write_byte(0x10, 0x01)
        for _ in range(2):
            self.write_byte(0x15, 0x0F)            # CMD_DDC_ABORT
            for _ in range(200):
                st = self.read_byte(0x16)
                if st & 0x80 or st & 0x38:         # done or error bits
                    break
                time.sleep(0.05)

    def read_edid_chunk(self, segment: int, offset: int, count: int) -> bytes:
        """One DDC FIFO transaction. Returns count-3 bytes starting at
        EDID[offset+3] — the first 3 bytes of every transaction are lost
        to the FL2000's 4-byte-aligned FIFO reads (reference driver quirk).
        """
        # Twice, per ITE: master-host select + DDC abort, to avoid FIFO lockup
        for _ in range(2):
            self.bit_set(0x10, 0)
            self.write_byte(0x15, 0x0F)
        self.write_byte(0x15, 0x09)    # FIFO clear
        # One dword programs regs 0x10..0x13: master=host, header=0xA0(EDID),
        # request offset, request count.
        self.write_dword(0x10, 0xA001 | ((offset & 0xFF) << 16) | (count << 24))
        # One dword programs regs 0x14..0x17: segment, CMD=3 (EDID read).
        self.write_dword(
            0x14, ((offset & 0xFF) << 16) | ((count & 0xFF) << 24) | 0x0300 | segment
        )
        time.sleep(0.01)
        out = bytearray()
        for _ in range(count - 3):
            out.append(self.read_byte(0x17))       # FIFO pop
        return bytes(out)

    def read_edid_block(self, block_id: int) -> bytes:
        """128-byte EDID block via the reference driver's 4x32-byte rounds
        with the leading-3-bytes patch dance."""
        blk = bytearray(128)
        seg, seg_off = block_id // 2, (block_id % 2) * 128
        int_stat = self.read_byte(0x06)
        if int_stat & 0x04:                        # DDC bus hang
            self.abort_ddc()
        self.clear_ddc_fifo()
        self.switch_bank(0)
        for rnd in range(4):
            off = 32 * rnd
            if rnd == 0 and block_id == 0:
                blk[0:3] = b"\x00\xff\xff"         # constant EDID header bytes
            else:
                head = self.read_edid_chunk(seg, seg_off + off - 3, 6)
                blk[off:off + 3] = head[:3]
            body = self.read_edid_chunk(seg, seg_off + off, 32)
            blk[off + 3:off + 32] = body[:29]
        return bytes(blk)


def cmd_dump(fl: FL2000) -> None:
    print("== register dump ==")
    for off in VGA_REGS:
        val = fl.reg_read(off)
        print(f"  [0x{off:04X}] = 0x{val:08X}")


def cmd_detect(fl: FL2000) -> None:
    print("== ITE IT66121 HDMI transmitter detect (i2c 0x4C) ==")
    try:
        dword, st = fl.i2c_read32(I2C_ADDR_ITE, 0)
    except TimeoutError as exc:
        print(f"  i2c timeout: {exc}")
        return
    vendor = dword & 0xFFFF
    device = (dword >> 16) & 0xFFF
    print(f"  raw=0x{dword:08X} byte_status=0x{st:X} → vendor=0x{vendor:04X} device=0x{device:03X}")
    if vendor == ITE_VENDOR_ID and device == ITE_DEVICE_ID:
        rev = (dword >> 28) & 0xF
        print(f"  ✔ IT66121 found (revision {rev})")
    elif st != 0:
        print("  ✘ no ACK from 0x4C — probably not an ITE-based dongle")
    else:
        print("  ✘ unexpected ID — some other transmitter chip?")


def parse_edid(edid: bytes) -> None:
    mfg = int.from_bytes(edid[8:10], "big")
    name = "".join(chr(((mfg >> s) & 0x1F) + ord("A") - 1) for s in (10, 5, 0))
    product = int.from_bytes(edid[10:12], "little")
    week, year = edid[16], 1990 + edid[17]
    print(f"  manufacturer={name} product=0x{product:04X} made week {week}/{year}")
    for d in range(54, 126, 18):
        desc = edid[d:d + 18]
        if desc[0] == desc[1] == 0:
            if desc[3] == 0xFC:
                print(f"  monitor name: {desc[5:18].decode(errors='replace').strip()}")
        else:
            pixclk = int.from_bytes(desc[0:2], "little") * 10_000
            hact = desc[2] | ((desc[4] & 0xF0) << 4)
            vact = desc[5] | ((desc[7] & 0xF0) << 4)
            hblank = desc[3] | ((desc[4] & 0x0F) << 8)
            vblank = desc[6] | ((desc[7] & 0x0F) << 8)
            refresh = pixclk / ((hact + hblank) * (vact + vblank)) if hact else 0
            print(f"  detailed timing: {hact}x{vact} @ {refresh:.1f} Hz "
                  f"(pixel clock {pixclk / 1e6:.2f} MHz)")


def cmd_edid(fl: FL2000) -> None:
    print("== EDID via IT66121 DDC master ==")
    ite = IT66121(fl)
    dword, st = fl.i2c_read32(I2C_ADDR_ITE, 0)
    if st or (dword & 0xFFFF) != ITE_VENDOR_ID:
        print("  no IT66121 — trying direct DDC at 0x50 (VGA wiring)")
        d0, st0 = fl.i2c_read32(I2C_ADDR_DDC, 0)
        print(f"  dword0=0x{d0:08X} (st=0x{st0:X})")
        return

    print("  resetting + powering up IT66121…")
    ite.reset()
    ite.power_up()
    hpd, rxsense = ite.hpd()
    print(f"  HPD={'yes' if hpd else 'NO'} RxSense={'yes' if rxsense else 'no'}")
    if not hpd:
        print("  ✘ no monitor detected on the HDMI connector — is it plugged in?")
        return

    edid = ite.read_edid_block(0)
    good = sum(edid) % 256 == 0
    print(f"  block 0 checksum: {'ok' if good else 'BAD'}")
    blocks = [edid]
    if good and edid[126] and edid[126] <= 7:
        for i in range(1, edid[126] + 1):
            ext = ite.read_edid_block(i)
            print(f"  block {i} checksum: {'ok' if sum(ext) % 256 == 0 else 'BAD'}")
            blocks.append(ext)
    path = "edid.bin"
    with open(path, "wb") as fh:
        fh.write(b"".join(blocks))
    print(f"  wrote {path} ({len(blocks) * 128} bytes)")
    if good:
        parse_edid(edid)


def main() -> None:
    what = sys.argv[1] if len(sys.argv) > 1 else "all"
    fl = FL2000()
    print(f"FL2000 opened (bus {fl.dev.bus} addr {fl.dev.address}, "
          f"speed={'SuperSpeed' if fl.dev.speed == 4 else 'HighSpeed' if fl.dev.speed == 3 else fl.dev.speed})")
    if what in ("dump", "all"):
        cmd_dump(fl)
    if what in ("detect", "all"):
        cmd_detect(fl)
    if what in ("edid", "all"):
        cmd_edid(fl)


if __name__ == "__main__":
    main()
