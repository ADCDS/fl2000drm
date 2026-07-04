#!/usr/bin/env python3
"""First-light display test for FL2000DX + IT66121 HDMI dongle.

Programs a video mode from userspace and streams RGB565 test-pattern frames
over the bulk endpoint. Sequences reproduced from the official GPL reference
driver (see notes/PROTOCOL.md).

Usage:  sudo python3 fl2000_display.py [--mode 640|800] [--dvi] [--static] [--seconds N]
"""

import argparse
import sys
import time

import os

import usb.core
import usb.util

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fl2000_probe import FL2000, IT66121, I2C_ADDR_ITE, ITE_VENDOR_ID

BULK_EP = 0x01


class Mode:
    def __init__(self, width, height, freq, h1, h2, v1, v2, pll, vic, pixclk):
        self.width, self.height, self.freq = width, height, freq
        self.h_sync_reg_1, self.h_sync_reg_2 = h1, h2
        self.v_sync_reg_1, self.v_sync_reg_2 = v1, v2
        self.pll, self.vic, self.pixclk = pll, vic, pixclk


# From big_table_16bit_r0 with fl2000_hdmi_compliance_tweak applied (640x480).
MODES = {
    "640": Mode(640, 480, 60, 0x2800320, 0x600091, 0x1E0020D, 0x2420024,
                0x003F6119, vic=1, pixclk=800 * 525 * 60),
    "800": Mode(800, 600, 60, 0x3200420, 0x8000D9, 0x2580274, 0x1C4001C,
                0x00080102, vic=0, pixclk=1056 * 628 * 60),
}


def reg_bit_set(fl, off, bit):
    fl.reg_write(off, fl.reg_read(off) | (1 << bit))


def reg_bit_clear(fl, off, bit):
    fl.reg_write(off, fl.reg_read(off) & ~(1 << bit))


def fl2000_set_mode(fl: FL2000, mode: Mode) -> None:
    """fl2000_monitor_set_resolution() for bulk + RGB565 + ZLP end-of-frame."""
    fl.reg_write(0x802C, mode.pll)
    reg_bit_set(fl, 0x8048, 15)                  # app reset, self-clearing
    if fl.reg_read(0x802C) != mode.pll:
        raise IOError("PLL readback mismatch")

    for bit in (22, 24, 19, 21, 13, 27, 28, 29):  # BIA/isoch/EOF-type off
        reg_bit_clear(fl, 0x803C, bit)
    reg_bit_set(fl, 0x803C, 28)                  # EOF = zero-length packet

    for bit in (28, 6, 31, 24, 25, 26, 27):      # clean color/compression cfg
        reg_bit_clear(fl, 0x8004, bit)
    reg_bit_set(fl, 0x8004, 0)                   # reset VGA CCS
    reg_bit_set(fl, 0x8004, 6)                   # RGB565 output
    reg_bit_set(fl, 0x8004, 7)                   # enable external DAC (→ ITE)

    for off, val in ((0x8008, mode.h_sync_reg_1), (0x800C, mode.h_sync_reg_2),
                     (0x8010, mode.v_sync_reg_1), (0x8014, mode.v_sync_reg_2)):
        fl.reg_write(off, val)
        if fl.reg_read(off) != val:
            raise IOError(f"timing reg 0x{off:04X} readback mismatch")

    fl.reg_write(0x801C, fl.reg_read(0x801C) & 0xC000FFFF)   # clear iso cfg
    reg_bit_set(fl, 0x0070, 13)
    reg_bit_clear(fl, 0x8088, 10)                # from fl2000_dongle_init_fl2000dx


def ite_av_mute(ite: IT66121, mute: int) -> None:
    ite.switch_bank(0)
    ite.write_masked(0xC1, 0x01, mute)           # GCP.SetAVMute
    ite.write_byte(0xC6, 0x03)                   # general pkt: enable+repeat


def ite_send_avi_infoframe(ite: IT66121, mode: Mode) -> None:
    db = [0] * 13
    db[0] = (0 << 5) | (1 << 4)                  # RGB, active-format present
    if mode.vic in (4, 16):                      # 720p/1080p → 16:9, ITU709
        db[1] = 8 | (2 << 4) | (2 << 6)
    else:                                        # 4:3, ITU601
        db[1] = 8 | (1 << 4) | (1 << 6)
    db[3] = mode.vic
    checksum = (0x100 - sum(db) - (0x82 + 0x02 + 13)) & 0xFF

    ite.switch_bank(1)
    ite.write_dword(0x58, db[0] | (db[1] << 8) | (db[2] << 16) | (db[3] << 24))
    ite.write_dword(0x5C, db[4] | (checksum << 8) | (db[5] << 16) | (db[6] << 24))
    ite.write_dword(0x60, db[7] | (db[8] << 8) | (db[9] << 16) | (db[10] << 24))
    ite.write_dword(0x64, db[11] | (db[12] << 8))
    ite.switch_bank(0)
    ite.write_byte(0xCD, 0x03)                   # AVI infoframe: enable+repeat


def ite_enable_video(ite: IT66121, mode: Mode, dvi: bool) -> None:
    """fl2000_hdmi_enable_video_output()"""
    level_high = mode.pixclk > 80_000_000

    ite.write_byte(0x04, 0x09)                   # SW_RST: VID_RST | HDCP_RST
    # input mode: RGB, single-edge, separate syncs (+bit0 per reference)
    cur = ite.read_byte(0x70)
    cur &= ~((3 << 6) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 5))
    ite.write_byte(0x70, cur | 0x01)
    # CSC bypass
    ite.write_masked(0x0F, 0x10, 0x10)
    cur = ite.read_byte(0x72)
    cur &= ~(3 | (1 << 5) | (1 << 7) | (1 << 6))
    ite.write_byte(0x72, cur)
    ite.write_byte(0xC0, 0 if dvi else 1)        # DVI / HDMI mode
    # AFE setup by pixel clock band
    ite.write_byte(0x61, 0x10)                   # AFE_DRV_RST
    if level_high:
        ite.write_masked(0x62, 0x90, 0x80)
        ite.write_masked(0x64, 0x89, 0x80)
        ite.write_masked(0x68, 0x10, 0x80)
    else:
        ite.write_masked(0x62, 0x90, 0x10)
        ite.write_masked(0x64, 0x89, 0x09)
        ite.write_masked(0x68, 0x10, 0x10)
    ite.write_masked(0x04, 0x28, 0x00)           # release REF_RST | VID_RST
    ite.write_byte(0x61, 0x00)
    ite.write_byte(0x04, 0x01)                   # keep only HDCP_RST
    ite.switch_bank(0)                           # fire AFE
    ite.write_byte(0x61, 0x00)


def make_bars_rgb565(width: int, height: int) -> bytes:
    colors = [0xFFFF, 0xFFE0, 0x07FF, 0x07E0, 0xF81F, 0xF800, 0x001F, 0x0000]
    bar_w = width // len(colors)
    row = bytearray()
    for x in range(width):
        c = colors[min(x // bar_w, len(colors) - 1)]
        row += c.to_bytes(2, "little")
    return bytes(row) * height


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=MODES, default="640")
    ap.add_argument("--dvi", action="store_true", help="DVI mode instead of HDMI")
    ap.add_argument("--static", action="store_true", help="no scrolling")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()
    mode = MODES[args.mode]

    fl = FL2000()
    ite = IT66121(fl)
    print(f"device open; setting {mode.width}x{mode.height}@{mode.freq} RGB565")

    dword, st = fl.i2c_read32(I2C_ADDR_ITE, 0)
    if st or (dword & 0xFFFF) != ITE_VENDOR_ID:
        sys.exit("IT66121 not responding")

    # dongle + transmitter bring-up (fl2000_dongle_card_initialize order)
    reg_bit_set(fl, 0x8048, 15)                  # dongle reset
    ite.reset()
    ite.power_up()
    hpd, _ = ite.hpd()
    print(f"  HPD={'yes' if hpd else 'NO'}")

    # monitor-connect prep (fl2000_monitor_connect)
    reg_bit_clear(fl, 0x0078, 17)
    reg_bit_set(fl, 0x0070, 19)                  # reject U2
    reg_bit_set(fl, 0x0070, 20)                  # reject U1

    fl2000_set_mode(fl, mode)
    print("  FL2000 mode registers programmed")

    ite_av_mute(ite, 1)
    ite_send_avi_infoframe(ite, mode)
    ite_enable_video(ite, mode, args.dvi)
    ite_av_mute(ite, 0)
    print(f"  IT66121 video output enabled ({'DVI' if args.dvi else 'HDMI'})")

    usb.util.claim_interface(fl.dev, 0)
    fl.dev.set_interface_altsetting(0, 1)        # alt 1 exposes bulk EP
    print("  interface 0 alt 1 selected; streaming…")

    frame = bytearray(make_bars_rgb565(mode.width, mode.height))
    frame_bytes = len(frame)
    deadline = time.monotonic() + args.seconds
    sent = 0
    t0 = time.monotonic()
    try:
        while time.monotonic() < deadline:
            if not args.static:
                shift = (sent * 8) % mode.width * 2
                view = bytes(frame[shift:]) + bytes(frame[:shift])
            else:
                view = bytes(frame)
            fl.dev.write(BULK_EP, view, timeout=2000)
            fl.dev.write(BULK_EP, b"", timeout=2000)   # ZLP = end of frame
            sent += 1
    except KeyboardInterrupt:
        pass
    finally:
        dt = time.monotonic() - t0
        if sent:
            mbps = sent * frame_bytes / dt / 1e6
            print(f"  sent {sent} frames in {dt:.1f}s → {sent / dt:.1f} fps "
                  f"({mbps:.1f} MB/s)")
        usb.util.release_interface(fl.dev, 0)


if __name__ == "__main__":
    main()
