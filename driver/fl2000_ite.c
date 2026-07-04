// SPDX-License-Identifier: GPL-2.0
/*
 * ITE IT66121 HDMI transmitter support, as wired behind the FL2000 I2C
 * master on HDMI dongles. Sequences ported from the reference driver's
 * fl2000_hdmi.c and validated on hardware from userspace first.
 *
 * The IT66121 has byte-wide registers; the FL2000 I2C engine moves 4 bytes
 * per transaction, so byte access is read-modify-write on aligned dwords.
 * A hardware quirk loses the first 3 bytes of every DDC FIFO transaction;
 * the EDID reader compensates the same way the reference driver does.
 */
#include <linux/delay.h>

#include "fl2000_drm.h"

#define ITE_ADDR		0x4C
#define ITE_VENDOR_ID		0x4954	/* "IT" */
#define ITE_DEVICE_ID		0x612

static int ite_read_byte(struct fl2000 *fl, u8 off, u8 *val)
{
	u32 dw;
	int ret;

	ret = fl2000_i2c_read32(fl, ITE_ADDR, off & ~3, &dw);
	if (ret)
		return ret;
	*val = dw >> ((off & 3) * 8);
	return 0;
}

static int ite_write_byte(struct fl2000 *fl, u8 off, u8 val)
{
	u32 dw;
	int shift = (off & 3) * 8;
	int ret;

	ret = fl2000_i2c_read32(fl, ITE_ADDR, off & ~3, &dw);
	if (ret)
		return ret;
	dw &= ~(0xFF << shift);
	dw |= val << shift;
	return fl2000_i2c_write32(fl, ITE_ADDR, off & ~3, dw);
}

static int ite_write_masked(struct fl2000 *fl, u8 off, u8 mask, u8 val)
{
	u8 cur;
	int ret;

	ret = ite_read_byte(fl, off, &cur);
	if (ret)
		return ret;
	return ite_write_byte(fl, off, (cur & ~mask) | (val & mask));
}

bool fl2000_ite_detect(struct fl2000 *fl)
{
	u32 id;

	if (fl2000_i2c_read32(fl, ITE_ADDR, 0, &id))
		return false;
	return (id & 0xFFFF) == ITE_VENDOR_ID &&
	       ((id >> 16) & 0xFFF) == ITE_DEVICE_ID;
}

/* {reg, mask, value} rows from fl2000_hdmi_power_up() */
static const u8 ite_power_up_seq[][3] = {
	{ 0x0F, 0x78, 0x38 },	/* power on GRCLK */
	{ 0x05, 0x01, 0x00 },	/* power on PCLK */
	{ 0x61, 0x20, 0x00 },	/* power on DRV */
	{ 0x62, 0x44, 0x00 },	/* power on XPLL */
	{ 0x64, 0x40, 0x00 },	/* power on IPLL */
	{ 0x61, 0x10, 0x00 },	/* DRV reset off */
	{ 0x62, 0x08, 0x08 },	/* XP_RESETB */
	{ 0x64, 0x04, 0x04 },	/* IP_RESETB */
	{ 0x6A, 0xFF, 0x70 },
	{ 0x66, 0xFF, 0x1F },
	{ 0x63, 0xFF, 0x38 },
	{ 0x0F, 0x78, 0x08 },	/* power on IACLK */
};

int fl2000_ite_init(struct fl2000 *fl)
{
	int i, ret;

	mutex_lock(&fl->ite_lock);

	ret = ite_write_masked(fl, 0x04, BIT(5), BIT(5));	/* SW reset */
	if (ret)
		goto out;
	msleep(300);

	for (i = 0; i < ARRAY_SIZE(ite_power_up_seq); i++) {
		const u8 *e = ite_power_up_seq[i];

		if (e[1] == 0xFF)
			ret = ite_write_byte(fl, e[0], e[2]);
		else
			ret = ite_write_masked(fl, e[0], e[1], e[2]);
		if (ret)
			goto out;
	}
out:
	mutex_unlock(&fl->ite_lock);
	return ret;
}

/* returns 1 = monitor present, 0 = absent, negative = error */
int fl2000_ite_hpd(struct fl2000 *fl)
{
	u8 st;
	int ret;

	mutex_lock(&fl->ite_lock);
	ret = ite_read_byte(fl, 0x0E, &st);
	mutex_unlock(&fl->ite_lock);
	if (ret)
		return ret;
	return !!(st & BIT(6));
}

static int ite_clear_ddc_fifo(struct fl2000 *fl)
{
	int ret;

	ret = ite_write_byte(fl, 0x10, 0x01);	/* DDC master = host */
	if (ret)
		return ret;
	return ite_write_byte(fl, 0x15, 0x09);	/* FIFO clear command */
}

static int ite_abort_ddc(struct fl2000 *fl)
{
	u8 sw_rst, status;
	int i, t, ret;

	ret = ite_read_byte(fl, 0x04, &sw_rst);
	if (ret)
		return ret;
	ret = ite_write_masked(fl, 0x20, BIT(0), 0);	/* CPDesire off */
	if (ret)
		return ret;
	ret = ite_write_byte(fl, 0x04, sw_rst | BIT(0));/* HDCP reset */
	if (ret)
		return ret;
	ret = ite_write_byte(fl, 0x10, 0x01);
	if (ret)
		return ret;

	for (i = 0; i < 2; i++) {
		ret = ite_write_byte(fl, 0x15, 0x0F);	/* DDC abort */
		if (ret)
			return ret;
		for (t = 0; t < 200; t++) {
			ret = ite_read_byte(fl, 0x16, &status);
			if (ret)
				return ret;
			if (status & (BIT(7) | BIT(5) | BIT(4) | BIT(3)))
				break;
			msleep(50);
		}
	}
	return 0;
}

/*
 * One DDC FIFO transaction of @count bytes starting at @offset; only
 * count-3 bytes come back (starting at offset+3) — the leading 3 bytes of
 * every transaction are lost to the 4-byte-aligned FIFO access.
 */
static int ite_read_edid_chunk(struct fl2000 *fl, u8 segment, u8 offset,
			       u8 count, u8 *out)
{
	int i, ret;

	/* twice, per ITE app note, to avoid FIFO lockup */
	for (i = 0; i < 2; i++) {
		ret = ite_write_masked(fl, 0x10, BIT(0), BIT(0));
		if (ret)
			return ret;
		ret = ite_write_byte(fl, 0x15, 0x0F);
		if (ret)
			return ret;
	}
	ret = ite_write_byte(fl, 0x15, 0x09);
	if (ret)
		return ret;

	/* one dword = regs 0x10..0x13: master host, header 0xA0, off, count */
	ret = fl2000_i2c_write32(fl, ITE_ADDR, 0x10,
				 0xA001 | (offset << 16) | (count << 24));
	if (ret)
		return ret;
	/* one dword = regs 0x14..0x17: segment, command 3 = EDID read */
	ret = fl2000_i2c_write32(fl, ITE_ADDR, 0x14,
				 segment | 0x0300 | (offset << 16) |
				 (count << 24));
	if (ret)
		return ret;

	/* DDC engine fetches up to 32 bytes at 100 kHz: ~4 ms */
	usleep_range(4000, 5000);

	for (i = 0; i < count - 3; i++) {
		ret = ite_read_byte(fl, 0x17, &out[i]);	/* FIFO pop */
		if (ret)
			return ret;
	}
	return 0;
}

static int ite_read_edid_block(struct fl2000 *fl, u8 block, u8 *buf)
{
	u8 seg = block / 2;
	u8 seg_off = (block % 2) * 128;
	u8 tmp[32];
	u8 int_stat;
	int rnd, ret;

	ret = ite_read_byte(fl, 0x06, &int_stat);
	if (ret)
		return ret;
	if (int_stat & BIT(2)) {	/* DDC bus hang */
		ret = ite_abort_ddc(fl);
		if (ret)
			return ret;
	}
	ret = ite_clear_ddc_fifo(fl);
	if (ret)
		return ret;
	ret = ite_write_byte(fl, 0x0F, 0);	/* bank 0 */
	if (ret)
		return ret;

	for (rnd = 0; rnd < 4; rnd++) {
		u8 off = 32 * rnd;

		/* first 3 bytes of each 32-byte round */
		if (rnd == 0 && block == 0) {
			buf[0] = 0x00;	/* constant EDID header bytes */
			buf[1] = 0xFF;
			buf[2] = 0xFF;
		} else {
			ret = ite_read_edid_chunk(fl, seg,
						  seg_off + off - 3, 6, tmp);
			if (ret)
				return ret;
			memcpy(buf + off, tmp, 3);
		}

		ret = ite_read_edid_chunk(fl, seg, seg_off + off, 32, tmp);
		if (ret)
			return ret;
		memcpy(buf + off + 3, tmp, 29);
	}
	return 0;
}

/* matches the drm_edid_read_custom() read_block callback signature */
int fl2000_ite_read_edid_block(void *ctx, u8 *buf, unsigned int block,
			       size_t len)
{
	struct fl2000 *fl = ctx;
	u8 full[128];
	int ret;

	if (len > sizeof(full))
		return -EINVAL;

	mutex_lock(&fl->ite_lock);
	ret = ite_read_edid_block(fl, block, full);
	mutex_unlock(&fl->ite_lock);
	if (ret)
		return ret;

	memcpy(buf, full, len);
	return 0;
}

static int ite_av_mute(struct fl2000 *fl, int mute)
{
	int ret;

	ret = ite_write_byte(fl, 0x0F, 0);		/* bank 0 */
	if (ret)
		return ret;
	ret = ite_write_masked(fl, 0xC1, BIT(0), mute);	/* GCP.SetAVMute */
	if (ret)
		return ret;
	return ite_write_byte(fl, 0xC6, 0x03);	/* general pkt on + repeat */
}

static int ite_send_avi_infoframe(struct fl2000 *fl,
				  const struct fl2000_mode *mode)
{
	u8 db[13] = {};
	u8 checksum;
	int i, sum = 0, ret;

	db[0] = BIT(4);				/* RGB, active format valid */
	if (mode->vic == 4 || mode->vic == 16)
		db[1] = 8 | (2 << 4) | (2 << 6);	/* 16:9, ITU709 */
	else
		db[1] = 8 | (1 << 4) | (1 << 6);	/* 4:3, ITU601 */
	if (!mode->vic)
		db[2] = 2 << 2;		/* IT mode: explicit full-range RGB */
	db[3] = mode->vic;

	for (i = 0; i < ARRAY_SIZE(db); i++)
		sum += db[i];
	checksum = 0x100 - ((sum + 0x82 + 0x02 + 13) & 0xFF);

	ret = ite_write_byte(fl, 0x0F, 1);	/* bank 1 */
	if (ret)
		return ret;
	ret = fl2000_i2c_write32(fl, ITE_ADDR, 0x58,
				 db[0] | (db[1] << 8) | (db[2] << 16) |
				 (db[3] << 24));
	if (ret)
		return ret;
	ret = fl2000_i2c_write32(fl, ITE_ADDR, 0x5C,
				 db[4] | (checksum << 8) | (db[5] << 16) |
				 (db[6] << 24));
	if (ret)
		return ret;
	ret = fl2000_i2c_write32(fl, ITE_ADDR, 0x60,
				 db[7] | (db[8] << 8) | (db[9] << 16) |
				 (db[10] << 24));
	if (ret)
		return ret;
	ret = fl2000_i2c_write32(fl, ITE_ADDR, 0x64, db[11] | (db[12] << 8));
	if (ret)
		return ret;
	ret = ite_write_byte(fl, 0x0F, 0);	/* bank 0 */
	if (ret)
		return ret;
	return ite_write_byte(fl, 0xCD, 0x03);	/* AVI pkt on + repeat */
}

static int ite_video_on(struct fl2000 *fl, const struct fl2000_mode *mode)
{
	u32 htotal = mode->h_sync_1 & 0xFFFF;
	u32 vtotal = mode->v_sync_1 & 0xFFFF;
	u64 pixclk = (u64)htotal * vtotal * mode->refresh;
	bool high = pixclk > 80000000ULL;
	u8 val;
	int ret;

	ret = ite_write_byte(fl, 0x04, 0x09);	/* VID_RST | HDCP_RST */
	if (ret)
		return ret;

	/* input mode: RGB, separate sync, single edge */
	ret = ite_read_byte(fl, 0x70, &val);
	if (ret)
		return ret;
	val &= ~((3 << 6) | BIT(5) | BIT(4) | BIT(3) | BIT(2));
	ret = ite_write_byte(fl, 0x70, val | 0x01);
	if (ret)
		return ret;

	/* CSC bypass */
	ret = ite_write_masked(fl, 0x0F, BIT(4), BIT(4));
	if (ret)
		return ret;
	ret = ite_read_byte(fl, 0x72, &val);
	if (ret)
		return ret;
	ret = ite_write_byte(fl, 0x72,
			     val & ~(3 | BIT(5) | BIT(6) | BIT(7)));
	if (ret)
		return ret;

	ret = ite_write_byte(fl, 0xC0, 1);	/* HDMI mode (0 = DVI) */
	if (ret)
		return ret;

	/* AFE config by pixel clock band */
	ret = ite_write_byte(fl, 0x61, BIT(4));	/* AFE_DRV_RST */
	if (ret)
		return ret;
	ret = ite_write_masked(fl, 0x62, 0x90, high ? 0x80 : 0x10);
	if (ret)
		return ret;
	ret = ite_write_masked(fl, 0x64, 0x89, high ? 0x80 : 0x09);
	if (ret)
		return ret;
	ret = ite_write_masked(fl, 0x68, 0x10, high ? 0x80 : 0x10);
	if (ret)
		return ret;

	ret = ite_write_masked(fl, 0x04, BIT(5) | BIT(3), 0);
	if (ret)
		return ret;
	ret = ite_write_byte(fl, 0x61, 0x00);
	if (ret)
		return ret;
	ret = ite_write_byte(fl, 0x04, 0x01);	/* keep only HDCP reset */
	if (ret)
		return ret;
	ret = ite_write_byte(fl, 0x0F, 0);	/* fire AFE */
	if (ret)
		return ret;
	return ite_write_byte(fl, 0x61, 0x00);
}

int fl2000_ite_enable_video(struct fl2000 *fl, const struct fl2000_mode *mode)
{
	int ret;

	mutex_lock(&fl->ite_lock);
	ret = ite_av_mute(fl, 1);
	if (ret)
		goto out;
	ret = ite_send_avi_infoframe(fl, mode);
	if (ret)
		goto out;
	ret = ite_video_on(fl, mode);
	if (ret)
		goto out;
	ret = ite_av_mute(fl, 0);
out:
	mutex_unlock(&fl->ite_lock);
	return ret;
}

int fl2000_ite_video_off(struct fl2000 *fl)
{
	int ret;

	mutex_lock(&fl->ite_lock);
	ret = ite_av_mute(fl, 1);
	if (!ret)
		ret = ite_write_byte(fl, 0x61, BIT(4));	/* TMDS driver off */
	mutex_unlock(&fl->ite_lock);
	return ret;
}

/* raw SYS_STATUS (0x0E): bit6 HPD, bit5 RxSense, bit4 video stable */
int fl2000_ite_status(struct fl2000 *fl, u8 *sys_status)
{
	int ret;

	mutex_lock(&fl->ite_lock);
	ret = ite_read_byte(fl, 0x0E, sys_status);
	mutex_unlock(&fl->ite_lock);
	return ret;
}
