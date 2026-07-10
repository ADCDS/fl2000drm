/* SPDX-License-Identifier: GPL-2.0 */
/*
 * fl2000drm - DRM driver for Fresco Logic FL2000DX USB display adapters
 *
 * Register protocol derived from Fresco Logic's GPL reference driver.
 */
#ifndef FL2000_DRM_H
#define FL2000_DRM_H

#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <drm/drm_simple_kms_helper.h>

struct fl2000_mode {
	u16 width;
	u16 height;
	u16 refresh;
	u32 h_sync_1;	/* reg 0x8008: width<<16 | htotal */
	u32 h_sync_2;	/* reg 0x800C: hsync<<16 | (hsync+hbp+1) */
	u32 v_sync_1;	/* reg 0x8010: height<<16 | vtotal */
	u32 v_sync_2;	/* reg 0x8014: low 16 = vsync+vbp+1 */
	u32 pll;	/* reg 0x802C, bulk transfer PLL */
	u8 vic;		/* CEA VIC for the AVI infoframe, 0 if none */
	u8 vsync_time;	/* vsync width in lines (for DRM mode timings) */
	u8 v_back_porch;
};

struct fl2000 {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector connector;

	struct usb_device *udev;
	struct usb_interface *intf;
	bool usb3;
	bool ite_present;

	/* serializes all EP0 register access and I2C sequences */
	struct mutex hw_lock;
	/* serializes logical IT66121 operations (DDC reads, mode setup) */
	struct mutex ite_lock;

	/*
	 * Triple-buffered wire-format frames: atomic updates convert into
	 * write_buf and publish it as ready_buf; the stream worker claims
	 * ready_buf as send_buf. Neither side ever blocks on the other.
	 */
	spinlock_t frame_swap_lock;
	void *write_buf;
	void *ready_buf;
	void *send_buf;
	bool frame_ready;
	size_t frame_len;
	const struct fl2000_mode *mode;
	u32 wire_bpp;			/* 2 = RGB565, 3 = RGB888 */
	bool streaming;
	struct work_struct stream_work;

	/*
	 * Pipelined bulk transfers: a ring of URBs kept in flight so the
	 * device-side FIFO never starves on producer jitter. Bulk URBs on
	 * one pipe complete in submission order, so a counting semaphore
	 * plus round-robin slot reuse is race-free.
	 */
	struct fl2000_xfer {
		struct urb *urb;
		u8 *buf;	/* usb_alloc_coherent, pre-mapped for DMA */
		dma_addr_t dma;
	} xfers[64];
	struct semaphore xfer_sem;
	unsigned int xfer_ring;
	atomic_t stream_error;
	struct workqueue_struct *stream_wq;

	/* EDID cache so connector probes never hit the slow DDC path */
	u8 edid_cache[4 * 128];
	int edid_len;
	bool edid_valid;
};

#define to_fl2000(x) container_of(x, struct fl2000, drm)

#define FL2000_CHUNK_SIZE	(64 * 1024)

/* fl2000_hw.c */
int fl2000_reg_read(struct fl2000 *fl, u16 offset, u32 *val);
int fl2000_reg_write(struct fl2000 *fl, u16 offset, u32 val);
int fl2000_i2c_read32(struct fl2000 *fl, u8 addr, u8 offset, u32 *val);
int fl2000_i2c_write32(struct fl2000 *fl, u8 addr, u8 offset, u32 val);
int fl2000_hw_reset(struct fl2000 *fl);
int fl2000_hw_dongle_init(struct fl2000 *fl);
int fl2000_hw_set_mode(struct fl2000 *fl, const struct fl2000_mode *mode,
		       u32 wire_bpp);
int fl2000_hw_stream_prep(struct fl2000 *fl);
const struct fl2000_mode *fl2000_hw_find_mode(int width, int height,
					      int refresh);
const struct fl2000_mode *fl2000_hw_mode_table(int *count);

#define FL2000_NUM_XFERS	ARRAY_SIZE(((struct fl2000 *)0)->xfers)

/* fl2000_ite.c */
bool fl2000_ite_detect(struct fl2000 *fl);
int fl2000_ite_init(struct fl2000 *fl);
int fl2000_ite_hpd(struct fl2000 *fl);
int fl2000_ite_read_edid_block(void *ctx, u8 *buf, unsigned int block,
			       size_t len);
int fl2000_ite_enable_video(struct fl2000 *fl, const struct fl2000_mode *mode);
int fl2000_ite_video_off(struct fl2000 *fl);
int fl2000_ite_status(struct fl2000 *fl, u8 *sys_status);

#endif
