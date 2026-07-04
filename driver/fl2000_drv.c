// SPDX-License-Identifier: GPL-2.0
/*
 * fl2000drm - DRM driver for Fresco Logic FL2000DX USB-to-HDMI/VGA adapters
 *
 * Modeled on the mainline gm12u320/udl drivers: one simple display pipe,
 * GEM shmem dumb buffers, shadow-plane atomic updates converted to the
 * wire format and streamed continuously over the bulk endpoint (the chip
 * has no framebuffer of its own — the host feeds every scanout frame).
 *
 * Frames are triple buffered between the atomic commit path and the USB
 * streaming worker so a slow or wedged transfer can never stall the
 * compositor, and connector probes are served from an EDID cache so the
 * slow DDC-over-USB path never runs inside a userspace ioctl.
 */
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/bitops.h>
#include <linux/vmalloc.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>

#include "fl2000_drm.h"

#define DRIVER_NAME	"fl2000drm"

/*
 * Wire format override for experiments, runtime-changeable; takes effect
 * on the next display enable (e.g. kscreen output off/on):
 * 0 = auto, 1 = RGB565, 2 = RGB888 (B,G,R bytes), 3 = BGR888 (R,G,B bytes),
 * 4 = XRGB32 (4 bytes/pixel + reg 0x8004 bit 28 experiment)
 */
static int fmt;
module_param(fmt, int, 0644);
MODULE_PARM_DESC(fmt, "wire format: 0=auto 1=RGB565 2=RGB888 3=BGR888 4=XRGB32");

/* rough payload budget per link speed, bytes/second */
#define FL2000_BUDGET_USB2	 38000000ULL
#define FL2000_BUDGET_USB3	400000000ULL
/* above this RGB888 rate on USB3, fall back to RGB565 on the wire */
#define FL2000_RGB888_LIMIT	320000000ULL

static u32 fl2000_wire_bpp_for(struct fl2000 *fl, const struct fl2000_mode *m)
{
	u64 bw888 = (u64)m->width * m->height * m->refresh * 3;

	switch (fmt) {
	case 1:
		return 2;
	case 2:
	case 3:
		return 3;
	case 4:
		return 4;
	}
	if (!fl->usb3)
		return 2;
	return bw888 > FL2000_RGB888_LIMIT ? 2 : 3;
}

static bool fl2000_mode_fits_link(struct fl2000 *fl,
				  const struct fl2000_mode *m)
{
	u64 bw = (u64)m->width * m->height * m->refresh *
		 fl2000_wire_bpp_for(fl, m);

	return bw <= (fl->usb3 ? FL2000_BUDGET_USB3 : FL2000_BUDGET_USB2);
}

/* device gone or reset: retrying is pointless */
static bool fl2000_usb_err_fatal(int err)
{
	return err == -ENODEV || err == -ENOENT || err == -ESHUTDOWN ||
	       err == -EPROTO || err == -EILSEQ;
}

/*
 * Pipelined streaming: a ring of URBs stays queued on the bulk endpoint so
 * the device-side FIFO never starves while the worker copies the next
 * chunk (the device NAK-paces the pipe to its scanout rate). Bulk URBs on
 * a single pipe complete in submission order, so round-robin slot reuse
 * guarded by a counting semaphore is race-free.
 */
static void fl2000_xfer_complete(struct urb *urb)
{
	struct fl2000 *fl = urb->context;

	if (urb->status)
		atomic_cmpxchg(&fl->stream_error, 0, urb->status);
	up(&fl->xfer_sem);
}

static int fl2000_submit_chunk(struct fl2000 *fl, const void *data,
			       size_t len)
{
	struct fl2000_xfer *x;
	int ret;

	if (down_timeout(&fl->xfer_sem, msecs_to_jiffies(1000)))
		return -ETIMEDOUT;

	x = &fl->xfers[fl->xfer_ring];
	fl->xfer_ring = (fl->xfer_ring + 1) % FL2000_NUM_XFERS;

	if (len)
		memcpy(x->buf, data, len);
	usb_fill_bulk_urb(x->urb, fl->udev, usb_sndbulkpipe(fl->udev, 1),
			  x->buf, len, fl2000_xfer_complete, fl);
	/* buffers are pre-mapped: no per-submit IOMMU map/unmap traffic */
	x->urb->transfer_dma = x->dma;
	x->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	ret = usb_submit_urb(x->urb, GFP_KERNEL);
	if (ret) {
		/* roll back so ring order stays aligned with completions */
		fl->xfer_ring = (fl->xfer_ring + FL2000_NUM_XFERS - 1) %
				FL2000_NUM_XFERS;
		up(&fl->xfer_sem);
	}
	return ret;
}

static void fl2000_stream_work(struct work_struct *work)
{
	struct fl2000 *fl = container_of(work, struct fl2000, stream_work);
	u64 t_win = ktime_get_ns(), t_frame;
	u32 frames = 0, frame_us, min_us = U32_MAX, max_us = 0;
	int errors = 0;
	int idx;

	if (!drm_dev_enter(&fl->drm, &idx))
		return;

	while (READ_ONCE(fl->streaming)) {
		size_t off = 0;
		int ret = 0;

		t_frame = ktime_get_ns();

		spin_lock(&fl->frame_swap_lock);
		if (fl->frame_ready) {
			swap(fl->send_buf, fl->ready_buf);
			fl->frame_ready = false;
		}
		spin_unlock(&fl->frame_swap_lock);

		while (off < fl->frame_len) {
			size_t len = min_t(size_t, FL2000_CHUNK_SIZE,
					   fl->frame_len - off);

			ret = fl2000_submit_chunk(fl, fl->send_buf + off, len);
			if (ret)
				break;
			off += len;
		}
		if (!ret)	/* zero-length packet = end of frame */
			ret = fl2000_submit_chunk(fl, NULL, 0);
		if (!ret)	/* async errors surface via completions */
			ret = atomic_xchg(&fl->stream_error, 0);

		if (ret) {
			if (fl2000_usb_err_fatal(ret)) {
				drm_dbg(&fl->drm,
					"device gone (%d), stream off\n", ret);
				break;
			}
			if (++errors > 50) {
				drm_err(&fl->drm,
					"bulk streaming failed (%d), stopping\n",
					ret);
				break;
			}
			/* best effort resync of the frame boundary */
			fl2000_submit_chunk(fl, NULL, 0);
			msleep(20);
		} else {
			errors = 0;
		}

		/* pacing + link health telemetry, roughly every 5 s at 60 Hz */
		frame_us = div_u64(ktime_get_ns() - t_frame, 1000);
		min_us = min(min_us, frame_us);
		max_us = max(max_us, frame_us);
		if (++frames == 300) {
			u64 win_ms = div_u64(ktime_get_ns() - t_win, 1000000);
			u8 ite_st = 0xFF;

			if (fl->ite_present)
				fl2000_ite_status(fl, &ite_st);
			drm_info(&fl->drm,
				 "stream: 300 frames in %llu ms (frame %u..%u us), ite=0x%02X\n",
				 win_ms, min_us, max_us, ite_st);
			frames = 0;
			min_us = U32_MAX;
			max_us = 0;
			t_win = ktime_get_ns();
		}
		cond_resched();
	}

	drm_dev_exit(idx);
}

/*
 * EDID cache
 */

static void fl2000_refresh_edid_cache(struct fl2000 *fl)
{
	int blocks, i;

	fl->edid_valid = false;

	if (fl2000_ite_read_edid_block(fl, fl->edid_cache, 0, 128))
		return;

	blocks = min_t(int, fl->edid_cache[126], 3) + 1;
	for (i = 1; i < blocks; i++) {
		if (fl2000_ite_read_edid_block(fl, fl->edid_cache + i * 128,
					       i, 128))
			return;
	}
	fl->edid_len = blocks * 128;
	fl->edid_valid = true;
}

/*
 * Display pipe
 */

static enum drm_mode_status
fl2000_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
		       const struct drm_display_mode *mode)
{
	struct fl2000 *fl = to_fl2000(pipe->crtc.dev);
	const struct fl2000_mode *m;

	m = fl2000_hw_find_mode(mode->hdisplay, mode->vdisplay,
				drm_mode_vrefresh(mode));
	if (!m)
		return MODE_BAD;
	if (!fl2000_mode_fits_link(fl, m))
		return MODE_BAD;
	return MODE_OK;
}

static void fl2000_free_frames(struct fl2000 *fl)
{
	int i;

	for (i = 0; i < FL2000_NUM_XFERS; i++) {
		usb_free_urb(fl->xfers[i].urb);
		if (fl->xfers[i].buf)
			usb_free_coherent(fl->udev, FL2000_CHUNK_SIZE,
					  fl->xfers[i].buf, fl->xfers[i].dma);
		fl->xfers[i].urb = NULL;
		fl->xfers[i].buf = NULL;
	}
	vfree(fl->write_buf);
	vfree(fl->ready_buf);
	vfree(fl->send_buf);
	fl->write_buf = NULL;
	fl->ready_buf = NULL;
	fl->send_buf = NULL;
	fl->frame_ready = false;
}

static void fl2000_kill_xfers(struct fl2000 *fl)
{
	int i;

	for (i = 0; i < FL2000_NUM_XFERS; i++)
		if (fl->xfers[i].urb)
			usb_kill_urb(fl->xfers[i].urb);
}

static void fl2000_pipe_enable(struct drm_simple_display_pipe *pipe,
			       struct drm_crtc_state *crtc_state,
			       struct drm_plane_state *plane_state)
{
	struct fl2000 *fl = to_fl2000(pipe->crtc.dev);
	struct drm_display_mode *mode = &crtc_state->mode;
	const struct fl2000_mode *m;
	int idx, ret;

	m = fl2000_hw_find_mode(mode->hdisplay, mode->vdisplay,
				drm_mode_vrefresh(mode));
	if (!m)
		return;

	if (!drm_dev_enter(&fl->drm, &idx))
		return;

	fl->wire_bpp = fl2000_wire_bpp_for(fl, m);
	fl->frame_len = (size_t)m->width * m->height * fl->wire_bpp;
	fl->write_buf = vzalloc(fl->frame_len);
	fl->ready_buf = vzalloc(fl->frame_len);
	fl->send_buf = vzalloc(fl->frame_len);
	if (!fl->write_buf || !fl->ready_buf || !fl->send_buf)
		goto err_free;

	sema_init(&fl->xfer_sem, FL2000_NUM_XFERS);
	fl->xfer_ring = 0;
	atomic_set(&fl->stream_error, 0);
	for (int i = 0; i < FL2000_NUM_XFERS; i++) {
		fl->xfers[i].urb = usb_alloc_urb(0, GFP_KERNEL);
		fl->xfers[i].buf = usb_alloc_coherent(fl->udev,
						      FL2000_CHUNK_SIZE,
						      GFP_KERNEL,
						      &fl->xfers[i].dma);
		if (!fl->xfers[i].urb || !fl->xfers[i].buf)
			goto err_free;
	}
	fl->mode = m;

	ret = fl2000_hw_stream_prep(fl);
	if (ret)
		goto err_free;
	ret = fl2000_hw_set_mode(fl, m, fl->wire_bpp);
	if (ret) {
		drm_err(&fl->drm, "mode set failed: %d\n", ret);
		goto err_free;
	}
	if (fl->ite_present) {
		ret = fl2000_ite_enable_video(fl, m);
		if (ret)
			drm_warn(&fl->drm, "IT66121 setup failed: %d\n", ret);
	}

	{
		u32 r8004 = 0, r803c = 0;

		fl2000_reg_read(fl, 0x8004, &r8004);
		fl2000_reg_read(fl, 0x803C, &r803c);
		drm_info(&fl->drm,
			 "streaming %ux%u@%u, %u bytes/px (fmt=%d), 0x8004=0x%08X 0x803C=0x%08X\n",
			 m->width, m->height, m->refresh, fl->wire_bpp, fmt,
			 r8004, r803c);
	}

	WRITE_ONCE(fl->streaming, true);
	queue_work(fl->stream_wq, &fl->stream_work);
	drm_dev_exit(idx);
	return;

err_free:
	fl2000_free_frames(fl);
	drm_dev_exit(idx);
}

static void fl2000_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct fl2000 *fl = to_fl2000(pipe->crtc.dev);
	int idx;

	WRITE_ONCE(fl->streaming, false);
	cancel_work_sync(&fl->stream_work);
	fl2000_kill_xfers(fl);

	if (drm_dev_enter(&fl->drm, &idx)) {
		if (fl->ite_present)
			fl2000_ite_video_off(fl);
		fl2000_hw_reset(fl);
		drm_dev_exit(idx);
	}

	fl2000_free_frames(fl);
	fl->mode = NULL;
}

static void fl2000_pipe_update(struct drm_simple_display_pipe *pipe,
			       struct drm_plane_state *old_state)
{
	struct fl2000 *fl = to_fl2000(pipe->crtc.dev);
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_shadow_plane_state *shadow =
		to_drm_shadow_plane_state(state);
	struct drm_framebuffer *fb = state->fb;
	struct drm_rect damage, full;
	struct iosys_map dst;
	int idx;

	if (!fb || !fl->write_buf || !fl->mode)
		return;
	if (!drm_atomic_helper_damage_merged(old_state, state, &damage))
		return;
	if (!drm_dev_enter(&fl->drm, &idx))
		return;

	/*
	 * Convert the full frame into write_buf (owned by the commit path,
	 * commits are serialized by DRM), then publish it. Damage only
	 * gates no-op commits for now.
	 */
	full = DRM_RECT_INIT(0, 0, fl->mode->width, fl->mode->height);
	iosys_map_set_vaddr(&dst, fl->write_buf);

	if (fl->wire_bpp == 2)
		drm_fb_xrgb8888_to_rgb565(&dst, NULL, shadow->data, fb, &full,
					  &shadow->fmtcnv_state, false);
	else if (fl->wire_bpp == 4)
		drm_fb_memcpy(&dst, NULL, shadow->data, fb, &full);
	else if (fmt == 3)
		drm_fb_xrgb8888_to_bgr888(&dst, NULL, shadow->data, fb, &full,
					  &shadow->fmtcnv_state);
	else
		drm_fb_xrgb8888_to_rgb888(&dst, NULL, shadow->data, fb, &full,
					  &shadow->fmtcnv_state);

	/*
	 * The FL2000 consumes the bulk stream as 64-bit words with the two
	 * 32-bit halves reversed — the reference driver applies the same
	 * swap to every surface (pixel_swap() in fl2000_ioctl.c). Solid
	 * colors are invariant under this, which is why test patterns look
	 * fine without it while fine detail turns to confetti.
	 */
	{
		u64 *p = fl->write_buf;
		size_t i, n = fl->frame_len / sizeof(u64);

		for (i = 0; i < n; i++)
			p[i] = rol64(p[i], 32);
	}

	spin_lock(&fl->frame_swap_lock);
	swap(fl->write_buf, fl->ready_buf);
	fl->frame_ready = true;
	spin_unlock(&fl->frame_swap_lock);

	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs fl2000_pipe_funcs = {
	.mode_valid = fl2000_pipe_mode_valid,
	.enable = fl2000_pipe_enable,
	.disable = fl2000_pipe_disable,
	.update = fl2000_pipe_update,
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
};

static const uint32_t fl2000_pipe_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const uint64_t fl2000_pipe_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};

/*
 * Connector
 */

static int fl2000_connector_get_modes(struct drm_connector *connector)
{
	struct fl2000 *fl = to_fl2000(connector->dev);
	const struct drm_edid *drm_edid = NULL;
	int count;

	if (fl->ite_present) {
		if (!fl->edid_valid)
			fl2000_refresh_edid_cache(fl);
		if (fl->edid_valid)
			drm_edid = drm_edid_alloc(fl->edid_cache,
						  fl->edid_len);
	}

	drm_edid_connector_update(connector, drm_edid);

	if (drm_edid) {
		count = drm_edid_connector_add_modes(connector);
		drm_edid_free(drm_edid);
	} else {
		count = drm_add_modes_noedid(connector, 1024, 768);
		drm_set_preferred_mode(connector, 640, 480);
	}

	/*
	 * Also offer every mode the scanout tables support on this link —
	 * panels behind VGA converters often omit their native mode from
	 * the (adapter-provided) EDID. Timings are decoded from the table's
	 * register values so they match the hardware exactly (CVT would
	 * round 1366 up to 1368 and the mode would never validate).
	 * Duplicates are merged by the probe helper.
	 */
	{
		const struct fl2000_mode *tbl;
		int i, n;

		tbl = fl2000_hw_mode_table(&n);
		for (i = 0; i < n; i++) {
			const struct fl2000_mode *m = &tbl[i];
			struct drm_display_mode *dmode;
			u32 hdisp, htotal, hsync, hbp, hfront;
			u32 vdisp, vtotal, vsync, vbp, vfront;

			if (!fl2000_mode_fits_link(fl, m))
				continue;

			hdisp = m->h_sync_1 >> 16;
			htotal = m->h_sync_1 & 0xFFFF;
			hsync = m->h_sync_2 >> 16;
			hbp = (m->h_sync_2 & 0xFFFF) - hsync - 1;
			hfront = htotal - hdisp - hsync - hbp;
			vdisp = m->v_sync_1 >> 16;
			vtotal = m->v_sync_1 & 0xFFFF;
			vsync = m->vsync_time;
			vbp = m->v_back_porch;
			vfront = vtotal - vdisp - vsync - vbp;

			dmode = drm_mode_create(connector->dev);
			if (!dmode)
				continue;
			dmode->clock = htotal * vtotal * m->refresh / 1000;
			dmode->hdisplay = hdisp;
			dmode->hsync_start = hdisp + hfront;
			dmode->hsync_end = hdisp + hfront + hsync;
			dmode->htotal = htotal;
			dmode->vdisplay = vdisp;
			dmode->vsync_start = vdisp + vfront;
			dmode->vsync_end = vdisp + vfront + vsync;
			dmode->vtotal = vtotal;
			dmode->type = DRM_MODE_TYPE_DRIVER;
			drm_mode_set_name(dmode);
			drm_mode_probed_add(connector, dmode);
			count++;
		}
	}
	return count;
}

static enum drm_connector_status
fl2000_connector_detect(struct drm_connector *connector, bool force)
{
	struct fl2000 *fl = to_fl2000(connector->dev);
	int hpd;

	if (!fl->ite_present)
		return connector_status_connected;

	hpd = fl2000_ite_hpd(fl);
	if (hpd < 0)
		return connector_status_unknown;
	if (!hpd) {
		fl->edid_valid = false;
		return connector_status_disconnected;
	}
	return connector_status_connected;
}

static const struct drm_connector_helper_funcs fl2000_connector_helper_funcs = {
	.get_modes = fl2000_connector_get_modes,
};

static const struct drm_connector_funcs fl2000_connector_funcs = {
	.detect = fl2000_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/*
 * DRM device
 */

DEFINE_DRM_GEM_FOPS(fl2000_fops);

static const struct drm_driver fl2000_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.fops = &fl2000_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
	.name = DRIVER_NAME,
	.desc = "Fresco Logic FL2000 USB display",
	.date = "20260704",
	.major = 1,
	.minor = 1,
};

static const struct drm_mode_config_funcs fl2000_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int fl2000_probe(struct usb_interface *intf,
			const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct fl2000 *fl;
	struct drm_device *drm;
	int ret, i;

	if (intf->cur_altsetting->desc.bInterfaceNumber != 0)
		return -ENODEV;

	fl = devm_drm_dev_alloc(&intf->dev, &fl2000_drm_driver,
				struct fl2000, drm);
	if (IS_ERR(fl))
		return PTR_ERR(fl);
	drm = &fl->drm;

	fl->udev = udev;
	fl->intf = intf;
	fl->usb3 = udev->speed >= USB_SPEED_SUPER;
	mutex_init(&fl->hw_lock);
	mutex_init(&fl->ite_lock);
	spin_lock_init(&fl->frame_swap_lock);
	INIT_WORK(&fl->stream_work, fl2000_stream_work);

	fl->stream_wq = alloc_workqueue("fl2000-stream",
					WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!fl->stream_wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(&intf->dev,
				       (void (*)(void *))destroy_workqueue,
				       fl->stream_wq);
	if (ret)
		return ret;

	/* streaming bulk endpoint lives on interface 0 altsetting 1 */
	ret = usb_set_interface(udev, 0, 1);
	if (ret)
		return ret;

	ret = fl2000_hw_reset(fl);
	if (ret)
		return dev_err_probe(&intf->dev, ret,
				     "device not responding\n");

	fl->ite_present = fl2000_ite_detect(fl);
	if (fl->ite_present) {
		ret = fl2000_ite_init(fl);
		if (ret)
			dev_warn(&intf->dev,
				 "IT66121 init failed: %d\n", ret);
		/*
		 * Give hotplug detect a moment to settle after the
		 * transmitter reset (VGA converter dongles can be slow to
		 * assert HPD), then warm the EDID cache so the first
		 * connector probes are instant.
		 */
		for (i = 0; i < 60; i++) {
			if (fl2000_ite_hpd(fl) == 1) {
				fl2000_refresh_edid_cache(fl);
				break;
			}
			msleep(50);
		}
	}

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = 640;
	drm->mode_config.max_width = 1920;
	drm->mode_config.min_height = 350;
	drm->mode_config.max_height = 1200;
	drm->mode_config.funcs = &fl2000_mode_config_funcs;

	ret = drm_connector_init(drm, &fl->connector, &fl2000_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret)
		return ret;
	drm_connector_helper_add(&fl->connector,
				 &fl2000_connector_helper_funcs);
	fl->connector.polled = DRM_CONNECTOR_POLL_CONNECT |
			       DRM_CONNECTOR_POLL_DISCONNECT;

	ret = drm_simple_display_pipe_init(drm, &fl->pipe, &fl2000_pipe_funcs,
					   fl2000_pipe_formats,
					   ARRAY_SIZE(fl2000_pipe_formats),
					   fl2000_pipe_modifiers,
					   &fl->connector);
	if (ret)
		return ret;

	drm_plane_enable_fb_damage_clips(&fl->pipe.plane);
	drm_mode_config_reset(drm);

	usb_set_intfdata(intf, fl);

	drm_kms_helper_poll_init(drm);

	ret = drm_dev_register(drm, 0);
	if (ret) {
		drm_kms_helper_poll_fini(drm);
		return ret;
	}

	drm_fbdev_shmem_setup(drm, 32);

	dev_info(&intf->dev,
		 "FL2000DX on USB%c link%s, modes up to %s\n",
		 fl->usb3 ? '3' : '2',
		 fl->ite_present ? " with IT66121 HDMI transmitter" : "",
		 fl->usb3 ? "1920x1080@60" : "640x480@60");
	return 0;
}

static void fl2000_disconnect(struct usb_interface *intf)
{
	struct fl2000 *fl = usb_get_intfdata(intf);
	struct drm_device *drm = &fl->drm;

	drm_kms_helper_poll_fini(drm);
	drm_dev_unplug(drm);
	WRITE_ONCE(fl->streaming, false);
	cancel_work_sync(&fl->stream_work);
	fl2000_kill_xfers(fl);
	drm_atomic_helper_shutdown(drm);
}

static const struct usb_device_id fl2000_id_table[] = {
	{ USB_DEVICE(0x1D5C, 0x2000) },
	{},
};
MODULE_DEVICE_TABLE(usb, fl2000_id_table);

static struct usb_driver fl2000_usb_driver = {
	.name = DRIVER_NAME,
	.probe = fl2000_probe,
	.disconnect = fl2000_disconnect,
	.id_table = fl2000_id_table,
};

module_usb_driver(fl2000_usb_driver);

MODULE_DESCRIPTION("DRM driver for Fresco Logic FL2000DX USB display adapters");
MODULE_AUTHOR("adriel + Claude");
MODULE_LICENSE("GPL");
