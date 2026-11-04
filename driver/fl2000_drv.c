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
#include <linux/version.h>
#include <linux/vmalloc.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_cache.h>
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
#include <drm/drm_vblank.h>
#include <drm/drm_print.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 14, 0)
#include <drm/clients/drm_client_setup.h>
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
#include <drm/drm_client_setup.h>
#endif

#include "fl2000_drm.h"

#define DRIVER_NAME	"fl2000drm"

/* 6.17 dropped the swab argument (big-endian output got its own helper) */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 17, 0)
#define fl2000_fb_xrgb8888_to_rgb565(...) \
	drm_fb_xrgb8888_to_rgb565(__VA_ARGS__)
#else
#define fl2000_fb_xrgb8888_to_rgb565(...) \
	drm_fb_xrgb8888_to_rgb565(__VA_ARGS__, false)
#endif

/*
 * Wire format override for experiments, runtime-changeable; takes effect
 * on the next display enable (e.g. kscreen output off/on):
 * 0 = auto, 1 = RGB565, 2 = RGB888 (B,G,R bytes), 3 = BGR888 (R,G,B bytes),
 * 4 = XRGB32 (4 bytes/pixel + reg 0x8004 bit 28 experiment)
 */
static int fmt;
module_param(fmt, int, 0644);
MODULE_PARM_DESC(fmt, "wire format: 0=auto 1=RGB565 2=RGB888 3=BGR888 4=XRGB32");

/*
 * Frame pacing (the anti-blink fix). Delivered flat-out (NAK-locked at
 * FIFO full), the chip races its EOF latch against vsync and every few
 * seconds fast-drains one frame while scanning out black (paired
 * 10.6/22.7 ms frame times on the wire; ~20 blinks/min at 1080p).
 * Instead, a frequency-locked loop paces one frame per scanout period -
 * the period measured from the chip's own frame counter, since the PLL
 * formula is off by a mode-dependent 0.1-0.6% - and a small proportional
 * term on pre-submission ring backlog holds the delivery phase off the
 * racy full mark. Measured: 4x fewer blinks at 1080p, race signature
 * gone from the wire; the residual blinks are line-buffer underflows
 * (a separate, bandwidth-margin mechanism).
 */
static int pace = 1;
module_param(pace, int, 0644);
MODULE_PARM_DESC(pace, "frame pacing: 0=off 1=frequency-locked to the chip's scanout (default)");

/*
 * Late latch: while a compositor is animating, the stream waits up to this
 * long at a frame pickup for a commit that is about to land, instead of
 * repeating the old frame and halving the compositor's rate. The USB
 * backlog (tens of ms) absorbs the delay.
 */
static unsigned int latch_us = 6000;
module_param(latch_us, uint, 0644);
MODULE_PARM_DESC(latch_us, "max wait at frame pickup for an in-progress commit, us (0=off)");

/* VGA status register 0x8000 decode (reference register map) */
#define FL2000_STATUS_VGA_ERROR		BIT(1)
#define FL2000_STATUS_LBUF_HALT		BIT(2)
#define FL2000_STATUS_TD_DROP		BIT(4)
#define FL2000_STATUS_LBUF_OVERFLOW	BIT(8)
#define FL2000_STATUS_LBUF_UNDERFLOW	BIT(9)
#define FL2000_STATUS_FRAME_CNT(v)	(((v) >> 10) & 0xFFFF)
#define FL2000_STATUS_EVENT_MASK	(BIT(26) | BIT(30) | BIT(31))

/* rough payload budget per link speed, bytes/second */
#define FL2000_BUDGET_USB2	 38000000ULL
#define FL2000_BUDGET_USB3	400000000ULL
/* above this RGB888 rate on USB3, fall back to RGB565 on the wire */
#define FL2000_RGB888_LIMIT	320000000ULL
#define FL2000_VGA_MONITOR_STATUS	BIT(29)
#define FL2000_DDC_ADDR		0x50
#define FL2000_DDC_SEGMENT_ADDR	0x30

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
	atomic_dec(&fl->inflight);
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
	atomic_inc(&fl->inflight);
	ret = usb_submit_urb(x->urb, GFP_KERNEL);
	if (ret) {
		/* roll back so ring order stays aligned with completions */
		fl->xfer_ring = (fl->xfer_ring + FL2000_NUM_XFERS - 1) %
				FL2000_NUM_XFERS;
		atomic_dec(&fl->inflight);
		up(&fl->xfer_sem);
	}
	return ret;
}

/*
 * Flip-completion pacing. Without vblank support DRM completes each page
 * flip the moment the commit returns, so compositors time their frames
 * against nothing: on this output KWin fell to ~30 Hz under cursor motion
 * or ran faster than the chip's ~60 Hz scanout, and frames were repeated
 * or dropped (visible as cursor and scrolling judder). Instead, hold the
 * commit's event until the stream worker picks up its frame — the closest
 * thing this hardware has to a vblank — so userspace locks to the real
 * rate.
 *
 * Events carry the scheduled pickup time, not the send time: a commit
 * caught by the late latch must not push the compositor's next target
 * later, or the lateness would compound frame after frame. Same fields
 * drm_crtc_send_vblank_event() fills, minus its choice of timestamp.
 */
static void fl2000_send_event(struct fl2000 *fl,
			      struct drm_pending_vblank_event *event,
			      ktime_t stamp)
{
	struct timespec64 tv = ktime_to_timespec64(stamp);
	unsigned long flags;

	if (!event)
		return;
	event->pipe = drm_crtc_index(&fl->pipe.crtc);
	event->event.vbl.sequence = 0;
	event->event.vbl.tv_sec = tv.tv_sec;
	event->event.vbl.tv_usec = tv.tv_nsec / 1000;
	spin_lock_irqsave(&fl->drm.event_lock, flags);
	drm_send_event_timestamp_locked(&fl->drm, &event->base, stamp);
	spin_unlock_irqrestore(&fl->drm.event_lock, flags);
}

/* Returns an event the caller must send now, if any. */
static struct drm_pending_vblank_event *
fl2000_defer_event_locked(struct fl2000 *fl,
			  struct drm_pending_vblank_event *event)
{
	struct drm_pending_vblank_event *old;

	if (!fl->stream_alive)
		return event;
	/* normally empty: DRM waits for flip_done before the next commit */
	old = fl->pending_event;
	fl->pending_event = event;
	return old;
}

/* The stream is not (or no longer) running: never leave a flip pending. */
static void fl2000_stream_stopped(struct fl2000 *fl)
{
	struct drm_pending_vblank_event *event;

	spin_lock(&fl->frame_swap_lock);
	fl->stream_alive = false;
	event = fl->pending_event;
	fl->pending_event = NULL;
	spin_unlock(&fl->frame_swap_lock);
	fl2000_send_event(fl, event, ktime_get());
}

static void fl2000_stream_work(struct work_struct *work)
{
	struct fl2000 *fl = container_of(work, struct fl2000, stream_work);
	u64 t_win = ktime_get_ns(), t_frame;
	bool animating = false;	/* the previous pickup completed a flip */
	u32 frames = 0, frame_us, min_us = U32_MAX, max_us = 0;
	int errors = 0;
	int idx;

	if (!drm_dev_enter(&fl->drm, &idx)) {
		fl2000_stream_stopped(fl);
		return;
	}

	while (READ_ONCE(fl->streaming)) {
		struct drm_pending_vblank_event *event;
		ktime_t stamp;
		size_t off = 0;
		int ret = 0;

		t_frame = ktime_get_ns();

		/*
		 * Pacing phase detector: ring backlog measured BEFORE
		 * submitting the next frame. (Sampling after submission is
		 * meaningless — the ring is always near-full then, which
		 * drove an earlier servo into a starve/catch-up limit cycle.)
		 */
		if (pace)
			fl->pace_occ = atomic_read(&fl->inflight);

		/* the pickup's nominal time: the pacing deadline when pacing */
		stamp = pace && fl->next_frame_ns ?
			ns_to_ktime(fl->next_frame_ns) : ktime_get();
		if (animating && latch_us && !READ_ONCE(fl->frame_ready) &&
		    !READ_ONCE(fl->pending_event))
			wait_event_hrtimeout(fl->frame_wq,
					     READ_ONCE(fl->frame_ready) ||
					     READ_ONCE(fl->pending_event) ||
					     !READ_ONCE(fl->streaming),
					     ns_to_ktime((u64)latch_us *
							 NSEC_PER_USEC));

		spin_lock(&fl->frame_swap_lock);
		if (fl->frame_ready) {
			swap(fl->send_buf, fl->ready_buf);
			fl->frame_ready = false;
		}
		event = fl->pending_event;
		fl->pending_event = NULL;
		spin_unlock(&fl->frame_swap_lock);
		/* this frame goes out now: complete the flip that made it */
		animating = !!event;
		fl2000_send_event(fl, event, stamp);

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

		frame_us = div_u64(ktime_get_ns() - t_frame, 1000);

		if (pace && fl->frame_period_ns) {
			/*
			 * Frequency-locked pacing on an absolute deadline
			 * chain. The period comes from measuring the chip's
			 * own frame counter (the PLL formula is off by a
			 * mode-dependent 0.1-0.6%, and integrator servos
			 * wind up through the starve/catch-up asymmetry —
			 * v2 and v3 both heartbeat-cycled). Only a small
			 * proportional phase term on the pre-submission
			 * backlog remains; overshoot self-corrects because
			 * deadlines are absolute, and a stall resyncs the
			 * chain to now.
			 */
			s64 err = (s64)fl->pace_occ - fl->pace_occ_target;
			u64 now = ktime_get_ns();

			if (!fl->next_frame_ns ||
			    (s64)(now - fl->next_frame_ns) >
			    2 * (s64)fl->frame_period_ns)
				fl->next_frame_ns = now;
			fl->next_frame_ns += fl->frame_period_ns + err * 500;
			if (fl->next_frame_ns > now + 2000) {
				u32 us = div_u64(fl->next_frame_ns - now,
						 1000) - 1;

				usleep_range(us, us + 50);
			}
		}

		/* link health telemetry, roughly every 5 s at 60 Hz */
		min_us = min(min_us, frame_us);
		max_us = max(max_us, frame_us);
		if (++frames == 300) {
			u64 win_ns = ktime_get_ns() - t_win;
			u64 win_ms = div_u64(win_ns, 1000000);
			u8 ite_st = 0xFF;
			u32 st = 0, hwf, hwd;

			if (fl->ite_present)
				fl2000_ite_status(fl, &ite_st);
			fl2000_reg_read(fl, 0x8000, &st);
			hwf = FL2000_STATUS_FRAME_CNT(st);
			hwd = (hwf - fl->last_frame_cnt) & 0xFFFF;
			drm_info(&fl->drm,
				 "stream: 300 frames in %llu ms (frame %u..%u us), ite=0x%02X hwfrm=+%u st=0x%08X%s%s%s intr=%d\n",
				 win_ms, min_us, max_us, ite_st, hwd, st,
				 st & FL2000_STATUS_LBUF_UNDERFLOW ? " UNDER" : "",
				 st & FL2000_STATUS_LBUF_OVERFLOW ? " OVER" : "",
				 st & FL2000_STATUS_LBUF_HALT ? " HALT" : "",
				 atomic_read(&fl->intr_count));
			/*
			 * FLL rate reference: the chip free-runs, so its
			 * frame counter over the window is the true scanout
			 * rate regardless of what we delivered. First valid
			 * sample locks directly, then a 3:1 EMA; ±2% of the
			 * PLL-formula value as sanity bounds.
			 */
			if (pace && hwd >= 250 && hwd <= 360) {
				u64 meas = div_u64(win_ns, hwd);

				if (meas > div_u64(fl->period_pll_ns * 98, 100) &&
				    meas < div_u64(fl->period_pll_ns * 102, 100)) {
					if (fl->frame_period_ns ==
					    fl->period_pll_ns)
						fl->frame_period_ns = meas;
					else
						fl->frame_period_ns =
							(3 * fl->frame_period_ns +
							 meas) / 4;
				}
			}
			if (pace)
				drm_info(&fl->drm,
					 "pace: period=%lluns occ=%d/%u\n",
					 fl->frame_period_ns, fl->pace_occ,
					 fl->pace_occ_target);
			fl->last_frame_cnt = hwf;
			if (st & (FL2000_STATUS_LBUF_OVERFLOW |
				  FL2000_STATUS_LBUF_UNDERFLOW))
				fl2000_reg_write(fl, 0x8000, st);
			frames = 0;
			min_us = U32_MAX;
			max_us = 0;
			t_win = ktime_get_ns();
		}
		cond_resched();
	}

	fl2000_stream_stopped(fl);
	drm_dev_exit(idx);
}

/*
 * EDID cache + async connector status.
 *
 * Everything userspace can reach (detect/get_modes ioctls) is served from
 * cached state; the actual HPD and DDC traffic runs in status_work. A DDC
 * read is slow and can require bus-hang recovery — running it inside a
 * connector ioctl blocks the X server (single-threaded).
 */

static int fl2000_vga_read_edid_block(struct fl2000 *fl, u8 *buf,
				      unsigned int block)
{
	u8 segment = block / 2;
	u8 base = (block % 2) * 128;
	int i, j, ret;

	if (block >= ARRAY_SIZE(fl->edid_cache) / 128)
		return -EINVAL;

	/*
	 * The FL2000 I2C engine always transfers four data bytes. Repeating
	 * the segment value keeps the DDC segment pointer at the requested
	 * value for all four bytes of the write.
	 */
	if (segment) {
		u32 data = segment * 0x01010101;

		ret = fl2000_i2c_write32(fl, FL2000_DDC_SEGMENT_ADDR,
					 segment, data);
		if (ret)
			return ret;
	}

	for (i = 0; i < 128; i += sizeof(u32)) {
		u32 data;

		ret = fl2000_i2c_read32(fl, FL2000_DDC_ADDR, base + i, &data);
		if (ret)
			return ret;
		for (j = 0; j < sizeof(data); j++)
			buf[i + j] = data >> (j * 8);
	}

	return 0;
}

static void fl2000_refresh_edid_cache(struct fl2000 *fl)
{
	u8 buf[sizeof(fl->edid_cache)];
	int blocks, i;

	mutex_lock(&fl->edid_lock);
	fl->edid_valid = false;
	mutex_unlock(&fl->edid_lock);

	if (fl->ite_present) {
		if (fl2000_ite_read_edid_block(fl, buf, 0, 128))
			return;
	} else if (fl2000_vga_read_edid_block(fl, buf, 0)) {
		return;
	}

	blocks = min_t(int, buf[126], 3) + 1;
	for (i = 1; i < blocks; i++) {
		if (fl->ite_present) {
			if (fl2000_ite_read_edid_block(fl, buf + i * 128,
						       i, 128))
				return;
		} else if (fl2000_vga_read_edid_block(fl, buf + i * 128, i)) {
			return;
		}
	}

	mutex_lock(&fl->edid_lock);
	memcpy(fl->edid_cache, buf, blocks * 128);
	fl->edid_len = blocks * 128;
	fl->edid_valid = true;
	mutex_unlock(&fl->edid_lock);
}

#define FL2000_EDID_MAX_ATTEMPTS 3

static void fl2000_status_work(struct work_struct *work)
{
	struct fl2000 *fl = container_of(work, struct fl2000, status_work);
	bool changed = false;
	u32 status;
	int hpd, idx, ret;

	if (!drm_dev_enter(&fl->drm, &idx))
		return;

	if (fl->ite_present) {
		hpd = fl2000_ite_hpd(fl);
	} else {
		ret = fl2000_reg_read(fl, 0x8000, &status);
		if (ret)
			goto out;
		hpd = !!(status & FL2000_VGA_MONITOR_STATUS);
	}
	if (hpd < 0)
		goto out;

	if (atomic_xchg(&fl->hpd_status, hpd) != hpd) {
		changed = true;
		fl->edid_attempts = 0;
		if (!hpd) {
			mutex_lock(&fl->edid_lock);
			fl->edid_valid = false;
			mutex_unlock(&fl->edid_lock);
		}
	}

	if (hpd && !fl->edid_valid &&
	    fl->edid_attempts < FL2000_EDID_MAX_ATTEMPTS) {
		fl->edid_attempts++;
		fl2000_refresh_edid_cache(fl);
		if (fl->edid_valid)
			changed = true;
	}
	if (changed)
		drm_kms_helper_hotplug_event(&fl->drm);
out:
	drm_dev_exit(idx);
}

/*
 * Interrupt EP: EP3 IN on interface 2 (1-byte payload, 4 ms service
 * interval). The chip raises it for VGA status events — monitor/EDID
 * hotplug, frame drops, line-buffer overflow/underflow. On each event we
 * read reg 0x8000 (clears the self-clearing bits), log it, and write the
 * sticky lbuf bits back to clear them. Reverse-engineering aid for the
 * frame-boundary blink; hotplug events also feed the status worker.
 */
static struct usb_driver fl2000_usb_driver;

static void fl2000_intr_complete(struct urb *urb)
{
	struct fl2000 *fl = urb->context;

	if (urb->status)
		return;		/* poisoned or device gone: stop quietly */
	atomic_inc(&fl->intr_count);
	queue_work(system_unbound_wq, &fl->intr_work);
}

static void fl2000_intr_work(struct work_struct *work)
{
	struct fl2000 *fl = container_of(work, struct fl2000, intr_work);
	u32 st = 0;
	int idx;

	if (!drm_dev_enter(&fl->drm, &idx))
		return;

	if (!fl2000_reg_read(fl, 0x8000, &st)) {
		dev_info_ratelimited(&fl->intf->dev,
			"intr #%d st=0x%08X frame=%u%s%s%s%s%s\n",
			atomic_read(&fl->intr_count), st,
			FL2000_STATUS_FRAME_CNT(st),
			st & FL2000_STATUS_LBUF_UNDERFLOW ? " LBUF-UNDER" : "",
			st & FL2000_STATUS_LBUF_OVERFLOW ? " LBUF-OVER" : "",
			st & FL2000_STATUS_LBUF_HALT ? " LBUF-HALT" : "",
			st & FL2000_STATUS_TD_DROP ? " TD-DROP" : "",
			st & FL2000_STATUS_VGA_ERROR ? " VGA-ERR" : "");
		if (st & (FL2000_STATUS_LBUF_OVERFLOW |
			  FL2000_STATUS_LBUF_UNDERFLOW))
			fl2000_reg_write(fl, 0x8000, st);
		if (st & FL2000_STATUS_EVENT_MASK)
			queue_work(system_unbound_wq, &fl->status_work);
	}

	usb_submit_urb(fl->intr_urb, GFP_KERNEL);
	drm_dev_exit(idx);
}

static void fl2000_intr_start(struct fl2000 *fl)
{
	struct usb_interface *intf = usb_ifnum_to_if(fl->udev, 2);
	struct usb_endpoint_descriptor *desc;

	if (!intf || usb_find_int_in_endpoint(intf->cur_altsetting, &desc))
		return;
	if (usb_driver_claim_interface(&fl2000_usb_driver, intf, fl))
		return;

	fl->intr_urb = usb_alloc_urb(0, GFP_KERNEL);
	fl->intr_buf = usb_alloc_coherent(fl->udev, 8, GFP_KERNEL,
					  &fl->intr_dma);
	if (!fl->intr_urb || !fl->intr_buf)
		goto err_free;

	usb_fill_int_urb(fl->intr_urb, fl->udev,
			 usb_rcvintpipe(fl->udev, usb_endpoint_num(desc)),
			 fl->intr_buf, usb_endpoint_maxp(desc),
			 fl2000_intr_complete, fl, desc->bInterval);
	fl->intr_urb->transfer_dma = fl->intr_dma;
	fl->intr_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	if (usb_submit_urb(fl->intr_urb, GFP_KERNEL))
		goto err_free;

	fl->intr_intf = intf;
	dev_info(&fl->intf->dev, "interrupt EP armed (ep %u, interval %d)\n",
		 usb_endpoint_num(desc), desc->bInterval);
	return;

err_free:
	usb_free_coherent(fl->udev, 8, fl->intr_buf, fl->intr_dma);
	usb_free_urb(fl->intr_urb);
	fl->intr_urb = NULL;
	fl->intr_buf = NULL;
	usb_set_intfdata(intf, NULL);
	usb_driver_release_interface(&fl2000_usb_driver, intf);
}

static void fl2000_intr_stop(struct fl2000 *fl)
{
	struct usb_interface *intf = fl->intr_intf;

	if (!intf)
		return;
	fl->intr_intf = NULL;
	usb_poison_urb(fl->intr_urb);
	cancel_work_sync(&fl->intr_work);
	usb_set_intfdata(intf, NULL);
	usb_driver_release_interface(&fl2000_usb_driver, intf);
	usb_free_coherent(fl->udev, 8, fl->intr_buf, fl->intr_dma);
	usb_free_urb(fl->intr_urb);
	fl->intr_urb = NULL;
	fl->intr_buf = NULL;
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
	kfree(fl->read_buf);
	vfree(fl->convert_buf);
	vfree(fl->write_buf);
	vfree(fl->ready_buf);
	vfree(fl->send_buf);
	fl->read_buf = NULL;
	fl->convert_buf = NULL;
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
	fl->read_buf = kmalloc_array(ALIGN(m->width + 3, 4), sizeof(u32),
				     GFP_KERNEL);
	fl->convert_buf = vzalloc(fl->frame_len);
	fl->write_buf = vzalloc(fl->frame_len);
	fl->ready_buf = vzalloc(fl->frame_len);
	fl->send_buf = vzalloc(fl->frame_len);
	if (!fl->read_buf || !fl->convert_buf || !fl->write_buf ||
	    !fl->ready_buf || !fl->send_buf)
		goto err_free;

	sema_init(&fl->xfer_sem, FL2000_NUM_XFERS);
	fl->xfer_ring = 0;
	atomic_set(&fl->stream_error, 0);
	atomic_set(&fl->inflight, 0);
	fl->next_frame_ns = 0;
	fl->pace_occ = 0;
	fl->pace_occ_target = clamp_t(u32,
				      (fl->frame_len / FL2000_CHUNK_SIZE + 1) / 4,
				      4, 24);
	{
		/* approximate scanout period from the PLL fields (the
		 * reference clock is off by a mode-dependent fraction of a
		 * percent; the FLL refines this from the hw frame counter)
		 */
		u32 mult = (m->pll >> 16) & 0xFF;
		u32 presc = (m->pll >> 8) & 0x3;
		u32 divisor = m->pll & 0xFF;
		u64 pclk = div_u64(10000000ULL * mult, presc * divisor);

		fl->period_pll_ns = div64_u64(
			(u64)(m->h_sync_1 & 0xFFF) * (m->v_sync_1 & 0xFFF) *
			1000000000ULL, pclk);
		fl->frame_period_ns = fl->period_pll_ns;
	}
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
			 "streaming %ux%u@%u, %u bytes/px (fmt=%d pace=%d period=%lluns), 0x8004=0x%08X 0x803C=0x%08X\n",
			 m->width, m->height, m->refresh, fl->wire_bpp, fmt,
			 pace, fl->frame_period_ns, r8004, r803c);
	}

	spin_lock(&fl->frame_swap_lock);
	fl->stream_alive = true;
	spin_unlock(&fl->frame_swap_lock);
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
	fl2000_stream_stopped(fl);
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
	struct drm_framebuffer line_fb = {
		.format = fb ? fb->format : NULL,
	};
	struct drm_rect damage, line;
	struct iosys_map dst, src, read;
	unsigned int dst_pitch;
	size_t offset, read_len, aligned_len;
	int idx, y;
	/* ours from here on: completed at frame pickup, or right away */
	struct drm_pending_vblank_event *event = pipe->crtc.state->event;

	pipe->crtc.state->event = NULL;

	if (!fb || !fl->write_buf || !fl->mode)
		goto send_now;
	if (!drm_atomic_helper_damage_merged(old_state, state, &damage)) {
		/* nothing new to show: still complete at the next pickup */
		spin_lock(&fl->frame_swap_lock);
		event = fl2000_defer_event_locked(fl, event);
		spin_unlock(&fl->frame_swap_lock);
		wake_up(&fl->frame_wq);
		goto send_now;
	}
	if (!drm_dev_enter(&fl->drm, &idx))
		goto send_now;

	/*
	 * Retain undamaged pixels in a commit-owned buffer: the rotating
	 * streaming buffers may contain older frames. Only read damaged
	 * pixels from the framebuffer, which can live in slow GPU memory.
	 */
	dst_pitch = fl->mode->width * fl->wire_bpp;
	offset = (size_t)(damage.y1 - (state->src_y >> 16)) * dst_pitch +
		 (damage.x1 - (state->src_x >> 16)) * fl->wire_bpp;
	iosys_map_set_vaddr(&dst, fl->convert_buf + offset);

	/*
	 * Read aligned chunks with DRM's WC-aware copy helper rather than
	 * ordinary memcpy from GPU memory. The temporary framebuffer only
	 * describes the cached scanline for the format conversion helpers.
	 * Leave the tail unrounded so the final scanline is never overread.
	 */
	line.x1 = damage.x1 & 3;
	line.y1 = 0;
	line.x2 = line.x1 + drm_rect_width(&damage);
	line.y2 = 1;
	read_len = line.x2 * sizeof(u32);
	aligned_len = round_down(read_len, 16);
	line_fb.width = line.x2;
	line_fb.height = 1;
	line_fb.pitches[0] = read_len;
	iosys_map_set_vaddr(&read, fl->read_buf);
	src = shadow->data[0];
	iosys_map_incr(&src, (size_t)damage.y1 * fb->pitches[0] +
			     (damage.x1 - line.x1) * sizeof(u32));

	for (y = damage.y1; y < damage.y2; y++) {
		drm_memcpy_from_wc(&read, &src, aligned_len);
		iosys_map_memcpy_from(fl->read_buf + aligned_len, &src,
				      aligned_len, read_len - aligned_len);
		if (fl->wire_bpp == 2)
			fl2000_fb_xrgb8888_to_rgb565(&dst, &dst_pitch, &read,
						     &line_fb, &line,
						     &shadow->fmtcnv_state);
		else if (fl->wire_bpp == 4)
			drm_fb_memcpy(&dst, &dst_pitch, &read, &line_fb, &line);
		else if (fmt == 3)
			drm_fb_xrgb8888_to_bgr888(&dst, &dst_pitch, &read,
						  &line_fb, &line,
						  &shadow->fmtcnv_state);
		else
			drm_fb_xrgb8888_to_rgb888(&dst, &dst_pitch, &read,
						  &line_fb, &line,
						  &shadow->fmtcnv_state);
		iosys_map_incr(&src, fb->pitches[0]);
		iosys_map_incr(&dst, dst_pitch);
	}

	memcpy(fl->write_buf, fl->convert_buf, fl->frame_len);

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
	event = fl2000_defer_event_locked(fl, event);
	spin_unlock(&fl->frame_swap_lock);
	wake_up(&fl->frame_wq);

	drm_dev_exit(idx);
send_now:
	fl2000_send_event(fl, event, ktime_get());
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

	mutex_lock(&fl->edid_lock);
	if (fl->edid_valid)
		drm_edid = drm_edid_alloc(fl->edid_cache, fl->edid_len);
	mutex_unlock(&fl->edid_lock);

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
	int idx;

	/*
	 * Cached state only — no USB traffic here. The worker re-reads HPD
	 * (ITE HPD or native VGA monitor status) and fires a hotplug event
	 * on change, so a stale answer lasts at most one poll period.
	 */
	if (drm_dev_enter(&fl->drm, &idx)) {
		queue_work(system_unbound_wq, &fl->status_work);
		drm_dev_exit(idx);
	}

	return atomic_read(&fl->hpd_status) ? connector_status_connected :
					      connector_status_disconnected;
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

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 17, 0)
/*
 * Before 6.17, DRM_GEM_SHMEM_DRIVER_OPS imports by mapping the sg_table,
 * so PRIME imports (Xorg output slaving: i915 renders, we scan out) must
 * be DMA-mapped against the USB host controller — the USB interface
 * device itself has no dma_mask and the default import path WARNs and
 * fails, which leaves the slaved output permanently black. From 6.17 the
 * shmem ops import without mapping (CPU access only), which is all the
 * frame update path needs.
 */
static struct drm_gem_object *
fl2000_gem_prime_import(struct drm_device *drm, struct dma_buf *dma_buf)
{
	struct fl2000 *fl = to_fl2000(drm);

	if (!fl->dmadev)
		return ERR_PTR(-ENODEV);
	return drm_gem_prime_import_dev(drm, dma_buf, fl->dmadev);
}
#endif

static const struct drm_driver fl2000_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.fops = &fl2000_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 17, 0)
	.gem_prime_import = fl2000_gem_prime_import,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
	DRM_FBDEV_SHMEM_DRIVER_OPS,
#endif
	.name = DRIVER_NAME,
	.desc = "Fresco Logic FL2000 USB display",
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 14, 0)
	.date = "20260711",
#endif
	.major = 1,
	.minor = 13,
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
	mutex_init(&fl->edid_lock);
	spin_lock_init(&fl->frame_swap_lock);
	init_waitqueue_head(&fl->frame_wq);
	INIT_WORK(&fl->stream_work, fl2000_stream_work);
	INIT_WORK(&fl->status_work, fl2000_status_work);
	INIT_WORK(&fl->intr_work, fl2000_intr_work);

	fl->stream_wq = alloc_workqueue("fl2000-stream",
					WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!fl->stream_wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(&intf->dev,
				       (void (*)(void *))destroy_workqueue,
				       fl->stream_wq);
	if (ret)
		return ret;

#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 17, 0)
	fl->dmadev = usb_intf_get_dma_device(intf);
	if (fl->dmadev) {
		ret = devm_add_action_or_reset(&intf->dev,
					       (void (*)(void *))put_device,
					       fl->dmadev);
		if (ret)
			return ret;
	} else {
		dev_warn(&intf->dev,
			 "no DMA device, buffer sharing (PRIME) disabled\n");
	}
#endif

	/* streaming bulk endpoint lives on interface 0 altsetting 1 */
	ret = usb_set_interface(udev, 0, 1);
	if (ret)
		return ret;

	ret = fl2000_hw_reset(fl);
	if (ret)
		return dev_err_probe(&intf->dev, ret,
				     "device not responding\n");

	fl->ite_present = fl2000_ite_detect(fl);
	if (!fl->ite_present) {
		/*
		 * VGA-only dongles need the 0x8020 detect bits or the DAC
		 * never outputs; on IT66121 dongles the same bits corrupt
		 * the 0x8004 format latch (phantom readback bits) and the
		 * bulk pipe stalls NAK-forever at high modes.
		 */
		ret = fl2000_hw_dongle_init(fl);
		if (ret)
			return dev_err_probe(&intf->dev, ret,
					     "dongle init failed\n");
	}

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
				atomic_set(&fl->hpd_status, 1);
				fl2000_refresh_edid_cache(fl);
				break;
			}
			msleep(50);
		}
	}

	fl2000_intr_start(fl);

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = 640;
	drm->mode_config.max_width = 2560;
	drm->mode_config.min_height = 350;
	drm->mode_config.max_height = 1200;
	drm->mode_config.funcs = &fl2000_mode_config_funcs;

	ret = drm_connector_init(drm, &fl->connector, &fl2000_connector_funcs,
				 fl->ite_present ? DRM_MODE_CONNECTOR_HDMIA :
						   DRM_MODE_CONNECTOR_VGA);
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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 13, 0)
	drm_client_setup(drm, NULL);
#else
	drm_fbdev_shmem_setup(drm, 32);
#endif

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
	struct drm_device *drm;

	/* the claimed interrupt interface tears down with the main one */
	if (!fl || intf != fl->intf)
		return;
	drm = &fl->drm;

	fl2000_intr_stop(fl);
	drm_kms_helper_poll_fini(drm);
	drm_dev_unplug(drm);
	cancel_work_sync(&fl->status_work);
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
