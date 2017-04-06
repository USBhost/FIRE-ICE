/*
 * drivers/video/tegra/host/gk20a/channel_gk20a.c
 *
 * GK20A Graphics channel
 *
 * Copyright (c) 2011-2016, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/nvhost.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/highmem.h> /* need for nvmap.h*/
#include <trace/events/gk20a.h>
#include <linux/scatterlist.h>
#include <linux/file.h>
#include <linux/anon_inodes.h>
#include <linux/dma-buf.h>

#include "debug_gk20a.h"
#include "gk20a.h"

#if defined(CONFIG_TEGRA_GK20A_DEBUG_SESSION)
#include "dbg_gpu_gk20a.h"
#endif

#include "hw_ram_gk20a.h"
#include "hw_fifo_gk20a.h"
#include "hw_pbdma_gk20a.h"
#include "hw_ccsr_gk20a.h"
#include "hw_ltc_gk20a.h"

#define NVMAP_HANDLE_PARAM_SIZE 1

static struct channel_gk20a *acquire_unused_channel(struct fifo_gk20a *f);
static void release_used_channel(struct fifo_gk20a *f, struct channel_gk20a *c);

static void free_priv_cmdbuf(struct channel_gk20a *c,
			     struct priv_cmd_entry *e);
static void recycle_priv_cmdbuf(struct channel_gk20a *c);

static int channel_gk20a_alloc_priv_cmdbuf(struct channel_gk20a *c);
static void channel_gk20a_free_priv_cmdbuf(struct channel_gk20a *c);

static int channel_gk20a_commit_userd(struct channel_gk20a *c);
static int channel_gk20a_setup_userd(struct channel_gk20a *c);
static int channel_gk20a_setup_ramfc(struct channel_gk20a *c,
			u64 gpfifo_base, u32 gpfifo_entries);

static void channel_gk20a_bind(struct channel_gk20a *ch_gk20a);
static void channel_gk20a_unbind(struct channel_gk20a *ch_gk20a);

static int channel_gk20a_alloc_inst(struct gk20a *g,
				struct channel_gk20a *ch);
static void channel_gk20a_free_inst(struct gk20a *g,
				struct channel_gk20a *ch);

static int channel_gk20a_update_runlist(struct channel_gk20a *c,
					bool add);
static void gk20a_free_error_notifiers(struct channel_gk20a *ch);

static struct channel_gk20a *acquire_unused_channel(struct fifo_gk20a *f)
{
	struct channel_gk20a *ch = NULL;
	int chid;

	mutex_lock(&f->ch_inuse_mutex);
	for (chid = 0; chid < f->num_channels; chid++) {
		if (!f->channel[chid].in_use) {
			f->channel[chid].in_use = true;
			ch = &f->channel[chid];
			break;
		}
	}
	mutex_unlock(&f->ch_inuse_mutex);

	return ch;
}

static void release_used_channel(struct fifo_gk20a *f, struct channel_gk20a *c)
{
	mutex_lock(&f->ch_inuse_mutex);
	f->channel[c->hw_chid].in_use = false;
	mutex_unlock(&f->ch_inuse_mutex);
}

int channel_gk20a_commit_va(struct channel_gk20a *c)
{
	u64 addr;
	u32 addr_lo;
	u32 addr_hi;
	void *inst_ptr;

	gk20a_dbg_fn("");

	inst_ptr = c->inst_block.cpuva;
	if (!inst_ptr)
		return -ENOMEM;

	addr = gk20a_mm_iova_addr(c->vm->pdes.sgt->sgl);
	addr_lo = u64_lo32(addr >> 12);
	addr_hi = u64_hi32(addr);

	gk20a_dbg_info("pde pa=0x%llx addr_lo=0x%x addr_hi=0x%x",
		   (u64)addr, addr_lo, addr_hi);

	gk20a_mem_wr32(inst_ptr, ram_in_page_dir_base_lo_w(),
		ram_in_page_dir_base_target_vid_mem_f() |
		ram_in_page_dir_base_vol_true_f() |
		ram_in_page_dir_base_lo_f(addr_lo));

	gk20a_mem_wr32(inst_ptr, ram_in_page_dir_base_hi_w(),
		ram_in_page_dir_base_hi_f(addr_hi));

	gk20a_mem_wr32(inst_ptr, ram_in_adr_limit_lo_w(),
		 u64_lo32(c->vm->va_limit) | 0xFFF);

	gk20a_mem_wr32(inst_ptr, ram_in_adr_limit_hi_w(),
		ram_in_adr_limit_hi_f(u64_hi32(c->vm->va_limit)));

	return 0;
}

static int channel_gk20a_commit_userd(struct channel_gk20a *c)
{
	u32 addr_lo;
	u32 addr_hi;
	void *inst_ptr;

	gk20a_dbg_fn("");

	inst_ptr = c->inst_block.cpuva;
	if (!inst_ptr)
		return -ENOMEM;

	addr_lo = u64_lo32(c->userd_iova >> ram_userd_base_shift_v());
	addr_hi = u64_hi32(c->userd_iova);

	gk20a_dbg_info("channel %d : set ramfc userd 0x%16llx",
		c->hw_chid, (u64)c->userd_iova);

	gk20a_mem_wr32(inst_ptr, ram_in_ramfc_w() + ram_fc_userd_w(),
		 pbdma_userd_target_vid_mem_f() |
		 pbdma_userd_addr_f(addr_lo));

	gk20a_mem_wr32(inst_ptr, ram_in_ramfc_w() + ram_fc_userd_hi_w(),
		 pbdma_userd_target_vid_mem_f() |
		 pbdma_userd_hi_addr_f(addr_hi));

	return 0;
}

static int channel_gk20a_set_schedule_params(struct channel_gk20a *c,
				u32 timeslice_timeout)
{
	void *inst_ptr;
	int shift = 3;
	int value = timeslice_timeout;

	inst_ptr = c->inst_block.cpuva;
	if (!inst_ptr)
		return -ENOMEM;

	/* disable channel */
	gk20a_writel(c->g, ccsr_channel_r(c->hw_chid),
		gk20a_readl(c->g, ccsr_channel_r(c->hw_chid)) |
		ccsr_channel_enable_clr_true_f());

	/* preempt the channel */
	WARN_ON(gk20a_fifo_preempt_channel(c->g, c->hw_chid));

	/* value field is 8 bits long */
	while (value >= 1 << 8) {
		value >>= 1;
		shift++;
	}

	/* time slice register is only 18bits long */
	if ((value << shift) >= 1<<19) {
		pr_err("Requested timeslice value is clamped to 18 bits\n");
		value = 255;
		shift = 10;
	}

	/* set new timeslice */
	gk20a_mem_wr32(inst_ptr, ram_fc_eng_timeslice_w(),
		value | (shift << 12) |
		fifo_eng_timeslice_enable_true_f());

	/* enable channel */
	gk20a_writel(c->g, ccsr_channel_r(c->hw_chid),
		gk20a_readl(c->g, ccsr_channel_r(c->hw_chid)) |
		ccsr_channel_enable_set_true_f());

	return 0;
}

static int channel_gk20a_setup_ramfc(struct channel_gk20a *c,
				u64 gpfifo_base, u32 gpfifo_entries)
{
	void *inst_ptr;

	gk20a_dbg_fn("");

	inst_ptr = c->inst_block.cpuva;
	if (!inst_ptr)
		return -ENOMEM;

	memset(inst_ptr, 0, ram_fc_size_val_v());

	gk20a_mem_wr32(inst_ptr, ram_fc_gp_base_w(),
		pbdma_gp_base_offset_f(
		u64_lo32(gpfifo_base >> pbdma_gp_base_rsvd_s())));

	gk20a_mem_wr32(inst_ptr, ram_fc_gp_base_hi_w(),
		pbdma_gp_base_hi_offset_f(u64_hi32(gpfifo_base)) |
		pbdma_gp_base_hi_limit2_f(ilog2(gpfifo_entries)));

	gk20a_mem_wr32(inst_ptr, ram_fc_signature_w(),
		 pbdma_signature_hw_valid_f() | pbdma_signature_sw_zero_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_formats_w(),
		pbdma_formats_gp_fermi0_f() |
		pbdma_formats_pb_fermi1_f() |
		pbdma_formats_mp_fermi0_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_pb_header_w(),
		pbdma_pb_header_priv_user_f() |
		pbdma_pb_header_method_zero_f() |
		pbdma_pb_header_subchannel_zero_f() |
		pbdma_pb_header_level_main_f() |
		pbdma_pb_header_first_true_f() |
		pbdma_pb_header_type_inc_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_subdevice_w(),
		pbdma_subdevice_id_f(1) |
		pbdma_subdevice_status_active_f() |
		pbdma_subdevice_channel_dma_enable_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_target_w(), pbdma_target_engine_sw_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_acquire_w(),
		pbdma_acquire_retry_man_2_f() |
		pbdma_acquire_retry_exp_2_f() |
		pbdma_acquire_timeout_exp_max_f() |
		pbdma_acquire_timeout_man_max_f() |
		pbdma_acquire_timeout_en_disable_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_eng_timeslice_w(),
		fifo_eng_timeslice_timeout_128_f() |
		fifo_eng_timeslice_timescale_3_f() |
		fifo_eng_timeslice_enable_true_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_pb_timeslice_w(),
		fifo_pb_timeslice_timeout_16_f() |
		fifo_pb_timeslice_timescale_0_f() |
		fifo_pb_timeslice_enable_true_f());

	gk20a_mem_wr32(inst_ptr, ram_fc_chid_w(), ram_fc_chid_id_f(c->hw_chid));

	return 0;
}

static int channel_gk20a_setup_userd(struct channel_gk20a *c)
{
	BUG_ON(!c->userd_cpu_va);

	gk20a_dbg_fn("");

	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_put_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_get_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_ref_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_put_hi_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_ref_threshold_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_gp_top_level_get_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_gp_top_level_get_hi_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_get_hi_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_gp_get_w(), 0);
	gk20a_mem_wr32(c->userd_cpu_va, ram_userd_gp_put_w(), 0);

	return 0;
}

static void channel_gk20a_bind(struct channel_gk20a *ch_gk20a)
{
	struct gk20a *g = ch_gk20a->g;
	struct fifo_gk20a *f = &g->fifo;
	struct fifo_engine_info_gk20a *engine_info =
		f->engine_info + ENGINE_GR_GK20A;

	u32 inst_ptr = ch_gk20a->inst_block.cpu_pa
		>> ram_in_base_shift_v();

	gk20a_dbg_info("bind channel %d inst ptr 0x%08x",
		ch_gk20a->hw_chid, inst_ptr);

	ch_gk20a->bound = true;

	gk20a_writel(g, ccsr_channel_r(ch_gk20a->hw_chid),
		(gk20a_readl(g, ccsr_channel_r(ch_gk20a->hw_chid)) &
		 ~ccsr_channel_runlist_f(~0)) |
		 ccsr_channel_runlist_f(engine_info->runlist_id));

	gk20a_writel(g, ccsr_channel_inst_r(ch_gk20a->hw_chid),
		ccsr_channel_inst_ptr_f(inst_ptr) |
		ccsr_channel_inst_target_vid_mem_f() |
		ccsr_channel_inst_bind_true_f());

	gk20a_writel(g, ccsr_channel_r(ch_gk20a->hw_chid),
		(gk20a_readl(g, ccsr_channel_r(ch_gk20a->hw_chid)) &
		 ~ccsr_channel_enable_set_f(~0)) |
		 ccsr_channel_enable_set_true_f());
}

static void channel_gk20a_unbind(struct channel_gk20a *ch_gk20a)
{
	struct gk20a *g = ch_gk20a->g;

	gk20a_dbg_fn("");

	if (ch_gk20a->bound)
		gk20a_writel(g, ccsr_channel_inst_r(ch_gk20a->hw_chid),
			ccsr_channel_inst_ptr_f(0) |
			ccsr_channel_inst_bind_false_f());

	ch_gk20a->bound = false;

	/*
	 * if we are agrressive then we can destroy the syncpt
	 * resource at this point
	 * if not, then it will be destroyed at channel_free()
	 */
	if (ch_gk20a->sync && ch_gk20a->sync->syncpt_aggressive_destroy) {
		ch_gk20a->sync->destroy(ch_gk20a->sync);
		ch_gk20a->sync = NULL;
	}
}

static int channel_gk20a_alloc_inst(struct gk20a *g,
				struct channel_gk20a *ch)
{
	struct device *d = dev_from_gk20a(g);
	int err = 0;
	dma_addr_t iova;

	gk20a_dbg_fn("");

	ch->inst_block.size = ram_in_alloc_size_v();
	ch->inst_block.cpuva = dma_alloc_coherent(d,
					ch->inst_block.size,
					&iova,
					GFP_KERNEL);
	if (!ch->inst_block.cpuva) {
		gk20a_err(d, "%s: memory allocation failed\n", __func__);
		err = -ENOMEM;
		goto clean_up;
	}

	ch->inst_block.iova = iova;
	ch->inst_block.cpu_pa = gk20a_get_phys_from_iova(d,
							ch->inst_block.iova);
	if (!ch->inst_block.cpu_pa) {
		gk20a_err(d, "%s: failed to get physical address\n", __func__);
		err = -ENOMEM;
		goto clean_up;
	}

	gk20a_dbg_info("channel %d inst block physical addr: 0x%16llx",
		ch->hw_chid, (u64)ch->inst_block.cpu_pa);

	gk20a_dbg_fn("done");
	return 0;

clean_up:
	gk20a_err(d, "fail");
	channel_gk20a_free_inst(g, ch);
	return err;
}

static void channel_gk20a_free_inst(struct gk20a *g,
				struct channel_gk20a *ch)
{
	struct device *d = dev_from_gk20a(g);

	if (ch->inst_block.cpuva)
		dma_free_coherent(d, ch->inst_block.size,
				ch->inst_block.cpuva, ch->inst_block.iova);
	ch->inst_block.cpuva = NULL;
	ch->inst_block.iova = 0;
	memset(&ch->inst_block, 0, sizeof(struct inst_desc));
}

static int channel_gk20a_update_runlist(struct channel_gk20a *c, bool add)
{
	return gk20a_fifo_update_runlist(c->g, 0, c->hw_chid, add, true);
}

void gk20a_disable_channel_no_update(struct channel_gk20a *ch)
{
	/* ensure no fences are pending */
	if (ch->sync)
		ch->sync->set_min_eq_max(ch->sync);

	/* disable channel */
	gk20a_writel(ch->g, ccsr_channel_r(ch->hw_chid),
		     gk20a_readl(ch->g,
		     ccsr_channel_r(ch->hw_chid)) |
		     ccsr_channel_enable_clr_true_f());
}

int gk20a_wait_channel_idle(struct channel_gk20a *ch)
{
	bool channel_idle = false;
	unsigned long end_jiffies = jiffies +
		msecs_to_jiffies(gk20a_get_gr_idle_timeout(ch->g));

	do {
		mutex_lock(&ch->jobs_lock);
		channel_idle = list_empty(&ch->jobs);
		mutex_unlock(&ch->jobs_lock);
		if (channel_idle)
			break;

		usleep_range(1000, 3000);
	} while (time_before(jiffies, end_jiffies)
			|| !tegra_platform_is_silicon());

	if (!channel_idle) {
		gk20a_err(dev_from_gk20a(ch->g), "jobs not freed for channel %d\n",
				ch->hw_chid);
		return -EBUSY;
	}

	return 0;
}

void gk20a_disable_channel(struct channel_gk20a *ch,
			   bool finish,
			   unsigned long finish_timeout)
{
	if (finish) {
		int err = gk20a_channel_finish(ch, finish_timeout);
		WARN_ON(err);
	}

	/* disable the channel from hw and increment syncpoints */
	gk20a_disable_channel_no_update(ch);

	gk20a_wait_channel_idle(ch);

	/* preempt the channel */
	gk20a_fifo_preempt_channel(ch->g, ch->hw_chid);

	/* remove channel from runlist */
	channel_gk20a_update_runlist(ch, false);
}

#if defined(CONFIG_GK20A_CYCLE_STATS)

static void gk20a_free_cycle_stats_buffer(struct channel_gk20a *ch)
{
	/* disable existing cyclestats buffer */
	mutex_lock(&ch->cyclestate.cyclestate_buffer_mutex);
	if (ch->cyclestate.cyclestate_buffer_handler) {
		dma_buf_vunmap(ch->cyclestate.cyclestate_buffer_handler,
				ch->cyclestate.cyclestate_buffer);
		dma_buf_put(ch->cyclestate.cyclestate_buffer_handler);
		ch->cyclestate.cyclestate_buffer_handler = NULL;
		ch->cyclestate.cyclestate_buffer = NULL;
		ch->cyclestate.cyclestate_buffer_size = 0;
	}
	mutex_unlock(&ch->cyclestate.cyclestate_buffer_mutex);
}

static int gk20a_channel_cycle_stats(struct channel_gk20a *ch,
		       struct nvhost_cycle_stats_args *args)
{
	struct dma_buf *dmabuf;
	void *virtual_address;

	if (args->nvmap_handle && !ch->cyclestate.cyclestate_buffer_handler) {

		/* set up new cyclestats buffer */
		dmabuf = dma_buf_get(args->nvmap_handle);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);
		virtual_address = dma_buf_vmap(dmabuf);
		if (!virtual_address)
			return -ENOMEM;

		ch->cyclestate.cyclestate_buffer_handler = dmabuf;
		ch->cyclestate.cyclestate_buffer = virtual_address;
		ch->cyclestate.cyclestate_buffer_size = dmabuf->size;
		return 0;

	} else if (!args->nvmap_handle &&
			ch->cyclestate.cyclestate_buffer_handler) {
		gk20a_free_cycle_stats_buffer(ch);
		return 0;

	} else if (!args->nvmap_handle &&
			!ch->cyclestate.cyclestate_buffer_handler) {
		/* no requst from GL */
		return 0;

	} else {
		pr_err("channel already has cyclestats buffer\n");
		return -EINVAL;
	}
}
#endif

static int gk20a_init_error_notifier(struct channel_gk20a *ch,
		struct nvhost_set_error_notifier *args)
{
	struct dma_buf *dmabuf;
	void *va;
	u64 end;

	if (unlikely(args->offset >
		     U64_MAX - sizeof(struct nvhost_notification)))
		return -EINVAL;

	end = args->offset + sizeof(struct nvhost_notification);

	if (!args->mem) {
		pr_err("gk20a_init_error_notifier: invalid memory handle\n");
		return -EINVAL;
	}

	dmabuf = dma_buf_get(args->mem);

	gk20a_free_error_notifiers(ch);

	if (IS_ERR(dmabuf)) {
		pr_err("Invalid handle: %d\n", args->mem);
		return -EINVAL;
	}

	if (end > dmabuf->size || end < sizeof(struct nvhost_notification)) {
		dma_buf_put(dmabuf);
		pr_err("gk20a_init_error_notifier: invalid offset\n");
		return -EINVAL;
	}

	/* map handle */
	va = dma_buf_vmap(dmabuf);
	if (!va) {
		dma_buf_put(dmabuf);
		pr_err("Cannot map notifier handle\n");
		return -ENOMEM;
	}

	ch->error_notifier = va + args->offset;
	ch->error_notifier_va = va;
	memset(ch->error_notifier, 0, sizeof(struct nvhost_notification));

	/* set channel notifiers pointer */
	mutex_lock(&ch->error_notifier_mutex);
	ch->error_notifier_ref = dmabuf;
	mutex_unlock(&ch->error_notifier_mutex);

	return 0;
}

void gk20a_set_error_notifier(struct channel_gk20a *ch, __u32 error)
{
	bool notifier_set = false;

	mutex_lock(&ch->error_notifier_mutex);
	if (ch->error_notifier_ref) {
		struct timespec time_data;
		u64 nsec;
		getnstimeofday(&time_data);
		nsec = ((u64)time_data.tv_sec) * 1000000000u +
				(u64)time_data.tv_nsec;
		ch->error_notifier->time_stamp.nanoseconds[0] =
				(u32)nsec;
		ch->error_notifier->time_stamp.nanoseconds[1] =
				(u32)(nsec >> 32);
		ch->error_notifier->info32 = error;
		ch->error_notifier->status = 0xffff;

		notifier_set = true;
	}
	mutex_unlock(&ch->error_notifier_mutex);

	if (notifier_set)
		gk20a_err(dev_from_gk20a(ch->g),
		    "error notifier set to %d for ch %d", error, ch->hw_chid);
}

static void gk20a_free_error_notifiers(struct channel_gk20a *ch)
{
	mutex_lock(&ch->error_notifier_mutex);
	if (ch->error_notifier_ref) {
		dma_buf_vunmap(ch->error_notifier_ref, ch->error_notifier_va);
		dma_buf_put(ch->error_notifier_ref);
		ch->error_notifier_ref = NULL;
		ch->error_notifier = NULL;
		ch->error_notifier_va = NULL;
	}
	mutex_unlock(&ch->error_notifier_mutex);
}

void gk20a_free_channel(struct channel_gk20a *ch, bool finish)
{
	struct gk20a *g = ch->g;
	struct device *d = dev_from_gk20a(g);
	struct fifo_gk20a *f = &g->fifo;
	struct gr_gk20a *gr = &g->gr;
	struct vm_gk20a *ch_vm = ch->vm;
	unsigned long timeout = gk20a_get_gr_idle_timeout(g);
#if defined(CONFIG_TEGRA_GK20A_DEBUG_SESSION)
	struct dbg_session_gk20a *dbg_s;
#endif

	gk20a_dbg_fn("");

	/* if engine reset was deferred, perform it now */
	mutex_lock(&f->deferred_reset_mutex);
	if (g->fifo.deferred_reset_pending) {
		gk20a_dbg(gpu_dbg_intr | gpu_dbg_gpu_dbg, "engine reset was"
			   " deferred, running now");
		fifo_gk20a_finish_mmu_fault_handling(g, g->fifo.mmu_fault_engines);
		g->fifo.mmu_fault_engines = 0;
		g->fifo.deferred_reset_pending = false;
	}
	mutex_unlock(&f->deferred_reset_mutex);

	if (!ch->bound)
		return;

	if (!gk20a_channel_as_bound(ch))
		goto unbind;

	gk20a_dbg_info("freeing bound channel context, timeout=%ld",
			timeout);

	gk20a_disable_channel(ch, finish && !ch->has_timedout, timeout);

	gk20a_free_error_notifiers(ch);

	/* release channel ctx */
	gk20a_free_channel_ctx(ch);

	gk20a_gr_flush_channel_tlb(gr);

	memset(&ch->ramfc, 0, sizeof(struct mem_desc_sub));

	/* free gpfifo */
	if (ch->vm && ch->gpfifo.gpu_va)
		gk20a_gmmu_unmap(ch_vm, ch->gpfifo.gpu_va,
			ch->gpfifo.size, gk20a_mem_flag_none);
	if (ch->gpfifo.cpu_va)
		dma_free_coherent(d, ch->gpfifo.size,
			ch->gpfifo.cpu_va, ch->gpfifo.iova);
	ch->gpfifo.cpu_va = NULL;
	ch->gpfifo.iova = 0;

	memset(&ch->gpfifo, 0, sizeof(struct gpfifo_desc));

#if defined(CONFIG_GK20A_CYCLE_STATS)
	gk20a_free_cycle_stats_buffer(ch);
#endif

	channel_gk20a_free_priv_cmdbuf(ch);

	if (ch->sync) {
		ch->sync->destroy(ch->sync);
		ch->sync = NULL;
	}

	/* release channel binding to the as_share */
	gk20a_as_release_share(ch_vm->as_share);

unbind:
	channel_gk20a_unbind(ch);
	channel_gk20a_free_inst(g, ch);

	gk20a_vm_put(ch->vm); /* Don't use VM after this. */
	ch->vm = NULL;
	ch->vpr = false;
	WARN_ON(ch->sync);

	/* unlink all debug sessions */
	mutex_lock(&ch->dbg_s_lock);

#if defined(CONFIG_TEGRA_GK20A_DEBUG_SESSION)
	list_for_each_entry(dbg_s, &ch->dbg_s_list, dbg_s_list_node) {
		dbg_s->ch = NULL;
		list_del_init(&dbg_s->dbg_s_list_node);
	}
#endif

	mutex_unlock(&ch->dbg_s_lock);

	/* ALWAYS last */
	release_used_channel(f, ch);
}

int gk20a_channel_release(struct inode *inode, struct file *filp)
{
	struct channel_gk20a *ch = (struct channel_gk20a *)filp->private_data;
	struct gk20a *g;
	int err;

	if (!ch || !ch->g)
		return -EFAULT;

	g = ch->g;
	trace_gk20a_channel_release(dev_name(&g->dev->dev));

	err = gk20a_busy(ch->g->dev);
	if (err) {
		gk20a_err(dev_from_gk20a(g), "failed to release channel %d",
			ch->hw_chid);
		return err;
	}
	gk20a_free_channel(ch, true);
	gk20a_idle(ch->g->dev);

	gk20a_put_client(g);
	filp->private_data = NULL;
	return 0;
}

static struct channel_gk20a *gk20a_open_new_channel(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	struct channel_gk20a *ch;

	ch = acquire_unused_channel(f);
	if (ch == NULL) {
		/* TBD: we want to make this virtualizable */
		gk20a_err(dev_from_gk20a(g), "out of hw chids");
		return 0;
	}

	ch->g = g;

	if (channel_gk20a_alloc_inst(g, ch)) {
		ch->in_use = false;
		gk20a_err(dev_from_gk20a(g),
			   "failed to open gk20a channel, out of inst mem");

		return 0;
	}
	g->ops.fifo.bind_channel(ch);
	ch->pid = current->pid;

	/* reset timeout counter and update timestamp */
	ch->timeout_accumulated_ms = 0;
	ch->timeout_gpfifo_get = 0;
	/* set gr host default timeout */
	ch->timeout_ms_max = gk20a_get_gr_idle_timeout(g);
	ch->timeout_debug_dump = true;
	ch->has_timedout = false;
	ch->obj_class = 0;

	/* The channel is *not* runnable at this point. It still needs to have
	 * an address space bound and allocate a gpfifo and grctx. */

	init_waitqueue_head(&ch->notifier_wq);
	init_waitqueue_head(&ch->semaphore_wq);
	init_waitqueue_head(&ch->submit_wq);

	return ch;
}

static int __gk20a_channel_open(struct gk20a *g, struct file *filp)
{
	int err;
	struct channel_gk20a *ch;

	trace_gk20a_channel_open(dev_name(&g->dev->dev));

	err = gk20a_get_client(g);
	if (err) {
		gk20a_err(dev_from_gk20a(g),
			"failed to get client ref");
		return err;
	}

	err = gk20a_busy(g->dev);
	if (err) {
		gk20a_put_client(g);
		gk20a_err(dev_from_gk20a(g), "failed to power on, %d", err);
		return err;
	}
	ch = gk20a_open_new_channel(g);
	gk20a_idle(g->dev);
	if (!ch) {
		gk20a_put_client(g);
		gk20a_err(dev_from_gk20a(g),
			"failed to get f");
		return -ENOMEM;
	}

	filp->private_data = ch;
	return 0;
}

int gk20a_channel_open(struct inode *inode, struct file *filp)
{
	struct gk20a *g = container_of(inode->i_cdev,
			struct gk20a, channel.cdev);
	return __gk20a_channel_open(g, filp);
}

/* allocate private cmd buffer.
   used for inserting commands before/after user submitted buffers. */
static int channel_gk20a_alloc_priv_cmdbuf(struct channel_gk20a *c)
{
	struct device *d = dev_from_gk20a(c->g);
	struct vm_gk20a *ch_vm = c->vm;
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	struct priv_cmd_entry *e;
	u32 i = 0, size;
	int err = 0;
	struct sg_table *sgt;
	dma_addr_t iova;

	/* Kernel can insert gpfifos before and after user gpfifos.
	   Before user gpfifos, kernel inserts fence_wait, which takes
	   syncpoint_a (2 dwords) + syncpoint_b (2 dwords) = 4 dwords.
	   After user gpfifos, kernel inserts fence_get, which takes
	   wfi (2 dwords) + syncpoint_a (2 dwords) + syncpoint_b (2 dwords)
	   = 6 dwords.
	   Worse case if kernel adds both of them for every user gpfifo,
	   max size of priv_cmdbuf is :
	   (gpfifo entry number * (2 / 3) * (4 + 6) * 4 bytes */
	size = roundup_pow_of_two(
		c->gpfifo.entry_num * 2 * 10 * sizeof(u32) / 3);

	q->mem.base_cpuva = dma_alloc_coherent(d, size,
					&iova,
					GFP_KERNEL);
	if (!q->mem.base_cpuva) {
		gk20a_err(d, "%s: memory allocation failed\n", __func__);
		err = -ENOMEM;
		goto clean_up;
	}

	q->mem.base_iova = iova;
	q->mem.size = size;

	err = gk20a_get_sgtable(d, &sgt,
			q->mem.base_cpuva, q->mem.base_iova, size);
	if (err) {
		gk20a_err(d, "%s: failed to create sg table\n", __func__);
		goto clean_up;
	}

	memset(q->mem.base_cpuva, 0, size);

	q->base_gpuva = gk20a_gmmu_map(ch_vm, &sgt,
					size,
					0, /* flags */
					gk20a_mem_flag_none);
	if (!q->base_gpuva) {
		gk20a_err(d, "ch %d : failed to map gpu va"
			   "for priv cmd buffer", c->hw_chid);
		err = -ENOMEM;
		goto clean_up_sgt;
	}

	q->size = q->mem.size / sizeof (u32);

	INIT_LIST_HEAD(&q->head);
	INIT_LIST_HEAD(&q->free);

	/* pre-alloc 25% of priv cmdbuf entries and put them on free list */
	for (i = 0; i < q->size / 4; i++) {
		e = kzalloc(sizeof(struct priv_cmd_entry), GFP_KERNEL);
		if (!e) {
			gk20a_err(d, "ch %d: fail to pre-alloc cmd entry",
				c->hw_chid);
			err = -ENOMEM;
			goto clean_up_sgt;
		}
		e->pre_alloc = true;
		list_add(&e->list, &q->free);
	}

	gk20a_free_sgtable(&sgt);

	return 0;

clean_up_sgt:
	gk20a_free_sgtable(&sgt);
clean_up:
	channel_gk20a_free_priv_cmdbuf(c);
	return err;
}

static void channel_gk20a_free_priv_cmdbuf(struct channel_gk20a *c)
{
	struct device *d = dev_from_gk20a(c->g);
	struct vm_gk20a *ch_vm = c->vm;
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	struct priv_cmd_entry *e;
	struct list_head *pos, *tmp, *head;

	if (q->size == 0)
		return;

	if (q->base_gpuva)
		gk20a_gmmu_unmap(ch_vm, q->base_gpuva,
				q->mem.size, gk20a_mem_flag_none);
	if (q->mem.base_cpuva)
		dma_free_coherent(d, q->mem.size,
			q->mem.base_cpuva, q->mem.base_iova);
	q->mem.base_cpuva = NULL;
	q->mem.base_iova = 0;

	/* free used list */
	head = &q->head;
	list_for_each_safe(pos, tmp, head) {
		e = container_of(pos, struct priv_cmd_entry, list);
		free_priv_cmdbuf(c, e);
	}

	/* free free list */
	head = &q->free;
	list_for_each_safe(pos, tmp, head) {
		e = container_of(pos, struct priv_cmd_entry, list);
		e->pre_alloc = false;
		free_priv_cmdbuf(c, e);
	}

	memset(q, 0, sizeof(struct priv_cmd_queue));
}

/* allocate a cmd buffer with given size. size is number of u32 entries */
int gk20a_channel_alloc_priv_cmdbuf(struct channel_gk20a *c, u32 orig_size,
			     struct priv_cmd_entry **entry)
{
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	struct priv_cmd_entry *e;
	struct list_head *node;
	u32 free_count;
	u32 size = orig_size;
	bool no_retry = false;

	gk20a_dbg_fn("size %d", orig_size);

	*entry = NULL;

	/* if free space in the end is less than requested, increase the size
	 * to make the real allocated space start from beginning. */
	if (q->put + size > q->size)
		size = orig_size + (q->size - q->put);

	gk20a_dbg_info("ch %d: priv cmd queue get:put %d:%d",
			c->hw_chid, q->get, q->put);

TRY_AGAIN:
	free_count = (q->size - (q->put - q->get) - 1) % q->size;

	if (size > free_count) {
		if (!no_retry) {
			recycle_priv_cmdbuf(c);
			no_retry = true;
			goto TRY_AGAIN;
		} else
			return -EAGAIN;
	}

	if (unlikely(list_empty(&q->free))) {

		gk20a_dbg_info("ch %d: run out of pre-alloc entries",
			c->hw_chid);

		e = kzalloc(sizeof(struct priv_cmd_entry), GFP_KERNEL);
		if (!e) {
			gk20a_err(dev_from_gk20a(c->g),
				"ch %d: fail to allocate priv cmd entry",
				c->hw_chid);
			return -ENOMEM;
		}
	} else  {
		node = q->free.next;
		list_del(node);
		e = container_of(node, struct priv_cmd_entry, list);
	}

	e->size = orig_size;
	e->gp_get = c->gpfifo.get;
	e->gp_put = c->gpfifo.put;
	e->gp_wrap = c->gpfifo.wrap;

	/* if we have increased size to skip free space in the end, set put
	   to beginning of cmd buffer (0) + size */
	if (size != orig_size) {
		e->ptr = q->mem.base_cpuva;
		e->gva = q->base_gpuva;
		q->put = orig_size;
	} else {
		e->ptr = q->mem.base_cpuva + q->put;
		e->gva = q->base_gpuva + q->put * sizeof(u32);
		q->put = (q->put + orig_size) & (q->size - 1);
	}

	/* we already handled q->put + size > q->size so BUG_ON this */
	BUG_ON(q->put > q->size);

	/* add new entry to head since we free from head */
	list_add(&e->list, &q->head);

	*entry = e;

	gk20a_dbg_fn("done");

	return 0;
}

/* Don't call this to free an explict cmd entry.
 * It doesn't update priv_cmd_queue get/put */
static void free_priv_cmdbuf(struct channel_gk20a *c,
			     struct priv_cmd_entry *e)
{
	struct priv_cmd_queue *q = &c->priv_cmd_q;

	if (!e)
		return;

	list_del(&e->list);

	if (unlikely(!e->pre_alloc))
		kfree(e);
	else {
		memset(e, 0, sizeof(struct priv_cmd_entry));
		e->pre_alloc = true;
		list_add(&e->list, &q->free);
	}
}

/* free entries if they're no longer being used */
static void recycle_priv_cmdbuf(struct channel_gk20a *c)
{
	struct priv_cmd_queue *q = &c->priv_cmd_q;
	struct priv_cmd_entry *e, *tmp;
	struct list_head *head = &q->head;
	bool wrap_around, found = false;

	gk20a_dbg_fn("");

	/* Find the most recent free entry. Free it and everything before it */
	list_for_each_entry(e, head, list) {

		gk20a_dbg_info("ch %d: cmd entry get:put:wrap %d:%d:%d "
			"curr get:put:wrap %d:%d:%d",
			c->hw_chid, e->gp_get, e->gp_put, e->gp_wrap,
			c->gpfifo.get, c->gpfifo.put, c->gpfifo.wrap);

		wrap_around = (c->gpfifo.wrap != e->gp_wrap);
		if (e->gp_get < e->gp_put) {
			if (c->gpfifo.get >= e->gp_put ||
			    wrap_around) {
				found = true;
				break;
			} else
				e->gp_get = c->gpfifo.get;
		} else if (e->gp_get > e->gp_put) {
			if (wrap_around &&
			    c->gpfifo.get >= e->gp_put) {
				found = true;
				break;
			} else
				e->gp_get = c->gpfifo.get;
		}
	}

	if (found)
		q->get = (e->ptr - q->mem.base_cpuva) + e->size;
	else {
		gk20a_dbg_info("no free entry recycled");
		return;
	}

	list_for_each_entry_safe_continue(e, tmp, head, list) {
		free_priv_cmdbuf(c, e);
	}

	gk20a_dbg_fn("done");
}


static int gk20a_alloc_channel_gpfifo(struct channel_gk20a *c,
				      struct nvhost_alloc_gpfifo_args *args)
{
	struct gk20a *g = c->g;
	struct device *d = dev_from_gk20a(g);
	struct vm_gk20a *ch_vm;
	u32 gpfifo_size;
	int err = 0;
	struct sg_table *sgt;
	dma_addr_t iova;

	/* Kernel can insert one extra gpfifo entry before user submitted gpfifos
	   and another one after, for internal usage. Triple the requested size. */
	gpfifo_size = roundup_pow_of_two(args->num_entries * 3);

	if (args->flags & NVHOST_ALLOC_GPFIFO_FLAGS_VPR_ENABLED)
		c->vpr = true;

	/* an address space needs to have been bound at this point.   */
	if (!gk20a_channel_as_bound(c)) {
		gk20a_err(d,
			    "not bound to an address space at time of gpfifo"
			    " allocation.  Attempting to create and bind to"
			    " one...");
		return -EINVAL;
	}
	ch_vm = c->vm;

	c->cmds_pending = false;
	c->last_submit_fence.valid = false;

	c->ramfc.offset = 0;
	c->ramfc.size = ram_in_ramfc_s() / 8;

	if (c->gpfifo.cpu_va) {
		gk20a_err(d, "channel %d :"
			   "gpfifo already allocated", c->hw_chid);
		return -EEXIST;
	}

	c->gpfifo.size = gpfifo_size * sizeof(struct gpfifo);
	c->gpfifo.cpu_va = (struct gpfifo *)dma_alloc_coherent(d,
						c->gpfifo.size,
						&iova,
						GFP_KERNEL);
	if (!c->gpfifo.cpu_va) {
		gk20a_err(d, "%s: memory allocation failed\n", __func__);
		err = -ENOMEM;
		goto clean_up;
	}

	c->gpfifo.iova = iova;
	c->gpfifo.entry_num = gpfifo_size;

	c->gpfifo.get = c->gpfifo.put = 0;

	err = gk20a_get_sgtable(d, &sgt,
			c->gpfifo.cpu_va, c->gpfifo.iova, c->gpfifo.size);
	if (err) {
		gk20a_err(d, "%s: failed to allocate sg table\n", __func__);
		goto clean_up;
	}

	c->gpfifo.gpu_va = gk20a_gmmu_map(ch_vm,
					&sgt,
					c->gpfifo.size,
					0, /* flags */
					gk20a_mem_flag_none);
	if (!c->gpfifo.gpu_va) {
		gk20a_err(d, "channel %d : failed to map"
			   " gpu_va for gpfifo", c->hw_chid);
		err = -ENOMEM;
		goto clean_up_sgt;
	}

	gk20a_dbg_info("channel %d : gpfifo_base 0x%016llx, size %d",
		c->hw_chid, c->gpfifo.gpu_va, c->gpfifo.entry_num);

	channel_gk20a_setup_ramfc(c, c->gpfifo.gpu_va, c->gpfifo.entry_num);

	channel_gk20a_setup_userd(c);
	channel_gk20a_commit_userd(c);

	/* TBD: setup engine contexts */

	err = channel_gk20a_alloc_priv_cmdbuf(c);
	if (err)
		goto clean_up_unmap;

	err = channel_gk20a_update_runlist(c, true);
	if (err)
		goto clean_up_unmap;

	gk20a_free_sgtable(&sgt);

	gk20a_dbg_fn("done");
	return 0;

clean_up_unmap:
	gk20a_gmmu_unmap(ch_vm, c->gpfifo.gpu_va,
		c->gpfifo.size, gk20a_mem_flag_none);
clean_up_sgt:
	gk20a_free_sgtable(&sgt);
clean_up:
	dma_free_coherent(d, c->gpfifo.size,
		c->gpfifo.cpu_va, c->gpfifo.iova);
	c->gpfifo.cpu_va = NULL;
	c->gpfifo.iova = 0;
	memset(&c->gpfifo, 0, sizeof(struct gpfifo_desc));
	gk20a_err(d, "fail");
	return err;
}

static inline int wfi_cmd_size(void)
{
	return 2;
}
void add_wfi_cmd(struct priv_cmd_entry *cmd, int *i)
{
	/* wfi */
	cmd->ptr[(*i)++] = 0x2001001E;
	/* handle, ignored */
	cmd->ptr[(*i)++] = 0x00000000;
}

static inline bool check_gp_put(struct gk20a *g,
				struct channel_gk20a *c)
{
	u32 put;
	/* gp_put changed unexpectedly since last update? */
	put = gk20a_bar1_readl(g,
	       c->userd_gpu_va + 4 * ram_userd_gp_put_w());
	if (c->gpfifo.put != put) {
		/*TBD: BUG_ON/teardown on this*/
		gk20a_err(dev_from_gk20a(g), "gp_put changed unexpectedly "
			   "since last update");
		c->gpfifo.put = put;
		return false; /* surprise! */
	}
	return true; /* checked out ok */
}

/* Update with this periodically to determine how the gpfifo is draining. */
static inline u32 update_gp_get(struct gk20a *g,
				struct channel_gk20a *c)
{
	u32 new_get = gk20a_bar1_readl(g,
		c->userd_gpu_va + sizeof(u32) * ram_userd_gp_get_w());
	if (new_get < c->gpfifo.get)
		c->gpfifo.wrap = !c->gpfifo.wrap;
	c->gpfifo.get = new_get;
	return new_get;
}

static inline u32 gp_free_count(struct channel_gk20a *c)
{
	return (c->gpfifo.entry_num - (c->gpfifo.put - c->gpfifo.get) - 1) %
		c->gpfifo.entry_num;
}

bool gk20a_channel_update_and_check_timeout(struct channel_gk20a *ch,
		u32 timeout_delta_ms)
{
	u32 gpfifo_get = update_gp_get(ch->g, ch);
	/* Count consequent timeout isr */
	if (gpfifo_get == ch->timeout_gpfifo_get) {
		/* we didn't advance since previous channel timeout check */
		ch->timeout_accumulated_ms += timeout_delta_ms;
	} else {
		/* first timeout isr encountered */
		ch->timeout_accumulated_ms = timeout_delta_ms;
	}

	ch->timeout_gpfifo_get = gpfifo_get;

	return ch->g->timeouts_enabled &&
		ch->timeout_accumulated_ms > ch->timeout_ms_max;
}


/* Issue a syncpoint increment *preceded* by a wait-for-idle
 * command.  All commands on the channel will have been
 * consumed at the time the fence syncpoint increment occurs.
 */
static int gk20a_channel_submit_wfi(struct channel_gk20a *c)
{
	struct priv_cmd_entry *cmd = NULL;
	struct gk20a *g = c->g;
	u32 free_count;
	int err;

	if (c->has_timedout)
		return -ETIMEDOUT;

	if (!c->sync) {
		c->sync = gk20a_channel_sync_create(c);
		if (!c->sync)
			return -ENOMEM;
	}

	update_gp_get(g, c);
	free_count = gp_free_count(c);
	if (unlikely(!free_count)) {
		gk20a_err(dev_from_gk20a(g),
			   "not enough gpfifo space");
		return -EAGAIN;
	}

	err = c->sync->incr_wfi(c->sync, &cmd, &c->last_submit_fence);
	if (unlikely(err))
		return err;

	WARN_ON(!c->last_submit_fence.wfi);

	c->gpfifo.cpu_va[c->gpfifo.put].entry0 = u64_lo32(cmd->gva);
	c->gpfifo.cpu_va[c->gpfifo.put].entry1 = u64_hi32(cmd->gva) |
		pbdma_gp_entry1_length_f(cmd->size);

	c->gpfifo.put = (c->gpfifo.put + 1) & (c->gpfifo.entry_num - 1);

	/* save gp_put */
	cmd->gp_put = c->gpfifo.put;

	gk20a_bar1_writel(g,
		c->userd_gpu_va + 4 * ram_userd_gp_put_w(),
		c->gpfifo.put);

	gk20a_dbg_info("post-submit put %d, get %d, size %d",
		c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

	return 0;
}

static u32 get_gp_free_count(struct channel_gk20a *c)
{
	update_gp_get(c->g, c);
	return gp_free_count(c);
}

static void trace_write_pushbuffer(struct channel_gk20a *c, struct gpfifo *g)
{
	void *mem = NULL;
	unsigned int words;
	u64 offset;
	struct dma_buf *dmabuf = NULL;

	if (gk20a_debug_trace_cmdbuf) {
		u64 gpu_va = (u64)g->entry0 |
			(u64)((u64)pbdma_gp_entry1_get_hi_v(g->entry1) << 32);
		int err;

		words = pbdma_gp_entry1_length_v(g->entry1);
		err = gk20a_vm_find_buffer(c->vm, gpu_va, &dmabuf, &offset);
		if (!err)
			mem = dma_buf_vmap(dmabuf);
	}

	if (mem) {
		u32 i;
		/*
		 * Write in batches of 128 as there seems to be a limit
		 * of how much you can output to ftrace at once.
		 */
		for (i = 0; i < words; i += 128U) {
			trace_gk20a_push_cmdbuf(
				c->g->dev->name,
				0,
				min(words - i, 128U),
				offset + i * sizeof(u32),
				mem);
		}
		dma_buf_vunmap(dmabuf, mem);
	}
}

static int gk20a_channel_add_job(struct channel_gk20a *c,
				 struct gk20a_channel_fence *fence)
{
	struct vm_gk20a *vm = c->vm;
	struct channel_gk20a_job *job = NULL;
	struct mapped_buffer_node **mapped_buffers = NULL;
	int err = 0, num_mapped_buffers;

	/* job needs reference to this vm */
	gk20a_vm_get(vm);

	err = gk20a_vm_get_buffers(vm, &mapped_buffers, &num_mapped_buffers);
	if (err) {
		gk20a_vm_put(vm);
		return err;
	}

	job = kzalloc(sizeof(*job), GFP_KERNEL);
	if (!job) {
		gk20a_vm_put_buffers(vm, mapped_buffers, num_mapped_buffers);
		gk20a_vm_put(vm);
		return -ENOMEM;
	}

	job->num_mapped_buffers = num_mapped_buffers;
	job->mapped_buffers = mapped_buffers;
	job->fence = *fence;

	mutex_lock(&c->jobs_lock);
	list_add_tail(&job->list, &c->jobs);
	mutex_unlock(&c->jobs_lock);

	return 0;
}

void gk20a_channel_update(struct channel_gk20a *c, int nr_completed)
{
	struct vm_gk20a *vm = c->vm;
	struct channel_gk20a_job *job, *n;

	wake_up(&c->submit_wq);

	mutex_lock(&c->jobs_lock);
	list_for_each_entry_safe(job, n, &c->jobs, list) {
		bool completed = WARN_ON(!c->sync) ||
			c->sync->is_expired(c->sync, &job->fence);
		if (!completed)
			break;

		gk20a_vm_put_buffers(vm, job->mapped_buffers,
				job->num_mapped_buffers);

		/* job is done. release its reference to vm */
		gk20a_vm_put(vm);

		list_del_init(&job->list);
		kfree(job);
		gk20a_idle(c->g->dev);
	}
	mutex_unlock(&c->jobs_lock);
}

static int gk20a_submit_channel_gpfifo(struct channel_gk20a *c,
				struct nvhost_gpfifo *gpfifo,
				u32 num_entries,
				struct nvhost_fence *fence,
				u32 flags)
{
	struct gk20a *g = c->g;
	struct device *d = dev_from_gk20a(g);
	int err = 0;
	int i;
	struct priv_cmd_entry *wait_cmd = NULL;
	struct priv_cmd_entry *incr_cmd = NULL;
	/* we might need two extra gpfifo entries - one for pre fence
	 * and one for post fence. */
	const int extra_entries = 2;

	if (c->has_timedout)
		return -ETIMEDOUT;

	if ((flags & (NVHOST_SUBMIT_GPFIFO_FLAGS_FENCE_WAIT |
		      NVHOST_SUBMIT_GPFIFO_FLAGS_FENCE_GET)) &&
	    !fence)
		return -EINVAL;

	if (!c->sync) {
		c->sync = gk20a_channel_sync_create(c);
		if (!c->sync)
			return -ENOMEM;
	}

#ifdef CONFIG_DEBUG_FS
	/* update debug settings */
	if (g->ops.ltc.sync_debugfs)
		g->ops.ltc.sync_debugfs(g);
#endif

	gk20a_dbg_info("channel %d", c->hw_chid);

	/* gk20a_channel_update releases this ref. */
	err = gk20a_busy(g->dev);
	if (err) {
		gk20a_err(d, "failed to host gk20a to submit gpfifo");
		return err;
	}

	trace_gk20a_channel_submit_gpfifo(c->g->dev->name,
					  c->hw_chid,
					  num_entries,
					  flags,
					  fence->syncpt_id, fence->value);
	check_gp_put(g, c);
	update_gp_get(g, c);

	gk20a_dbg_info("pre-submit put %d, get %d, size %d",
		c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

	/* an address space needs to have been bound at this point.   */
	if (!gk20a_channel_as_bound(c)) {
		gk20a_err(d,
				"not bound to an address space at time of gpfifo"
				" submit.  Attempting to create and bind to"
				" one...");
		return -EINVAL;
	}

	/* Invalidate tlb if it's dirty...                                   */
	/* TBD: this should be done in the cmd stream, not with PRIs.        */
	/* We don't know what context is currently running...                */
	/* Note also: there can be more than one context associated with the */
	/* address space (vm).   */
	gk20a_mm_tlb_invalidate(c->vm);

	/* Make sure we have enough space for gpfifo entries. If not,
	 * wait for signals from completed submits */
	if (gp_free_count(c) < num_entries + extra_entries) {
		err = wait_event_interruptible(c->submit_wq,
			get_gp_free_count(c) >= num_entries + extra_entries ||
			c->has_timedout);
	}

	if (c->has_timedout) {
		err = -ETIMEDOUT;
		goto clean_up;
	}

	if (err) {
		gk20a_err(d, "not enough gpfifo space");
		err = -EAGAIN;
		goto clean_up;
	}

	/*
	 * optionally insert syncpt wait in the beginning of gpfifo submission
	 * when user requested and the wait hasn't expired.
	 * validate that the id makes sense, elide if not
	 * the only reason this isn't being unceremoniously killed is to
	 * keep running some tests which trigger this condition
	 */
	if (flags & NVHOST_SUBMIT_GPFIFO_FLAGS_FENCE_WAIT) {
		if (flags & NVHOST_SUBMIT_GPFIFO_FLAGS_SYNC_FENCE)
			err = c->sync->wait_fd(c->sync, fence->syncpt_id,
					&wait_cmd);
		else
			err = c->sync->wait_syncpt(c->sync, fence->syncpt_id,
					fence->value, &wait_cmd);
	}
	if (err)
		goto clean_up;


	/* always insert syncpt increment at end of gpfifo submission
	   to keep track of method completion for idle railgating */
	if (flags & NVHOST_SUBMIT_GPFIFO_FLAGS_FENCE_GET &&
			flags & NVHOST_SUBMIT_GPFIFO_FLAGS_SYNC_FENCE)
		err = c->sync->incr_user_fd(c->sync, &incr_cmd,
					    &c->last_submit_fence,
					    &fence->syncpt_id);
	else if (flags & NVHOST_SUBMIT_GPFIFO_FLAGS_FENCE_GET)
		err = c->sync->incr_user_syncpt(c->sync, &incr_cmd,
						&c->last_submit_fence,
						&fence->syncpt_id,
						&fence->value);
	else
		err = c->sync->incr(c->sync, &incr_cmd,
				    &c->last_submit_fence);
	if (err)
		goto clean_up;

	if (wait_cmd) {
		c->gpfifo.cpu_va[c->gpfifo.put].entry0 =
			u64_lo32(wait_cmd->gva);
		c->gpfifo.cpu_va[c->gpfifo.put].entry1 =
			u64_hi32(wait_cmd->gva) |
			pbdma_gp_entry1_length_f(wait_cmd->size);
		trace_write_pushbuffer(c, &c->gpfifo.cpu_va[c->gpfifo.put]);

		c->gpfifo.put = (c->gpfifo.put + 1) &
			(c->gpfifo.entry_num - 1);

		/* save gp_put */
		wait_cmd->gp_put = c->gpfifo.put;
	}

	for (i = 0; i < num_entries; i++) {
		c->gpfifo.cpu_va[c->gpfifo.put].entry0 =
			gpfifo[i].entry0; /* cmd buf va low 32 */
		c->gpfifo.cpu_va[c->gpfifo.put].entry1 =
			gpfifo[i].entry1; /* cmd buf va high 32 | words << 10 */
		trace_write_pushbuffer(c, &c->gpfifo.cpu_va[c->gpfifo.put]);
		c->gpfifo.put = (c->gpfifo.put + 1) &
			(c->gpfifo.entry_num - 1);
	}

	if (incr_cmd) {
		c->gpfifo.cpu_va[c->gpfifo.put].entry0 =
			u64_lo32(incr_cmd->gva);
		c->gpfifo.cpu_va[c->gpfifo.put].entry1 =
			u64_hi32(incr_cmd->gva) |
			pbdma_gp_entry1_length_f(incr_cmd->size);
		trace_write_pushbuffer(c, &c->gpfifo.cpu_va[c->gpfifo.put]);

		c->gpfifo.put = (c->gpfifo.put + 1) &
			(c->gpfifo.entry_num - 1);

		/* save gp_put */
		incr_cmd->gp_put = c->gpfifo.put;
	}

	trace_gk20a_channel_submitted_gpfifo(c->g->dev->name,
					     c->hw_chid,
					     num_entries,
					     flags,
					     fence->syncpt_id, fence->value);

	/* TODO! Check for errors... */
	gk20a_channel_add_job(c, &c->last_submit_fence);

	c->cmds_pending = true;
	gk20a_bar1_writel(g,
		c->userd_gpu_va + 4 * ram_userd_gp_put_w(),
		c->gpfifo.put);

	gk20a_dbg_info("post-submit put %d, get %d, size %d",
		c->gpfifo.put, c->gpfifo.get, c->gpfifo.entry_num);

	gk20a_dbg_fn("done");
	return err;

clean_up:
	gk20a_err(d, "fail");
	free_priv_cmdbuf(c, wait_cmd);
	free_priv_cmdbuf(c, incr_cmd);
	gk20a_idle(g->dev);
	return err;
}

void gk20a_remove_channel_support(struct channel_gk20a *c)
{

}

int gk20a_init_channel_support(struct gk20a *g, u32 chid)
{
	struct channel_gk20a *c = g->fifo.channel+chid;
	c->g = g;
	c->in_use = false;
	c->hw_chid = chid;
	c->bound = false;
	c->remove_support = gk20a_remove_channel_support;
	mutex_init(&c->error_notifier_mutex);
	mutex_init(&c->jobs_lock);
	INIT_LIST_HEAD(&c->jobs);
#if defined(CONFIG_GK20A_CYCLE_STATS)
	mutex_init(&c->cyclestate.cyclestate_buffer_mutex);
#endif
	INIT_LIST_HEAD(&c->dbg_s_list);
	mutex_init(&c->dbg_s_lock);

	return 0;
}

int gk20a_channel_finish(struct channel_gk20a *ch, unsigned long timeout)
{
	int err = 0;

	if (!ch->cmds_pending)
		return 0;

	/* Do not wait for a timedout channel */
	if (ch->has_timedout)
		return -ETIMEDOUT;

	if (!(ch->last_submit_fence.valid && ch->last_submit_fence.wfi)) {
		gk20a_dbg_fn("issuing wfi, incr to finish the channel");
		err = gk20a_channel_submit_wfi(ch);
	}
	if (err)
		return err;

	BUG_ON(!(ch->last_submit_fence.valid && ch->last_submit_fence.wfi));

	gk20a_dbg_fn("waiting for channel to finish thresh:%d",
		      ch->last_submit_fence.thresh);

	err = ch->sync->wait_cpu(ch->sync, &ch->last_submit_fence, timeout);
	if (WARN_ON(err))
		dev_warn(dev_from_gk20a(ch->g),
			 "timed out waiting for gk20a channel to finish");
	else
		ch->cmds_pending = false;

	return err;
}

static int gk20a_channel_wait_semaphore(struct channel_gk20a *ch,
					ulong id, u32 offset,
					u32 payload, long timeout)
{
	struct platform_device *pdev = ch->g->dev;
	struct dma_buf *dmabuf;
	void *data;
	u32 *semaphore;
	int ret = 0;
	long remain;

	/* do not wait if channel has timed out */
	if (ch->has_timedout)
		return -ETIMEDOUT;

	dmabuf = dma_buf_get(id);
	if (IS_ERR(dmabuf)) {
		gk20a_err(&pdev->dev, "invalid notifier nvmap handle 0x%lx",
			   id);
		return -EINVAL;
	}

	data = dma_buf_kmap(dmabuf, offset >> PAGE_SHIFT);
	if (!data) {
		gk20a_err(&pdev->dev, "failed to map notifier memory");
		ret = -EINVAL;
		goto cleanup_put;
	}

	semaphore = data + (offset & ~PAGE_MASK);

	remain = wait_event_interruptible_timeout(
			ch->semaphore_wq,
			*semaphore == payload || ch->has_timedout,
			timeout);

	if (remain == 0 && *semaphore != payload)
		ret = -ETIMEDOUT;
	else if (remain < 0)
		ret = remain;

	dma_buf_kunmap(dmabuf, offset >> PAGE_SHIFT, data);
cleanup_put:
	dma_buf_put(dmabuf);
	return ret;
}

static int gk20a_channel_wait(struct channel_gk20a *ch,
			      struct nvhost_wait_args *args)
{
	struct device *d = dev_from_gk20a(ch->g);
	struct dma_buf *dmabuf;
	struct notification *notif;
	struct timespec tv;
	u64 jiffies;
	ulong id;
	u32 offset;
	unsigned long timeout;
	int remain, ret = 0;
	u64 end;

	gk20a_dbg_fn("");

	if (ch->has_timedout)
		return -ETIMEDOUT;

	if (args->timeout == NVHOST_NO_TIMEOUT)
		timeout = MAX_SCHEDULE_TIMEOUT;
	else
		timeout = (u32)msecs_to_jiffies(args->timeout);

	switch (args->type) {
	case NVHOST_WAIT_TYPE_NOTIFIER:
		id = args->condition.notifier.nvmap_handle;
		offset = args->condition.notifier.offset;

		if (unlikely(offset > U32_MAX - sizeof(struct notification)))
			return -EINVAL;

		end = offset + sizeof(struct notification);

		dmabuf = dma_buf_get(id);
		if (IS_ERR(dmabuf)) {
			gk20a_err(d, "invalid notifier nvmap handle 0x%lx",
				   id);
			return -EINVAL;
		}

		if (end > dmabuf->size || end < sizeof(struct notification)) {
			dma_buf_put(dmabuf);
			gk20a_err(d, "invalid notifier offset\n");
			return -EINVAL;
		}

		notif = dma_buf_vmap(dmabuf);
		if (!notif) {
			gk20a_err(d, "failed to map notifier memory");
			return -ENOMEM;
		}

		notif = (struct notification *)((uintptr_t)notif + offset);

		/* user should set status pending before
		 * calling this ioctl */
		remain = wait_event_interruptible_timeout(
				ch->notifier_wq,
				notif->status == 0 || ch->has_timedout,
				timeout);

		if (remain == 0 && notif->status != 0) {
			ret = -ETIMEDOUT;
			goto notif_clean_up;
		} else if (remain < 0) {
			ret = -EINTR;
			goto notif_clean_up;
		}

		/* TBD: fill in correct information */
		jiffies = get_jiffies_64();
		jiffies_to_timespec(jiffies, &tv);
		notif->timestamp.nanoseconds[0] = tv.tv_nsec;
		notif->timestamp.nanoseconds[1] = tv.tv_sec;
		notif->info32 = 0xDEADBEEF; /* should be object name */
		notif->info16 = ch->hw_chid; /* should be method offset */

notif_clean_up:
		dma_buf_vunmap(dmabuf, notif);
		return ret;

	case NVHOST_WAIT_TYPE_SEMAPHORE:
		ret = gk20a_channel_wait_semaphore(ch,
				args->condition.semaphore.nvmap_handle,
				args->condition.semaphore.offset,
				args->condition.semaphore.payload,
				timeout);

		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int gk20a_channel_set_priority(struct channel_gk20a *ch,
		u32 priority)
{
	u32 timeslice_timeout;
	/* set priority of graphics channel */
	switch (priority) {
	case NVHOST_PRIORITY_LOW:
		/* 64 << 3 = 512us */
		timeslice_timeout = 64;
		break;
	case NVHOST_PRIORITY_MEDIUM:
		/* 128 << 3 = 1024us */
		timeslice_timeout = 128;
		break;
	case NVHOST_PRIORITY_HIGH:
		/* 255 << 3 = 2048us */
		timeslice_timeout = 255;
		break;
	default:
		pr_err("Unsupported priority");
		return -EINVAL;
	}
	channel_gk20a_set_schedule_params(ch,
			timeslice_timeout);
	return 0;
}

static int gk20a_channel_zcull_bind(struct channel_gk20a *ch,
			    struct nvhost_zcull_bind_args *args)
{
	struct gk20a *g = ch->g;
	struct gr_gk20a *gr = &g->gr;

	gk20a_dbg_fn("");

	return gr_gk20a_bind_ctxsw_zcull(g, gr, ch,
				args->gpu_va, args->mode);
}

/* in this context the "channel" is the host1x channel which
 * maps to *all* gk20a channels */
int gk20a_channel_suspend(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;
	bool channels_in_use = false;
	int err;

	gk20a_dbg_fn("");

	/* wait for engine idle */
	err = gk20a_fifo_wait_engine_idle(g);
	if (err)
		return err;

	for (chid = 0; chid < f->num_channels; chid++) {
		if (f->channel[chid].in_use) {

			gk20a_dbg_info("suspend channel %d", chid);
			/* disable channel */
			gk20a_writel(g, ccsr_channel_r(chid),
				gk20a_readl(g, ccsr_channel_r(chid)) |
				ccsr_channel_enable_clr_true_f());
			/* preempt the channel */
			gk20a_fifo_preempt_channel(g, chid);

			channels_in_use = true;
		}
	}

	if (channels_in_use) {
		gk20a_fifo_update_runlist(g, 0, ~0, false, true);

		for (chid = 0; chid < f->num_channels; chid++) {
			if (f->channel[chid].in_use)
				channel_gk20a_unbind(&f->channel[chid]);
		}
	}

	gk20a_dbg_fn("done");
	return 0;
}

/* in this context the "channel" is the host1x channel which
 * maps to *all* gk20a channels */
int gk20a_channel_resume(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;
	bool channels_in_use = false;

	gk20a_dbg_fn("");

	for (chid = 0; chid < f->num_channels; chid++) {
		if (f->channel[chid].in_use) {
			gk20a_dbg_info("resume channel %d", chid);
			g->ops.fifo.bind_channel(&f->channel[chid]);
			channels_in_use = true;
		}
	}

	if (channels_in_use)
		gk20a_fifo_update_runlist(g, 0, ~0, true, true);

	gk20a_dbg_fn("done");
	return 0;
}

void gk20a_channel_semaphore_wakeup(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	u32 chid;

	gk20a_dbg_fn("");

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *c = g->fifo.channel+chid;
		if (c->in_use)
			wake_up_interruptible_all(&c->semaphore_wq);
	}
}

static int gk20a_ioctl_channel_submit_gpfifo(
	struct channel_gk20a *ch,
	struct nvhost_submit_gpfifo_args *args)
{
	void *gpfifo;
	u32 size;
	int ret = 0;

	gk20a_dbg_fn("");

	if (ch->has_timedout)
		return -ETIMEDOUT;

	size = args->num_entries * sizeof(struct nvhost_gpfifo);

	gpfifo = kzalloc(size, GFP_KERNEL);
	if (!gpfifo)
		return -ENOMEM;

	if (copy_from_user(gpfifo,
			   (void __user *)(uintptr_t)args->gpfifo, size)) {
		ret = -EINVAL;
		goto clean_up;
	}

	ret = gk20a_submit_channel_gpfifo(ch, gpfifo, args->num_entries,
					&args->fence, args->flags);

clean_up:
	kfree(gpfifo);
	return ret;
}

void gk20a_init_channel(struct gpu_ops *gops)
{
	gops->fifo.bind_channel = channel_gk20a_bind;
}

long gk20a_channel_ioctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	struct channel_gk20a *ch = filp->private_data;
	struct platform_device *dev = ch->g->dev;
	u8 buf[NVHOST_IOCTL_CHANNEL_MAX_ARG_SIZE] = {0};
	int err = 0;

	if ((_IOC_TYPE(cmd) != NVHOST_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVHOST_IOCTL_CHANNEL_LAST) ||
		(_IOC_SIZE(cmd) > NVHOST_IOCTL_CHANNEL_MAX_ARG_SIZE))
		return -EFAULT;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	switch (cmd) {
	case NVHOST_IOCTL_CHANNEL_OPEN:
	{
		int fd;
		struct file *file;
		char *name;

		err = get_unused_fd_flags(O_RDWR);
		if (err < 0)
			break;
		fd = err;

		name = kasprintf(GFP_KERNEL, "nvhost-%s-fd%d",
				dev_name(&dev->dev), fd);
		if (!name) {
			err = -ENOMEM;
			put_unused_fd(fd);
			break;
		}

		file = anon_inode_getfile(name, filp->f_op, NULL, O_RDWR);
		kfree(name);
		if (IS_ERR(file)) {
			err = PTR_ERR(file);
			put_unused_fd(fd);
			break;
		}
		fd_install(fd, file);

		err = __gk20a_channel_open(ch->g, file);
		if (err) {
			put_unused_fd(fd);
			fput(file);
			break;
		}

		((struct nvhost_channel_open_args *)buf)->channel_fd = fd;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_NVMAP_FD:
		break;
	case NVHOST_IOCTL_CHANNEL_ALLOC_OBJ_CTX:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			return err;
		}
		err = gk20a_alloc_obj_ctx(ch,
				(struct nvhost_alloc_obj_ctx_args *)buf);
		gk20a_idle(dev);
		break;
	case NVHOST_IOCTL_CHANNEL_ALLOC_GPFIFO:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			return err;
		}
		err = gk20a_alloc_channel_gpfifo(ch,
				(struct nvhost_alloc_gpfifo_args *)buf);
		gk20a_idle(dev);
		break;
	case NVHOST_IOCTL_CHANNEL_SUBMIT_GPFIFO:
		err = gk20a_ioctl_channel_submit_gpfifo(ch,
				(struct nvhost_submit_gpfifo_args *)buf);
		break;
	case NVHOST_IOCTL_CHANNEL_WAIT:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			return err;
		}
		err = gk20a_channel_wait(ch,
				(struct nvhost_wait_args *)buf);
		gk20a_idle(dev);
		break;
	case NVHOST_IOCTL_CHANNEL_ZCULL_BIND:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			return err;
		}
		err = gk20a_channel_zcull_bind(ch,
				(struct nvhost_zcull_bind_args *)buf);
		gk20a_idle(dev);
		break;
	case NVHOST_IOCTL_CHANNEL_SET_ERROR_NOTIFIER:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			return err;
		}
		err = gk20a_init_error_notifier(ch,
				(struct nvhost_set_error_notifier *)buf);
		gk20a_idle(dev);
		break;
#ifdef CONFIG_GK20A_CYCLE_STATS
	case NVHOST_IOCTL_CHANNEL_CYCLE_STATS:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			return err;
		}
		err = gk20a_channel_cycle_stats(ch,
				(struct nvhost_cycle_stats_args *)buf);
		gk20a_idle(dev);
		break;
#endif
	case NVHOST_IOCTL_CHANNEL_SET_TIMEOUT:
	{
		u32 timeout =
			(u32)((struct nvhost_set_timeout_args *)buf)->timeout;
		gk20a_dbg(gpu_dbg_gpu_dbg, "setting timeout (%d ms) for chid %d",
			   timeout, ch->hw_chid);
		ch->timeout_ms_max = timeout;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_TIMEOUT_EX:
	{
		u32 timeout =
			(u32)((struct nvhost_set_timeout_args *)buf)->timeout;
		bool timeout_debug_dump = !((u32)
			((struct nvhost_set_timeout_ex_args *)buf)->flags &
			(1 << NVHOST_TIMEOUT_FLAG_DISABLE_DUMP));
		gk20a_dbg(gpu_dbg_gpu_dbg, "setting timeout (%d ms) for chid %d",
			   timeout, ch->hw_chid);
		ch->timeout_ms_max = timeout;
		ch->timeout_debug_dump = timeout_debug_dump;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_TIMEDOUT:
		((struct nvhost_get_param_args *)buf)->value =
			ch->has_timedout;
		break;
	case NVHOST_IOCTL_CHANNEL_SET_PRIORITY:
		err = gk20a_busy(dev);
		if (err) {
			dev_err(&dev->dev,
				"%s: failed to host gk20a for ioctl cmd: 0x%x",
				__func__, cmd);
			return err;
		}
		gk20a_channel_set_priority(ch,
			((struct nvhost_set_priority_args *)buf)->priority);
		gk20a_idle(dev);
		break;
	default:
		dev_err(&dev->dev, "unrecognized ioctl cmd: 0x%x", cmd);
		err = -ENOTTY;
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}
