/*
 * Tegra Graphics Host Client Module
 *
 * Copyright (c) 2010-2016, NVIDIA Corporation. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/slab.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/file.h>
#include <linux/clk.h>
#include <linux/hrtimer.h>
#include <linux/export.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/tegra-soc.h>
#include <linux/anon_inodes.h>

#include <trace/events/nvhost.h>

#include <linux/io.h>
#include <linux/string.h>

#include <linux/nvhost.h>
#include <linux/nvhost_ioctl.h>

#include <mach/gpufuse.h>

#include "debug.h"
#include "bus_client.h"
#include "dev.h"
#include "class_ids.h"
#include "chip_support.h"
#include "nvhost_acm.h"

#include "nvhost_syncpt.h"
#include "nvhost_channel.h"
#include "nvhost_job.h"
#include "nvhost_hwctx.h"
#include "nvhost_sync.h"

DEFINE_MUTEX(channel_lock);

static int validate_reg(struct platform_device *ndev, u32 offset, int count)
{
	int err = 0;
	struct resource *r;
	struct nvhost_device_data *pdata = platform_get_drvdata(ndev);

	/* check if offset is u32 aligned */
	if (offset & 3)
		return -EINVAL;

	r = platform_get_resource(pdata->master ? pdata->master : ndev,
			IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&ndev->dev, "failed to get memory resource\n");
		return -ENODEV;
	}

	if (offset + 4 * count > resource_size(r)
			|| (offset + 4 * count < offset))
		err = -EPERM;

	return err;
}

int validate_max_size(struct platform_device *ndev, u32 size)
{
	struct resource *r;

	/* check if size is non-zero and u32 aligned */
	if (!size || size & 3)
		return -EINVAL;

	r = platform_get_resource(ndev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&ndev->dev, "failed to get memory resource\n");
		return -ENODEV;
	}

	if (size > resource_size(r))
		return -EPERM;

	return 0;
}

static __iomem void *get_aperture(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);

	if (pdata->master)
		pdata = platform_get_drvdata(pdata->master);

	return pdata->aperture[0];
}

void host1x_writel(struct platform_device *pdev, u32 r, u32 v)
{
	void __iomem *addr = get_aperture(pdev) + r;
	nvhost_dbg(dbg_reg, " d=%s r=0x%x v=0x%x", pdev->name, r, v);
	writel(v, addr);
}
EXPORT_SYMBOL_GPL(host1x_writel);

u32 host1x_readl(struct platform_device *pdev, u32 r)
{
	void __iomem *addr = get_aperture(pdev) + r;
	u32 v = readl(addr);
	nvhost_dbg(dbg_reg, " d=%s r=0x%x v=0x%x", pdev->name, r, v);
	return v;
}
EXPORT_SYMBOL_GPL(host1x_readl);

int nvhost_read_module_regs(struct platform_device *ndev,
			u32 offset, int count, u32 *values)
{
	void __iomem *p = get_aperture(ndev);
	int err;

	if (!p)
		return -ENODEV;

	/* verify offset */
	err = validate_reg(ndev, offset, count);
	if (err)
		return err;

	err = nvhost_module_busy(ndev);
	if (err)
		return err;

	p += offset;
	while (count--) {
		*(values++) = readl(p);
		p += 4;
	}
	rmb();
	nvhost_module_idle(ndev);

	return 0;
}

int nvhost_write_module_regs(struct platform_device *ndev,
			u32 offset, int count, const u32 *values)
{
	int err;
	void __iomem *p = get_aperture(ndev);

	if (!p)
		return -ENODEV;

	/* verify offset */
	err = validate_reg(ndev, offset, count);
	if (err)
		return err;

	err = nvhost_module_busy(ndev);
	if (err)
		return err;

	p += offset;
	while (count--) {
		writel(*(values++), p);
		p += 4;
	}
	wmb();
	nvhost_module_idle(ndev);

	return 0;
}

struct nvhost_channel_userctx {
	struct nvhost_channel *ch;
	struct nvhost_hwctx *hwctx;
	struct nvhost_job *job;
	u32 timeout;
	u32 priority;
	int clientid;
	bool timeout_debug_dump;
};

static int nvhost_channelrelease(struct inode *inode, struct file *filp)
{
	struct nvhost_channel_userctx *priv = filp->private_data;

	mutex_lock(&channel_lock);
	if (!priv->ch || !priv->ch->dev) {
		mutex_unlock(&channel_lock);
		return 0;
	}
	trace_nvhost_channel_release(dev_name(&priv->ch->dev->dev));

	filp->private_data = NULL;

	nvhost_module_remove_client(priv->ch->dev, priv);

	if (priv->hwctx) {
		struct nvhost_channel *ch = priv->ch;
		struct nvhost_hwctx *ctx = priv->hwctx;

		mutex_lock(&ch->submitlock);
		if (ch->cur_ctx == ctx)
			ch->cur_ctx = NULL;
		mutex_unlock(&ch->submitlock);

		priv->hwctx->h->put(priv->hwctx);
	}

	if (priv->job)
		nvhost_job_put(priv->job);

	mutex_unlock(&channel_lock);
	nvhost_putchannel(priv->ch, 1);
	kfree(priv);
	return 0;
}

static int __nvhost_channelopen(struct inode *inode,
		struct nvhost_channel *ch,
		struct file *filp)
{
	struct nvhost_channel_userctx *priv;
	struct nvhost_device_data *pdata;
	int ret;

	if (inode) {
		pdata = container_of(inode->i_cdev,
				struct nvhost_device_data, cdev);
		ret = nvhost_channel_map(pdata, &ch);
		if (ret) {
			pr_err("%s: failed to map channel, error: %d\n",
					__func__, ret);
			return ret;
		}
	} else {
		if (!ch || !ch->dev) {
			pr_err("%s: NULL channel request to get\n", __func__);
			return -EINVAL;
		}
		pdata = platform_get_drvdata(ch->dev);
		if (!pdata->exclusive)
			nvhost_getchannel(ch);
		else
			return -EBUSY;
	}

	mutex_lock(&channel_lock);
	if (!ch || !ch->dev) {
		mutex_unlock(&channel_lock);
		return -EINVAL;
	}
	trace_nvhost_channel_open(dev_name(&ch->dev->dev));

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto fail;

	filp->private_data = priv;
	priv->ch = ch;
	if (nvhost_module_add_client(ch->dev, priv))
		goto fail_add_client;

	if (ch->ctxhandler && ch->ctxhandler->alloc) {
		ret = nvhost_module_busy(ch->dev);
		if (ret)
			goto fail_priv;

		priv->hwctx = ch->ctxhandler->alloc(ch->ctxhandler, ch);
		nvhost_module_idle(ch->dev);
		if (!priv->hwctx)
			goto fail_priv;
	}
	priv->priority = NVHOST_PRIORITY_MEDIUM;
	priv->clientid = atomic_add_return(1,
			&nvhost_get_host(ch->dev)->clientid);
	/* If we wrapped around to 0, increment again */
	if (!priv->clientid)
		priv->clientid = atomic_add_return(1,
				&nvhost_get_host(ch->dev)->clientid);
	pdata = dev_get_drvdata(ch->dev->dev.parent);
	priv->timeout = pdata->nvhost_timeout_default;
	priv->timeout_debug_dump = true;
	if (!tegra_platform_is_silicon())
		priv->timeout = 0;
	mutex_unlock(&channel_lock);
	return 0;

fail_priv:
	nvhost_module_remove_client(ch->dev, priv);
fail_add_client:
	kfree(priv);
fail:
	nvhost_putchannel(ch, 1);
	mutex_unlock(&channel_lock);

	return -ENOMEM;
}

static int nvhost_channelopen(struct inode *inode, struct file *filp)
{
	return __nvhost_channelopen(inode, NULL, filp);
}

void nvhost_set_notifier(struct nvhost_channel *ch, __u32 error)
{
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
		dev_err(&ch->dev->dev, "error notifier set to %d\n", error);
	}
}

void nvhost_free_error_notifiers(struct nvhost_channel *ch)
{
	if (ch->error_notifier_ref) {
		dma_buf_vunmap(ch->error_notifier_ref, ch->error_notifier_va);
		dma_buf_put(ch->error_notifier_ref);
		ch->error_notifier_ref = 0;
		ch->error_notifier = 0;
		ch->error_notifier_va = 0;
	}
}

static int nvhost_init_error_notifier(struct nvhost_channel *ch,
		struct nvhost_set_error_notifier *args) {
	void *va;
	u64 end;
	struct dma_buf *dmabuf;

	if (unlikely(args->offset >
		     U64_MAX - sizeof(struct nvhost_notification)))
		return -EINVAL;

	end = args->offset + sizeof(struct nvhost_notification);

	if (!args->mem) {
		dev_err(&ch->dev->dev, "invalid memory handle\n");
		return -EINVAL;
	}

	dmabuf = dma_buf_get(args->mem);

	if (ch->error_notifier_ref)
		nvhost_free_error_notifiers(ch);

	if (IS_ERR(dmabuf)) {
		dev_err(&ch->dev->dev, "Invalid handle: %d\n", args->mem);
		return -EINVAL;
	}

	if (end > dmabuf->size || end < sizeof(struct nvhost_notification)) {
		dma_buf_put(dmabuf);
		pr_err("%s: invalid offset\n", __func__);
		return -EINVAL;
	}

	/* map handle and clear error notifier struct */
	va = dma_buf_vmap(dmabuf);
	if (!va) {
		dma_buf_put(dmabuf);
		dev_err(&ch->dev->dev, "Cannot map notifier handle\n");
		return -ENOMEM;
	}

	/* set channel notifiers pointer */
	ch->error_notifier_ref = dmabuf;
	ch->error_notifier = va + args->offset;
	ch->error_notifier_va = va;
	memset(ch->error_notifier, 0, sizeof(struct nvhost_notification));
	return 0;

}

static int nvhost_ioctl_channel_submit(struct nvhost_channel_userctx *ctx,
		struct nvhost_submit_args *args)
{
	struct nvhost_job *job;
	int num_cmdbufs = args->num_cmdbufs;
	int num_relocs = args->num_relocs;
	int num_waitchks = args->num_waitchks;
	int num_syncpt_incrs = args->num_syncpt_incrs;
	struct nvhost_cmdbuf __user *cmdbufs =
		(struct nvhost_cmdbuf *)(uintptr_t)args->cmdbufs;
	struct nvhost_cmdbuf __user *cmdbuf_exts =
		(struct nvhost_cmdbuf *)(uintptr_t)args->cmdbuf_exts;
	struct nvhost_reloc __user *relocs =
		(struct nvhost_reloc *)(uintptr_t)args->relocs;
	struct nvhost_reloc_shift __user *reloc_shifts =
		(struct nvhost_reloc_shift *)(uintptr_t)args->reloc_shifts;
	struct nvhost_waitchk __user *waitchks =
		(struct nvhost_waitchk *)(uintptr_t)args->waitchks;
	struct nvhost_syncpt_incr __user *syncpt_incrs =
		(struct nvhost_syncpt_incr *)(uintptr_t)args->syncpt_incrs;
	u32 __user *waitbases = (u32 *)(uintptr_t)args->waitbases;
	u32 __user *fences = (u32 *)(uintptr_t)args->fences;
	u32 __user *class_ids = (u32 *)(uintptr_t)args->class_ids;

	struct nvhost_master *host = nvhost_get_host(ctx->ch->dev);
	u32 *local_waitbases = NULL, *local_class_ids = NULL;
	int err, i, hwctx_syncpt_idx = -1;

	if (num_syncpt_incrs > host->info.nb_pts)
		return -EINVAL;

	job = nvhost_job_alloc(ctx->ch,
			ctx->hwctx,
			num_cmdbufs,
			num_relocs,
			num_waitchks,
			num_syncpt_incrs);
	if (!job)
		return -ENOMEM;

	job->num_relocs = args->num_relocs;
	job->num_waitchk = args->num_waitchks;
	job->num_syncpts = args->num_syncpt_incrs;
	job->priority = ctx->priority;
	job->clientid = ctx->clientid;

	/* mass copy class_ids */
	if (args->class_ids) {
		local_class_ids = kzalloc(sizeof(u32) * num_cmdbufs,
			GFP_KERNEL);
		if (!local_class_ids) {
			err = -ENOMEM;
			goto fail;
		}
		err = copy_from_user(local_class_ids, class_ids,
			sizeof(u32) * num_cmdbufs);
		if (err) {
			err = -EINVAL;
			goto fail;
		}
	}

	for (i = 0; i < num_cmdbufs; ++i) {
		struct nvhost_cmdbuf cmdbuf;
		struct nvhost_cmdbuf_ext cmdbuf_ext;
		u32 class_id = class_ids ? local_class_ids[i] : 0;

		err = copy_from_user(&cmdbuf, cmdbufs + i, sizeof(cmdbuf));
		if (err)
			goto fail;

		cmdbuf_ext.pre_fence = -1;
		if (cmdbuf_exts)
			err = copy_from_user(&cmdbuf_ext,
					cmdbuf_exts + i, sizeof(cmdbuf_ext));
		if (err)
			cmdbuf_ext.pre_fence = -1;

		nvhost_job_add_gather(job, cmdbuf.mem, cmdbuf.words,
				      cmdbuf.offset, class_id,
				      cmdbuf_ext.pre_fence);
	}

	kfree(local_class_ids);
	local_class_ids = NULL;

	err = copy_from_user(job->relocarray,
			relocs, sizeof(*relocs) * num_relocs);
	if (err)
		goto fail;

	err = copy_from_user(job->relocshiftarray,
			reloc_shifts, sizeof(*reloc_shifts) * num_relocs);
	if (err)
		goto fail;

	err = copy_from_user(job->waitchk,
			waitchks, sizeof(*waitchks) * num_waitchks);
	if (err)
		goto fail;

	/* mass copy waitbases */
	if (args->waitbases) {
		local_waitbases = kzalloc(sizeof(u32) * num_syncpt_incrs,
			GFP_KERNEL);
		if (!local_waitbases) {
			err = -ENOMEM;
			goto fail;
		}

		err = copy_from_user(local_waitbases, waitbases,
			sizeof(u32) * num_syncpt_incrs);
		if (err) {
			err = -EINVAL;
			goto fail;
		}
	}

	/* set valid id for hwctx_syncpt_idx if hwctx does not provide one */
	if (!ctx->hwctx || ctx->hwctx->h->syncpt == NVSYNCPT_INVALID)
		hwctx_syncpt_idx = 0;

	/*
	 * Go through each syncpoint from userspace. Here we:
	 * - Copy syncpoint information
	 * - Validate each syncpoint
	 * - Determine waitbase for each syncpoint
	 * - Determine the index of hwctx syncpoint in the table
	 */

	for (i = 0; i < num_syncpt_incrs; ++i) {
		u32 waitbase;
		struct nvhost_syncpt_incr sp;

		/* Copy */
		err = copy_from_user(&sp, syncpt_incrs + i, sizeof(sp));
		if (err)
			goto fail;

		/* Validate */
		if (sp.syncpt_id >= host->info.nb_pts) {
			err = -EINVAL;
			goto fail;
		}

		/* Determine waitbase */
		if (waitbases && local_waitbases[i] != NVSYNCPT_INVALID)
			waitbase = local_waitbases[i];
		else
			waitbase = nvhost_syncpt_get_waitbase(job->ch,
				sp.syncpt_id);

		/* Store */
		job->sp[i].id = sp.syncpt_id;
		job->sp[i].incrs = sp.syncpt_incrs;
		job->sp[i].waitbase = waitbase;

		/* Find hwctx syncpoint */
		if (ctx->hwctx && (job->sp[i].id == ctx->hwctx->h->syncpt))
			hwctx_syncpt_idx = i;
	}

	/* not needed anymore */
	kfree(local_waitbases);
	local_waitbases = NULL;

	/* Is hwctx_syncpt_idx valid? */
	if (hwctx_syncpt_idx == -1) {
		err = -EINVAL;
		goto fail;
	}

	job->hwctx_syncpt_idx = hwctx_syncpt_idx;

	trace_nvhost_channel_submit(ctx->ch->dev->name,
		job->num_gathers, job->num_relocs, job->num_waitchk,
		job->sp[job->hwctx_syncpt_idx].id,
		job->sp[job->hwctx_syncpt_idx].incrs);

	nvhost_module_busy(ctx->ch->dev);
	err = nvhost_job_pin(job, &nvhost_get_host(ctx->ch->dev)->syncpt);
	nvhost_module_idle(ctx->ch->dev);
	if (err)
		goto fail;

	if (args->timeout)
		job->timeout = min(ctx->timeout, args->timeout);
	else
		job->timeout = ctx->timeout;
	job->timeout_debug_dump = ctx->timeout_debug_dump;

	err = nvhost_channel_submit(job);
	if (err)
		goto fail_submit;

	/* Deliver multiple fences back to the userspace */
	if (fences)
		for (i = 0; i < num_syncpt_incrs; ++i) {
			u32 fence = job->sp[i].fence;
			err = copy_to_user(fences, &fence, sizeof(u32));
			if (err)
				break;
			fences++;
		}

	/* Deliver the fence using the old mechanism _only_ if a single
	 * syncpoint is used. */

	if (args->flags & BIT(NVHOST_SUBMIT_FLAG_SYNC_FENCE_FD)) {
		struct nvhost_ctrl_sync_fence_info pts[num_syncpt_incrs];

		for (i = 0; i < num_syncpt_incrs; i++) {
			pts[i].id = job->sp[i].id;
			pts[i].thresh = job->sp[i].fence;
		}

		err = nvhost_sync_create_fence_fd(ctx->ch->dev,
				pts, num_syncpt_incrs, "fence", &args->fence);
		if (err)
			goto fail;
	} else if (num_syncpt_incrs == 1)
		args->fence = job->sp[job->hwctx_syncpt_idx].fence;
	else
		args->fence = 0;

	nvhost_job_put(job);

	return 0;

fail_submit:
	nvhost_job_unpin(job);
fail:
	nvhost_job_put(job);
	kfree(local_class_ids);
	kfree(local_waitbases);
	return err;
}

static int moduleid_to_index(struct platform_device *dev, u32 moduleid)
{
	int i;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	for (i = 0; i < NVHOST_MODULE_MAX_CLOCKS; i++) {
		if (pdata->clocks[i].moduleid == moduleid)
			return i;
	}

	/* Old user space is sending a random number in args. Return clock
	 * zero in these cases. */
	return 0;
}

static int nvhost_ioctl_channel_set_rate(struct nvhost_channel_userctx *ctx,
	struct nvhost_clk_rate_args *arg)
{
	u32 moduleid = (arg->moduleid >> NVHOST_MODULE_ID_BIT_POS)
			& ((1 << NVHOST_MODULE_ID_BIT_WIDTH) - 1);
	u32 attr = (arg->moduleid >> NVHOST_CLOCK_ATTR_BIT_POS)
			& ((1 << NVHOST_CLOCK_ATTR_BIT_WIDTH) - 1);
	int index = moduleid ?
			moduleid_to_index(ctx->ch->dev, moduleid) : 0;

	return nvhost_module_set_rate(ctx->ch->dev,
			ctx, arg->rate, index, attr);
}

static int nvhost_ioctl_channel_get_rate(struct nvhost_channel_userctx *ctx,
	u32 moduleid, u32 *rate)
{
	int index = moduleid ? moduleid_to_index(ctx->ch->dev, moduleid) : 0;

	return nvhost_module_get_rate(ctx->ch->dev,
			(unsigned long *)rate, index);
}

static int nvhost_ioctl_channel_module_regrdwr(
	struct nvhost_channel_userctx *ctx,
	struct nvhost_ctrl_module_regrdwr_args *args)
{
	u32 num_offsets = args->num_offsets;
	u32 __user *offsets = (u32 *)(uintptr_t)args->offsets;
	u32 __user *values = (u32 *)(uintptr_t)args->values;
	u32 vals[64];
	struct platform_device *ndev;

	trace_nvhost_ioctl_channel_module_regrdwr(args->id,
		args->num_offsets, args->write);

	/* Check that there is something to read and that block size is
	 * u32 aligned */
	if (num_offsets == 0 || args->block_size & 3)
		return -EINVAL;

	ndev = ctx->ch->dev;

	while (num_offsets--) {
		int err;
		u32 offs;
		int remaining = args->block_size >> 2;

		if (get_user(offs, offsets))
			return -EFAULT;

		offsets++;
		while (remaining) {
			int batch = min(remaining, 64);
			if (args->write) {
				if (copy_from_user(vals, values,
						batch * sizeof(u32)))
					return -EFAULT;

				err = nvhost_write_module_regs(ndev,
					offs, batch, vals);
				if (err)
					return err;
			} else {
				err = nvhost_read_module_regs(ndev,
						offs, batch, vals);
				if (err)
					return err;

				if (copy_to_user(values, vals,
						batch * sizeof(u32)))
					return -EFAULT;
			}

			remaining -= batch;
			offs += batch * sizeof(u32);
			values += batch;
		}
	}

	return 0;
}

static u32 create_mask(u32 *words, int num)
{
	int i;
	u32 word = 0;
	for (i = 0; i < num; i++) {
		if (!words[i] || words[i] > 31)
			continue;
		word |= BIT(words[i]);
	}

	return word;
}

static long nvhost_channelctl(struct file *filp,
	unsigned int cmd, unsigned long arg)
{
	struct nvhost_channel_userctx *priv = filp->private_data;
	struct device *dev;
	u8 buf[NVHOST_IOCTL_CHANNEL_MAX_ARG_SIZE];
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

	if (!priv->ch->dev) {
		pr_warn("Channel already unmapped\n");
		return -EFAULT;
	}

	dev = &priv->ch->dev->dev;
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
				dev_name(dev), fd);
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

		err = __nvhost_channelopen(NULL, priv->ch, file);
		if (err) {
			put_unused_fd(fd);
			fput(file);
			break;
		}

		((struct nvhost_channel_open_args *)buf)->channel_fd = fd;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_SYNCPOINTS:
	{
		struct nvhost_device_data *pdata = \
			platform_get_drvdata(priv->ch->dev);
		((struct nvhost_get_param_args *)buf)->value =
			create_mask(pdata->syncpts, NVHOST_MODULE_MAX_SYNCPTS);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_SYNCPOINT:
	{
		struct nvhost_device_data *pdata = \
			platform_get_drvdata(priv->ch->dev);
		struct nvhost_get_param_arg *arg =
			(struct nvhost_get_param_arg *)buf;
		if (arg->param >= NVHOST_MODULE_MAX_SYNCPTS)
			return -EINVAL;
		/* if we already have required syncpt then return it ... */
		if (pdata->syncpts[arg->param]) {
			arg->value = pdata->syncpts[arg->param];
			break;
		}
		/* ... otherwise get a new syncpt dynamically */
		arg->value = nvhost_get_syncpt_host_managed(pdata->pdev,
							    arg->param);
		if (!arg->value)
			return -EAGAIN;
		/* ... and store it for further references */
		pdata->syncpts[arg->param] = arg->value;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_CLIENT_MANAGED_SYNCPOINT:
	{
		char name[32];
		char set_name[32];
		struct nvhost_device_data *pdata =
			platform_get_drvdata(priv->ch->dev);
		struct nvhost_get_client_managed_syncpt_arg *args =
			(struct nvhost_get_client_managed_syncpt_arg *)buf;
		const char __user *args_name =
			(const char __user *)(uintptr_t)args->name;

		if (args_name) {
			if (strncpy_from_user(name, args_name,
							sizeof(name)) < 0)
				return -EFAULT;
			name[sizeof(name) - 1] = '\0';
		} else {
			name[0] = '\0';
		}
		/* if we already have required syncpt then return it ... */
		if (pdata->client_managed_syncpt) {
			args->value = pdata->client_managed_syncpt;
			break;
		}
		/* ... otherwise get a new syncpt dynamically */
		snprintf(set_name, sizeof(set_name),
				"%s_%s", dev_name(&pdata->pdev->dev), name);
		args->value = nvhost_get_syncpt_client_managed(set_name);
		if (!args->value)
			return -EAGAIN;
		/* ... and store it for further references */
		pdata->client_managed_syncpt = args->value;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_FREE_CLIENT_MANAGED_SYNCPOINT:
	{
		struct nvhost_free_client_managed_syncpt_arg *args =
			(struct nvhost_free_client_managed_syncpt_arg *)buf;
		struct nvhost_device_data *pdata =
			platform_get_drvdata(priv->ch->dev);
		if (!args->value)
			break;
		if (args->value != pdata->client_managed_syncpt)
			return -EINVAL;
		nvhost_free_syncpt(args->value);
		pdata->client_managed_syncpt = 0;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_WAITBASES:
	{
		struct nvhost_device_data *pdata = \
			platform_get_drvdata(priv->ch->dev);
		((struct nvhost_get_param_args *)buf)->value =
			create_mask(pdata->waitbases,
					NVHOST_MODULE_MAX_WAITBASES);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_WAITBASE:
	{
		struct nvhost_device_data *pdata = \
			platform_get_drvdata(priv->ch->dev);
		struct nvhost_get_param_arg *arg =
			(struct nvhost_get_param_arg *)buf;
		if (arg->param >= NVHOST_MODULE_MAX_WAITBASES
				|| !pdata->waitbases[arg->param])
			return -EINVAL;
		arg->value = pdata->waitbases[arg->param];
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_MODMUTEXES:
	{
		struct nvhost_device_data *pdata = \
			platform_get_drvdata(priv->ch->dev);
		((struct nvhost_get_param_args *)buf)->value =
			create_mask(pdata->modulemutexes,
					NVHOST_MODULE_MAX_MODMUTEXES);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_MODMUTEX:
	{
		struct nvhost_device_data *pdata = \
			platform_get_drvdata(priv->ch->dev);
		struct nvhost_get_param_arg *arg =
			(struct nvhost_get_param_arg *)buf;
		if (arg->param >= NVHOST_MODULE_MAX_MODMUTEXES
				|| !pdata->modulemutexes[arg->param])
			return -EINVAL;
		arg->value = pdata->modulemutexes[arg->param];
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_NVMAP_FD:
		break;
	case NVHOST_IOCTL_CHANNEL_GET_CLK_RATE:
	{
		struct nvhost_clk_rate_args *arg =
				(struct nvhost_clk_rate_args *)buf;

		err = nvhost_ioctl_channel_get_rate(priv,
				arg->moduleid, &arg->rate);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_CLK_RATE:
	{
		struct nvhost_clk_rate_args *arg =
				(struct nvhost_clk_rate_args *)buf;

		err = nvhost_ioctl_channel_set_rate(priv, arg);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SET_TIMEOUT:
	{
		u32 timeout =
			(u32)((struct nvhost_set_timeout_args *)buf)->timeout;

		priv->timeout = timeout;
		dev_dbg(&priv->ch->dev->dev,
			"%s: setting buffer timeout (%d ms) for userctx 0x%p\n",
			__func__, priv->timeout, priv);
		if (priv->hwctx)
			priv->hwctx->timeout_ms_max = timeout;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_GET_TIMEDOUT:
		((struct nvhost_get_param_args *)buf)->value =
				priv->hwctx->has_timedout;
		break;
	case NVHOST_IOCTL_CHANNEL_SET_PRIORITY:
		priv->priority =
			(u32)((struct nvhost_set_priority_args *)buf)->priority;
		break;
	case NVHOST32_IOCTL_CHANNEL_MODULE_REGRDWR:
	{
		struct nvhost32_ctrl_module_regrdwr_args *args32 =
			(struct nvhost32_ctrl_module_regrdwr_args *)buf;
		struct nvhost_ctrl_module_regrdwr_args args;
		args.id = args32->id;
		args.num_offsets = args32->num_offsets;
		args.block_size = args32->block_size;
		args.offsets = args32->offsets;
		args.values = args32->values;
		args.write = args32->write;
		err = nvhost_ioctl_channel_module_regrdwr(priv, &args);
		break;
	}
	case NVHOST_IOCTL_CHANNEL_MODULE_REGRDWR:
		err = nvhost_ioctl_channel_module_regrdwr(priv, (void *)buf);
		break;
	case NVHOST32_IOCTL_CHANNEL_SUBMIT:
	{
		struct nvhost32_submit_args *args32 = (void *)buf;
		struct nvhost_submit_args args;

		memset(&args, 0, sizeof(args));
		args.submit_version = args32->submit_version;
		args.num_syncpt_incrs = args32->num_syncpt_incrs;
		args.num_cmdbufs = args32->num_cmdbufs;
		args.num_relocs = args32->num_relocs;
		args.num_waitchks = args32->num_waitchks;
		args.timeout = args32->timeout;
		args.syncpt_incrs = args32->syncpt_incrs;
		args.fence = args32->fence;

		args.cmdbufs = args32->cmdbufs;
		args.relocs = args32->relocs;
		args.reloc_shifts = args32->reloc_shifts;
		args.waitchks = args32->waitchks;
		args.waitbases = args32->waitbases;
		args.class_ids = args32->class_ids;
		args.fences = args32->fences;

		err = nvhost_ioctl_channel_submit(priv, &args);
		args32->fence = args.fence;
		break;
	}
	case NVHOST_IOCTL_CHANNEL_SUBMIT:
		err = nvhost_ioctl_channel_submit(priv, (void *)buf);
		break;
	case NVHOST_IOCTL_CHANNEL_SET_ERROR_NOTIFIER:
		err = nvhost_init_error_notifier(priv->ch,
			(struct nvhost_set_error_notifier *)buf);
		break;
	case NVHOST_IOCTL_CHANNEL_SET_TIMEOUT_EX:
	{
		u32 timeout =
			(u32)((struct nvhost_set_timeout_args *)buf)->timeout;
		bool timeout_debug_dump = !((u32)
			((struct nvhost_set_timeout_ex_args *)buf)->flags &
			(1 << NVHOST_TIMEOUT_FLAG_DISABLE_DUMP));
		priv->timeout = timeout;
		priv->timeout_debug_dump = timeout_debug_dump;
		dev_dbg(&priv->ch->dev->dev,
			"%s: setting buffer timeout (%d ms) for userctx 0x%p\n",
			__func__, priv->timeout, priv);
		if (priv->hwctx) {
			priv->hwctx->timeout_ms_max = timeout;
			priv->hwctx->timeout_debug_dump = timeout_debug_dump;
		}
		break;
	}
	default:
		nvhost_dbg_info("unrecognized ioctl cmd: 0x%x", cmd);
		err = -ENOTTY;
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}

static const struct file_operations nvhost_channelops = {
	.owner = THIS_MODULE,
	.release = nvhost_channelrelease,
	.open = nvhost_channelopen,
#ifdef CONFIG_COMPAT
	.compat_ioctl = nvhost_channelctl,
#endif
	.unlocked_ioctl = nvhost_channelctl
};

struct nvhost_hwctx *nvhost_channel_get_file_hwctx(int fd)
{
	struct nvhost_channel_userctx *userctx;
	struct file *f = fget(fd);
	if (!f)
		return 0;

	if (f->f_op != &nvhost_channelops) {
		fput(f);
		return 0;
	}

	userctx = (struct nvhost_channel_userctx *)f->private_data;
	fput(f);
	return userctx->hwctx;
}

static struct {
	int class_id;
	const char *dev_name;
} class_id_dev_name_map[] = {
	/*	{ NV_HOST1X_CLASS_ID, ""}, */
	{ NV_VIDEO_ENCODE_MPEG_CLASS_ID, "mpe" },
	{ NV_VIDEO_ENCODE_MSENC_CLASS_ID, "msenc" },
	{ NV_GRAPHICS_3D_CLASS_ID, "gr3d" },
	{ NV_GRAPHICS_GPU_CLASS_ID, "gpu"},
	{ NV_GRAPHICS_VIC_CLASS_ID, "vic"},
	{ NV_TSEC_CLASS_ID, "tsec" },
};

static struct {
	int module_id;
	const char *dev_name;
} module_id_dev_name_map[] = {
	{ NVHOST_MODULE_VI, "vi"},
	{ NVHOST_MODULE_ISP, "isp"},
	{ NVHOST_MODULE_MPE, "mpe"},
	{ NVHOST_MODULE_MSENC, "msenc"},
	{ NVHOST_MODULE_TSEC, "tsec"},
	{ NVHOST_MODULE_GPU, "gpu"},
	{ NVHOST_MODULE_VIC, "vic"},
};

static const char *get_device_name_for_dev(struct platform_device *dev)
{
	int i;
	/* first choice is to use the class id if specified */
	for (i = 0; i < ARRAY_SIZE(class_id_dev_name_map); i++) {
		struct nvhost_device_data *pdata = nvhost_get_devdata(dev);
		if (pdata->class == class_id_dev_name_map[i].class_id)
			return class_id_dev_name_map[i].dev_name;
	}

	/* second choice is module name if specified */
	for (i = 0; i < ARRAY_SIZE(module_id_dev_name_map); i++) {
		struct nvhost_device_data *pdata = nvhost_get_devdata(dev);
		if (pdata->moduleid == module_id_dev_name_map[i].module_id)
			return module_id_dev_name_map[i].dev_name;
	}

	/* last choice is to just use the given dev name */
	return dev->name;
}

static struct device *nvhost_client_device_create(
	struct platform_device *pdev, struct cdev *cdev,
	const char *cdev_name, dev_t devno,
	const struct file_operations *ops)
{
	struct nvhost_master *host = nvhost_get_host(pdev);
	const char *use_dev_name;
	struct device *dev;
	int err;

	nvhost_dbg_fn("");

	BUG_ON(!host);

	cdev_init(cdev, ops);
	cdev->owner = THIS_MODULE;

	err = cdev_add(cdev, devno, 1);
	if (err < 0) {
		dev_err(&pdev->dev,
			"failed to add cdev\n");
		return NULL;
	}
	use_dev_name = get_device_name_for_dev(pdev);

	dev = device_create(host->nvhost_class,
			NULL, devno, NULL,
			(pdev->id <= 0) ?
			IFACE_NAME "-%s%s" :
			IFACE_NAME "-%s%s.%d",
			cdev_name, use_dev_name, pdev->id);

	if (IS_ERR(dev)) {
		err = PTR_ERR(dev);
		dev_err(&pdev->dev,
			"failed to create %s %s device for %s\n",
			use_dev_name, cdev_name, pdev->name);
		return NULL;
	}

	return dev;
}

#define NVHOST_NUM_CDEV 4
int nvhost_client_user_init(struct platform_device *dev)
{
	dev_t devno;
	int err;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	/* reserve 3 minor #s for <dev>, and ctrl-<dev> */

	err = alloc_chrdev_region(&devno, 0, NVHOST_NUM_CDEV, IFACE_NAME);
	if (err < 0) {
		dev_err(&dev->dev, "failed to allocate devno\n");
		goto fail;
	}
	pdata->cdev_region = devno;

	pdata->node = nvhost_client_device_create(dev, &pdata->cdev,
				"", devno, &nvhost_channelops);
	if (pdata->node == NULL)
		goto fail;

	/* module control (npn-channel based, global) interface */
	if (pdata->ctrl_ops) {
		++devno;
		pdata->ctrl_node = nvhost_client_device_create(dev,
					&pdata->ctrl_cdev, "ctrl-",
					devno, pdata->ctrl_ops);
		if (pdata->ctrl_node == NULL)
			goto fail;
	}

	return 0;
fail:
	return err;
}

void nvhost_client_user_deinit(struct platform_device *dev)
{
	struct nvhost_master *nvhost_master = nvhost_get_host(dev);
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	if (pdata->node) {
		device_destroy(nvhost_master->nvhost_class, pdata->cdev.dev);
		cdev_del(&pdata->cdev);
	}

	if (pdata->as_node) {
		device_destroy(nvhost_master->nvhost_class, pdata->as_cdev.dev);
		cdev_del(&pdata->as_cdev);
	}

	if (pdata->ctrl_node) {
		device_destroy(nvhost_master->nvhost_class,
			       pdata->ctrl_cdev.dev);
		cdev_del(&pdata->ctrl_cdev);
	}

	unregister_chrdev_region(pdata->cdev_region, NVHOST_NUM_CDEV);
}

int nvhost_client_device_init(struct platform_device *dev)
{
	int err;
	struct nvhost_master *nvhost_master = nvhost_get_host(dev);
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	pdata->channels = kzalloc(pdata->num_channels *
					sizeof(struct nvhost_channel *),
					GFP_KERNEL);

	/* Create debugfs directory for the device */
	nvhost_device_debug_init(dev);

	err = nvhost_client_user_init(dev);
	if (err)
		goto fail;

	err = nvhost_device_list_add(dev);
	if (err)
		goto fail;

	if (pdata->scaling_init)
		pdata->scaling_init(dev);

	/* reset syncpoint values for this unit */
	err = nvhost_module_busy(nvhost_master->dev);
	if (err)
		goto fail_busy;

	nvhost_syncpt_reset_client(dev);
	nvhost_module_idle(nvhost_master->dev);

	/* Initialize dma parameters */
	dev->dev.dma_parms = &pdata->dma_parms;
	dma_set_max_seg_size(&dev->dev, UINT_MAX);

	dev_info(&dev->dev, "initialized\n");

	if (pdata->slave && !pdata->slave_initialized) {
		struct nvhost_device_data *slave_pdata =
					pdata->slave->dev.platform_data;
		slave_pdata->master = dev;
		pdata->slave->dev.parent = dev->dev.parent;
		platform_device_register(pdata->slave);
		pdata->slave_initialized = 1;
	}

	return 0;

fail_busy:
	/* Remove from nvhost device list */
	nvhost_device_list_remove(dev);
fail:
	/* Add clean-up */
	dev_err(&dev->dev, "failed to init client device\n");
	nvhost_client_user_deinit(dev);
	nvhost_device_debug_deinit(dev);
	return err;
}
EXPORT_SYMBOL(nvhost_client_device_init);

int nvhost_client_device_release(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	/* Release nvhost module resources */
	nvhost_module_deinit(dev);

	/* Remove from nvhost device list */
	nvhost_device_list_remove(dev);

	/* Release chardev and device node for user space */
	nvhost_client_user_deinit(dev);

	/* Remove debugFS */
	nvhost_device_debug_deinit(dev);

	/* Release all nvhost channel of dev*/
	nvhost_channel_release(pdata);

	return 0;
}
EXPORT_SYMBOL(nvhost_client_device_release);

int nvhost_client_device_get_resources(struct platform_device *dev)
{
	int i;
	void __iomem *regs = NULL;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	for (i = 0; i < dev->num_resources; i++) {
		struct resource *r = NULL;

		r = platform_get_resource(dev, IORESOURCE_MEM, i);
		/* We've run out of mem resources */
		if (!r)
			break;

		regs = devm_request_and_ioremap(&dev->dev, r);
		if (!regs)
			goto fail;

		pdata->aperture[i] = regs;
	}

	return 0;

fail:
	dev_err(&dev->dev, "failed to get register memory\n");

	return -ENXIO;
}
EXPORT_SYMBOL(nvhost_client_device_get_resources);

/* This is a simple wrapper around request_firmware that takes
 * 'fw_name' and if available applies a SOC relative path prefix to it.
 * The caller is responsible for calling release_firmware later.
 */
const struct firmware *
nvhost_client_request_firmware(struct platform_device *dev, const char *fw_name)
{
	struct nvhost_chip_support *op = nvhost_get_chip_ops();
	const struct firmware *fw;
	char *fw_path = NULL;
	int path_len, err;

	/* This field is NULL when calling from SYS_EXIT.
	   Add a check here to prevent crash in request_firmware */
	if (!current->fs) {
		BUG();
		return NULL;
	}

	if (!fw_name)
		return NULL;

	if (op->soc_name) {
		path_len = strlen(fw_name) + strlen(op->soc_name);
		path_len += 2; /* for the path separator and zero terminator*/

		fw_path = kzalloc(sizeof(*fw_path) * path_len,
				     GFP_KERNEL);
		if (!fw_path)
			return NULL;

		sprintf(fw_path, "%s/%s", op->soc_name, fw_name);
		fw_name = fw_path;
	}

	err = request_firmware(&fw, fw_name, &dev->dev);
	kfree(fw_path);
	if (err) {
		dev_err(&dev->dev, "failed to get firmware\n");
		return NULL;
	}

	/* note: caller must release_firmware */
	return fw;
}
EXPORT_SYMBOL(nvhost_client_request_firmware);
