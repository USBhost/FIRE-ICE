/*
 * drivers/video/tegra/nvmap/nvmap.c
 *
 * Memory manager for Tegra GPU
 *
 * Copyright (c) 2009-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define pr_fmt(fmt)	"nvmap: %s() " fmt, __func__

#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/io.h>
#include <linux/rbtree.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/export.h>

#include <asm/pgtable.h>
#include <asm/tlbflush.h>

#include <linux/nvmap.h>
#include <trace/events/nvmap.h>

#include "nvmap_priv.h"

/* private nvmap_handle flag for pinning duplicate detection */
#define NVMAP_HANDLE_VISITED (0x1ul << 31)

static phys_addr_t handle_phys(struct nvmap_handle *h)
{
	phys_addr_t addr;

	if (h->heap_pgalloc) {
		BUG_ON(!h->attachment->priv);
		addr = sg_dma_address(
				((struct sg_table *)h->attachment->priv)->sgl);
	} else {
		addr = h->carveout->base;
	}

	return addr;
}

/*
 * Do the actual pin. Just calls to the dma_buf code.
 */
int __nvmap_pin(struct nvmap_handle_ref *ref, phys_addr_t *phys)
{
	struct nvmap_handle *h = ref->handle;
	struct sg_table *sgt = NULL;

	atomic_inc(&ref->pin);

	/*
	 * We should not be using a bidirectional mapping here; however, nvmap
	 * does not really keep track of whether memory is readable, writable,
	 * or both so this keeps everyone happy.
	 */
	sgt = dma_buf_map_attachment(h->attachment, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt))
		goto err;
	*phys = sg_dma_address(sgt->sgl);
	trace_nvmap_pin(h->owner, h->owner ? h->owner->name : "unknown", h,
			atomic_read(&h->pin));
	return 0;

err:
	atomic_dec(&ref->pin);
	return PTR_ERR(sgt);
}

void __nvmap_unpin(struct nvmap_handle_ref *ref)
{
	struct nvmap_handle *h = ref->handle;

	/*
	 * If the handle has been pinned by other refs it is possible to arrive
	 * here: the passed ref has a 0 pin count. This is of course invalid.
	 */
	if (!atomic_add_unless(&ref->pin, -1, 0))
		goto done;

	dma_buf_unmap_attachment(h->attachment,
		h->attachment->priv, DMA_BIDIRECTIONAL);

done:
	trace_nvmap_unpin(h->owner, h->owner ? h->owner->name : "unknown", h,
			atomic_read(&h->pin));
}

int nvmap_pin_ids(struct nvmap_client *client, unsigned int nr,
		  struct nvmap_handle * const *ids)
{
	int i, err = 0;
	phys_addr_t phys;
	struct nvmap_handle_ref *ref;

	/*
	 * Just try and pin every handle.
	 */
	nvmap_ref_lock(client);
	for (i = 0; i < nr; i++) {
		if (!ids[i] || !virt_addr_valid(ids[i]))
			continue;

		ref = __nvmap_validate_locked(client, ids[i]);
		if (!ref) {
			err = -EPERM;
			goto err_cleanup;
		}
		if (!ref->handle) {
			err = -EINVAL;
			goto err_cleanup;
		}

		err = __nvmap_pin(ref, &phys);
		if (err)
			goto err_cleanup;
	}
	nvmap_ref_unlock(client);
	return 0;

err_cleanup:
	for (--i; i >= 0; i--) {
		if (!ids[i] || !virt_addr_valid(ids[i]))
			continue;

		/*
		 * We will get the ref again - the ref lock has yet to be given
		 * up so if this worked the first time it will work again.
		 */
		ref = __nvmap_validate_locked(client, ids[i]);
		__nvmap_unpin(ref);
	}
	nvmap_ref_unlock(client);
	return err;
}

/*
 * This will unpin every handle. If an error occurs on a handle later handles
 * will still be unpinned.
 */
void nvmap_unpin_ids(struct nvmap_client *client, unsigned int nr,
		     struct nvmap_handle * const *ids)
{
	int i;
	struct nvmap_handle_ref *ref;

	nvmap_ref_lock(client);
	for (i = 0; i < nr; i++) {
		if (!ids[i] || !virt_addr_valid(ids[i]))
			continue;

		ref = __nvmap_validate_locked(client, ids[i]);
		if (!ref) {
			pr_info("ref is null during unpin.\n");
			continue;
		}
		if (!ref->handle) {
			WARN(1, "ref->handle is NULL.\n");
			continue;
		}

		__nvmap_unpin(ref);
	}
	nvmap_ref_unlock(client);
}

void *__nvmap_kmap(struct nvmap_handle *h, unsigned int pagenum)
{
	phys_addr_t paddr;
	unsigned long kaddr;
	pgprot_t prot;
	struct vm_struct *area = NULL;

	if (!virt_addr_valid(h))
		return NULL;

	h = nvmap_handle_get(h);
	if (!h)
		return NULL;

	if (pagenum >= h->size >> PAGE_SHIFT)
		goto out;
	prot = nvmap_pgprot(h, PG_PROT_KERNEL);
	area = alloc_vm_area(PAGE_SIZE, NULL);
	if (!area)
		goto out;
	kaddr = (ulong)area->addr;

	if (h->heap_pgalloc)
		paddr = page_to_phys(nvmap_to_page(h->pgalloc.pages[pagenum]));
	else
		paddr = h->carveout->base + pagenum * PAGE_SIZE;

	ioremap_page_range(kaddr, kaddr + PAGE_SIZE, paddr, prot);
	return (void *)kaddr;
out:
	nvmap_handle_put(h);
	return NULL;
}

void __nvmap_kunmap(struct nvmap_handle *h, unsigned int pagenum,
		  void *addr)
{
	phys_addr_t paddr;
	struct vm_struct *area = NULL;

	if (!h ||
	    WARN_ON(!virt_addr_valid(h)) ||
	    WARN_ON(!addr))
		return;

	if (WARN_ON(pagenum >= h->size >> PAGE_SHIFT))
		return;

	if (h->heap_pgalloc)
		paddr = page_to_phys(nvmap_to_page(h->pgalloc.pages[pagenum]));
	else
		paddr = h->carveout->base + pagenum * PAGE_SIZE;

	if (h->flags != NVMAP_HANDLE_UNCACHEABLE &&
	    h->flags != NVMAP_HANDLE_WRITE_COMBINE) {
		dmac_flush_range(addr, addr + PAGE_SIZE);
		outer_flush_range(paddr, paddr + PAGE_SIZE); /* FIXME */
	}

	area = find_vm_area(addr);
	if (area)
		free_vm_area(area);
	else
		WARN(1, "Invalid address passed");
	nvmap_handle_put(h);
}

void *__nvmap_mmap(struct nvmap_handle *h)
{
	pgprot_t prot;
	void *vaddr = NULL;
	unsigned long adj_size;
	struct vm_struct *v;
	void *p;
	struct page **pages;

	if (!virt_addr_valid(h))
		return NULL;

	h = nvmap_handle_get(h);
	if (!h)
		return NULL;

	if (!h->alloc)
		return NULL;

	prot = nvmap_pgprot(h, PG_PROT_KERNEL);

	if (h->heap_pgalloc) {

		if (!h->vaddr) {
			pages = nvmap_pages(h->pgalloc.pages,
					    h->size >> PAGE_SHIFT);
			if (!pages)
				return NULL;
			vaddr = vm_map_ram(pages,
				h->size >> PAGE_SHIFT, -1, prot);
			nvmap_altfree(pages,
				(h->size >> PAGE_SHIFT) * sizeof(*pages));
		}
#ifdef NVMAP_LAZY_VFREE
		if (vaddr && atomic_long_cmpxchg(&h->vaddr, 0, (long)vaddr))
			vm_unmap_ram(vaddr, h->size >> PAGE_SHIFT);
		return h->vaddr;
#endif
		return vaddr;
	}

	/* carveout - explicitly map the pfns into a vmalloc area */
	if (!h->carveout)
		return NULL;

	adj_size = h->carveout->base & ~PAGE_MASK;
	adj_size += h->size;
	adj_size = PAGE_ALIGN(adj_size);

	v = alloc_vm_area(adj_size, 0);
	if (!v) {
		nvmap_handle_put(h);
		return NULL;
	}

	p = v->addr + (h->carveout->base & ~PAGE_MASK);
	ioremap_page_range((ulong)v->addr, (ulong)v->addr + adj_size,
		h->carveout->base & PAGE_MASK, prot);

	/* leave the handle ref count incremented by 1, so that
	 * the handle will not be freed while the kernel mapping exists.
	 * nvmap_handle_put will be called by unmapping this address */
	return p;
}

void __nvmap_munmap(struct nvmap_handle *h, void *addr)
{
	if (!h ||
	    WARN_ON(!virt_addr_valid(h)) ||
	    WARN_ON(!addr))
		return;

	/* Handle can be locked by cache maintenance in
	 * separate thread */
	if (h->heap_pgalloc) {
#ifndef NVMAP_LAZY_VFREE
		BUG_ON(h->vaddr);
		vm_unmap_ram(addr, h->size >> PAGE_SHIFT);
#endif
	} else {
		struct vm_struct *vm;
		addr -= (h->carveout->base & ~PAGE_MASK);
		vm = find_vm_area(addr);
		BUG_ON(!vm);
		free_vm_area(vm);
	}
	nvmap_handle_put(h);
}

static struct nvmap_client *nvmap_get_dmabuf_client(void)
{
	static struct nvmap_client *client;

	if (!client) {
		struct nvmap_client *temp;

		temp = __nvmap_create_client(nvmap_dev, "dmabuf_client");
		if (!temp)
			return NULL;
		if (cmpxchg(&client, NULL, temp))
			nvmap_client_put(temp);
	}
	BUG_ON(!client);
	return client;
}

static struct nvmap_handle_ref *__nvmap_alloc(struct nvmap_client *client,
					      size_t size, size_t align,
					      unsigned int flags,
					      unsigned int heap_mask)
{
	const unsigned int default_heap = NVMAP_HEAP_CARVEOUT_GENERIC;
	struct nvmap_handle_ref *r = NULL;
	int err;

	if (!virt_addr_valid(client))
		return ERR_PTR(-EINVAL);

	if (heap_mask == 0)
		heap_mask = default_heap;

	r = nvmap_create_handle(client, size);
	if (IS_ERR(r))
		return r;

	err = nvmap_alloc_handle(client, __nvmap_ref_to_id(r),
				 heap_mask, align,
				 0, /* kind n/a */
				 flags & ~(NVMAP_HANDLE_KIND_SPECIFIED |
					   NVMAP_HANDLE_COMPR_SPECIFIED));

	if (err) {
		nvmap_free_handle(client, __nvmap_ref_to_id(r));
		return ERR_PTR(err);
	}

	return r;
}

static void __nvmap_free(struct nvmap_client *client,
			 struct nvmap_handle_ref *r)
{
	struct nvmap_handle *handle = __nvmap_ref_to_id(r);

	if (!r ||
	    WARN_ON(!virt_addr_valid(client)) ||
	    WARN_ON(!virt_addr_valid(r)) ||
	    WARN_ON(!virt_addr_valid(handle)))
		return;

	nvmap_free_handle(client, handle);
}

struct dma_buf *nvmap_alloc_dmabuf(size_t size, size_t align,
				   unsigned int flags,
				   unsigned int heap_mask)
{
	struct dma_buf *dmabuf;
	struct nvmap_handle_ref *ref;
	struct nvmap_client *client = nvmap_get_dmabuf_client();

	ref = __nvmap_alloc(client, size, align, flags, heap_mask);
	if (!ref)
		return ERR_PTR(-ENOMEM);
	if (IS_ERR(ref))
		return (struct dma_buf *)ref;

	dmabuf = __nvmap_dmabuf_export_from_ref(ref);
	__nvmap_free(client, ref);
	return dmabuf;
}

void nvmap_handle_put(struct nvmap_handle *h)
{
	int cnt;

	if (WARN_ON(!virt_addr_valid(h)))
		return;
	cnt = atomic_dec_return(&h->ref);

	if (WARN_ON(cnt < 0)) {
		pr_err("%s: %s put to negative references\n",
			__func__, current->comm);
	} else if (cnt == 0)
		_nvmap_handle_free(h);
}

struct sg_table *__nvmap_sg_table(struct nvmap_client *client,
		struct nvmap_handle *h)
{
	struct sg_table *sgt = NULL;
	int err, npages;
	struct page **pages;

	if (!virt_addr_valid(h))
		return ERR_PTR(-EINVAL);

	h = nvmap_handle_get(h);
	if (!h)
		return ERR_PTR(-EINVAL);

	npages = PAGE_ALIGN(h->size) >> PAGE_SHIFT;
	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		err = -ENOMEM;
		goto err;
	}

	if (!h->heap_pgalloc) {
		err = sg_alloc_table(sgt, 1, GFP_KERNEL);
		if (err)
			goto err;
		sg_set_buf(sgt->sgl, phys_to_virt(handle_phys(h)), h->size);
	} else {
		pages = nvmap_pages(h->pgalloc.pages, npages);
		if (!pages) {
			err = -ENOMEM;
			goto err;
		}
		err = sg_alloc_table_from_pages(sgt, pages,
				npages, 0, h->size, GFP_KERNEL);
		nvmap_altfree(pages, npages * sizeof(*pages));
		if (err)
			goto err;
	}
	nvmap_handle_put(h);
	return sgt;

err:
	kfree(sgt);
	nvmap_handle_put(h);
	return ERR_PTR(err);
}

void __nvmap_free_sg_table(struct nvmap_client *client,
		struct nvmap_handle *h, struct sg_table *sgt)
{
	sg_free_table(sgt);
	kfree(sgt);
}
