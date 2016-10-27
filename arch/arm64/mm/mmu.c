/*
 * Based on arch/arm/mm/mmu.c
 *
 * Copyright (C) 1995-2005 Russell King
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/mman.h>
#include <linux/nodemask.h>
#include <linux/memblock.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/vmalloc.h>

#include <asm/cputype.h>
#include <asm/sections.h>
#include <asm/setup.h>
#include <asm/sizes.h>
#include <asm/tlb.h>
#include <asm/mmu_context.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "mm.h"

/*
 * Empty_zero_page is a special page that is used for zero-initialized data
 * and COW.
 */
unsigned long empty_zero_page;
EXPORT_SYMBOL(empty_zero_page);
unsigned long zero_page_mask;
static int zero_page_order = 3;

pgprot_t pgprot_default;
EXPORT_SYMBOL(pgprot_default);

static pmdval_t prot_sect_kernel;

struct cachepolicy {
	const char	policy[16];
	u64		mair;
	u64		tcr;
};

static struct cachepolicy cache_policies[] __initdata = {
	{
		.policy		= "uncached",
		.mair		= 0x44,			/* inner, outer non-cacheable */
		.tcr		= TCR_IRGN_NC | TCR_ORGN_NC,
	}, {
		.policy		= "writethrough",
		.mair		= 0xaa,			/* inner, outer write-through, read-allocate */
		.tcr		= TCR_IRGN_WT | TCR_ORGN_WT,
	}, {
		.policy		= "writeback",
		.mair		= 0xee,			/* inner, outer write-back, read-allocate */
		.tcr		= TCR_IRGN_WBnWA | TCR_ORGN_WBnWA,
	}
};

/*
 * These are useful for identifying cache coherency problems by allowing the
 * cache or the cache and writebuffer to be turned off. It changes the Normal
 * memory caching attributes in the MAIR_EL1 register.
 */
static int __init early_cachepolicy(char *p)
{
	int i;
	u64 tmp;

	for (i = 0; i < ARRAY_SIZE(cache_policies); i++) {
		int len = strlen(cache_policies[i].policy);

		if (memcmp(p, cache_policies[i].policy, len) == 0)
			break;
	}
	if (i == ARRAY_SIZE(cache_policies)) {
		pr_err("ERROR: unknown or unsupported cache policy: %s\n", p);
		return 0;
	}

	flush_cache_all();

	/*
	 * Modify MT_NORMAL attributes in MAIR_EL1.
	 */
	asm volatile(
	"	mrs	%0, mair_el1\n"
	"	bfi	%0, %1, #%2, #8\n"
	"	msr	mair_el1, %0\n"
	"	isb\n"
	: "=&r" (tmp)
	: "r" (cache_policies[i].mair), "i" (MT_NORMAL * 8));

	/*
	 * Modify TCR PTW cacheability attributes.
	 */
	asm volatile(
	"	mrs	%0, tcr_el1\n"
	"	bic	%0, %0, %2\n"
	"	orr	%0, %0, %1\n"
	"	msr	tcr_el1, %0\n"
	"	isb\n"
	: "=&r" (tmp)
	: "r" (cache_policies[i].tcr), "r" (TCR_IRGN_MASK | TCR_ORGN_MASK));

	flush_cache_all();

	return 0;
}
early_param("cachepolicy", early_cachepolicy);

/*
 * Adjust the PMD section entries according to the CPU in use.
 */
static void __init init_mem_pgprot(void)
{
	pteval_t default_pgprot;
	int i;

	default_pgprot = PTE_ATTRINDX(MT_NORMAL);
	prot_sect_kernel = PMD_TYPE_SECT | PMD_SECT_AF | PMD_ATTRINDX(MT_NORMAL);

#ifdef CONFIG_SMP
	/*
	 * Mark memory with the "shared" attribute for SMP systems
	 */
	default_pgprot |= PTE_SHARED;
	prot_sect_kernel |= PMD_SECT_S;
#endif

	for (i = 0; i < 16; i++) {
		unsigned long v = pgprot_val(protection_map[i]);
		protection_map[i] = __pgprot(v | default_pgprot);
	}

	pgprot_default = __pgprot(PTE_TYPE_PAGE | PTE_AF | default_pgprot);
}

pgprot_t phys_mem_access_prot(struct file *file, unsigned long pfn,
			      unsigned long size, pgprot_t vma_prot)
{
	if (!pfn_valid(pfn))
		return pgprot_noncached(vma_prot);
	else if (file->f_flags & O_SYNC)
		return pgprot_writecombine(vma_prot);
	return vma_prot;
}
EXPORT_SYMBOL(phys_mem_access_prot);

void __init *early_alloc(unsigned long sz)
{
	void *ptr = __va(memblock_alloc(sz, sz));
	memset(ptr, 0, sz);
	return ptr;
}

static void __init alloc_init_pte(pmd_t *pmd, unsigned long addr,
		unsigned long end, unsigned long pfn, unsigned int type)
{
	pte_t *pte;

	if (pmd_none(*pmd)) {
		pte = early_alloc(PTRS_PER_PTE * sizeof(pte_t));
		__pmd_populate(pmd, __pa(pte), PMD_TYPE_TABLE);
	}
	BUG_ON(pmd_bad(*pmd));

	pte = pte_offset_kernel(pmd, addr);
	do {
		switch (type) {
		case MT_NORMAL_NC:
			set_pte(pte, pfn_pte(pfn, PROT_NORMAL_NC));
			break;
		case MT_DEVICE_nGnRE:
			set_pte(pte, pfn_pte(pfn, PROT_DEVICE_nGnRE));
			break;
		case MT_MEMORY_KERNEL_EXEC:
			set_pte(pte, pfn_pte(pfn, PAGE_KERNEL_EXEC));
			break;
		default:
			BUG();
		}

		pfn++;
	} while (pte++, addr += PAGE_SIZE, addr != end);
}

static void __init alloc_init_pmd(pud_t *pud, unsigned long addr,
		unsigned long end, phys_addr_t phys, unsigned int type)
{
	pmd_t *pmd;
	unsigned long next;

	/*
	 * Check for initial section mappings in the pgd/pud and remove them.
	 */
	if (pud_none(*pud) || pud_bad(*pud)) {
		pmd = early_alloc(PTRS_PER_PMD * sizeof(pmd_t));
		pud_populate(&init_mm, pud, pmd);
	}

	pmd = pmd_offset(pud, addr);
	do {
		next = pmd_addr_end(addr, end);
		/* try section mapping first */
		if (((addr | next | phys) & ~SECTION_MASK) == 0) {
			pmd_t old_pmd = *pmd;
			switch (type) {
			case MT_NORMAL_NC:
				set_pmd(pmd, __pmd(phys | PROT_SECT_NORMAL_NC));
				break;
			case MT_DEVICE_nGnRE:
				set_pmd(pmd, __pmd(phys | PROT_SECT_DEVICE_nGnRE));
				break;
			case MT_MEMORY_KERNEL_EXEC:
				set_pmd(pmd, __pmd(phys | prot_sect_kernel));
				break;
			default:
				BUG();
			}
			/*
			 * Check for previous table entries created during
			 * boot (__create_page_tables) and flush them
			 */
			if (!pmd_none(old_pmd))
				flush_tlb_all();
		} else {
			alloc_init_pte(pmd, addr, next, __phys_to_pfn(phys), type);
		}
		phys += next - addr;
	} while (pmd++, addr = next, addr != end);
}

static void __init alloc_init_pud(pgd_t *pgd, unsigned long addr,
		unsigned long end, unsigned long phys, unsigned int type)
{
	pud_t *pud = pud_offset(pgd, addr);
	unsigned long next;

	do {
		next = pud_addr_end(addr, end);
		alloc_init_pmd(pud, addr, next, phys, type);
		phys += next - addr;
	} while (pud++, addr = next, addr != end);
}

/*
 * Create the page directory entries and any necessary page tables for the
 * mapping specified by 'md'.
 */
static void __init create_mapping(struct map_desc *io_desc)
{
	unsigned long addr, length, end, next;
	unsigned long virt = io_desc->virtual;
	unsigned long phys = __pfn_to_phys(io_desc->pfn);
	pgd_t *pgd;

	if (virt < VMALLOC_START) {
		pr_warning("BUG: not creating mapping for 0x%016lx at 0x%016lx - outside kernel range\n",
			   phys, virt);
		return;
	}

	addr = virt & PAGE_MASK;
	length = PAGE_ALIGN(io_desc->length + (virt & ~PAGE_MASK));

	pgd = pgd_offset_k(addr);
	end = addr + length;
	do {
		next = pgd_addr_end(addr, end);
		alloc_init_pud(pgd, addr, next, phys, io_desc->type);
		phys += next - addr;
	} while (pgd++, addr = next, addr != end);
}

/* To support old-style static device memory mapping. */
__init void iotable_init(struct map_desc *io_desc, int nr)
{
	struct map_desc *md;
	struct vm_struct *vm;

	vm = early_alloc(sizeof(*vm) * nr);

	for (md = io_desc; nr; md++, nr--) {
		create_mapping(md);
		vm->addr = (void *)(md->virtual & PAGE_MASK);
		vm->size = PAGE_ALIGN(md->length + (md->virtual & ~PAGE_MASK));
		vm->phys_addr = __pfn_to_phys(md->pfn);
		vm->flags = VM_IOREMAP;
		vm->caller = iotable_init;
		vm_area_add_early(vm++);
	}
}

__init void iotable_init_va(struct map_desc *io_desc, int nr)
{
	struct map_desc *md;
	struct vm_struct *vm;

	vm = early_alloc(sizeof(*vm) * nr);

	for (md = io_desc; nr; md++, nr--) {
		vm->addr = (void *)(md->virtual & PAGE_MASK);
		vm->size = PAGE_ALIGN(md->length + (md->virtual & ~PAGE_MASK));
		vm->phys_addr = __pfn_to_phys(md->pfn);
		vm->flags = VM_IOREMAP;
		vm->caller = iotable_init;
		vm_area_add_early(vm++);
	}
}

__init void iotable_init_mapping(struct map_desc *io_desc, int nr)
{
	struct map_desc *md;

	for (md = io_desc; nr; md++, nr--)
		create_mapping(md);
}

#ifdef CONFIG_EARLY_PRINTK
/*
 * Create an early I/O mapping using the pgd/pmd entries already populated
 * in head.S as this function is called too early to allocated any memory. The
 * mapping size is 2MB with 4KB pages or 64KB or 64KB pages.
 */
void __iomem *early_io_map(phys_addr_t phys, unsigned long virt)
{
	unsigned long size, mask;
	bool page64k = IS_ENABLED(CONFIG_ARM64_64K_PAGES);
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	/*
	 * No early pte entries with !ARM64_64K_PAGES configuration, so using
	 * sections (pmd).
	 */
	size = page64k ? PAGE_SIZE : SECTION_SIZE;
	mask = ~(size - 1);

	pgd = pgd_offset_k(virt);
	pud = pud_offset(pgd, virt);
	if (pud_none(*pud))
		return NULL;
	pmd = pmd_offset(pud, virt);

	if (page64k) {
		if (pmd_none(*pmd))
			return NULL;
		pte = pte_offset_kernel(pmd, virt);
		set_pte(pte, __pte((phys & mask) | PROT_DEVICE_nGnRE));
	} else {
		set_pmd(pmd, __pmd((phys & mask) | PROT_SECT_DEVICE_nGnRE));
	}

	return (void __iomem *)((virt & mask) + (phys & ~mask));
}
#endif

static void __init map_mem(void)
{
	struct memblock_region *reg;
	struct map_desc md;
	phys_addr_t limit;

	/*
	 * Temporarily limit the memblock range. We need to do this as
	 * create_mapping requires puds, pmds and ptes to be allocated from
	 * memory addressable from the initial direct kernel mapping.
	 *
	 * The initial direct kernel mapping, located at swapper_pg_dir,
	 * gives us PGDIR_SIZE memory starting from PHYS_OFFSET (which must be
	 * aligned to 2MB as per Documentation/arm64/booting.txt).
	 */
	limit = PHYS_OFFSET + PGDIR_SIZE;
	memblock_set_current_limit(limit);

	/* map all the memory banks */
	for_each_memblock(memory, reg) {
		phys_addr_t start = reg->base;
		phys_addr_t end = start + reg->size;

		md.virtual = __phys_to_virt(start);
		md.pfn = __phys_to_pfn(start);
		md.length = end - start;
		md.type = MT_MEMORY_KERNEL_EXEC;

		if (start >= end)
			break;

#ifndef CONFIG_ARM64_64K_PAGES
		/*
		 * For the first memory bank align the start address and
		 * current memblock limit to prevent create_mapping() from
		 * allocating pte page tables from unmapped memory.
		 * When 64K pages are enabled, the pte page table for the
		 * first PGDIR_SIZE is already present in swapper_pg_dir.
		 */
		if (start < limit)
			start = ALIGN(start, PMD_SIZE);
		if (end < limit) {
			limit = end & PMD_MASK;
			memblock_set_current_limit(limit);
		}
#endif

		create_mapping(&md);
	}
}

/*
 * paging_init() sets up the page tables, initialises the zone memory
 * maps and sets up the zero page.
 */
void __init paging_init(void)
{
	/*
	 * Maximum PGDIR_SIZE addressable via the initial direct kernel
	 * mapping in swapper_pg_dir.
	 */
	memblock_set_current_limit((PHYS_OFFSET & PGDIR_MASK) + PGDIR_SIZE);

	init_mem_pgprot();
	map_mem();

	/*
	 * Ask the machine support to map in the statically mapped devices.
	 */
	if (machine_desc->map_io)
		machine_desc->map_io();

	/*
	 * Finally flush the caches and tlb to ensure that we're in a
	 * consistent state.
	 */
	flush_cache_all();
	flush_tlb_all();

	/* allocate the zero page. */
	empty_zero_page = (ulong)early_alloc(PAGE_SIZE << zero_page_order);
	zero_page_mask = ((PAGE_SIZE << zero_page_order) - 1) & PAGE_MASK;

	bootmem_init();


	dma_contiguous_remap();
	/*
	 * TTBR0 is only used for the identity mapping at this stage. Make it
	 * point to zero page to avoid speculatively fetching new entries.
	 */
	cpu_set_reserved_ttbr0();
	flush_tlb_all();
}

/*
 * Enable the identity mapping to allow the MMU disabling.
 */
void setup_mm_for_reboot(void)
{
	cpu_switch_mm(idmap_pg_dir, &init_mm);
	flush_tlb_all();
}

/*
 * Check whether a kernel address is valid (derived from arch/x86/).
 */
int kern_addr_valid(unsigned long addr)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	if ((((long)addr) >> VA_BITS) != -1UL)
		return 0;

	pgd = pgd_offset_k(addr);
	if (pgd_none(*pgd))
		return 0;

	pud = pud_offset(pgd, addr);
	if (pud_none(*pud))
		return 0;

	pmd = pmd_offset(pud, addr);
	if (pmd_none(*pmd))
		return 0;

	pte = pte_offset_kernel(pmd, addr);
	if (pte_none(*pte))
		return 0;

	return pfn_valid(pte_pfn(*pte));
}
#ifdef CONFIG_SPARSEMEM_VMEMMAP
#ifdef CONFIG_ARM64_64K_PAGES
int __meminit vmemmap_populate(unsigned long start, unsigned long end, int node)
{
	return vmemmap_populate_basepages(start, end, node);
}
#else	/* !CONFIG_ARM64_64K_PAGES */
int __meminit vmemmap_populate(unsigned long start, unsigned long end, int node)
{
	unsigned long addr = start;
	unsigned long next;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;

	do {
		next = pmd_addr_end(addr, end);

		pgd = vmemmap_pgd_populate(addr, node);
		if (!pgd)
			return -ENOMEM;

		pud = vmemmap_pud_populate(pgd, addr, node);
		if (!pud)
			return -ENOMEM;

		pmd = pmd_offset(pud, addr);
		if (pmd_none(*pmd)) {
			void *p = NULL;

			p = vmemmap_alloc_block_buf(PMD_SIZE, node);
			if (!p)
				return -ENOMEM;

			set_pmd(pmd, __pmd(__pa(p) | prot_sect_kernel));
		} else
			vmemmap_verify((pte_t *)pmd, node, addr, next);
	} while (addr = next, addr != end);

	return 0;
}
#endif	/* CONFIG_ARM64_64K_PAGES */
void vmemmap_free(unsigned long start, unsigned long end)
{
}
#endif	/* CONFIG_SPARSEMEM_VMEMMAP */
