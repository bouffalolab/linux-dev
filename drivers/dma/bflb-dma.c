// SPDX-License-Identifier: GPL-2.0+
/*
 * Bouffalo Lab DMA Engine Driver
 *
 * Copyright (c) 2024 Bouffalo Lab
 * Author: qhli@bouffalolab.com
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/slab.h>
#include <linux/bitfield.h>
#include <dt-bindings/dma/bflb-dma.h>
#include "virt-dma.h"

/* Global DMA controller registers. */
#define BFLB_DMA_REG_INT_STATUS         0x0
#define BFLB_DMA_REG_INT_TC_STATUS      0x4
#define BFLB_DMA_REG_INT_TC_CLR         0x8
#define BFLB_DMA_REG_INT_ERR_STATUS     0xC
#define BFLB_DMA_REG_INT_ERR_CLR        0x10
#define BFLB_DMA_REG_RAW_INT_STATUS     0x14
#define BFLB_DMA_REG_RAW_INT_ERR_STATUS 0x18
#define BFLB_DMA_REG_ENABLED_CHAN       0x1C
#define BFLB_DMA_REG_SOFT_BREQ          0x20
#define BFLB_DMA_REG_SOFT_SREQ          0x24
#define BFLB_DMA_REG_SOFT_LBREQ         0x28
#define BFLB_DMA_REG_SOFT_LSREQ         0x2C
#define BFLB_DMA_REG_CONFIG             0x30
#define BFLB_DMA_REG_SYNC               0x34

/* DMA config register. */
#define BFLB_DMA_CFG_SMDMA_EN   BIT(0)
#define BFLB_DMA_CFG_ENDIAN     BIT(1)

/* Channel registers */
#define BFLB_DMA_CHAN_REG_BASE(i)       ((i) * 0x100)
#define BFLB_DMA_CHAN_REG_SRC_ADDR      0x0
#define BFLB_DMA_CHAN_REG_DST_ADDR      0x4
#define BFLB_DMA_CHAN_REG_LLI_ADDR      0x8
#define BFLB_DMA_CHAN_REG_CTRL          0xC
#define BFLB_DMA_CHAN_REG_CONFIG        0x10
#define BFLB_DMA_CHAN_REG_RSVD          0x1C

/* DMA channel control register. */
#define BFLB_DMA_CHAN_CTRL_XFER_SIZE    GENMASK(11, 0)
#define BFLB_DMA_CHAN_CTRL_SBURST       GENMASK(13, 12)
#define BFLB_DMA_CHAN_CTRL_DST_MIN_MODE BIT(14)
#define BFLB_DMA_CHAN_CTRL_DBURST       GENMASK(16, 15)
#define BFLB_DMA_CHAN_CTRL_DST_ADD_MODE BIT(17)
#define BFLB_DMA_CHAN_CTRL_SWIDTH       GENMASK(19, 18)
#define BFLB_DMA_CHAN_CTRL_DWIDTH       GENMASK(22, 21)
#define BFLB_DMA_CHAN_CTRL_FIX_CNT      GENMASK(25, 23)
#define BFLB_DMA_CHAN_CTRL_SRC_INC      BIT(26)
#define BFLB_DMA_CHAN_CTRL_DST_INC      BIT(27)
#define BFLB_DMA_CHAN_CTRL_PROT         GENMASK(30, 28)
#define BFLB_DMA_CHAN_CTRL_INT_EN       BIT(31)

/* DMA channeL config register. */
#define BFLB_DMA_CHAN_CONFIG_EN             BIT(0)
#define BFLB_DMA_CHAN_CONFIG_SRC_REQ        GENMASK(5, 1)
#define BFLB_DMA_CHAN_CONFIG_DST_REQ        GENMASK(10, 6)
#define BFLB_DMA_CHAN_CONFIG_FLOWCTRL       GENMASK(13, 11)
#define BFLB_DMA_CHAN_CONFIG_INT_ERR_MASK   BIT(14)
#define BFLB_DMA_CHAN_CONFIG_INT_TC_MASK    BIT(15)
#define BFLB_DMA_CHAN_CONFIG_LOCK           BIT(16)
#define BFLB_DMA_CHAN_CONFIG_ACTIVE         BIT(17)
#define BFLB_DMA_CHAN_CONFIG_HALT           BIT(18)
#define BFLB_DMA_CHAN_CONFIG_LLI_CNT        GENMASK(29, 20)

/* Maximum of transfer size of a LLI, not in bytes. */
#define BFLB_DMA_LLI_MAX_XFER_SIZE  4095
#define BFLB_DMA_LLI_MAX_XFER_BYTES (BFLB_DMA_LLI_MAX_XFER_SIZE * 4)
#define BFLB_DMA_LLI_XFER_SIZE_MASK 0xFFF

struct bflb_dma_lli {
	/* Source Address. */
	u32 src;
	/* Destination Address. */
	u32 dst;
	/* Address of the next lli. */
	u32 next;
	/* Control register of the lli. */
	u32 ctrl;
	/* Node on the lli_list of bflb_dma_txd. */
	struct list_head node;
	/* DMA address of this lli that is used by hardware. */
	dma_addr_t addr;
	/* Length of the LLI in bytes. */
	u32 len;
};

struct bflb_dma_txd {
	/* Virtual DMA descriptor */
	struct virt_dma_desc vd;
	/* Link list of lli nodes */
	struct list_head lli_list;
	/* Flag to indicate cyclic transfers */
	bool cyclic;
	enum dma_transfer_direction dir;
	/* Length of the transfer in bytes. */
	u32 len;
};

struct bflb_dma_pchan {
	/* Physical index to this channel */
	u32 id;
	/* Virtual memory base for the dma channel */
	void __iomem *base;
	/* The vchan currently being served by the pchan */
	struct bflb_dma_vchan *vchan;
	/* */
	struct bflb_dma_device *bd;
};

struct bflb_dma_vchan {
	/* The virtual channel */
	struct virt_dma_chan vc;
	/* The physical channel utilized by this channel */
	struct bflb_dma_pchan *pchan;
	/* The active transaction on this channel */
	struct bflb_dma_txd	*txd;
	/* Slave configuration for this channel */
	struct dma_slave_config cfg;
	/* Source and destination request ID for this channel */
	u8 src_req;
	u8 dst_req;
	/* Flag that indicating if the virtual channel has been configured. */
	bool configured;
	/* Flag that indicating if the virtual channel has been paused. */
	bool paused;
};

struct bflb_dma_device {
	/* DMA engine for this instance. */
	struct dma_device dma;
	void __iomem *base;
	struct clk *clk;
	/* Lock to use when change DMA controller global register. */
	spinlock_t lock;
	/* A pool for the LLI descriptors. */
	struct dma_pool *lli_pool;
	int *irqs;
	/* The number of physical channels. */
	unsigned int nr_pchans;
	/* Array of data for the physical channels. */
	struct bflb_dma_pchan *pchans;
	/* The number of virtual channels. */
	unsigned int nr_vchans;
	/* Array of data for the virtual channels. */
	struct bflb_dma_vchan *vchans;
};

static void pchan_update(struct bflb_dma_pchan *pchan, u32 reg,
		u32 mask, u32 val)
{
	u32 regval;

	regval = readl(pchan->base + reg);
	regval &= ~mask;
	regval |= val;
	writel(regval, pchan->base + reg);
}

static void pchan_writel(struct bflb_dma_pchan *pchan, u32 reg, u32 data)
{
	writel(data, pchan->base + reg);
}

static u32 pchan_readl(struct bflb_dma_pchan *pchan, u32 reg)
{
	return readl(pchan->base + reg);
}

static void dma_update(struct bflb_dma_device *bd, u32 reg, u32 mask, u32 val)
{
	u32 regval;

	regval = readl(bd->base + reg);
	regval &= ~mask;
	regval |= val;
	writel(regval, bd->base + reg);
}

static void dma_writel(struct bflb_dma_device *bd, u32 reg, u32 data)
{
	writel(data, bd->base + reg);
}

static u32 dma_readl(struct bflb_dma_device *bd, u32 reg)
{
	return readl(bd->base + reg);
}

#define to_bflb_dma(d)  container_of(d, struct bflb_dma_device, dma)

static struct device *chan2dev(struct dma_chan *chan)
{
	return &chan->dev->device;
}

#define to_bflb_vchan(c)    container_of(chan, struct bflb_dma_vchan, vc.chan)

#define to_bflb_txd(x)  container_of(x, struct bflb_dma_txd, vd.tx)

static void bflb_dma_free_lli(struct bflb_dma_device *bd,
		struct bflb_dma_lli *lli)
{
	list_del(&lli->node);
	dma_pool_free(bd->lli_pool, lli, lli->addr);
}

static struct bflb_dma_lli *bflb_dma_alloc_lli(struct bflb_dma_device *bd)
{
	dma_addr_t addr;
	struct bflb_dma_lli *lli;

	lli = dma_pool_alloc(bd->lli_pool, GFP_NOWAIT, &addr);
	if (!lli)
		return NULL;

	INIT_LIST_HEAD(&lli->node);
	lli->addr = addr;
	return lli;
}

static int bflb_dma_txd_add_lli(struct bflb_dma_txd *txd,
		struct bflb_dma_lli *lli)
{
	struct bflb_dma_lli *last;

	if (!list_empty(&txd->lli_list)) {
		last = list_last_entry(&txd->lli_list,
				struct bflb_dma_lli, node);
		last->next = lli->addr;
	}
	list_add_tail(&lli->node, &txd->lli_list);
	return 0;
}

static void bflb_dma_txd_circularize_lli(struct bflb_dma_txd *txd)
{
	struct bflb_dma_lli *first, *last;

	first = list_first_entry_or_null(&txd->lli_list,
			struct bflb_dma_lli, node);
	if (!first)
		return;

	last = list_last_entry(&txd->lli_list, struct bflb_dma_lli, node);
	last->next = first->addr;
}

static const u32 bflb_bus_width_tbl[] = {
	[DMA_SLAVE_BUSWIDTH_1_BYTE] = 0,
	[DMA_SLAVE_BUSWIDTH_2_BYTES] = 1,
	[DMA_SLAVE_BUSWIDTH_4_BYTES] = 2,
};

static const u32 bflb_bus_burst_tbl[] = {
	[1] = 0,
	[4] = 1,
	[8] = 2,
	[16] = 3,
};

static inline int bflb_dma_config_lli(struct bflb_dma_lli *lli,
		struct dma_slave_config *cfg, dma_addr_t src, dma_addr_t dst,
		u32 xfer_size, u32 bytes, enum dma_transfer_direction dir,
		u8 int_en)
{
	u32 width, burst;

	lli->src = src;
	lli->dst = dst;
	lli->next = 0;
	lli->ctrl = 0;
	lli->len = bytes;

	/* Set transfer size. */
	lli->ctrl |= FIELD_PREP(BFLB_DMA_CHAN_CTRL_XFER_SIZE, xfer_size);

	/* Source burst. */
	burst = bflb_bus_burst_tbl[cfg->src_maxburst];
	lli->ctrl |= FIELD_PREP(BFLB_DMA_CHAN_CTRL_SBURST, burst);

	/* Destination burst. */
	burst = bflb_bus_burst_tbl[cfg->dst_maxburst];
	lli->ctrl |= FIELD_PREP(BFLB_DMA_CHAN_CTRL_DBURST, burst);

	/* Source address width. */
	width = bflb_bus_width_tbl[cfg->src_addr_width];
	lli->ctrl |= FIELD_PREP(BFLB_DMA_CHAN_CTRL_SWIDTH, width);
	/* Destination address width. */
	width = bflb_bus_width_tbl[cfg->dst_addr_width];
	lli->ctrl |= FIELD_PREP(BFLB_DMA_CHAN_CTRL_DWIDTH, width);

	/* Source and destination address increase. */
	if (dir == DMA_MEM_TO_DEV) {
		lli->ctrl |= FIELD_PREP(BFLB_DMA_CHAN_CTRL_SRC_INC, 1);
		lli->ctrl |= FIELD_PREP(BFLB_DMA_CHAN_CTRL_DST_INC, 0);
	} else if (dir == DMA_DEV_TO_MEM) {
		lli->ctrl |= FIELD_PREP(BFLB_DMA_CHAN_CTRL_SRC_INC, 0);
		lli->ctrl |= FIELD_PREP(BFLB_DMA_CHAN_CTRL_DST_INC, 1);
	} else if (dir == DMA_MEM_TO_MEM) {
		lli->ctrl |= FIELD_PREP(BFLB_DMA_CHAN_CTRL_SRC_INC, 1);
		lli->ctrl |= FIELD_PREP(BFLB_DMA_CHAN_CTRL_DST_INC, 1);
	}

	/* Interrupt enable. */
	lli->ctrl |= FIELD_PREP(BFLB_DMA_CHAN_CTRL_INT_EN, int_en);
	return 0;
}

static struct bflb_dma_pchan *bflb_dma_get_pchan(struct bflb_dma_device *bd,
		struct bflb_dma_vchan *vchan)
{
	int i;
	unsigned long flags;
	struct bflb_dma_pchan *pchan = NULL;

	for (i = 0; i < bd->nr_pchans; i++) {
		pchan = &bd->pchans[i];

		spin_lock_irqsave(&bd->lock, flags);
		if (!pchan->vchan) {
			pchan->vchan = vchan;
			spin_unlock_irqrestore(&bd->lock, flags);
			break;
		}

		spin_unlock_irqrestore(&bd->lock, flags);
	}

	return pchan;
}

static inline bool bflb_dma_pchan_busy(struct bflb_dma_pchan *pchan)
{
	unsigned int val;

	val = pchan_readl(pchan, BFLB_DMA_CHAN_REG_CONFIG);

	return !!FIELD_GET(BFLB_DMA_CHAN_CONFIG_EN, val);
}

static void bflb_dma_terminate_pchan(struct bflb_dma_device *bd,
		struct bflb_dma_pchan *pchan)
{
	u32 cfg, pending;
	unsigned long flags;

	cfg = pchan_readl(pchan, BFLB_DMA_CHAN_REG_CONFIG);
	/* Disable this channel. */
	cfg &= ~BFLB_DMA_CHAN_CONFIG_EN;
	/* Mask complete and error interrupt. */
	cfg |= BFLB_DMA_CHAN_CONFIG_INT_TC_MASK;
	cfg |= BFLB_DMA_CHAN_CONFIG_INT_ERR_MASK;
	pchan_writel(pchan, BFLB_DMA_CHAN_REG_CONFIG, cfg);

	/* Clear interrupt and error status. */
	spin_lock_irqsave(&bd->lock, flags);
	pending = dma_readl(bd, BFLB_DMA_REG_INT_TC_STATUS);
	if (pending & BIT(pchan->id)) {
		dev_warn(bd->dma.dev,
		"terminating pchan %d still has pending irq\n", pchan->id);
	}
	dma_update(bd, BFLB_DMA_REG_INT_TC_CLR, BIT(pchan->id), BIT(pchan->id));
	dma_update(bd, BFLB_DMA_REG_INT_ERR_CLR, BIT(pchan->id), BIT(pchan->id));
	/* The physical channel is idle now. */
	pchan->vchan = NULL;
	spin_unlock_irqrestore(&bd->lock, flags);
}

static void bflb_dma_pause_pchan(struct bflb_dma_pchan *pchan)
{
	pchan_update(pchan, BFLB_DMA_CHAN_REG_CONFIG, BFLB_DMA_CHAN_CONFIG_HALT,
			BFLB_DMA_CHAN_CONFIG_HALT);
}

static void bflb_dma_resume_pchan(struct bflb_dma_pchan *pchan)
{
	pchan_update(pchan, BFLB_DMA_CHAN_REG_CONFIG,
			BFLB_DMA_CHAN_CONFIG_HALT, 0);
}

/* vc.lock must be held by caller. */
static int bflb_dma_start_next_txd(struct bflb_dma_vchan *vchan)
{
	unsigned long flags;
	struct bflb_dma_lli *lli;
	struct bflb_dma_device *bd = to_bflb_dma(vchan->vc.chan.device);
	struct virt_dma_desc *vd = vchan_next_desc(&vchan->vc);
	struct bflb_dma_pchan *pchan = vchan->pchan;
	struct bflb_dma_txd *txd = to_bflb_txd(&vd->tx);

	if (!vd) {
		dev_err(bd->dma.dev, "vchan is scheduled for nothing?\n");
		return -EINVAL;
	}

	if (vchan->txd) {
		dev_warn(bd->dma.dev, "vchan is already processing a txd\n");
		return 0;
	}

	/* Detach this descriptor from issued list in the virt channel. */
	list_del(&vd->node);

	vchan->txd = txd;

	/* Wait for channel inactive. */
	while (bflb_dma_pchan_busy(pchan))
		cpu_relax();

	lli = list_first_entry_or_null(&txd->lli_list,
			struct bflb_dma_lli, node);
	if (!lli) {
		/* TODO treat it as completed. */
		dev_warn(bd->dma.dev,
			"the txd has no LLIs, treat it as completed\n");
		return 0;
	}

	/* Configure this channel. */
	pchan_writel(pchan, BFLB_DMA_CHAN_REG_SRC_ADDR, lli->src);
	pchan_writel(pchan, BFLB_DMA_CHAN_REG_DST_ADDR, lli->dst);
	pchan_writel(pchan, BFLB_DMA_CHAN_REG_LLI_ADDR, lli->next);
	pchan_writel(pchan, BFLB_DMA_CHAN_REG_CTRL, lli->ctrl);

	u32 chan_cfg = pchan_readl(pchan, BFLB_DMA_CHAN_REG_CONFIG);
	/* Unmask interrupt. */
	chan_cfg &= ~BFLB_DMA_CHAN_CONFIG_INT_TC_MASK;
	chan_cfg &= ~BFLB_DMA_CHAN_CONFIG_INT_ERR_MASK;
	if (txd->dir == DMA_MEM_TO_MEM)
		chan_cfg |= FIELD_PREP(BFLB_DMA_CHAN_CONFIG_FLOWCTRL, 0);
	else if (txd->dir == DMA_MEM_TO_DEV)
		chan_cfg |= FIELD_PREP(BFLB_DMA_CHAN_CONFIG_FLOWCTRL, 1);
	else if (txd->dir == DMA_DEV_TO_MEM)
		chan_cfg |= FIELD_PREP(BFLB_DMA_CHAN_CONFIG_FLOWCTRL, 2);
	/* Configure the source and destination requests. */
	chan_cfg &= ~BFLB_DMA_CHAN_CONFIG_SRC_REQ;
	chan_cfg |= FIELD_PREP(BFLB_DMA_CHAN_CONFIG_SRC_REQ, vchan->src_req);
	chan_cfg &= ~BFLB_DMA_CHAN_CONFIG_DST_REQ;
	chan_cfg |= FIELD_PREP(BFLB_DMA_CHAN_CONFIG_DST_REQ, vchan->dst_req);
	/* Clear IRQ status for this pchan. */
	spin_lock_irqsave(&bd->lock, flags);
	dma_writel(bd, BFLB_DMA_REG_INT_TC_CLR, BIT(pchan->id));
	dma_writel(bd, BFLB_DMA_REG_INT_ERR_CLR, BIT(pchan->id));
	spin_unlock_irqrestore(&bd->lock, flags);

	dev_dbg(bd->dma.dev, "starting pchan %d\n", pchan->id);

	/* Start DMA transfer for this pchan. */
	chan_cfg |= BFLB_DMA_CHAN_CONFIG_EN;
	pchan_writel(pchan, BFLB_DMA_CHAN_REG_CONFIG, chan_cfg);
	return 0;
}

static void bflb_dma_put_pchan(struct bflb_dma_device *bd,
		struct bflb_dma_vchan *vchan)
{
	/* Ensure that the physical channel is stopped */
	bflb_dma_terminate_pchan(bd, vchan->pchan);

	vchan->pchan = NULL;
}

static int bflb_dma_start_transfer(struct bflb_dma_vchan *vchan)
{
	struct bflb_dma_pchan *pchan;
	struct bflb_dma_device *bd = to_bflb_dma(vchan->vc.chan.device);

	pchan = bflb_dma_get_pchan(bd, vchan);
	if (!pchan)
		return -EBUSY;

	dev_dbg(bd->dma.dev, "allocated pchan %d\n", pchan->id);

	/* Now the vchan is attached to the pchan. */
	vchan->pchan = pchan;
	bflb_dma_start_next_txd(vchan);
	return 0;
}

static irqreturn_t bflb_dma_interrupt(int irq, void *dev_id)
{
	bool sched = false;
	u32 pending, err_status;
	struct bflb_dma_txd *txd;
	struct bflb_dma_vchan *vchan;
	struct bflb_dma_pchan *pchan = dev_id;
	struct bflb_dma_device *bd = pchan->bd;

	spin_lock(&bd->lock);
	/* TODO handle DMA errors. */
	err_status = dma_readl(bd, BFLB_DMA_REG_INT_ERR_STATUS);
	if (err_status) {
		dev_err(bd->dma.dev, "error status %x, chan %d\n",
				err_status, pchan->id);
		dma_writel(bd, BFLB_DMA_REG_INT_ERR_CLR, err_status);
	}

	pending = dma_readl(bd, BFLB_DMA_REG_INT_TC_STATUS);
	if (unlikely(!(pending & BIT(pchan->id)))) {
		/* Invoked for nothing. */
		spin_unlock(&bd->lock);
		return IRQ_HANDLED;
	}

	/* Clear IRQ status for this pchan. */
	pchan_writel(pchan, BFLB_DMA_REG_INT_TC_CLR, BIT(pchan->id));
	spin_unlock(&bd->lock);

	vchan = pchan->vchan;
	if (!vchan) {
		/* Maybe the vchan has already been terminated. */
		dev_warn(bd->dma.dev, "no vchan attached on pchan %d\n",
				pchan->id);
		return IRQ_HANDLED;
	}

	spin_lock(&vchan->vc.lock);

	txd = vchan->txd;
	if (txd) {
		if (txd->cyclic) {
			/*
			 * The physical channel is currently used by this
			 * virtual channel unless the client terminates it
			 * explicitly, so just schedule complete callbacks.
			 */
			vchan_cyclic_callback(&txd->vd);
		} else {
			/* Prepare to serve the next txd. */
			vchan->txd = NULL;
			/* Schedule complete callbacks. */
			vchan_cookie_complete(&txd->vd);
		}
	}

	/*
	 * Since the current transfer is done, try to start the next one if any,
	 * otherwise free this physical channel.
	 */
	if (!vchan->txd) {
		if (vchan_next_desc(&vchan->vc)) {
			bflb_dma_start_next_txd(vchan);
		} else {
			/*
			 * A physical channel is free now, we need to check
			 * if any other virtual channel needs service.
			 */
			bflb_dma_put_pchan(bd, vchan);
			sched = true;
		}
	}

	spin_unlock(&vchan->vc.lock);

	if (sched) {
		list_for_each_entry(vchan, &bd->dma.channels,
				vc.chan.device_node) {
			spin_lock(&vchan->vc.lock);
			if (vchan_next_desc(&vchan->vc) &&
			    !vchan->pchan && !vchan->paused)
				bflb_dma_start_transfer(vchan);
			spin_unlock(&vchan->vc.lock);
		}
	}
	return IRQ_HANDLED;
}

static struct bflb_dma_txd *bflb_dma_alloc_txd(bool cyclic,
		enum dma_transfer_direction dir, u32 len)
{
	struct bflb_dma_txd *txd;

	txd = kzalloc(sizeof(*txd), GFP_NOWAIT);
	if (!txd)
		return NULL;

	INIT_LIST_HEAD(&txd->lli_list);
	txd->cyclic = cyclic;
	txd->dir = dir;
	txd->len = len;
	return txd;
}

static void bflb_dma_free_txd(struct bflb_dma_device *bd,
		struct bflb_dma_txd *txd)
{
	struct bflb_dma_lli *lli, *_lli;

	if (unlikely(!txd))
		return;

	list_for_each_entry_safe(lli, _lli, &txd->lli_list, node)
		bflb_dma_free_lli(bd, lli);

	kfree(txd);
}

static void bflb_dma_desc_free(struct virt_dma_desc *vd)
{
	struct bflb_dma_device *bd = to_bflb_dma(vd->tx.chan->device);
	struct bflb_dma_txd *txd = to_bflb_txd(&vd->tx);

	bflb_dma_free_txd(bd, txd);
}

/*
 * This is a synchronous way to terminate a channel, so device_synchronize
 * is not implemented.
 */
static int bflb_dma_terminate_all(struct dma_chan *chan)
{
	struct bflb_dma_device *bd = to_bflb_dma(chan->device);
	struct bflb_dma_vchan *vchan = to_bflb_vchan(chan);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&vchan->vc.lock, flags);

	/* Detach the vchan from the pchan even if the pchan is still working. */
	if (vchan->pchan)
		bflb_dma_put_pchan(bd, vchan);

	/*
	 * The pchan has already been disabled, thus the current pending txd
	 * should be aborted silently.
	 */
	if (vchan->txd) {
		bflb_dma_desc_free(&vchan->txd->vd);
		vchan->txd = NULL;
	}

	vchan_get_all_descriptors(&vchan->vc, &head);
	spin_unlock_irqrestore(&vchan->vc.lock, flags);

	vchan_dma_desc_free_list(&vchan->vc, &head);
	return 0;
}

static int bflb_dma_config(struct dma_chan *chan,
		struct dma_slave_config *config)
{
	u32 burst;
	struct bflb_dma_vchan *vchan = to_bflb_vchan(chan);

	/* Reject definitely invalid configurations */
	if (config->src_addr_width > DMA_SLAVE_BUSWIDTH_4_BYTES ||
			config->dst_addr_width > DMA_SLAVE_BUSWIDTH_4_BYTES)
		return -EINVAL;

	burst = config->src_maxburst;
	if (burst != 1 && burst != 4 && burst != 8 && burst != 16)
		return -EINVAL;

	burst = config->dst_maxburst;
	if (burst != 1 && burst != 4 && burst != 8 && burst != 16)
		return -EINVAL;

	memcpy(&vchan->cfg, config, sizeof(vchan->cfg));
	vchan->configured = true;
	return 0;
}

static int bflb_dma_pause(struct dma_chan *chan)
{
	unsigned long flags;
	struct bflb_dma_vchan *vchan = to_bflb_vchan(chan);

	dev_dbg(chan2dev(chan), "vchan(%u %u) pause\n",
			vchan->src_req, vchan->dst_req);
	spin_lock_irqsave(&vchan->vc.lock, flags);
	if (vchan->pchan)
		bflb_dma_pause_pchan(vchan->pchan);
	vchan->paused = true;
	spin_unlock_irqrestore(&vchan->vc.lock, flags);

	return 0;
}

static int bflb_dma_resume(struct dma_chan *chan)
{
	unsigned long flags;
	struct bflb_dma_vchan *vchan = to_bflb_vchan(chan);

	dev_dbg(chan2dev(chan), "vchan(%u %u) resume\n",
			vchan->src_req, vchan->dst_req);
	spin_lock_irqsave(&vchan->vc.lock, flags);
	if (vchan->pchan)
		bflb_dma_resume_pchan(vchan->pchan);
	vchan->paused = false;
	spin_unlock_irqrestore(&vchan->vc.lock, flags);

	return 0;
}

static u32 bflb_dma_get_chan_residue(struct bflb_dma_vchan *vchan)
{
	size_t bytes = 0;
	u32 src, dst, done;
	bool busy_lli = true;
	struct bflb_dma_txd *txd;
	struct bflb_dma_lli *lli;
	struct bflb_dma_pchan *pchan;

	pchan = vchan->pchan;
	txd = vchan->txd;

	if (!pchan || !txd)
		return 0;

	if (txd->cyclic)
		return 0;

	/* Get remain count of current node in link list */
	u32 cfg = pchan_readl(pchan, BFLB_DMA_CHAN_REG_CONFIG);
	u32 lli_cnt = FIELD_GET(BFLB_DMA_CHAN_CONFIG_LLI_CNT, cfg);

	/* Loop through the preceding nodes to get total remaining bytes */
	list_for_each_entry(lli, &txd->lli_list, node) {
		if (lli_cnt > 0) {
			/* The leading lli_cnt LLIs has already been processed. */
			lli_cnt--;
		} else if (busy_lli) {
			/* The hardware is currently processing the LLI. */
			busy_lli = false;
			switch (txd->dir) {
			case DMA_MEM_TO_DEV:
			case DMA_MEM_TO_MEM:
				src = pchan_readl(pchan, BFLB_DMA_CHAN_REG_SRC_ADDR);
				if (src > lli->src) {
					done = src - lli->src;
					if (lli->len > done)
						bytes += lli->len - done;
				}
				break;

			case DMA_DEV_TO_MEM:
				dst = pchan_readl(pchan, BFLB_DMA_CHAN_REG_DST_ADDR);
				if (dst > lli->dst) {
					done = dst - lli->dst;
					if (lli->len > done)
						bytes += lli->len - done;
				}
				break;

			default:
				bytes = 0;
				break;
			}
		} else {
			/* The LLI has not been processed by the hardware. */
			bytes += lli->len;
		}
	}

	return bytes;
}

static enum dma_status bflb_dma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *state)
{
	size_t bytes;
	enum dma_status ret;
	unsigned long flags;
	struct virt_dma_desc *vd;
	struct bflb_dma_txd *txd;
	struct bflb_dma_vchan *vchan = to_bflb_vchan(chan);

	ret = dma_cookie_status(chan, cookie, state);
	if (ret == DMA_COMPLETE || !state)
		return ret;

	spin_lock_irqsave(&vchan->vc.lock, flags);

	ret = dma_cookie_status(chan, cookie, state);
	if (ret == DMA_COMPLETE) {
		spin_unlock_irqrestore(&vchan->vc.lock, flags);
		return ret;
	}

	vd = vchan_find_desc(&vchan->vc, cookie);
	if (vd) {
		/*
		 * This txd is still on the issued_list, which means that
		 * it is not processed yet.
		 */
		txd = to_bflb_txd(&vd->tx);
		bytes = txd->len;
	} else if (vchan->txd->vd.tx.cookie == cookie) {
		bytes = bflb_dma_get_chan_residue(vchan);
	} else {
		bytes = 0;
	}

	spin_unlock_irqrestore(&vchan->vc.lock, flags);

	dma_set_residue(state, bytes);
	return ret;
}

static void bflb_dma_issue_pending(struct dma_chan *chan)
{
	unsigned long flags;
	struct bflb_dma_vchan *vchan = to_bflb_vchan(chan);

	spin_lock_irqsave(&vchan->vc.lock, flags);
	if (vchan_issue_pending(&vchan->vc)) {
		if (!vchan->pchan)
			bflb_dma_start_transfer(vchan);
	}
	spin_unlock_irqrestore(&vchan->vc.lock, flags);
}

static int bflb_dma_analyze_transfer(struct dma_slave_config *cfg,
		enum dma_transfer_direction dir, size_t len,
		size_t *xfer_size, size_t *stride_factor, size_t *partial_bytes)
{
	enum dma_slave_buswidth addr_width;

	/* Memory copy based on word is more effective.  */
	if (dir == DMA_MEM_TO_MEM) {
		*xfer_size = len >> 2;
		*stride_factor = 4;
		*partial_bytes = len & 3;
		return 0;
	}

	if (dir == DMA_MEM_TO_DEV)
		addr_width = cfg->src_addr_width;
	else if (dir == DMA_DEV_TO_MEM)
		addr_width = cfg->dst_addr_width;
	else
		addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	switch (addr_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		*partial_bytes = 0;
		*xfer_size = len;
		*stride_factor = 1;
		break;

	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		*partial_bytes = len & 1;
		*xfer_size = len >> 1;
		*stride_factor = 2;
		break;

	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		*partial_bytes = len & 3;
		*xfer_size = len >> 2;
		*stride_factor = 4;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

/*
 * Allocate one or multiple lli to set up a transfer.
 * Return 0 on success, an error code otherwise, and the caller
 * is supposed to free the transfer descriptor on failure.
 */
static int bflb_dma_prep_transfer(struct bflb_dma_device *bd,
		struct bflb_dma_vchan *vc, struct bflb_dma_txd *txd,
		dma_addr_t src, dma_addr_t dst, size_t len,
		enum dma_transfer_direction dir, u8 int_en)
{
	int err;
	u8 lli_int_en;
	struct bflb_dma_lli *lli;
	size_t xfers, xfer_size, stride_factor;
	struct dma_slave_config *cfg = &vc->cfg;
	size_t offset, bytes, remain_xfers, partial_bytes;

	err = bflb_dma_analyze_transfer(cfg, dir, len, &xfer_size,
			&stride_factor, &partial_bytes);
	if (err) {
		dev_err(bd->dma.dev, "Invalid transfer, %d\n", err);
		return err;
	}

	if (is_slave_direction(dir) && partial_bytes) {
		dev_err(bd->dma.dev, "Invalid slave transfer\n");
		return -EINVAL;
	}

	/* LLIs will be configured based on the config. so prepare it. */
	if (dir == DMA_MEM_TO_MEM && xfer_size) {
		cfg->src_addr_width = 4;
		cfg->dst_addr_width = 4;
		cfg->src_maxburst = 1;
		cfg->dst_maxburst = 1;
	}

	for (offset = 0; offset < xfer_size; offset += xfers) {
		lli = bflb_dma_alloc_lli(bd);
		if (!lli) {
			dev_warn(bd->dma.dev, "failed to allocate lli\n");
			return -ENOMEM;
		}

		lli_int_en = 0;
		remain_xfers = xfer_size - offset;

		/* Enable interrupt on last lli. */
		if (remain_xfers <= BFLB_DMA_LLI_MAX_XFER_SIZE) {
			xfers = remain_xfers;
			if (int_en && !partial_bytes)
				lli_int_en = 1;
		} else {
			xfers = BFLB_DMA_LLI_MAX_XFER_SIZE;
		}

		bytes = xfers * stride_factor;
		err = bflb_dma_config_lli(lli, cfg, src, dst, xfers, bytes,
				dir, lli_int_en);
		if (err) {
			dev_warn(bd->dma.dev, "failed to config lli\n");
			bflb_dma_free_lli(bd, lli);
			return err;
		}

		bflb_dma_txd_add_lli(txd, lli);

		if (dir == DMA_MEM_TO_DEV) {
			src += bytes;
		} else if (dir == DMA_DEV_TO_MEM) {
			dst += bytes;
		} else if (dir == DMA_MEM_TO_MEM) {
			src += bytes;
			dst += bytes;
		}
	}

	if (partial_bytes) {
		/* Handle the partial data by byte. */
		cfg->src_addr_width = 1;
		cfg->dst_addr_width = 1;
		cfg->src_maxburst = 1;
		cfg->dst_maxburst = 1;

		lli = bflb_dma_alloc_lli(bd);
		if (!lli) {
			dev_warn(bd->dma.dev, "failed to allocate lli\n");
			return -ENOMEM;
		}

		err = bflb_dma_config_lli(lli, cfg, src, dst, 1, partial_bytes,
				dir, 1);
		if (err) {
			dev_warn(bd->dma.dev, "failed to config lli\n");
			bflb_dma_free_lli(bd, lli);
			return err;
		}

		bflb_dma_txd_add_lli(txd, lli);
	}
	return 0;
}

static struct dma_async_tx_descriptor *
bflb_dma_prep_memcpy(struct dma_chan *chan, dma_addr_t dst, dma_addr_t src,
		size_t len, unsigned long flags)
{
	int ret;
	struct bflb_dma_txd *txd;
	struct bflb_dma_device *bd = to_bflb_dma(chan->device);
	struct bflb_dma_vchan *vchan = to_bflb_vchan(chan);

	if (!len)
		return NULL;

	txd = bflb_dma_alloc_txd(false, DMA_MEM_TO_MEM, len);
	if (!txd)
		return NULL;

	ret = bflb_dma_prep_transfer(bd, vchan, txd, src, dst, len,
			DMA_MEM_TO_MEM, 1);
	if (ret) {
		dev_err(bd->dma.dev, "failed to set up memcpy transfer, %d\n", ret);
		goto err_txd_free;
	}

	return vchan_tx_prep(&vchan->vc, &txd->vd, flags);

err_txd_free:
	bflb_dma_free_txd(bd, txd);
	return NULL;
}

static struct dma_async_tx_descriptor *
bflb_dma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction dir,
		unsigned long flags, void *context)
{
	size_t len;
	int ret, i;
	u32 tot_len = 0;
	struct scatterlist *sg;
	struct bflb_dma_txd *txd;
	dma_addr_t addr, src = 0, dst = 0;
	struct bflb_dma_device *bd = to_bflb_dma(chan->device);
	struct bflb_dma_vchan *vchan = to_bflb_vchan(chan);
	struct dma_slave_config *cfg = &vchan->cfg;

	/* Reject the request if the vchan is not event configured. */
	if (!vchan->configured)
		return NULL;

	if (!is_slave_direction(dir))
		return NULL;

	for_each_sg(sgl, sg, sg_len, i)
		tot_len += sg_dma_len(sg);

	txd = bflb_dma_alloc_txd(false, dir, tot_len);
	if (!txd)
		return NULL;

	for_each_sg(sgl, sg, sg_len, i) {
		addr = sg_dma_address(sg);
		len = sg_dma_len(sg);

		if (dir == DMA_MEM_TO_DEV) {
			src = addr;
			dst = cfg->dst_addr;
		} else {
			src = cfg->src_addr;
			dst = addr;
		}

		/* Only enable interrupt on the last lli of last transfer. */
		u8 int_en;

		int_en = i == (sg_len - 1);
		ret = bflb_dma_prep_transfer(bd, vchan, txd, src, dst,
				len, dir, int_en);
		if (ret) {
			dev_err(bd->dma.dev, "failed to prep slave_sg transfer, %d\n", ret);
			goto err_txd_free;
		}
	}

	return vchan_tx_prep(&vchan->vc, &txd->vd, flags);

err_txd_free:
	bflb_dma_free_txd(bd, txd);
	return NULL;
}

static struct dma_async_tx_descriptor *
bflb_prep_dma_cyclic(struct dma_chan *chan,
		dma_addr_t buf_addr, size_t buf_len,
		size_t period_len,
		enum dma_transfer_direction dir,
		unsigned long flags)
{
	int ret, i;
	struct bflb_dma_txd *txd;
	dma_addr_t src = 0, dst = 0;
	unsigned int periods = buf_len / period_len;
	struct bflb_dma_device *bd = to_bflb_dma(chan->device);
	struct bflb_dma_vchan *vchan = to_bflb_vchan(chan);
	struct dma_slave_config *cfg = &vchan->cfg;

	/* Reject the request if the vchan is not event configured. */
	if (!vchan->configured)
		return NULL;

	if (!is_slave_direction(dir))
		return NULL;

	txd = bflb_dma_alloc_txd(true, dir, (u32)buf_len);
	if (!txd)
		return NULL;

	for (i = 0; i < periods; i++) {
		if (dir == DMA_MEM_TO_DEV) {
			src = buf_addr + (period_len * i);
			dst = cfg->dst_addr;
		} else {
			src = cfg->src_addr;
			dst = buf_addr + (period_len * i);
		}

		ret = bflb_dma_prep_transfer(bd, vchan, txd, src, dst,
				period_len, dir, 1);
		if (ret) {
			dev_err(bd->dma.dev, "failed to prep cyclic transfer, %d\n", ret);
			goto err_txd_free;
		}
	}

	bflb_dma_txd_circularize_lli(txd);
	return vchan_tx_prep(&vchan->vc, &txd->vd, flags);

err_txd_free:
	bflb_dma_free_txd(bd, txd);
	return NULL;
}

static void bflb_dma_free_chan_resources(struct dma_chan *chan)
{
	struct bflb_dma_vchan *vchan = to_bflb_vchan(chan);

	/* Ensure all queued descriptors are freed */
	vchan_free_chan_resources(&vchan->vc);
}

static inline void bflb_dma_free(struct bflb_dma_device *bd)
{
	struct bflb_dma_vchan *vchan = NULL;
	struct bflb_dma_vchan *next;

	list_for_each_entry_safe(vchan,
			next, &bd->dma.channels, vc.chan.device_node) {
		list_del(&vchan->vc.chan.device_node);
		tasklet_kill(&vchan->vc.task);
	}
}

static struct dma_chan *bflb_dma_of_xlate(struct of_phandle_args *dma_spec,
		struct of_dma *ofdma)
{
	struct dma_chan *chan;
	struct bflb_dma_vchan *vchan;
	u8 src_req = FIELD_GET(DMA_SRC_REQ_MASK, dma_spec->args[0]);
	u8 dst_req = FIELD_GET(DMA_DST_REQ_MASK, dma_spec->args[0]);
	struct bflb_dma_device *bd = ofdma->of_dma_data;

	if (src_req > DMA_REQ_MAX || dst_req > DMA_REQ_MAX)
		return NULL;

	chan = dma_get_any_slave_channel(&bd->dma);
	if (!chan)
		return NULL;

	vchan = to_bflb_vchan(chan);
	vchan->src_req = src_req;
	vchan->dst_req = dst_req;
	return chan;
}

#define BFLB_DMA_BUS_WIDTH_MASK			\
	(BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |	\
	BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |	\
	BIT(DMA_SLAVE_BUSWIDTH_3_BYTES) |	\
	BIT(DMA_SLAVE_BUSWIDTH_4_BYTES))

static void bflb_dma_enable(struct bflb_dma_device *bd)
{
	u32 cfg;

	cfg = dma_readl(bd, BFLB_DMA_REG_CONFIG);
	cfg |= BFLB_DMA_CFG_SMDMA_EN;
	dma_writel(bd, BFLB_DMA_REG_CONFIG, cfg);
}

static void bflb_dma_disable(struct bflb_dma_device *bd)
{
	u32 cfg;

	cfg = dma_readl(bd, BFLB_DMA_REG_CONFIG);
	cfg &= ~BFLB_DMA_CFG_SMDMA_EN;
	dma_writel(bd, BFLB_DMA_REG_CONFIG, cfg);
}

static int bflb_dma_pchan_init(struct bflb_dma_device *bd,
		struct bflb_dma_pchan *pchan, int idx)
{
	int err;

	struct platform_device *pdev = to_platform_device(bd->dma.dev);

	pchan->bd = bd;
	pchan->id = idx;
	pchan->base = bd->base + BFLB_DMA_CHAN_REG_BASE(idx + 1);
	bd->irqs[idx] = platform_get_irq(pdev, idx);
	if (bd->irqs[idx] < 0) {
		dev_err(&pdev->dev, "unable to get IRQ with index %d, err %d\n",
				idx, bd->irqs[idx]);
		return bd->irqs[idx];
	}

	err = devm_request_irq(&pdev->dev, bd->irqs[idx], bflb_dma_interrupt, 0,
			dev_name(&pdev->dev), pchan);
	if (err) {
		dev_err(&pdev->dev, "unable to request IRQ with index %d, err %d\n",
				idx, err);
	}
	return err;
}

static int bflb_dma_probe(struct platform_device *pdev)
{
	int ret, i;
	struct bflb_dma_device *bd;
	struct device_node *np = pdev->dev.of_node;

	bd = devm_kzalloc(&pdev->dev, sizeof(*bd), GFP_KERNEL);
	if (!bd)
		return -ENOMEM;

	/* Remember the device itself. */
	bd->dma.dev = &pdev->dev;

	bd->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(bd->clk)) {
		dev_err(&pdev->dev, "unable to get clock\n");
		return PTR_ERR(bd->clk);
	}

	bd->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bd->base))
		return PTR_ERR(bd->base);

	ret = of_property_read_u32(np, "dma-channels", &bd->nr_pchans);
	if (ret) {
		dev_err(&pdev->dev, "can't get dma-channels\n");
		return ret;
	}

	ret = of_property_read_u32(np, "dma-requests", &bd->nr_vchans);
	if (ret) {
		dev_err(&pdev->dev, "can't get dma-requests\n");
		return ret;
	}

	dev_info(&pdev->dev, "dma-channels %d, dma-requests %d\n",
			bd->nr_pchans, bd->nr_vchans);

	bd->irqs = devm_kcalloc(&pdev->dev, bd->nr_pchans,
			sizeof(int), GFP_KERNEL);
	if (!bd->irqs)
		return -ENOMEM;

	/* Initialize the physical channels. */
	bd->pchans = devm_kcalloc(&pdev->dev, bd->nr_pchans,
			sizeof(struct bflb_dma_pchan), GFP_KERNEL);
	if (!bd->pchans)
		return -ENOMEM;

	for (i = 0; i < bd->nr_pchans; i++) {
		struct bflb_dma_pchan *pchan = &bd->pchans[i];

		ret = bflb_dma_pchan_init(bd, pchan, i);
		if (ret) {
			dev_info(&pdev->dev, "failed to initialise phy channel %d\n", i);
			return ret;
		}
	}

	/* Initialize the virtual channels. */
	bd->vchans = devm_kcalloc(&pdev->dev, bd->nr_vchans,
			sizeof(struct bflb_dma_vchan), GFP_KERNEL);
	if (!bd->vchans)
		return -ENOMEM;

	INIT_LIST_HEAD(&bd->dma.channels);
	for (i = 0; i < bd->nr_vchans; i++) {
		struct bflb_dma_vchan *vchan = &bd->vchans[i];

		vchan->vc.desc_free = bflb_dma_desc_free;
		vchan_init(&vchan->vc, &bd->dma);
	}

	/* Create a pool of consistent memory blocks for DMA linked list items. */
	bd->lli_pool = dmam_pool_create(dev_name(bd->dma.dev), bd->dma.dev,
			sizeof(struct bflb_dma_lli),
			__alignof__(struct bflb_dma_lli), 0);
	if (!bd->lli_pool) {
		dev_err(&pdev->dev, "unable to allocate DMA descriptor pool\n");
		return -ENOMEM;
	}

	/* TODO figure out the mask stuff */
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	platform_set_drvdata(pdev, bd);
	spin_lock_init(&bd->lock);

	/* Set DMA engine capabilities. */
	dma_set_max_seg_size(&pdev->dev, BFLB_DMA_LLI_MAX_XFER_BYTES);
	dma_cap_set(DMA_MEMCPY, bd->dma.cap_mask);
	dma_cap_set(DMA_SLAVE, bd->dma.cap_mask);
	dma_cap_set(DMA_CYCLIC, bd->dma.cap_mask);
	/* Install the following DMA operations. */
	bd->dma.device_free_chan_resources = bflb_dma_free_chan_resources;
	bd->dma.device_tx_status = bflb_dma_tx_status;
	bd->dma.device_issue_pending = bflb_dma_issue_pending;
	bd->dma.device_prep_dma_memcpy = bflb_dma_prep_memcpy;
	bd->dma.device_prep_slave_sg = bflb_dma_prep_slave_sg;
	bd->dma.device_prep_dma_cyclic = bflb_prep_dma_cyclic;
	bd->dma.device_config = bflb_dma_config;
	bd->dma.device_pause = bflb_dma_pause;
	bd->dma.device_resume = bflb_dma_resume;
	bd->dma.device_terminate_all = bflb_dma_terminate_all;
	bd->dma.src_addr_widths = BFLB_DMA_BUS_WIDTH_MASK;
	bd->dma.dst_addr_widths = BFLB_DMA_BUS_WIDTH_MASK;
	bd->dma.directions = BIT(DMA_MEM_TO_DEV) | BIT(DMA_DEV_TO_MEM) |
		BIT(DMA_MEM_TO_MEM);
	bd->dma.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;

	clk_prepare_enable(bd->clk);
	ret = dmaenginem_async_device_register(&bd->dma);
	if (ret) {
		dev_err(&pdev->dev, "failed to register DMA engine device\n");
		goto err_out;
	}

	/* Register a device tree parser for the controller. */
	ret = of_dma_controller_register(pdev->dev.of_node, bflb_dma_of_xlate, bd);
	if (ret) {
		dev_err(&pdev->dev, "of_dma_controller_register failed, %d\n", ret);
		goto err_out;
	}

	bflb_dma_enable(bd);
	dev_info(&pdev->dev, "Bouffalo Lab DMA Engine Driver Inited\n");
	return 0;

err_out:
	clk_disable_unprepare(bd->clk);
	return ret;
}

static int bflb_dma_remove(struct platform_device *pdev)
{
	int i;
	struct bflb_dma_device *bd = platform_get_drvdata(pdev);

	/* Mask all interrupts and disable all channels. */
	for (i = 0; i < bd->nr_pchans; i++) {
		u32 cfg;
		struct bflb_dma_pchan *pchan = &bd->pchans[i];

		cfg = pchan_readl(pchan, BFLB_DMA_CHAN_REG_CONFIG);
		cfg |= BFLB_DMA_CHAN_CONFIG_INT_TC_MASK;
		cfg |= BFLB_DMA_CHAN_CONFIG_INT_ERR_MASK;
		cfg &= ~BFLB_DMA_CHAN_CONFIG_EN;
		pchan_writel(pchan, BFLB_DMA_CHAN_REG_CONFIG, cfg);
	}
	bflb_dma_disable(bd);
	of_dma_controller_free(pdev->dev.of_node);
	bflb_dma_free(bd);
	clk_disable_unprepare(bd->clk);
	return 0;
}

static const struct of_device_id bflb_dma_of_match[] = {
	{ .compatible = "bflb,bl808-dma", .data = NULL },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, bflb_dma_of_match);

static struct platform_driver bflb_dma_driver = {
	.probe	= bflb_dma_probe,
	.remove	= bflb_dma_remove,
	.driver = {
		.name = "bflb-dma",
		.of_match_table = of_match_ptr(bflb_dma_of_match),
	},
};

module_platform_driver(bflb_dma_driver);

MODULE_AUTHOR("qhli@bouffalolab.com");
MODULE_DESCRIPTION("Bouffalo DMA Engine driver");
MODULE_LICENSE("GPL");
