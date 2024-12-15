// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2017-2024 Bouffalo Lab
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/mfd/syscon.h>
#include <linux/debugfs.h>
#include "spi-bflb.h"

#define BFLB_SPI_DRIVER_NAME	"bflb-spi"

/* Global controll register: pad as master/slave */
#define BL808_GLOBAL_PARAM_CFG0		0x510
#define BL808_MM_SPI_PAD_ROLE_MASK	BIT(27)
#define BL808_MM_SPI_PAD_AS_MASTER	BIT(27)

#define BL808_SPI_CFG0_REG	0x1b0
#define BL808_SPI_CFG0_CLK_DIV_MASK	GENMASK(4, 0)

/* TX/RX fifo size in bytes */
#define BFLB_SPI_MAX_TXF_SIZE	32

#define BFLB_XFER_DMA_TX 0x1
#define BFLB_XFER_DMA_RX 0x2

struct bflb_spi {
	/* Register base address */
	void __iomem *regs;
	/* SPI parent clock */
	struct clk *pclk;
	/* SPI bus clock */
	struct clk_hw clk_hw;
	/* Current clock divider */
	u32 bus_div;
	u32 spi_div;
	/* FIFO depth in bytes */
	unsigned int fifo_depth;
	u32 max_bits_per_word;
	/* Wake-up due to trigger from ISR, not used now */
	struct completion done;
	/* Global register map */
	struct regmap *syscon_regmap;
	struct device *dev;
	/* Current transfer info */
	u32 tx_len;
	u32 rx_len;
	const void *tx_buf;
	void *rx_buf;
	u8 xfer_stride;
	/* TX/RX FIFO register address for DMA. */
	phys_addr_t dma_txdr;
	phys_addr_t dma_rxdr;
	bool dma_avail;
	atomic_t state;
	/* Statistics */
	u64_stats_t txf_overflow;
	u64_stats_t txf_underflow;
	u64_stats_t rxf_overflow;
	u64_stats_t rxf_underflow;
	/* Debugfs entry */
	struct dentry *dbgfs_dir;
};

struct bflb_spi_dev_data {
	/* Config register */
	u32 cr_val;
	u32 cr_mask;
};

#define clk_hw_to_bflb_spi(hw)	container_of(hw, struct bflb_spi, clk_hw)

static struct dentry *bflb_spi_dbgfs_root;

/*
 * Push data into spi tx fifo
 * @param: spi - the spi controller handle
 *	   max - the maximum of words to transfer, -1 means no limit,
 *		 push as much data as possible.
 */
static u32 bflb_spi_txfifo_push(struct bflb_spi *spi, int max)
{
	u32 i;
	u32 word, n_words;

	if (!spi->tx_len || !max)
		return 0;

	/* Get available bytes of tx fifo */
	n_words = readl(spi->regs + SPI_FIFO_CFG1_REG_OFFSET);
	n_words = FIELD_GET(SPI_TXF_BYTES_MASK, n_words) / spi->xfer_stride;
	n_words = min(n_words, spi->tx_len);
	if (max > 0)
		n_words = min_t(u32, n_words, max);

	for (i = 0; i < n_words; i++) {
		switch (spi->xfer_stride) {
		case 1:
			word = *(u8 *)spi->tx_buf;
			break;

		case 2:
			word = *(u16 *)spi->tx_buf;
			break;

		case 3:
			word = *(u32 *)spi->tx_buf;
			break;

		case 4:
			word = *(u32 *)spi->tx_buf;
			break;

		default:
			dev_err(spi->dev, "invalid tx transfer stride %d\n",
					spi->xfer_stride);
			return 0;
		}
		spi->tx_buf += spi->xfer_stride;
		spi->tx_len--;
		writel(word, spi->regs + SPI_TXFIFO_REG_OFFSET);
	}

	return n_words;
}

/* Return number of words read out of RXFIFO */
static u32 bflb_spi_rxfifo_drain(struct bflb_spi *spi)
{
	u32 i, tmp, avail;

	avail = readl(spi->regs + SPI_FIFO_CFG1_REG_OFFSET);
	avail = FIELD_GET(SPI_RXF_BYTES_MASK, avail);
	avail /= spi->xfer_stride;

	for (i = 0; i < avail && spi->rx_len > 0; i++, spi->rx_len--) {
		if (!spi->rx_buf) {
			/* Just pop the data */
			tmp = readl(spi->regs + SPI_RXFIFO_REG_OFFSET);
			continue;
		}

		switch (spi->xfer_stride) {
		case 1:
			*(u8 *)spi->rx_buf = readl(spi->regs + SPI_RXFIFO_REG_OFFSET);
			break;

		case 2:
			*(u16 *)spi->rx_buf = readl(spi->regs + SPI_RXFIFO_REG_OFFSET);
			break;

		case 3:
			*(u32 *)spi->rx_buf = readl(spi->regs + SPI_RXFIFO_REG_OFFSET);
			break;

		case 4:
			*(u32 *)spi->rx_buf = readl(spi->regs + SPI_RXFIFO_REG_OFFSET);
			break;

		default:
			dev_err(spi->dev, "invalid rx transfer stride %d\n",
					spi->xfer_stride);
			return 0;
		}

		spi->rx_buf += spi->xfer_stride;
	}
	return avail;
}

static void bflb_spi_init(struct bflb_spi *spi)
{
	u32 tmp;

	/* Only master mode is supported now */
	regmap_update_bits(spi->syscon_regmap, BL808_GLOBAL_PARAM_CFG0,
		BL808_MM_SPI_PAD_ROLE_MASK, BL808_MM_SPI_PAD_AS_MASTER);
	/* Basic configurations */
	tmp = SPI_CR_CONT_XFER | FIELD_PREP(SPI_CR_FRAME_LEN_MASK, 0);
	writel(tmp, spi->regs + SPI_CONFIG_REG_OFFSET);

	/* Clear all pending interrupts */
	tmp = SPI_INT_CLR_ALL;
	/* Enable some interrupts */
	tmp |= SPI_INT_ENABLE_RXF_RDY | SPI_INT_ENABLE_FIFO_ERR;
	/* Mask all interrupts */
	tmp |= SPI_INT_MASK_ALL;
	writel(tmp, spi->regs + SPI_INT_REG_OFFSET);
	/* Clear pending fifo errors */
	tmp = SPI_FIFO_CLR_TX | SPI_FIFO_CLR_RX;
	writel(tmp, spi->regs + SPI_FIFO_CFG0_REG_OFFSET);
	/* No fifo threshold */
	tmp = 0;
	writel(tmp, spi->regs + SPI_FIFO_CFG1_REG_OFFSET);
}

static inline void bflb_spi_wait_idle(struct bflb_spi *bspi)
{
	while (readl(bspi->regs + SPI_BUS_BUSY_REG_OFFSET))
		cpu_relax();
}

static int bflb_spi_prepare_message(struct spi_controller *ctlr,
					struct spi_message *msg)
{
	u32 tmp;
	struct spi_device *spi_dev = msg->spi;
	struct bflb_spi *spi = spi_controller_get_devdata(ctlr);
	struct bflb_spi_dev_data *dev_data = spi_get_ctldata(spi_dev);

	tmp = readl(spi->regs + SPI_CONFIG_REG_OFFSET);
	tmp &= ~dev_data->cr_mask;
	tmp |= dev_data->cr_val;
	writel(tmp, spi->regs + SPI_CONFIG_REG_OFFSET);
	return 0;
}

static int bflb_spi_dev_setup(struct spi_device *spi)
{
	u8 tmp;
	struct bflb_spi_dev_data *dev_data;

	dev_data = spi_get_ctldata(spi);
	if (!dev_data) {
		dev_data = kzalloc(sizeof(*dev_data), GFP_KERNEL);
		if (!dev_data)
			return -ENOMEM;

		spi_set_ctldata(spi, dev_data);
	}

	dev_data->cr_mask |= SPI_CR_CLK_POL_HIGH;
	dev_data->cr_val |= (spi->mode & SPI_CPOL) ? SPI_CR_CLK_POL_HIGH : 0;

	dev_data->cr_mask |= SPI_CR_CLK_PHA_1;
	dev_data->cr_val |= (spi->mode & SPI_CPHA) ? SPI_CR_CLK_PHA_1 : 0;

	dev_data->cr_mask |= SPI_CR_LSBIT_FIRST;
	dev_data->cr_val |= (spi->mode & SPI_LSB_FIRST) ? SPI_CR_LSBIT_FIRST : 0;

	if (spi->bits_per_word & 7) {
		dev_err(&spi->dev, "bits_per_word %d is not a multiple of 8\n",
				spi->bits_per_word);
		return -EINVAL;
	}
	tmp = spi->bits_per_word / 8;
	tmp = tmp ? tmp - 1 : tmp;
	dev_data->cr_mask |= SPI_CR_FRAME_LEN_MASK;
	dev_data->cr_val |= FIELD_PREP(SPI_CR_FRAME_LEN_MASK, tmp);
	return 0;
}

static void bflb_spi_dev_cleanup(struct spi_device *spi)
{
	struct bflb_spi_dev_data *dev_data;

	dev_data = spi_get_ctldata(spi);
	spi_set_ctldata(spi, NULL);
	kfree(dev_data);
}

static int bflb_spi_prepare_controller(struct spi_controller *ctlr)
{
	u32 tmp;
	struct bflb_spi *spi = spi_controller_get_devdata(ctlr);

	tmp = readl(spi->regs + SPI_CONFIG_REG_OFFSET);
	tmp |= SPI_CR_MASTER_EN;
	tmp &= ~SPI_CR_SLAVE_EN;
	writel(tmp, spi->regs + SPI_CONFIG_REG_OFFSET);
	return 0;
}

static int bflb_spi_unprepare_controller(struct spi_controller *ctlr)
{
	u32 tmp;
	struct bflb_spi *spi = spi_controller_get_devdata(ctlr);

	tmp = readl(spi->regs + SPI_CONFIG_REG_OFFSET);
	tmp &= ~SPI_CR_MASTER_EN;
	tmp &= ~SPI_CR_SLAVE_EN;
	writel(tmp, spi->regs + SPI_CONFIG_REG_OFFSET);
	return 0;
}

static void bflb_spi_set_cs(struct spi_device *spi_dev, bool is_high)
{
}

static void bflb_spi_handle_err(struct spi_controller *ctlr,
				struct spi_message *msg)
{
	u32 tmp;
	struct bflb_spi *spi = spi_controller_get_devdata(ctlr);

	/* Disable controller */
	tmp = readl(spi->regs + SPI_CONFIG_REG_OFFSET);
	tmp &= ~SPI_CR_MASTER_EN;
	tmp &= ~SPI_CR_SLAVE_EN;
	writel(tmp, spi->regs + SPI_CONFIG_REG_OFFSET);

	/* Mask all interrupts */
	tmp = readl(spi->regs + SPI_INT_REG_OFFSET);
	tmp |= SPI_INT_MASK_ALL;
	writel(tmp, spi->regs + SPI_INT_REG_OFFSET);
}

static irqreturn_t bflb_spi_isr(int irq, void *dev_id)
{
	u32 tmp, pending;
	struct spi_controller *ctlr = dev_id;
	struct bflb_spi *spi = spi_controller_get_devdata(ctlr);

	tmp = readl(spi->regs + SPI_INT_REG_OFFSET);
	pending = FIELD_GET(SPI_INT_STS_ALL, tmp);
	/* This is not working */
	if (unlikely(pending & SPI_INT_STS_XFER_END))
		complete(&spi->done);

	if (pending & SPI_INT_STS_FIFO_ERR) {
		tmp = readl(spi->regs + SPI_FIFO_CFG0_REG_OFFSET);
		if (tmp & SPI_FIFO_STS_TXF_OVFL)
			u64_stats_inc(&spi->txf_overflow);
		if (tmp & SPI_FIFO_STS_TXF_UDFL)
			u64_stats_inc(&spi->txf_underflow);
		if (tmp & SPI_FIFO_STS_RXF_OVFL)
			u64_stats_inc(&spi->rxf_overflow);
		if (tmp & SPI_FIFO_STS_RXF_UDFL)
			u64_stats_inc(&spi->rxf_underflow);

		/* Clear pending fifo error status */
		tmp = readl(spi->regs + SPI_FIFO_CFG0_REG_OFFSET);
		tmp |= SPI_FIFO_CLR_TX | SPI_FIFO_CLR_RX;
		writel(tmp, spi->regs + SPI_FIFO_CFG0_REG_OFFSET);
		/* Clear pending transfer end interrupt */
		tmp = readl(spi->regs + SPI_INT_REG_OFFSET);
		tmp |= SPI_INT_CLR_XFER_END;
		writel(tmp, spi->regs + SPI_INT_REG_OFFSET);
	}

	if (pending & SPI_INT_STS_RXF_RDY) {
		u32 consumed;

		/*
		 * Pop some data first and pending rx fifo ready interrupt is
		 * auto-cleared after data is poped.
		 */
		consumed = bflb_spi_rxfifo_drain(spi);
		if (!spi->rx_len) {
			/* Mask rx fifo ready interrupt */
			tmp = readl(spi->regs + SPI_INT_REG_OFFSET);
			tmp |= SPI_INT_MASK_RXF_RDY;
			tmp |= SPI_INT_MASK_FIFO_ERR;
			writel(tmp, spi->regs + SPI_INT_REG_OFFSET);
			spi_finalize_current_transfer(ctlr);
		} else {
			/*
			 * Check if there is some tx work to do. Although the
			 * driver has tried to drain the rx fifo, the spi
			 * controller is still working concurrently, which means
			 * there could be new data in the rx fifo and tx fifo
			 * has more room for transfer right now. Rx fifo
			 * overflow could occur if the driver tries to push as
			 * much data as possible, and that's why the variable
			 * consumed is the clamp.
			 */
			bflb_spi_txfifo_push(spi, consumed);
		}
	}
	return IRQ_HANDLED;
}

static void bflb_spi_prepare_wait(struct bflb_spi *spi)
{
	reinit_completion(&spi->done);
}

static void bflb_spi_wait(struct bflb_spi *spi, u32 bit, int poll)
{
	if (poll) {
		while (!(readl(spi->regs + SPI_INT_REG_OFFSET) & bit))
			cpu_relax();
	} else {
		wait_for_completion(&spi->done);
	}
}

static void bflb_spi_flush_fifo(struct bflb_spi *spi)
{
	u32 i, tmp, avail;

	/* Clear pending fifo errors */
	tmp = readl(spi->regs + SPI_FIFO_CFG0_REG_OFFSET);
	tmp |= SPI_FIFO_CLR_TX | SPI_FIFO_CLR_RX;
	writel(tmp, spi->regs + SPI_FIFO_CFG0_REG_OFFSET);
	/* Pop data in the rx fifo */
	avail = readl(spi->regs + SPI_FIFO_CFG1_REG_OFFSET);
	avail = FIELD_GET(SPI_RXF_BYTES_MASK, avail);
	for (i = 0; i < avail; i++)
		tmp = readl(spi->regs + SPI_RXFIFO_REG_OFFSET);
}

static int bflb_spi_xfer_setup(struct bflb_spi *spi, struct spi_device *dev,
				struct spi_transfer *t)
{
	u32 tmp;

	if (t->bits_per_word & 7) {
		dev_err(spi->dev, "bits_per_word %d is not multiple of 8\n",
			t->bits_per_word);
		return -EINVAL;
	}

	spi->xfer_stride = t->bits_per_word / 8;
	if (t->len % spi->xfer_stride) {
		dev_err(spi->dev, "transfer bytes %d is not multiple of word size %d\n",
			t->len, spi->xfer_stride);
		return -EINVAL;
	}

	/* Set word size */
	tmp = readl(spi->regs + SPI_CONFIG_REG_OFFSET);
	tmp &= ~SPI_CR_FRAME_LEN_MASK;
	tmp |= FIELD_PREP(SPI_CR_FRAME_LEN_MASK, spi->xfer_stride - 1);
	writel(tmp, spi->regs + SPI_CONFIG_REG_OFFSET);
	/* Set transfer timing parameters */
	clk_set_rate(spi->clk_hw.clk, t->speed_hz);
	/* Set up transfer state */
	spi->tx_len = t->len / spi->xfer_stride;
	spi->rx_len = spi->tx_len;
	spi->tx_buf = t->tx_buf;
	spi->rx_buf = t->rx_buf;
	bflb_spi_flush_fifo(spi);
	if (readl(spi->regs + SPI_BUS_BUSY_REG_OFFSET))
		dev_warn(spi->dev, "note: spi bus is busy?!\n");
	return 0;
}

/*
 * Note: The polling transfer has not undergone sufficient testing, thus
 * it might not be that stable.
 */
static int __maybe_unused bflb_spi_transfer_one_poll(struct spi_controller *ctlr,
		struct spi_device *device, struct spi_transfer *t)
{
	u32 consumed;
	int err, poll;
	struct bflb_spi *spi = spi_controller_get_devdata(ctlr);

	err = bflb_spi_xfer_setup(spi, device, t);
	if (err < 0)
		return err;

	poll = 1;
	bflb_spi_prepare_wait(spi);
	bflb_spi_txfifo_push(spi, -1);
	while (spi->rx_len) {
		/* Wait for reception to complete */
		bflb_spi_wait(spi, SPI_INT_STS_RXF_RDY, poll);
		/* Read data out of the rx fifo */
		consumed = bflb_spi_rxfifo_drain(spi);
		bflb_spi_prepare_wait(spi);
		/* Enqueue words for transmission */
		bflb_spi_txfifo_push(spi, consumed);
	}

	return 0;
}

static int bflb_spi_transfer_one_irq(struct bflb_spi *bspi,
		struct spi_device *device, struct spi_transfer *t)
{
	int err;
	u32 tmp;

	err = bflb_spi_xfer_setup(bspi, device, t);
	if (err < 0)
		return err;

	tmp = bflb_spi_txfifo_push(bspi, -1);
	/*
	 * Unmask rx fifo ready interrupt for transaction, and fifo error
	 * interrupt for debugging.
	 */
	tmp = readl(bspi->regs + SPI_INT_REG_OFFSET);
	tmp &= ~SPI_INT_MASK_RXF_RDY;
	tmp &= ~SPI_INT_MASK_FIFO_ERR;
	writel(tmp, bspi->regs + SPI_INT_REG_OFFSET);
	/* 1 means that the transfer is in process. */
	return 1;
}

static void bflb_spi_dma_txcb(void *data)
{
	int state;
	struct spi_controller *ctlr = data;
	struct bflb_spi *bspi = spi_controller_get_devdata(ctlr);

	state = atomic_fetch_andnot(BFLB_XFER_DMA_TX, &bspi->state);
	/* The transaction is still in progress. */
	if (state & BFLB_XFER_DMA_RX)
		return;

	/*
	 * SPI TX DMA completion interrupt is triggered after the data is
	 * moved from memory to SPI TX FIFO, but that does not mean the
	 * SPI transaction is done.
	 */
	bflb_spi_wait_idle(bspi);
	spi_finalize_current_transfer(ctlr);
}

static void bflb_spi_dma_rxcb(void *data)
{
	int state;
	struct spi_controller *ctlr = data;
	struct bflb_spi *bspi = spi_controller_get_devdata(ctlr);

	state = atomic_fetch_andnot(BFLB_XFER_DMA_RX, &bspi->state);
	/* Usually SPI TX DMA completion interrupt is a step earlier. */
	if (unlikely(state & BFLB_XFER_DMA_TX))
		return;

	spi_finalize_current_transfer(ctlr);
}

static int bflb_spi_transfer_one_dma(struct bflb_spi *bspi,
	struct spi_controller *ctlr, struct spi_device *device,
	struct spi_transfer *t)
{
	int err;
	u32 tmp;
	u32 addr_width;
	struct dma_slave_config cfg;
	struct dma_async_tx_descriptor *txd = NULL, *rxd = NULL;

	err = bflb_spi_xfer_setup(bspi, device, t);
	if (err < 0)
		return err;

	addr_width = device->bits_per_word >> 3;
	/* bits_per_word of 24 needs a word. */
	if (addr_width == 3)
		addr_width = 4;

	atomic_set(&bspi->state, 0);
	if (t->tx_buf) {
		memset(&cfg, 0, sizeof(cfg));
		cfg.direction = DMA_MEM_TO_DEV;
		cfg.src_addr_width = addr_width;
		cfg.src_maxburst = 1;
		cfg.dst_addr = bspi->dma_txdr;
		cfg.dst_addr_width = addr_width;
		cfg.dst_maxburst = 1;

		err = dmaengine_slave_config(ctlr->dma_tx, &cfg);
		if (err) {
			dev_err(bspi->dev,
				"Failed to config dma tx, %d\n", err);
			return err;
		}

		txd = dmaengine_prep_slave_sg(ctlr->dma_tx, t->tx_sg.sgl,
			t->tx_sg.nents, DMA_MEM_TO_DEV, DMA_PREP_INTERRUPT);
		if (!txd) {
			dev_err(bspi->dev,
				"Failed to prepare dma tx, %d\n", err);
			return -EINVAL;
		}

		txd->callback = bflb_spi_dma_txcb;
		txd->callback_param = ctlr;
	}

	if (t->rx_buf) {
		memset(&cfg, 0, sizeof(cfg));
		cfg.direction = DMA_DEV_TO_MEM;
		cfg.src_addr = bspi->dma_rxdr;
		cfg.src_addr_width = addr_width;
		cfg.src_maxburst = 1;
		cfg.dst_addr_width = addr_width;
		cfg.dst_maxburst = 1;

		err = dmaengine_slave_config(ctlr->dma_rx, &cfg);
		if (err) {
			dmaengine_terminate_sync(ctlr->dma_tx);
			dev_err(bspi->dev,
				"Failed to config dma rx, %d\n", err);
			return -EINVAL;
		}

		rxd = dmaengine_prep_slave_sg(ctlr->dma_rx, t->rx_sg.sgl,
			t->rx_sg.nents, DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
		if (!rxd) {
			dmaengine_terminate_sync(ctlr->dma_tx);
			dev_err(bspi->dev,
				"Failed to prepare dma rx, %d\n", err);
			return -EINVAL;
		}

		rxd->callback = bflb_spi_dma_rxcb;
		rxd->callback_param = ctlr;
	}

	if (rxd) {
		ctlr->dma_rx->cookie = dmaengine_submit(rxd);
		atomic_or(BFLB_XFER_DMA_RX, &bspi->state);
		dma_async_issue_pending(ctlr->dma_rx);
	}

	if (txd) {
		ctlr->dma_tx->cookie = dmaengine_submit(txd);
		atomic_or(BFLB_XFER_DMA_TX, &bspi->state);
		dma_async_issue_pending(ctlr->dma_tx);
	}

	/*
	 * Mask RX FIFO ready interrupt because we only need DMA interrupt
	 * now.
	 * Unmask FIFO error interrupt for debugging.
	 */
	tmp = readl(bspi->regs + SPI_INT_REG_OFFSET);
	tmp |= SPI_INT_MASK_RXF_RDY;
	tmp &= ~SPI_INT_MASK_FIFO_ERR;
	writel(tmp, bspi->regs + SPI_INT_REG_OFFSET);

	/* Enable DMA requests. */
	tmp = readl(bspi->regs + SPI_FIFO_CFG0_REG_OFFSET);
	tmp |= SPI_FIFO_DMA_TX_EN;
	tmp |= SPI_FIFO_DMA_RX_EN;
	writel(tmp, bspi->regs + SPI_FIFO_CFG0_REG_OFFSET);
	/* 1 means that the transfer is in process. */
	return 1;
}

static bool bflb_spi_can_dma(struct spi_controller *master,
		struct spi_device *spi, struct spi_transfer *xfer)
{
	struct bflb_spi *bspi = spi_controller_get_devdata(master);

	/*
	 * It is a waste of resource to setup DMA and wait for the interrupt
	 * if the transfer is too short.
	 */
	return bspi->dma_avail ? xfer->len > bspi->fifo_depth : false;
}

static int bflb_spi_transfer_one(struct spi_controller *ctlr,
		struct spi_device *device, struct spi_transfer *t)
{
	bool use_dma = false;
	struct bflb_spi *bspi = spi_controller_get_devdata(ctlr);

	if (!t->len) {
		dev_warn(bspi->dev, "No data for transfer\n");
		return 0;
	}

	if (!t->tx_buf && !t->rx_buf) {
		dev_err(bspi->dev, "No buffers for transfer\n");
		return -EINVAL;
	}

	if (bspi->dma_avail)
		use_dma = ctlr->can_dma(ctlr, device, t);

	if (use_dma)
		return bflb_spi_transfer_one_dma(bspi, ctlr, device, t);

	return bflb_spi_transfer_one_irq(bspi, device, t);
}

static size_t __maybe_unused bflb_spi_get_max_xfer_size(struct spi_device *spi_dev)
{
	struct bflb_spi *spi = spi_controller_get_devdata(spi_dev->controller);

	return spi->fifo_depth;
}

static int dbgfs_stats_show(struct seq_file *m, void *unused)
{
	struct bflb_spi *spi = m->private;

	seq_printf(m, "tx fifo overflow\t\t%llu\n",
			u64_stats_read(&spi->txf_overflow));
	seq_printf(m, "tx fifo underflow\t\t%llu\n",
			u64_stats_read(&spi->txf_underflow));
	seq_printf(m, "rx fifo overflow\t\t%llu\n",
			u64_stats_read(&spi->rxf_overflow));
	seq_printf(m, "rx fifo underflow\t\t%llu\n",
			u64_stats_read(&spi->rxf_underflow));
	return 0;
}

DEFINE_SHOW_ATTRIBUTE(dbgfs_stats);

static void bflb_spi_ctlr_dbgfs_deinit(void *data)
{
	struct bflb_spi *spi = data;

	debugfs_remove_recursive(spi->dbgfs_dir);
}

static int bflb_spi_debugfs_init(struct bflb_spi *spi)
{
	spi->dbgfs_dir = debugfs_create_dir(dev_name(spi->dev), bflb_spi_dbgfs_root);
	debugfs_create_file("stats", 0444, spi->dbgfs_dir, spi, &dbgfs_stats_fops);
	return devm_add_action_or_reset(spi->dev, bflb_spi_ctlr_dbgfs_deinit, spi);
}

static unsigned long bflb_spi_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	u32 tmp;
	unsigned int bus_div, spi_div0, spi_div1;
	struct bflb_spi *spi = clk_hw_to_bflb_spi(hw);

	regmap_read(spi->syscon_regmap, BL808_SPI_CFG0_REG, &bus_div);
	bus_div = FIELD_GET(BL808_SPI_CFG0_CLK_DIV_MASK, bus_div);
	tmp = readl(spi->regs + SPI_PRD0_REG_OFFSET);
	spi_div0 = FIELD_GET(SPI_PRD0_PHASE0_MASK, tmp);
	spi_div1 = FIELD_GET(SPI_PRD0_PHASE1_MASK, tmp);
	return parent_rate / ((bus_div + 1) * (spi_div0 + 1 + spi_div1 + 1));
}

static void bflb_spi_clk_calc_div(unsigned long parent_rate, unsigned long rate,
				u32 *bus_div, u32 *spi_div)
{
	/* 2 * 256 is the maximum of controller's divider factor */
	if (rate >= parent_rate / (2 * 256)) {
		*bus_div = 1;
	} else {
		/* The maximum of bus divider factor */
		*bus_div = 32;
	}

	*spi_div = DIV_ROUND_UP_ULL(parent_rate, *bus_div * 2 * rate);
}

static int bflb_spi_clk_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	int need_resume = 0;
	u32 cr, tmp, bus_div, spi_div;
	struct bflb_spi *spi = clk_hw_to_bflb_spi(hw);

	/* Disable controller in the first place */
	cr = readl(spi->regs + SPI_CONFIG_REG_OFFSET);
	if (cr & SPI_CR_MASTER_EN) {
		need_resume = 1;
		tmp = cr & ~SPI_CR_MASTER_EN;
		writel(tmp, spi->regs + SPI_CONFIG_REG_OFFSET);
	}

	bus_div = spi->bus_div > 0 ? spi->bus_div - 1 : 0;
	spi_div = spi->spi_div > 0 ? spi->spi_div - 1 : 0;
	regmap_update_bits(spi->syscon_regmap, BL808_SPI_CFG0_REG,
			BL808_SPI_CFG0_CLK_DIV_MASK,
			FIELD_PREP(BL808_SPI_CFG0_CLK_DIV_MASK, bus_div));

	tmp = FIELD_PREP(SPI_PRD0_START_MASK, spi_div) |
		FIELD_PREP(SPI_PRD0_STOP_MASK, spi_div) |
		FIELD_PREP(SPI_PRD0_PHASE0_MASK, spi_div) |
		FIELD_PREP(SPI_PRD0_PHASE1_MASK, spi_div);
	writel(tmp, spi->regs + SPI_PRD0_REG_OFFSET);

	tmp = spi_div;
	writel(tmp, spi->regs + SPI_PRD1_REG_OFFSET);

	if (need_resume)
		writel(cr, spi->regs + SPI_CONFIG_REG_OFFSET);
	return 0;
}

static long bflb_spi_clk_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	struct bflb_spi *spi = clk_hw_to_bflb_spi(hw);

	/* Remember the clock dividers to speed up clock setting */
	bflb_spi_clk_calc_div(*parent_rate, rate, &spi->bus_div, &spi->spi_div);
	return *parent_rate / (spi->bus_div * 2 * spi->spi_div);
}

static const struct clk_ops bflb_spi_clk_ops = {
	.recalc_rate = bflb_spi_clk_recalc_rate,
	.set_rate = bflb_spi_clk_set_rate,
	.round_rate = bflb_spi_clk_round_rate,
};

static int bflb_spi_register_clk_div(struct bflb_spi *spi)
{
	char name[32];
	const char *pclk_name;
	struct clk_init_data init;

	pclk_name = __clk_get_name(spi->pclk);
	snprintf(name, sizeof(name), "%s-div", dev_name(spi->dev));
	init.name = name;
	init.ops = &bflb_spi_clk_ops;
	init.num_parents = 1;
	init.parent_names = (const char *[]){ pclk_name };
	init.flags = 0;
	spi->clk_hw.init = &init;
	devm_clk_hw_register_clkdev(spi->dev, &spi->clk_hw, "spi-div",
					dev_name(spi->dev));
	return devm_clk_hw_register(spi->dev, &spi->clk_hw);
}

static int bflb_spi_probe(struct platform_device *pdev)
{
	int ret, irq;
	struct bflb_spi *spi;
	struct resource *mem;
	struct device_node *np;
	bool dma_tx_ok = false;
	bool dma_rx_ok = false;
	struct spi_controller *ctlr;

	ctlr = devm_spi_alloc_master(&pdev->dev, sizeof(struct bflb_spi));
	if (!ctlr) {
		return dev_err_probe(&pdev->dev, -ENOMEM,
					"no memory for spi controller\n");
	}

	spi = spi_controller_get_devdata(ctlr);
	spi->dev = &pdev->dev;
	init_completion(&spi->done);
	platform_set_drvdata(pdev, ctlr);

	np = dev_of_node(&pdev->dev);
	spi->syscon_regmap = syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(spi->syscon_regmap)) {
		return dev_err_probe(spi->dev, PTR_ERR(spi->syscon_regmap),
					"syscon node to regmap failed\n");
	}

	spi->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &mem);
	if (IS_ERR(spi->regs)) {
		ret = PTR_ERR(spi->regs);
		return dev_err_probe(spi->dev, ret, "failed to ioremap\n");
	}

	/* Spin up the controller clock before manipulating registers */
	spi->pclk = devm_clk_get_prepared(&pdev->dev, NULL);
	if (IS_ERR(spi->pclk)) {
		ret = PTR_ERR(spi->pclk);
		return dev_err_probe(spi->dev, ret,
					"unable to get and prepare clock\n");
	}
	ret = clk_enable(spi->pclk);
	if (ret < 0)
		return dev_err_probe(spi->dev, ret, "failed to enable clock\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		clk_disable(spi->pclk);
		return dev_err_probe(spi->dev, -EINVAL, "failed to get irq\n");
	}

	ret = devm_request_irq(&pdev->dev, irq, bflb_spi_isr, 0,
				dev_name(&pdev->dev), ctlr);
	if (ret) {
		clk_disable(spi->pclk);
		return dev_err_probe(spi->dev, ret, "unable to request irq\n");
	}

	ctlr->dma_tx = dma_request_chan(spi->dev, "tx");
	if (IS_ERR(ctlr->dma_tx)) {
		if (PTR_ERR(ctlr->dma_tx) == -EPROBE_DEFER) {
			clk_disable(spi->pclk);
			return dev_err_probe(spi->dev, -EPROBE_DEFER,
					"Failed to get dma tx channel\n");
		}

		dev_warn(spi->dev, "Unable to get dma tx channel, %ld\n",
				PTR_ERR(ctlr->dma_tx));
	} else {
		spi->dma_txdr = mem->start + SPI_TXFIFO_REG_OFFSET;
		dma_tx_ok = true;
	}

	ctlr->dma_rx = dma_request_chan(spi->dev, "rx");
	if (IS_ERR(ctlr->dma_rx)) {
		if (PTR_ERR(ctlr->dma_rx) == -EPROBE_DEFER) {
			clk_disable(spi->pclk);
			return dev_err_probe(spi->dev, PTR_ERR(ctlr->dma_rx),
					"Failed to get dma rx channel");
		} else {
			dev_warn(spi->dev, "Unable to get dma rx channel, %ld\n",
				PTR_ERR(ctlr->dma_tx));
		}
	} else {
		spi->dma_rxdr = mem->start + SPI_RXFIFO_REG_OFFSET;
		dma_rx_ok = true;
	}

	if (dma_tx_ok && dma_rx_ok) {
		spi->dma_avail = true;
		ctlr->can_dma = bflb_spi_can_dma;
		dev_info(spi->dev, "DMA is available");
	} else {
		spi->dma_avail = false;
	}

	spi->fifo_depth = BFLB_SPI_MAX_TXF_SIZE;
	spi->max_bits_per_word = 32;
	of_property_read_u32(np, "bflb,fifo-depth", &spi->fifo_depth);
	of_property_read_u32(np, "bflb,max-bits-per-word",
				&spi->max_bits_per_word);
	dev_info(spi->dev, "fifo depth %dbytes, max bits per word %d\n",
			spi->fifo_depth, spi->max_bits_per_word);

	ctlr->bus_num = pdev->id;
	ctlr->mode_bits = SPI_CPHA | SPI_CPOL | SPI_CS_HIGH | SPI_LSB_FIRST;
	ctlr->max_speed_hz = 80000000;
	ctlr->min_speed_hz = 9766;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(16) |
				SPI_BPW_MASK(24) | SPI_BPW_MASK(32);
	ctlr->flags = SPI_CONTROLLER_MUST_TX | SPI_MASTER_GPIO_SS;
	ctlr->setup = bflb_spi_dev_setup;
	ctlr->cleanup = bflb_spi_dev_cleanup;
	ctlr->prepare_transfer_hardware = bflb_spi_prepare_controller;
	ctlr->unprepare_transfer_hardware = bflb_spi_unprepare_controller;
	ctlr->prepare_message = bflb_spi_prepare_message;
	ctlr->set_cs = bflb_spi_set_cs;
	ctlr->handle_err = bflb_spi_handle_err;
	ctlr->transfer_one = bflb_spi_transfer_one;
	ctlr->use_gpio_descriptors = true;
	pdev->dev.dma_mask = NULL;
	ctlr->dev.of_node = np;

	ret = bflb_spi_register_clk_div(spi);
	if (ret) {
		clk_disable(spi->pclk);
		return dev_err_probe(spi->dev, ret,
				"failed to register clock divider %d\n", ret);
	}
	bflb_spi_init(spi);

	ret = devm_spi_register_master(&pdev->dev, ctlr);
	if (ret < 0) {
		clk_disable(spi->pclk);
		return dev_err_probe(spi->dev, ret,
				"failed to register spi controller\n");
	}

	bflb_spi_debugfs_init(spi);
	return 0;
}

static void bflb_spi_remove(struct platform_device *pdev)
{
	u32 tmp;
	struct spi_controller *ctlr = platform_get_drvdata(pdev);
	struct bflb_spi *spi = spi_controller_get_devdata(ctlr);

	/* Disable all the interrupts */
	tmp = SPI_INT_CLR_ALL | SPI_INT_MASK_ALL;
	writel(tmp, spi->regs + SPI_INT_REG_OFFSET);
	/* Stop the clock. */
	clk_disable(spi->pclk);
	/* Disable the hardware. */
	bflb_spi_unprepare_controller(ctlr);
}

static const struct of_device_id bflb_spi_of_match[] = {
	{ .compatible = "bflb,bl808-spi", },
	{}
};

MODULE_DEVICE_TABLE(of, bflb_spi_of_match);

static struct platform_driver bflb_spi_driver = {
	.probe = bflb_spi_probe,
	.remove_new = bflb_spi_remove,
	.driver = {
		.name = BFLB_SPI_DRIVER_NAME,
		.of_match_table = bflb_spi_of_match,
	},
};

static int __init bflb_spi_driver_init(void)
{
	int err;

	bflb_spi_dbgfs_root = debugfs_create_dir(BFLB_SPI_DRIVER_NAME, NULL);
	err = platform_driver_register(&bflb_spi_driver);
	if (err)
		debugfs_remove_recursive(bflb_spi_dbgfs_root);
	return err;
}

static void __exit bflb_spi_driver_exit(void)
{
	platform_driver_unregister(&bflb_spi_driver);
	debugfs_remove_recursive(bflb_spi_dbgfs_root);
}

module_init(bflb_spi_driver_init);
module_exit(bflb_spi_driver_exit);

MODULE_AUTHOR("qhli@bouffalolab.com");
MODULE_DESCRIPTION("Bouffalo Lab SPI driver");
MODULE_LICENSE("GPL");
