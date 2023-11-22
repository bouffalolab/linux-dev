/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __BFLB_SPI_H__
#define __BFLB_SPI_H__

#include <linux/bits.h>

/* 0x00: spi config register */
#define SPI_CONFIG_REG_OFFSET	(0x0)

#define SPI_CR_MASTER_EN		BIT(0)
#define SPI_CR_SLAVE_EN			BIT(1)
#define SPI_CR_FRAME_LEN_MASK		GENMASK(3, 2)
#define SPI_CR_CLK_POL_HIGH		BIT(4)
#define SPI_CR_CLK_PHA_1		BIT(5)
#define SPI_CR_LSBIT_FIRST		BIT(6)
#define SPI_CR_LSBYTE_FIRST		BIT(7)
#define SPI_CR_IGN_RX_DATA		BIT(8)
#define SPI_CR_CONT_XFER		BIT(9)
#define SPI_CR_3WIRE			BIT(10)
#define SPI_CR_DEGLITCH_EN		BIT(11)
#define SPI_CR_DEGLITCH_CYCLE_MASK	GENMASK(15, 12)

/* 0x04: spi interrupt register */
#define SPI_INT_REG_OFFSET	(0x4)

#define SPI_INT_STS_XFER_END		BIT(0)
#define SPI_INT_STS_TXF_RDY		BIT(1)
#define SPI_INT_STS_RXF_RDY		BIT(2)
#define SPI_INT_STS_TIMEOUT		BIT(3)
#define SPI_INT_STS_TXF_UNDERFLOW	BIT(4)
#define SPI_INT_STS_FIFO_ERR		BIT(5)
#define SPI_INT_STS_ALL			GENMASK(5, 0)

#define SPI_INT_MASK_XFER_END		BIT(8)
#define SPI_INT_MASK_TXF_RDY		BIT(9)
#define SPI_INT_MASK_RXF_RDY		BIT(10)
#define SPI_INT_MASK_TX_TIMEOUT		BIT(11)
#define SPI_INT_MASK_TXF_UNDERFLOW	BIT(12)
#define SPI_INT_MASK_FIFO_ERR		BIT(13)
#define SPI_INT_MASK_ALL		GENMASK(13, 8)

#define SPI_INT_CLR_XFER_END		BIT(16)
#define SPI_INT_CLR_TX_TIMEOUT		BIT(19)
#define SPI_INT_CLR_TXF_UNDERFLOW	BIT(20)
#define SPI_INT_CLR_ALL			GENMASK(21, 16)

#define SPI_INT_ENABLE_XFER_END		BIT(24)
#define SPI_INT_ENABLE_TXF_RDY		BIT(25)
#define SPI_INT_ENABLE_RXF_RDY		BIT(26)
#define SPI_INT_ENABLE_TX_TIMEOUT	BIT(27)
#define SPI_INT_ENABLE_TXF_UNDERFLOW	BIT(28)
#define SPI_INT_ENABLE_FIFO_ERR		BIT(29)
#define SPI_INT_ENABLE_ALL		GENMASK(29, 24)

/* 0x08: spi bus busy indicator register */
#define SPI_BUS_BUSY_REG_OFFSET	(0x8)

#define SPI_BUSY_BUSY	BIT(0)

/* 0x10: spi prd 0 register */
#define SPI_PRD0_REG_OFFSET	(0x10)

#define SPI_PRD0_START_MASK	GENMASK(7, 0)
#define SPI_PRD0_STOP_MASK	GENMASK(15, 8)
#define SPI_PRD0_PHASE0_MASK	GENMASK(23, 16)
#define SPI_PRD0_PHASE1_MASK	GENMASK(31, 24)

/* 0x14: spi prd 1 register */
#define SPI_PRD1_REG_OFFSET	(0x14)

#define SPI_PRD1_ITVL_MASK	GENMASK(7, 0)

/* 0x18: spi rx data ignore register */
#define SPI_RXD_IGN_REG_OFFSET	(0x18)

#define SPI_RXD_IGN_START_MASK	GENMASK(31, 5)
#define SPI_RXD_IGN_STOP_MASK	GENMASK(4, 0)

/* 0x1C: spi transfer timeout register */
#define SPI_TIMEOUT_REG_OFFSET	(0x1C)

/* 0x80: spi fifo config 0 register */
#define SPI_FIFO_CFG0_REG_OFFSET	(0x80)

#define SPI_FIFO_DMA_TX_EN	BIT(0)
#define SPI_FIFO_DMA_RX_EN	BIT(1)
#define SPI_FIFO_CLR_TX		BIT(2)
#define SPI_FIFO_CLR_RX		BIT(3)
#define SPI_FIFO_STS_TXF_OVFL	BIT(4)
#define SPI_FIFO_STS_TXF_UDFL	BIT(5)
#define SPI_FIFO_STS_RXF_OVFL	BIT(6)
#define SPI_FIFO_STS_RXF_UDFL	BIT(7)

/* 0x84: spi fifo config 1 register */
#define SPI_FIFO_CFG1_REG_OFFSET	(0x84)

/* threshold for which tx/rx fifo becomes available */
#define SPI_RXF_THRESH_MASK	GENMASK(28, 24)
#define SPI_TXF_THRESH_MASK	GENMASK(20, 16)
/* available bytes in tx/rx fifo */
#define SPI_TXF_BYTES_MASK	GENMASK(5, 0)
#define SPI_RXF_BYTES_MASK	GENMASK(13, 8)

/* 0x88: spi tx fifo register */
#define SPI_TXFIFO_REG_OFFSET	(0x88)

/* 0x8C: spi rx fifo register */
#define SPI_RXFIFO_REG_OFFSET	(0x8C)

/* __BFLB_SPI_H__ */
#endif
