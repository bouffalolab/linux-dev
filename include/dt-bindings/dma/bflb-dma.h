/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This header provides macros for BFLB DMA bindings.
 *
 * Copyright (c) 2024 Bouffalo Lab
 */

#ifndef __DT_BINDINGS_DMA_BFLB_DMA_H__
#define __DT_BINDINGS_DMA_BFLB_DMA_H__

/* DMA2 requests. */
#define DMA_REQ_NONE        0
#define DMA_REQ_MAX         22

#define DMA_REQ_UART3_RX    0
#define DMA_REQ_UART3_TX    1
#define DMA_REQ_SPI1_RX     2
#define DMA_REQ_SPI1_TX     3
#define DMA_REQ_I2C2_RX     6
#define DMA_REQ_I2C2_TX     7
#define DMA_REQ_I2C3_RX     8
#define DMA_REQ_I2C3_TX     9
#define DMA_REQ_DSI_RX      10
#define DMA_REQ_DSI_TX      11
#define DMA_REQ_DBI_TX      22

#define DMA_SRC_REQ_MASK    0x001F
#define DMA_DST_REQ_MASK    0x03E0

#define DMA_REQ_INFO(src, dst)  (((src) & 0x1F) | \
                                (((dst) & 0x1F) << 5))
#endif /* __DT_BINDINGS_DMA_BFLB_DMA_H__ */
