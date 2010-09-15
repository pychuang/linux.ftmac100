/*
 * Faraday FTMAC100 Ethernet
 *
 * (C) Copyright 2009 Faraday Technology
 * Po-Yu Chuang <ratbert@faraday-tech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/version.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/mii.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/io.h>

#include "ftmac100.h"

#define USE_NAPI

#define DRV_NAME	"ftmac100"
#define DRV_VERSION	"0.1"

#define RX_QUEUE_ENTRIES	128	/* must be power of 2 */
#define TX_QUEUE_ENTRIES	16	/* must be power of 2 */

#define MAX_PKT_SIZE		1518
#define RX_BUF_SIZE		2044	/* must be smaller than 0x7ff */

/******************************************************************************
 * priveate data
 *****************************************************************************/
struct ftmac100_descs {
	struct ftmac100_rxdes 	rxdes[RX_QUEUE_ENTRIES];
	struct ftmac100_txdes	txdes[TX_QUEUE_ENTRIES];
};

struct ftmac100_priv
{
	struct resource		*res;
	void			*base_addr;
	int			irq;

	struct ftmac100_descs	*descs;
	dma_addr_t		descs_dma_addr;

	unsigned int		rx_pointer;
	unsigned int		tx_clean_pointer;
	unsigned int		tx_pointer;
	unsigned int		tx_pending;

	spinlock_t		hw_lock;
	spinlock_t		rx_lock;
	spinlock_t		tx_lock;

	struct net_device	*dev;
#ifdef USE_NAPI
	struct napi_struct	napi;
#endif

	struct net_device_stats	stats;

	struct mii_if_info	mii;
};

/******************************************************************************
 * internal functions (hardware register access)
 *****************************************************************************/
#define INT_MASK_RX_DISABLED	(FTMAC100_INT_XPKT_OK		|	\
				 FTMAC100_INT_XPKT_LOST		|	\
				 FTMAC100_INT_RPKT_LOST		|	\
				 FTMAC100_INT_AHB_ERR		|	\
				 FTMAC100_INT_PHYSTS_CHG)

#define INT_MASK_ALL_ENABLED	(INT_MASK_RX_DISABLED		|	\
				 FTMAC100_INT_RPKT_FINISH	|	\
				 FTMAC100_INT_NORXBUF)

#define	INT_MASK_ALL_DISABLED	0

#ifdef USE_NAPI
static inline void ftmac100_disable_rxint(struct ftmac100_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->hw_lock, flags);
	iowrite32(INT_MASK_RX_DISABLED, priv->base_addr + FTMAC100_OFFSET_IMR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);
}
#endif

static inline void ftmac100_enable_all_int(struct ftmac100_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->hw_lock, flags);
	iowrite32(INT_MASK_ALL_ENABLED, priv->base_addr + FTMAC100_OFFSET_IMR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);
}

static inline void ftmac100_disable_all_int(struct ftmac100_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->hw_lock, flags);
	iowrite32(INT_MASK_ALL_DISABLED, priv->base_addr + FTMAC100_OFFSET_IMR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);
}

static inline void ftmac100_set_receive_ring_base(struct ftmac100_priv *priv, 
		dma_addr_t addr)
{
	iowrite32(addr, priv->base_addr + FTMAC100_OFFSET_RXR_BADR);
}

static inline void ftmac100_set_transmit_ring_base(struct ftmac100_priv *priv, 
		dma_addr_t addr)
{
	iowrite32(addr, priv->base_addr + FTMAC100_OFFSET_TXR_BADR);
}

static inline void ftmac100_txdma_start_polling(struct ftmac100_priv *priv)
{
	iowrite32(1, priv->base_addr + FTMAC100_OFFSET_TXPD);
}

static int ftmac100_reset(struct ftmac100_priv *priv)
{
	unsigned long flags;
	int i;

	/* NOTE: reset clears all registers */

	spin_lock_irqsave(&priv->hw_lock, flags);
	iowrite32(FTMAC100_MACCR_SW_RST, priv->base_addr + FTMAC100_OFFSET_MACCR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);

	for (i = 0; i < 5; i++) {
		int maccr;

		spin_lock_irqsave(&priv->hw_lock, flags);
		maccr = ioread32(priv->base_addr + FTMAC100_OFFSET_MACCR);
		spin_unlock_irqrestore(&priv->hw_lock, flags);
		if (!(maccr & FTMAC100_MACCR_SW_RST)) {
			/*
			 * FTMAC100_MACCR_SW_RST cleared does not indicate
			 * that hardware reset completed (what the f*ck).
			 * We still need to wait for a while.
			 */
			msleep_interruptible(1);
			return 0;
		}

		msleep_interruptible(10);
	}

	dev_err(&priv->dev->dev, "software reset failed\n");
	return -EIO;
}

static void ftmac100_set_mac(struct ftmac100_priv *priv, const unsigned char *mac)
{
	unsigned int maddr = mac[0] << 8 | mac[1];
	unsigned int laddr = mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5];

	iowrite32(maddr, priv->base_addr + FTMAC100_OFFSET_MAC_MADR);
	iowrite32(laddr, priv->base_addr + FTMAC100_OFFSET_MAC_LADR);
}

static int ftmac100_start_hw(struct ftmac100_priv *priv)
{
	unsigned long flags;
	int maccr;

	if (ftmac100_reset(priv))
		return -EIO;

	/* setup ring buffer base registers */

	spin_lock_irqsave(&priv->hw_lock, flags);
	ftmac100_set_receive_ring_base(priv,
		priv->descs_dma_addr + offsetof(struct ftmac100_descs, rxdes));
	ftmac100_set_transmit_ring_base(priv,
		priv->descs_dma_addr + offsetof(struct ftmac100_descs, txdes));

	iowrite32(FTMAC100_APTC_RXPOLL_CNT(1), priv->base_addr + FTMAC100_OFFSET_APTC);

	ftmac100_set_mac(priv, priv->dev->dev_addr);

	maccr = FTMAC100_MACCR_XMT_EN |
		FTMAC100_MACCR_RCV_EN |
		FTMAC100_MACCR_XDMA_EN |
		FTMAC100_MACCR_RDMA_EN |
		FTMAC100_MACCR_CRC_APD |
		FTMAC100_MACCR_FULLDUP |
		FTMAC100_MACCR_RX_RUNT |
		FTMAC100_MACCR_RX_BROADPKT;

	iowrite32(maccr, priv->base_addr + FTMAC100_OFFSET_MACCR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);

	return 0;
}

static void ftmac100_stop_hw(struct ftmac100_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->hw_lock, flags);
	iowrite32(0, priv->base_addr + FTMAC100_OFFSET_MACCR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);
}

/******************************************************************************
 * internal functions (receive descriptor)
 *****************************************************************************/
static inline int ftmac100_rxdes_first_segment(struct ftmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTMAC100_RXDES0_FRS;
}

static inline int ftmac100_rxdes_last_segment(struct ftmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTMAC100_RXDES0_LRS;
}

static inline int ftmac100_rxdes_owned_by_dma(struct ftmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTMAC100_RXDES0_RXDMA_OWN;
}

static inline void ftmac100_rxdes_set_dma_own(struct ftmac100_rxdes *rxdes)
{
	/* clear status bits */
	rxdes->rxdes0 = FTMAC100_RXDES0_RXDMA_OWN;
}

static inline int ftmac100_rxdes_rx_error(struct ftmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTMAC100_RXDES0_RX_ERR;
}

static inline int ftmac100_rxdes_crc_error(struct ftmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTMAC100_RXDES0_CRC_ERR;
}

static inline int ftmac100_rxdes_frame_too_long(struct ftmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTMAC100_RXDES0_FTL;
}

static inline int ftmac100_rxdes_runt(struct ftmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTMAC100_RXDES0_RUNT;
}

static inline int ftmac100_rxdes_odd_nibble(struct ftmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTMAC100_RXDES0_RX_ODD_NB;
}

static inline unsigned int ftmac100_rxdes_frame_length(struct ftmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTMAC100_RXDES0_RFL;
}

static inline int ftmac100_rxdes_multicast(struct ftmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTMAC100_RXDES0_MULTICAST;
}

static inline void ftmac100_rxdes_set_buffer_size(struct ftmac100_rxdes *rxdes,
		unsigned int size)
{
	rxdes->rxdes1 = (rxdes->rxdes1 & FTMAC100_RXDES1_EDORR) |
			 FTMAC100_RXDES1_RXBUF_SIZE(size);
}

static inline void ftmac100_rxdes_set_end_of_ring(struct ftmac100_rxdes *rxdes)
{
	rxdes->rxdes1 |= FTMAC100_RXDES1_EDORR;
}

static inline void ftmac100_rxdes_set_dma_addr(struct ftmac100_rxdes *rxdes, dma_addr_t addr)
{
	rxdes->rxdes2 = addr;
}

static inline dma_addr_t ftmac100_rxdes_get_dma_addr(struct ftmac100_rxdes *rxdes)
{
	return rxdes->rxdes2;
}

/* rxdes3 is not used by hardware, we use it to keep track of buffer */
static inline void ftmac100_rxdes_set_va(struct ftmac100_rxdes *rxdes, void *addr)
{
	rxdes->rxdes3 = (unsigned int)addr;
}

static inline void *ftmac100_rxdes_get_va(struct ftmac100_rxdes *rxdes)
{
	return (void *)rxdes->rxdes3;
}

/******************************************************************************
 * internal functions (receive)
 *****************************************************************************/
static inline int ftmac100_next_rx_pointer(int pointer)
{
	return (pointer + 1) & (RX_QUEUE_ENTRIES - 1);
}

static inline void ftmac100_rx_pointer_advance(struct ftmac100_priv *priv)
{
	priv->rx_pointer = ftmac100_next_rx_pointer(priv->rx_pointer);
}

static inline struct ftmac100_rxdes *ftmac100_current_rxdes(struct ftmac100_priv *priv)
{
	return &priv->descs->rxdes[priv->rx_pointer];
}

static struct ftmac100_rxdes *ftmac100_rx_locate_first_segment(
		struct ftmac100_priv *priv)
{
	struct ftmac100_rxdes *rxdes = ftmac100_current_rxdes(priv);

	while (!ftmac100_rxdes_owned_by_dma(rxdes)) {
		if (ftmac100_rxdes_first_segment(rxdes))
			return rxdes;

		ftmac100_rxdes_set_dma_own(rxdes);
		ftmac100_rx_pointer_advance(priv);
		rxdes = ftmac100_current_rxdes(priv);
	}

	return NULL;
}

static int ftmac100_rx_packet_error(struct ftmac100_priv *priv, struct ftmac100_rxdes *rxdes)
{
	int error = 0;

	if (unlikely(ftmac100_rxdes_rx_error(rxdes))) {
		if (printk_ratelimit())
			dev_info(&priv->dev->dev, "rx err\n");

		priv->stats.rx_errors++;
		error = 1;
	}

	if (unlikely(ftmac100_rxdes_crc_error(rxdes))) {
		if (printk_ratelimit())
			dev_info(&priv->dev->dev, "rx crc err\n");

		priv->stats.rx_crc_errors++;
		error = 1;
	}

	if (unlikely(ftmac100_rxdes_frame_too_long(rxdes))) {
		if (printk_ratelimit())
			dev_info(&priv->dev->dev, "rx frame too long\n");

		priv->stats.rx_length_errors++;
		error = 1;
	}

	if (unlikely(ftmac100_rxdes_runt(rxdes))) {
		if (printk_ratelimit())
			dev_info(&priv->dev->dev, "rx runt\n");

		priv->stats.rx_length_errors++;
		error = 1;
	}

	if (unlikely(ftmac100_rxdes_odd_nibble(rxdes))) {
		if (printk_ratelimit())
			dev_info(&priv->dev->dev, "rx odd nibble\n");

		priv->stats.rx_length_errors++;
		error = 1;
	}

	return error;
}

static void ftmac100_rx_drop_packet(struct ftmac100_priv *priv)
{
	struct ftmac100_rxdes *rxdes = ftmac100_current_rxdes(priv);
	int done = 0;

	if (printk_ratelimit())
		dev_dbg(&priv->dev->dev, "drop packet %p\n", rxdes);

	do {
		if (ftmac100_rxdes_last_segment(rxdes))
			done = 1;

		ftmac100_rxdes_set_dma_own(rxdes);
		ftmac100_rx_pointer_advance(priv);
		rxdes = ftmac100_current_rxdes(priv);
	} while (!done && !ftmac100_rxdes_owned_by_dma(rxdes));

	priv->stats.rx_dropped++;
}

static int ftmac100_rx_packet(struct ftmac100_priv *priv, int *processed)
{
	unsigned long flags;
	struct ftmac100_rxdes *rxdes;
	struct sk_buff *skb;
	int length;
	int copied = 0;
	int done = 0;

	spin_lock_irqsave(&priv->rx_lock, flags);
	rxdes = ftmac100_rx_locate_first_segment(priv);
	spin_unlock_irqrestore(&priv->rx_lock, flags);
	if (!rxdes)
		return 0;

	if (unlikely(ftmac100_rx_packet_error(priv, rxdes))) {
		spin_lock_irqsave(&priv->rx_lock, flags);
		ftmac100_rx_drop_packet(priv);
		spin_unlock_irqrestore(&priv->rx_lock, flags);
		return 1;
	}

	/* start processing */

	length = ftmac100_rxdes_frame_length(rxdes);

	skb = dev_alloc_skb(length + NET_IP_ALIGN);
	if (unlikely(!skb)) {
		if (printk_ratelimit())
			dev_err(&priv->dev->dev, "rx skb alloc failed\n");

		spin_lock_irqsave(&priv->rx_lock, flags);
		ftmac100_rx_drop_packet(priv);
		spin_unlock_irqrestore(&priv->rx_lock, flags);
		return 1;
	}

	if (unlikely(ftmac100_rxdes_multicast(rxdes)))
		priv->stats.multicast++;

	skb_reserve(skb, NET_IP_ALIGN);

	do {
		dma_addr_t d = ftmac100_rxdes_get_dma_addr(rxdes);
		void *buf = ftmac100_rxdes_get_va(rxdes);
		int size;

		size = min(length - copied, RX_BUF_SIZE);

		dma_sync_single_for_cpu(NULL, d, RX_BUF_SIZE, DMA_FROM_DEVICE);
		memcpy(skb_put(skb, size), buf, size);

		copied += size;

		if (ftmac100_rxdes_last_segment(rxdes))
			done = 1;

		dma_sync_single_for_device(NULL, d, RX_BUF_SIZE, DMA_FROM_DEVICE);

		spin_lock_irqsave(&priv->rx_lock, flags);

		ftmac100_rxdes_set_dma_own(rxdes);

		ftmac100_rx_pointer_advance(priv);
		rxdes = ftmac100_current_rxdes(priv);

		spin_unlock_irqrestore(&priv->rx_lock, flags);
	} while (!done && copied < length);

	skb->protocol = eth_type_trans(skb, priv->dev);

	/* push packet to protocol stack */

#ifdef USE_NAPI
	netif_receive_skb(skb);
#else
	netif_rx(skb);
#endif

	priv->dev->last_rx = jiffies;

	priv->stats.rx_packets++;
	priv->stats.rx_bytes += skb->len;

	(*processed)++;

	return 1;
}

/******************************************************************************
 * internal functions (transmit descriptor)
 *****************************************************************************/
static inline void ftmac100_txdes_reset(struct ftmac100_txdes *txdes)
{
	/* clear all except end of ring bit */
	txdes->txdes0 = 0;
	txdes->txdes1 &= FTMAC100_TXDES1_EDOTR;
	txdes->txdes2 = 0;
	txdes->txdes3 = 0;
}

static inline int ftmac100_txdes_owned_by_dma(struct ftmac100_txdes *txdes)
{
	return txdes->txdes0 & FTMAC100_TXDES0_TXDMA_OWN;
}

static inline void ftmac100_txdes_set_dma_own(struct ftmac100_txdes *txdes)
{
	txdes->txdes0 |= FTMAC100_TXDES0_TXDMA_OWN;
}

static inline int ftmac100_txdes_excessive_collision(struct ftmac100_txdes *txdes)
{
	return txdes->txdes0 & FTMAC100_TXDES0_TXPKT_EXSCOL;
}

static inline int ftmac100_txdes_late_collision(struct ftmac100_txdes *txdes)
{
	return txdes->txdes0 & FTMAC100_TXDES0_TXPKT_LATECOL;
}

static inline void ftmac100_txdes_set_end_of_ring(struct ftmac100_txdes *txdes)
{
	txdes->txdes1 |= FTMAC100_TXDES1_EDOTR;
}

static inline void ftmac100_txdes_set_first_segment(struct ftmac100_txdes *txdes)
{
	txdes->txdes1 |= FTMAC100_TXDES1_FTS;
}

static inline void ftmac100_txdes_set_last_segment(struct ftmac100_txdes *txdes)
{
	txdes->txdes1 |= FTMAC100_TXDES1_LTS;
}

static inline void ftmac100_txdes_set_txint(struct ftmac100_txdes *txdes)
{
	txdes->txdes1 |= FTMAC100_TXDES1_TXIC;
}

static inline void ftmac100_txdes_set_buffer_size(struct ftmac100_txdes *txdes, unsigned int len)
{
	txdes->txdes1 |= FTMAC100_TXDES1_TXBUF_SIZE(len);
}

static inline void ftmac100_txdes_set_dma_addr(struct ftmac100_txdes *txdes, dma_addr_t addr)
{
	txdes->txdes2 = addr;
}

/* txdes3 is not used by hardware, we use it to keep track of socket buffer */
static inline void ftmac100_txdes_set_skb(struct ftmac100_txdes *txdes, struct sk_buff *skb)
{
	txdes->txdes3 = (unsigned int)skb;
}

static inline struct sk_buff *ftmac100_txdes_get_skb(struct ftmac100_txdes *txdes)
{
	return (struct sk_buff *)txdes->txdes3;
}

/******************************************************************************
 * internal functions (transmit)
 *****************************************************************************/
static inline int ftmac100_next_tx_pointer(int pointer)
{
	return (pointer + 1) & (TX_QUEUE_ENTRIES - 1);
}

static inline void ftmac100_tx_pointer_advance(struct ftmac100_priv *priv)
{
	priv->tx_pointer = ftmac100_next_tx_pointer(priv->tx_pointer);
}

static inline void ftmac100_tx_clean_pointer_advance(struct ftmac100_priv *priv)
{
	priv->tx_clean_pointer = ftmac100_next_tx_pointer(priv->tx_clean_pointer);
}

static inline struct ftmac100_txdes *ftmac100_current_txdes(struct ftmac100_priv *priv)
{
	return &priv->descs->txdes[priv->tx_pointer];
}

static inline struct ftmac100_txdes *ftmac100_current_clean_txdes(struct ftmac100_priv *priv)
{
	return &priv->descs->txdes[priv->tx_clean_pointer];
}

static int ftmac100_tx_complete_packet(struct ftmac100_priv *priv)
{
	struct ftmac100_txdes *txdes;
	struct sk_buff *skb;

	if (priv->tx_pending == 0)
		return 0;

	txdes = ftmac100_current_clean_txdes(priv);

	if (ftmac100_txdes_owned_by_dma(txdes))
		return 0;

	skb = ftmac100_txdes_get_skb(txdes);

	if (unlikely(ftmac100_txdes_excessive_collision(txdes) ||
			ftmac100_txdes_late_collision(txdes))) {
		/*
		 * packet transmitted to ethernet lost due to late collision
		 * or excessive collision
		 */
		priv->stats.tx_aborted_errors++;
	} else {
		priv->stats.tx_packets++;
		priv->stats.tx_bytes += skb->len;
	}

	skb_dma_unmap(NULL, skb, DMA_TO_DEVICE);

	dev_kfree_skb_irq(skb);

	ftmac100_txdes_reset(txdes);

	ftmac100_tx_clean_pointer_advance(priv);

	priv->tx_pending--;
	netif_wake_queue(priv->dev);

	return 1;
}

static void ftmac100_tx_complete(struct ftmac100_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->tx_lock, flags);
	while (ftmac100_tx_complete_packet(priv));
	spin_unlock_irqrestore(&priv->tx_lock, flags);
}

static int ftmac100_xmit(struct sk_buff *skb, struct ftmac100_priv *priv)
{
	struct ftmac100_txdes *txdes;
	unsigned int len = (skb->len < ETH_ZLEN) ? ETH_ZLEN : skb->len;
	unsigned long flags;

	txdes = ftmac100_current_txdes(priv);
	ftmac100_tx_pointer_advance(priv);

	/* setup TX descriptor */

	spin_lock_irqsave(&priv->tx_lock, flags);
	ftmac100_txdes_set_skb(txdes, skb);
	ftmac100_txdes_set_dma_addr(txdes, skb_shinfo(skb)->dma_maps[0]);

	ftmac100_txdes_set_first_segment(txdes);
	ftmac100_txdes_set_last_segment(txdes);
	ftmac100_txdes_set_txint(txdes);
	ftmac100_txdes_set_buffer_size(txdes, len);

	priv->tx_pending++;
	if (priv->tx_pending == TX_QUEUE_ENTRIES) {
		if (printk_ratelimit())
			dev_info(&priv->dev->dev, "tx queue full\n");

		netif_stop_queue(priv->dev);
	}

	/* start transmit */

	wmb();
	ftmac100_txdes_set_dma_own(txdes);
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	spin_lock_irqsave(&priv->hw_lock, flags);
	ftmac100_txdma_start_polling(priv);
	spin_unlock_irqrestore(&priv->hw_lock, flags);
	priv->dev->trans_start = jiffies;

	return NETDEV_TX_OK;
}

/******************************************************************************
 * internal functions (buffer)
 *****************************************************************************/
static void ftmac100_free_buffers(struct ftmac100_priv *priv)
{
	int i;

	for (i = 0; i < RX_QUEUE_ENTRIES; i += 2) {
		struct ftmac100_rxdes *rxdes = &priv->descs->rxdes[i];
		dma_addr_t d = ftmac100_rxdes_get_dma_addr(rxdes);
		void *page = ftmac100_rxdes_get_va(rxdes);

		if (d)
			dma_unmap_single(NULL, d, PAGE_SIZE, DMA_FROM_DEVICE);

		if (page != NULL)
			free_page((unsigned long)page);
	}

	for (i = 0; i < TX_QUEUE_ENTRIES; i++) {
		struct ftmac100_txdes *txdes = &priv->descs->txdes[i];
		struct sk_buff *skb = ftmac100_txdes_get_skb(txdes);

		if (skb) {
			skb_dma_unmap(NULL, skb, DMA_TO_DEVICE);
			dev_kfree_skb(skb);
		}
	}

	dma_free_coherent(NULL, sizeof(struct ftmac100_descs), priv->descs,
							priv->descs_dma_addr);
}

static int ftmac100_alloc_buffers(struct ftmac100_priv *priv)
{
	int i;

	priv->descs = dma_alloc_coherent(NULL, sizeof(struct ftmac100_descs),
				&priv->descs_dma_addr, GFP_KERNEL | GFP_DMA);
	if (priv->descs == NULL)
		return -ENOMEM;

	memset(priv->descs, 0, sizeof(struct ftmac100_descs));

	/* initialize RX ring */

	ftmac100_rxdes_set_end_of_ring(&priv->descs->rxdes[RX_QUEUE_ENTRIES - 1]);

	for (i = 0; i < RX_QUEUE_ENTRIES; i += 2) {
		struct ftmac100_rxdes *rxdes = &priv->descs->rxdes[i];
		void *page;
		dma_addr_t d;

		page = (void *)__get_free_page(GFP_KERNEL | GFP_DMA);
		if (page == NULL)
			goto err;

		d = dma_map_single(NULL, page, PAGE_SIZE, DMA_FROM_DEVICE);
		if (dma_mapping_error(NULL, d)) {
			free_page((unsigned long)page);
			goto err;
		}

		/*
		 * The hardware enforces a sub-2K maximum packet size, so we
		 * put two buffers on every hardware page.
		 */
		ftmac100_rxdes_set_va(rxdes, page);
		ftmac100_rxdes_set_va(rxdes + 1, page + PAGE_SIZE / 2);

		ftmac100_rxdes_set_dma_addr(rxdes, d);
		ftmac100_rxdes_set_dma_addr(rxdes + 1, d + PAGE_SIZE / 2);

		ftmac100_rxdes_set_buffer_size(rxdes, RX_BUF_SIZE);
		ftmac100_rxdes_set_buffer_size(rxdes + 1, RX_BUF_SIZE);

		ftmac100_rxdes_set_dma_own(rxdes);
		ftmac100_rxdes_set_dma_own(rxdes + 1);
	}

	/* initialize TX ring */

	ftmac100_txdes_set_end_of_ring(&priv->descs->txdes[TX_QUEUE_ENTRIES - 1]);
	return 0;

err:
	ftmac100_free_buffers(priv);
	return -ENOMEM;
}

/******************************************************************************
 * struct mii_if_info functions
 *****************************************************************************/
static int ftmac100_mdio_read(struct net_device *dev, int phy_id, int reg)
{
	struct ftmac100_priv *priv = netdev_priv(dev);
	unsigned long flags;
	int phycr;
	int i;

	phycr = FTMAC100_PHYCR_PHYAD(phy_id) |
		FTMAC100_PHYCR_REGAD(reg) |
		FTMAC100_PHYCR_MIIRD;

	spin_lock_irqsave(&priv->hw_lock, flags);
	iowrite32(phycr, priv->base_addr + FTMAC100_OFFSET_PHYCR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);

	for (i = 0; i < 10; i++) {
		phycr = ioread32(priv->base_addr + FTMAC100_OFFSET_PHYCR);

		if ((phycr & FTMAC100_PHYCR_MIIRD) == 0)
			return phycr & FTMAC100_PHYCR_MIIRDATA;

		msleep(1);
	}

	dev_err(&dev->dev, "mdio read timed out\n");
	return 0xffff;
}

static void ftmac100_mdio_write(struct net_device *dev, int phy_id, int reg, int data)
{
	struct ftmac100_priv *priv = netdev_priv(dev);
	unsigned long flags;
	int phycr;
	int i;

	phycr = FTMAC100_PHYCR_PHYAD(phy_id) |
		FTMAC100_PHYCR_REGAD(reg) |
		FTMAC100_PHYCR_MIIWR;

	data = FTMAC100_PHYWDATA_MIIWDATA(data);

	spin_lock_irqsave(&priv->hw_lock, flags);
	iowrite32(data, priv->base_addr + FTMAC100_OFFSET_PHYWDATA);
	iowrite32(phycr, priv->base_addr + FTMAC100_OFFSET_PHYCR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);

	for (i = 0; i < 10; i++) {
		phycr = ioread32(priv->base_addr + FTMAC100_OFFSET_PHYCR);

		if ((phycr & FTMAC100_PHYCR_MIIWR) == 0)
			return;

		msleep(1);
	}

	dev_err(&dev->dev, "mdio write timed out\n");
}

/******************************************************************************
 * struct ethtool_ops functions
 *****************************************************************************/
static void ftmac100_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strcpy(info->driver, DRV_NAME);
	strcpy(info->version, DRV_VERSION);
	strcpy(info->bus_info, dev_name(&dev->dev));
}

static int ftmac100_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct ftmac100_priv *priv = netdev_priv(dev);
	return mii_ethtool_gset(&priv->mii, cmd);
}

static int ftmac100_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct ftmac100_priv *priv = netdev_priv(dev);
	return mii_ethtool_sset(&priv->mii, cmd);
}

static int ftmac100_nway_reset(struct net_device *dev)
{
	struct ftmac100_priv *priv = netdev_priv(dev);
	return mii_nway_restart(&priv->mii);
}

static u32 ftmac100_get_link(struct net_device *dev)
{
	struct ftmac100_priv *priv = netdev_priv(dev);
	return mii_link_ok(&priv->mii);
}

static struct ethtool_ops ftmac100_ethtool_ops = {
	.set_settings		= ftmac100_set_settings,
	.get_settings		= ftmac100_get_settings,
	.get_drvinfo		= ftmac100_get_drvinfo,
	.nway_reset		= ftmac100_nway_reset,
	.get_link		= ftmac100_get_link,
};

/******************************************************************************
 * interrupt handler
 *****************************************************************************/
static irqreturn_t ftmac100_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct ftmac100_priv *priv = netdev_priv(dev);
	unsigned long flags;
	unsigned int status;
	unsigned int imr;

	spin_lock_irqsave(&priv->hw_lock, flags);
	status = ioread32(priv->base_addr + FTMAC100_OFFSET_ISR);
	imr = ioread32(priv->base_addr + FTMAC100_OFFSET_IMR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);

	status &= imr;
	if (status & (FTMAC100_INT_RPKT_FINISH | FTMAC100_INT_NORXBUF)) {
		/*
		 * FTMAC100_INT_RPKT_FINISH:
		 *	RX DMA has received packets into RX buffer successfully
		 *
		 * FTMAC100_INT_NORXBUF:
		 *	RX buffer unavailable
		 */
#ifdef USE_NAPI
		/* Disable interrupts for polling */
		ftmac100_disable_rxint(priv);

		napi_schedule(&priv->napi);
#else
		int rx = 0;

		while (ftmac100_rx_packet(priv, &rx));
#endif
	}

	if (status & FTMAC100_INT_NORXBUF) {
		/* RX buffer unavailable */
		if (printk_ratelimit())
			dev_info(&dev->dev, "INT_NORXBUF\n");

		priv->stats.rx_over_errors++;
	}

	if (status & (FTMAC100_INT_XPKT_OK | FTMAC100_INT_XPKT_LOST)) {
		/*
		 * FTMAC100_INT_XPKT_OK:
		 * 	 packet transmitted to ethernet successfully
		 *
		 * FTMAC100_INT_XPKT_LOST:
		 *	packet transmitted to ethernet lost due to late
		 *	collision or excessive collision
		 */
		ftmac100_tx_complete(priv);
	}

	if (status & FTMAC100_INT_RPKT_LOST) {
		/* received packet lost due to RX FIFO full */
		if (printk_ratelimit())
			dev_info(&dev->dev, "INT_RPKT_LOST\n");

		priv->stats.rx_fifo_errors++;
	}

	if (status & FTMAC100_INT_AHB_ERR) {
		/* AHB error */
		if (printk_ratelimit())
			dev_info(&dev->dev, "INT_AHB_ERR\n");

		/* do nothing */
	}

	if (status & FTMAC100_INT_PHYSTS_CHG) {
		/* PHY link status change */
		if (printk_ratelimit())
			dev_info(&dev->dev, "INT_PHYSTS_CHG\n");

		mii_check_link(&priv->mii);
	}

	return IRQ_HANDLED;
}

/******************************************************************************
 * struct napi_struct functions
 *****************************************************************************/
#ifdef USE_NAPI
static int ftmac100_poll(struct napi_struct *napi, int budget)
{
	struct ftmac100_priv *priv = container_of(napi, struct ftmac100_priv, napi);
	int retry;
	int rx = 0;

	do {
		retry = ftmac100_rx_packet(priv, &rx);
	} while (retry && rx < budget);

	if (!retry || rx < budget) {
		/* stop polling */
		napi_complete(napi);
		ftmac100_enable_all_int(priv);
	}

	return rx;
}
#endif

/******************************************************************************
 * struct net_device_ops functions
 *****************************************************************************/
static int ftmac100_open(struct net_device *dev)
{
	struct ftmac100_priv *priv = netdev_priv(dev);
	int err;

	err = ftmac100_alloc_buffers(priv);
	if (err) {
		dev_err(&dev->dev, "failed to allocate buffers\n");
		goto err_alloc;
	}

	err = request_irq(priv->irq, ftmac100_interrupt, IRQF_SHARED, dev->name, dev);
	if (err) {
		dev_err(&dev->dev, "failed to request irq %d\n", priv->irq);
		goto err_irq;
	}

	priv->rx_pointer = 0;
	priv->tx_clean_pointer = 0;
	priv->tx_pointer = 0;
	spin_lock_init(&priv->hw_lock);
	spin_lock_init(&priv->rx_lock);
	spin_lock_init(&priv->tx_lock);
	priv->tx_pending = 0;

	err = ftmac100_start_hw(priv);
	if (err)
		goto err_hw;

#ifdef USE_NAPI
	napi_enable(&priv->napi);
#endif
	netif_start_queue(dev);

	ftmac100_enable_all_int(priv);

	return 0;

err_hw:
	free_irq(priv->irq, dev);
err_irq:
	ftmac100_free_buffers(priv);
err_alloc:
	return err;
}

static int ftmac100_stop(struct net_device *dev)
{
	struct ftmac100_priv *priv = netdev_priv(dev);

	ftmac100_disable_all_int(priv);
	netif_stop_queue(dev);
#ifdef USE_NAPI
	napi_disable(&priv->napi);
#endif
	ftmac100_stop_hw(priv);
	free_irq(priv->irq, dev);
	ftmac100_free_buffers(priv);

	return 0;
}

static int ftmac100_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ftmac100_priv *priv = netdev_priv(dev);

	if (unlikely(skb->len > MAX_PKT_SIZE)) {
		if (printk_ratelimit())
			dev_dbg(&dev->dev, "tx packet too big\n");

		priv->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	if (unlikely(skb_dma_map(NULL, skb, DMA_TO_DEVICE) != 0)) {
		/* drop packet */
		if (printk_ratelimit())
			dev_err(&dev->dev, "map socket buffer failed\n");

		priv->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	return ftmac100_xmit(skb, priv);
}

static struct net_device_stats *ftmac100_get_stats(struct net_device *dev)
{
	struct ftmac100_priv *priv = netdev_priv(dev);
	return &priv->stats;
}

/* optional */
static int ftmac100_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct ftmac100_priv *priv = netdev_priv(dev);
	struct mii_ioctl_data *data = if_mii(ifr);

	return generic_mii_ioctl(&priv->mii, data, cmd, NULL);
}

static struct net_device_ops ftmac100_netdev_ops = {
	.ndo_open	= ftmac100_open,
	.ndo_stop	= ftmac100_stop,
	.ndo_start_xmit	= ftmac100_hard_start_xmit,
	.ndo_get_stats	= ftmac100_get_stats,
	.ndo_do_ioctl	= ftmac100_do_ioctl,
};

/******************************************************************************
 * struct platform_driver functions
 *****************************************************************************/
static int ftmac100_remove(struct platform_device *pdev)
{
	struct net_device *dev;
	struct ftmac100_priv *priv;

	dev = platform_get_drvdata(pdev);
	if (dev == NULL)
		return 0;

	platform_set_drvdata(pdev, NULL);

	priv = netdev_priv(dev);

	unregister_netdev(dev);

	if (priv->base_addr != NULL)
		iounmap(priv->base_addr);

	if (priv->res != NULL)
		release_resource(priv->res);

	free_netdev(dev);

	return 0;
}

static int ftmac100_probe(struct platform_device *pdev)
{
	struct resource		*res;
	int			irq;
	struct net_device *dev;
	struct ftmac100_priv *priv;
	int err;

	if (pdev == NULL)
		return -ENODEV;

	if ((res = platform_get_resource(pdev, IORESOURCE_MEM, 0)) == 0) {
		return -ENXIO;
	}

	if ((irq = platform_get_irq(pdev, 0)) < 0) {
		return irq;
	}

	/* setup net_device */

	dev = alloc_etherdev(sizeof(struct ftmac100_priv));
	if (dev == NULL) {
		err = -ENOMEM;
		goto err_out;
	}

	SET_NETDEV_DEV(dev, &pdev->dev);

	dev->netdev_ops		= &ftmac100_netdev_ops;
	dev->ethtool_ops	= &ftmac100_ethtool_ops;

	platform_set_drvdata(pdev, dev);

	/* setup private data */

	priv = netdev_priv(dev);
	priv->dev = dev;

#ifdef USE_NAPI
	/* initialize NAPI */
	netif_napi_add(dev, &priv->napi, ftmac100_poll, 64);
#endif

	/* map io memory */

	priv->res = request_mem_region(res->start, res->end - res->start,
			dev_name(&pdev->dev));
	if (priv->res == NULL) {
		dev_err(&pdev->dev, "Could not reserve memory region\n");
		err = -ENOMEM;
		goto err_out;
	}

	priv->base_addr = ioremap(res->start, res->end - res->start);
	if (priv->base_addr == NULL) {
		dev_err(&pdev->dev, "Failed to ioremap ethernet registers\n");
		err = -EIO;
		goto err_out;
	}

	priv->irq = irq;

	/* initialize struct mii_if_info */

	priv->mii.phy_id	= 0;
	priv->mii.phy_id_mask	= 0x1f;
	priv->mii.reg_num_mask	= 0x1f;
	priv->mii.dev		= dev;
	priv->mii.mdio_read	= ftmac100_mdio_read;
	priv->mii.mdio_write	= ftmac100_mdio_write;

	/* register network device */

	err = register_netdev(dev);
	if (err) {
		dev_err(&pdev->dev, "Failed to register netdev\n");
		goto err_out;
	}

	dev_info(&dev->dev, "irq %d, mapped at %p\n", priv->irq, priv->base_addr);

	if (is_zero_ether_addr(dev->dev_addr)) {
		random_ether_addr(dev->dev_addr);
		dev_info(&dev->dev, "generated random MAC address "
			"%.2x:%.2x:%.2x:%.2x:%.2x:%.2x.\n",
			dev->dev_addr[0], dev->dev_addr[1],
			dev->dev_addr[2], dev->dev_addr[3],
			dev->dev_addr[4], dev->dev_addr[5]);
	}

	return 0;

err_out:
	ftmac100_remove(pdev);
	return err;
}

static struct platform_driver ftmac100_driver = {
	.probe		= ftmac100_probe,
	.remove		= ftmac100_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

/******************************************************************************
 * initialization / finalization
 *****************************************************************************/
static int __init ftmac100_init(void)
{
	printk(KERN_INFO "Loading " DRV_NAME ": version " DRV_VERSION " ...\n");
	return platform_driver_register(&ftmac100_driver);
}

static void __exit ftmac100_exit(void)
{
	platform_driver_unregister(&ftmac100_driver);
}

module_init(ftmac100_init);
module_exit(ftmac100_exit);

MODULE_AUTHOR("Po-Yu Chuang <ratbert@faraday-tech.com>");
MODULE_DESCRIPTION("FTMAC100 driver");
MODULE_LICENSE("GPL");
