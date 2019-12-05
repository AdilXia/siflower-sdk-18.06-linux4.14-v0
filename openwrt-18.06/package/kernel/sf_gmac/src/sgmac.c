#include "sgmac.h"
#include <linux/circ_buf.h>
#include <linux/crc32.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/reset.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/clk.h>
#include <net/vxlan.h>
#include <sf16a18.h>
#ifdef CONFIG_SFAX8_FACTORY_READ
#include <linux/sfax8_factory_read.h>
#endif

struct sgmac_priv *g_priv = NULL;

struct sgmac_dma_desc {
	__le32 flags;		// desc0
	__le32 buf_size;	// desc1
	__le32 buf1_addr;	// desc2	/* Buffer 1 Address Pointer */
	__le32 buf2_addr;	// desc3	/* Buffer 2 Address Pointer */
	__le32 ext_status;	// desc4
	__le32 res;		// desc5
	__le32 timestamp_low;	// desc6
	__le32 timestamp_high;	// desc7
};

struct sgmac_extra_stats {
	/* Transmit errors */
	unsigned long tx_jabber;
	unsigned long tx_frame_flushed;
	unsigned long tx_payload_error;
	unsigned long tx_ip_header_error;
	unsigned long tx_local_fault;
	unsigned long tx_remote_fault;
	/* Receive errors */
	unsigned long rx_watchdog;
	unsigned long rx_da_filter_fail;
	unsigned long rx_payload_error;
	unsigned long rx_ip_header_error;
	/* Tx/Rx IRQ errors */
	unsigned long tx_process_stopped;
	unsigned long rx_buf_unav;
	unsigned long rx_process_stopped;
	unsigned long tx_early;
	unsigned long fatal_bus_error;
};

struct sgmac_priv {
	struct sgmac_dma_desc *dma_rx; // the whole list of rx desc
	struct sk_buff **rx_skbuff;    // the whole list of rx buf
	unsigned int rx_tail;	  // next rx desc owned by dma
	unsigned int rx_head;	  // next rx desc owned by driver

	struct sgmac_dma_desc *dma_tx; // the whole list of tx desc
	struct sk_buff **tx_skbuff;    // the whole list of tx buf
	unsigned int tx_head;	  // next tx desc owned by dma
	unsigned int tx_tail;	  // next tx desc owned by driver
	int tx_irq_cnt; // when we xmit 32 frames, we will set tx irq

	void __iomem *base;
	unsigned int dma_buf_sz;
	dma_addr_t dma_rx_phy; // first rx desc address
	dma_addr_t dma_tx_phy; // first tx desc address

#ifdef CONFIG_SFAX8_RGMII_GMAC
	struct clk *eth_byp_ref_clk;
#endif
#ifdef CONFIG_SFAX8_PTP
	struct clk *eth_tsu_clk;
#endif
	struct clk *eth_bus_clk;
	struct net_device *ndev;
	struct device *dev;
	struct napi_struct napi;

	unsigned long vids[BITS_TO_LONGS(
			VLAN_N_VID)]; // set bit of VLAN_N_VID to save vids
	u16 vids_num;

	const char *gswitch;
	int max_macs; // the number of mac address filter registers we have
	struct sgmac_extra_stats xstats;

	spinlock_t stats_lock;
	char rx_pause; // Flag to enable rx of pause frames
	char tx_pause; // Flag to enable tx of pause frames
	int wolopts;
	struct work_struct tx_timeout_work;

#ifdef CONFIG_SFAX8_PTP
	int hwts_tx_en;
	int hwts_rx_en;
	struct ptp_clock *ptp_clock;
	struct ptp_clock_info ptp_clock_ops;
	spinlock_t ptp_lock;
#endif

	int link;   // PHY's last seen link state, 0 is no link and 1 is link.
	int speed;  // PHY's last set duplex mode.
	int duplex; // PHY's last set speed.
	struct phy_device *phydev; // Pointer to attached PHY device.
	struct mii_bus *bus;       // Pointer to the current MII bus.
	struct device_node *phy_node;

	struct rtnl_link_stats64 netstats;
    int pmt_irq;
};

/* EMAC_Soft_Clkgate[3] */
#define EMAC_ETH_BYPREF_CLK		(1 << 2) /* supply for gmii transmit clk */
#define EMAC_ETH_TSU_CLK		(1 << 3) /* supply for timestamp unit */

/* We have 66(NET_IP_ALIGN + NET_SKB_PAD) bytes headroom before, and GMAC
 * hardware only needs 32 bytes, so it's enough for GMAC driver.
 * But mac80211 needs 112(extra_tx_headroom + IEEE80211_ENCRYPT_HEADROOM) bytes
 * headroom, so mac80211 will resize the skb and copy the buffer of the skb,
 * which is unused cost.
 * So now we expand the head, and mac80211 will not copy the buffer.
 * */
#define EXTER_HEADROOM 80

/* GMAC Configuration Settings */
#define MAX_MTU 9000
// austin: PAUSE_TIME is 0xffff by default
#define PAUSE_TIME 0x400

#define DMA_RX_RING_SZ 256
#define DMA_TX_RING_SZ 64

/* DMA descriptor ring helpers */
// if n < (s - 1), x++; if n = s - 1, n = 0
#define dma_ring_incr(n, s) (((n) + 1) & ((s)-1))
// Measure the non-wrapping occupancy of a buffer:
// This returns the number of consecutive items[2] that can be extracted from
// the buffer without having to wrap back to the beginning of the buffer.
// if h < t, dma_ring_space(h, t, s) = t - h - 1;
// if h >= t, dma_ring_space(h, t, s) = s + t - h - 1;
#define dma_ring_space(h, t, s) CIRC_SPACE(h, t, s)
// Measure the occupancy of a buffer:
// This returns the number of items currently occupying a buffer[2].
// if h < t , dma_ring_cnt(h, t, s) = s + h -t
// if h >= t, dma_ring_cnt(h, t, s) = h - t;
#define dma_ring_cnt(h, t, s) CIRC_CNT(h, t, s)

#define tx_dma_ring_space(p)                                                   \
	dma_ring_space((p)->tx_head, (p)->tx_tail, DMA_TX_RING_SZ)

static int sgmac_stop(struct net_device *ndev);
/* GMAC Descriptor Access Helpers */
static inline void desc_set_buf_len(struct sgmac_dma_desc *p, u32 buf_sz)
{
	if (buf_sz > MAX_DESC_BUF_SZ)
		p->buf_size = cpu_to_le32(
				MAX_DESC_BUF_SZ |
				(buf_sz - MAX_DESC_BUF_SZ)
						<< DESC_BUFFER2_SZ_OFFSET);
	else
		p->buf_size = cpu_to_le32(buf_sz);
}

static inline int desc_get_buf_len(struct sgmac_dma_desc *p)
{
	u32 len = le32_to_cpu(p->buf_size);
	return (len & DESC_BUFFER1_SZ_MASK) +
	       ((len & DESC_BUFFER2_SZ_MASK) >> DESC_BUFFER2_SZ_OFFSET);
}

static inline void
desc_init_rx_desc(struct sgmac_dma_desc *p, int ring_size, int buf_sz)
{
	struct sgmac_dma_desc *end = p + ring_size - 1;

	memset(p, 0, sizeof(*p) * ring_size);

	for (; p <= end; p++)
		desc_set_buf_len(p, buf_sz);

	end->buf_size |= cpu_to_le32(RXDESC1_END_RING);
}

static inline void desc_init_tx_desc(struct sgmac_dma_desc *p, u32 ring_size)
{
	memset(p, 0, sizeof(*p) * ring_size);
	p[ring_size - 1].flags = cpu_to_le32(TXDESC_END_RING);
}

static inline int desc_get_owner(struct sgmac_dma_desc *p)
{
	return le32_to_cpu(p->flags) & DESC_OWN;
}

static inline void desc_set_rx_owner(struct sgmac_dma_desc *p)
{
	/* Clear all fields and set the owner */
	p->flags = cpu_to_le32(DESC_OWN);
}

static inline void desc_set_tx_owner(struct sgmac_dma_desc *p, u32 flags)
{
	u32 tmpflags = le32_to_cpu(p->flags);
	tmpflags &= TXDESC_END_RING;
	tmpflags |= flags | DESC_OWN;
	p->flags = cpu_to_le32(tmpflags);
}

static inline void desc_clear_tx_owner(struct sgmac_dma_desc *p)
{
	u32 tmpflags = le32_to_cpu(p->flags);
	tmpflags &= TXDESC_END_RING;
	p->flags = cpu_to_le32(tmpflags);
}

static inline int desc_get_tx_ls(struct sgmac_dma_desc *p)
{
	return le32_to_cpu(p->flags) & TXDESC_LAST_SEG;
}

static inline int desc_get_tx_fs(struct sgmac_dma_desc *p)
{
	return le32_to_cpu(p->flags) & TXDESC_FIRST_SEG;
}

static inline u32 desc_get_buf_addr(struct sgmac_dma_desc *p)
{
	return le32_to_cpu(p->buf1_addr);
}

static inline void
desc_set_buf_addr(struct sgmac_dma_desc *p, u32 paddr, int len)
{
	p->buf1_addr = cpu_to_le32(paddr);
	if (len > MAX_DESC_BUF_SZ)
		p->buf2_addr = cpu_to_le32(paddr + MAX_DESC_BUF_SZ);
}

static inline void
desc_set_buf_addr_and_size(struct sgmac_dma_desc *p, u32 paddr, int len)
{
	desc_set_buf_len(p, len);
	desc_set_buf_addr(p, paddr, len);
}

static inline int desc_get_rx_frame_len(struct sgmac_dma_desc *p)
{
	u32 data, len;
	data = le32_to_cpu(p->flags);
	len = (data & RXDESC_FRAME_LEN_MASK) >> RXDESC_FRAME_LEN_OFFSET;
	return len;
}

static inline u32 desc_get_tx_timestamp_status(struct sgmac_dma_desc *p)
{
	return le32_to_cpu(p->flags) & TXDESC_TX_TS_STATUS;
}

static inline u32 desc_get_rx_timestamp_status(struct sgmac_dma_desc *p)
{
	u64 ns;
	ns = p->timestamp_low;
	ns += p->timestamp_high * 1000000000ULL;
	return ns;
}

static inline u32 desc_get_timestamp(struct sgmac_dma_desc *p)
{
	return le32_to_cpu(p->flags) & TXDESC_TX_TS_STATUS;
}

static void sgmac_dma_flush_tx_fifo(struct sgmac_priv *priv)
{
	int timeout = 1000;
	u32 reg = readl(priv->base + GMAC_DMA_OPERATION);
	writel(reg | DMA_OPERATION_FTF, priv->base + GMAC_DMA_OPERATION);

	while ((timeout-- > 0) &&
			readl(priv->base + GMAC_DMA_OPERATION) & DMA_OPERATION_FTF)
		udelay(1);
}

static int desc_get_tx_status(struct sgmac_priv *priv, struct sgmac_dma_desc *p)
{
	struct sgmac_extra_stats *x = &priv->xstats;
	u32 status = le32_to_cpu(p->flags);

	if (!(status & TXDESC_ERROR_SUMMARY))
		return 0;

	netdev_dbg(priv->ndev, "tx desc error = 0x%08x\n", status);
	if (status & TXDESC_JABBER_TIMEOUT)
		x->tx_jabber++;
	if (status & TXDESC_FRAME_FLUSHED)
		x->tx_frame_flushed++;
	if (status & TXDESC_UNDERFLOW_ERR)
		sgmac_dma_flush_tx_fifo(priv);
	if (status & TXDESC_IP_HEADER_ERR)
		x->tx_ip_header_error++;
	// we don't have these bit
	/*
	if (status & TXDESC_LOCAL_FAULT)
		x->tx_local_fault++;
	if (status & TXDESC_REMOTE_FAULT)
		x->tx_remote_fault++;
	*/
	if (status & TXDESC_PAYLOAD_CSUM_ERR)
		x->tx_payload_error++;

	return -1;
}

static int desc_get_rx_status(struct sgmac_priv *priv, struct sgmac_dma_desc *p)
{
	struct sgmac_extra_stats *x = &priv->xstats;
	int ret = CHECKSUM_UNNECESSARY;
	u32 status = le32_to_cpu(p->flags);
	u32 ext_status = le32_to_cpu(p->ext_status);

	if (status & RXDESC_DA_FILTER_FAIL) {
		netdev_dbg(priv->ndev, "GMAC RX : Dest Address filter fail\n");
		x->rx_da_filter_fail++;
		return -1;
	}

	/* All frames should fit into a single buffer */
	if (!(status & RXDESC_FIRST_SEG) || !(status & RXDESC_LAST_SEG))
		return -1;

	/* Check if packet has checksum already */
	if ((status & RXDESC_FRAME_TYPE) && (status & RXDESC_EXT_STATUS) &&
			!(ext_status & RXDESC_IP_PAYLOAD_MASK))
		ret = CHECKSUM_NONE;

	netdev_dbg(priv->ndev,
			"rx status - frame type=%d, csum = %d, ext stat %08x\n",
			(status & RXDESC_FRAME_TYPE) ? 1 : 0, ret, ext_status);

	if (!(status & RXDESC_ERROR_SUMMARY))
		return ret;

	/* Handle any errors */
	if (status & (RXDESC_DESCRIPTOR_ERR | RXDESC_OVERFLOW_ERR |
				     RXDESC_CHECKSUM_ERR | RXDESC_LENGTH_ERR |
				     RXDESC_CRC_ERR))
		return -1;

	if (status & RXDESC_EXT_STATUS) {
		if (ext_status & RXDESC_IP_HEADER_ERR)
			x->rx_ip_header_error++;
		if (ext_status & RXDESC_IP_PAYLOAD_ERR)
			x->rx_payload_error++;
		netdev_dbg(priv->ndev, "IP checksum error - stat %08x\n",
				ext_status);
		return CHECKSUM_NONE;
	}

	return ret;
}

static inline void sgmac_mac_enable(struct sgmac_priv *priv)
{
	u32 value = readl(priv->base + GMAC_CONTROL);
	value |= GMAC_CONTROL_RE | GMAC_CONTROL_TE;
	writel(value, priv->base + GMAC_CONTROL);

	value = readl(priv->base + GMAC_DMA_OPERATION);
	value |= DMA_OPERATION_ST | DMA_OPERATION_SR;
	writel(value, priv->base + GMAC_DMA_OPERATION);
}

static inline void sgmac_mac_disable(struct sgmac_priv *priv)
{
	u32 value = readl(priv->base + GMAC_DMA_OPERATION);
	value &= ~(DMA_OPERATION_ST | DMA_OPERATION_SR);
	writel(value, priv->base + GMAC_DMA_OPERATION);
	value = readl(priv->base + GMAC_CONTROL);
	value &= ~(GMAC_CONTROL_TE | GMAC_CONTROL_RE);
	writel(value, priv->base + GMAC_CONTROL);
}

static void
sgmac_set_mac_addr(struct sgmac_priv *priv, unsigned char *addr, int num)
{
	u32 data;
	if (addr) {
		data = (addr[5] << 8) | addr[4] | (num ? GMAC_ADDR_AE : 0);
		writel(data, priv->base + GMAC_ADDR_HIGH(num));
		data = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) |
		       addr[0];
		writel(data, priv->base + GMAC_ADDR_LOW(num));
	} else {
		writel(0, priv->base + GMAC_ADDR_HIGH(num));
		writel(0, priv->base + GMAC_ADDR_LOW(num));
	}
}

static void sgmac_set_mdc_clk_div(struct sgmac_priv *priv)
{
	int value = readl(priv->base + GMAC_GMII_ADDR);
	value &= ~GMAC_GMII_ADDR_CR_MASK;
	/* In sf16a18, the CSR clk is 250MHz */
	value |= GMAC_GMII_ADDR_CR;
	writel(value, priv->base + GMAC_GMII_ADDR);
}

/**
 * sgmac_mdio_remove - MDIO remove function.
 * @priv:	Pointer to ARC EMAC private data structure.
 *
 * Unregisters the MDIO and frees any associate memory for MII bus.
 */
static int sgmac_mdio_remove(struct sgmac_priv *priv)
{
	mdiobus_unregister(priv->bus);
	mdiobus_free(priv->bus);
	priv->bus = NULL;
	return 0;
}

static int sgmac_phy_wait_rw_not_busy(struct sgmac_priv *priv)
{
	int value = readl(priv->base + GMAC_GMII_ADDR);
	int i = 0;
	while ((value & GMAC_GMII_ADDR_GB) && (i < 100000)) {
		value = readl(priv->base + GMAC_GMII_ADDR);
		i++;
	}
	return value;
}

static int sgmac_phy_read(struct mii_bus *bus, int phy_addr, int phy_reg)
{
	struct sgmac_priv *priv = bus->priv;
	int addr_value;
	addr_value = sgmac_phy_wait_rw_not_busy(priv);
	// clear the data first
	writel(0xffffffff, priv->base + GMAC_GMII_DATA);
	// set address flag
	addr_value = (addr_value & (~GMAC_GMII_ADDR_GR)) |
		     ((phy_reg << 6) & GMAC_GMII_ADDR_GR);
	// set phy addr
	addr_value = (addr_value & (~GMAC_GMII_ADDR_PA)) |
		     ((phy_addr << 11) & GMAC_GMII_ADDR_PA);
	// set read flag
	addr_value &= ~GMAC_GMII_ADDR_GW;
	// set GB flag to indica is busy now
	addr_value |= GMAC_GMII_ADDR_GB;
	writel(addr_value, priv->base + GMAC_GMII_ADDR);
	// wait for complete
	sgmac_phy_wait_rw_not_busy(priv);
	return (readl(priv->base + GMAC_GMII_DATA) & GMAC_GMII_DATA_GD);
}

static int
sgmac_phy_write(struct mii_bus *bus, int phy_addr, int phy_reg, u16 data_value)
{
	struct sgmac_priv *priv = bus->priv;
	int addr_value;
	addr_value = sgmac_phy_wait_rw_not_busy(priv);
	// clear the data first
	writel(data_value & GMAC_GMII_DATA_GD, priv->base + GMAC_GMII_DATA);
	// set address flag
	addr_value = (addr_value & (~GMAC_GMII_ADDR_GR)) |
		     ((phy_reg << 6) & GMAC_GMII_ADDR_GR);
	// set phy addr
	addr_value = (addr_value & (~GMAC_GMII_ADDR_PA)) |
		     ((phy_addr << 11) & GMAC_GMII_ADDR_PA);
	// set write flag
	addr_value |= GMAC_GMII_ADDR_GW;
	// set GB flag to indica is busy now
	addr_value |= GMAC_GMII_ADDR_GB;
	writel(addr_value, priv->base + GMAC_GMII_ADDR);
	// wait for complete
	sgmac_phy_wait_rw_not_busy(priv);
	return 0;
}

/**
 * sgmac_adjust_link - Adjust the PHY link duplex.
 * @ndev:	Pointer to the net_device structure.
 *
 * This function is called to change the speed and duplex setting of gmac
 * after auto negotiation is done by the PHY.
 */
static void sgmac_adjust_link(struct net_device *ndev)
{
	struct sgmac_priv *priv = netdev_priv(ndev);
	struct phy_device *phydev = priv->phydev;
	unsigned int reg, state_changed = 0;

	if (priv->link != phydev->link) {
		priv->link = phydev->link;
		state_changed = 1;
	}

	reg = readl(priv->base + GMAC_CONTROL);
	if (priv->speed != phydev->speed) {
		priv->speed = phydev->speed;
		state_changed = 1;
		reg &= ~GMAC_CONTROL_SPD_MASK;
		if (priv->speed == SPEED_10)
			reg |= GMAC_SPEED_10M;
		else if (priv->speed == SPEED_100)
			reg |= GMAC_SPEED_100M;
		else if (priv->speed == SPEED_1000)
			reg |= GMAC_SPEED_1000M;
	}
	if (priv->duplex != phydev->duplex) {

		if (DUPLEX_FULL == phydev->duplex)
			reg |= GMAC_CONTROL_DM;
		else
			reg &= ~GMAC_CONTROL_DM;

		priv->duplex = phydev->duplex;
		state_changed = 1;
	}
	writel(reg, priv->base + GMAC_CONTROL);

	if (state_changed)
		phy_print_status(phydev);

}

int mdio_read_ext(int phyaddr, int phyreg, int *phydata)
{
	int phy_value = sgmac_phy_read(g_priv->bus, phyaddr, phyreg);
	*phydata = phy_value;
	return 0;
}
EXPORT_SYMBOL(mdio_read_ext);

int mdio_write_ext(int phyaddr, int phyreg, int phydata)
{
	sgmac_phy_write(g_priv->bus, phyaddr, phyreg, phydata);
	return 0;
}
EXPORT_SYMBOL(mdio_write_ext);

int smi_read(int reg_addr, int *pdata)
{
	int data;
	sgmac_phy_write(g_priv->bus, 0, 31, 0xe);
	sgmac_phy_write(g_priv->bus, 0, 23, reg_addr);
	sgmac_phy_write(g_priv->bus, 0, 21, 1);
	data = sgmac_phy_read(g_priv->bus, 0, 25);
	*pdata = data;

	return 0;
}

int smi_write(int reg_addr, int data)
{
	sgmac_phy_write(g_priv->bus, 0, 31, 0xe);
	sgmac_phy_write(g_priv->bus, 0, 23, reg_addr);
	sgmac_phy_write(g_priv->bus, 0, 24, data);
	sgmac_phy_write(g_priv->bus, 0, 21, 0x3);

	return 0;
}

int rtl8367c_getAsicReg(int reg, int *pValue)
{
	int regData;
	int retVal;

	retVal = smi_read(reg, &regData);
	if(retVal != 0)
		return -1;

	*pValue = regData;

	return 0;
}

int rtl8367c_setAsicReg(int reg, int value)
{
	int retVal;

	retVal = smi_write(reg, value);
	if(retVal != 0)
		return -1;

	return 0;
}

int rtl8367c_setAsicRegBit(int reg, int bit, int value)
{
	int regData;
	int retVal;

	if(bit >= 16)
		return -1;

	retVal = smi_read(reg, &regData);
	if(retVal != 0)
		return -1;

	if(value)
		regData = regData | (1 << bit);
	else
		regData = regData & (~(1 << bit));

	retVal = smi_write(reg, regData);
	if(retVal != 0)
		return -1;

	return 0;
}

int rtl8367c_setAsicRegBits(int reg, int bits, int value)
{
	int regData;
	int retVal;
	int bitsShift;
	int valueShifted;

	if(bits >= (1 << 16) )
		return -1;

	bitsShift = 0;
	while(!(bits & (1 << bitsShift)))
	{
		bitsShift++;
		if(bitsShift >= 16)
			return -1;
	}
	valueShifted = value << bitsShift;

	if(valueShifted > 0xFFFF)
		return -1;

	retVal = smi_read(reg, &regData);
	if(retVal != 0)
		return -1;

	regData = regData & (~bits);
	regData = regData | (valueShifted & bits);

	retVal = smi_write(reg, regData);
	if(retVal != 0)
		return -1;

	return 0;
}

int rtk_extPort_rgmii_init(int port)
{
	int retVal;
	int ext_id;
	int regValue;
	int reg_data = 0, mode = 1, forcemode = 1, nway = 0, link = 1, speed = 2, duplex = 1, txpause = 1, rxpause = 1;

	ext_id = port - 15;

	if( (retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_BYPASS_LINE_RATE, ext_id, 0)) != 0)
		return retVal;

	if( (retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_MAC8_SEL_SGMII_OFFSET, 0)) != 0)
		return retVal;

	if( (retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_MAC8_SEL_HSGMII_OFFSET, 0)) != 0)
		return retVal;

	if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_DIGITAL_INTERFACE_SELECT, RTL8367C_SELECT_GMII_0_MASK << (ext_id * RTL8367C_SELECT_GMII_1_OFFSET), mode)) != 0)
		return retVal;

	reg_data |= forcemode << 12;
	//reg_data |= mstfault << 9;
	//reg_data |= mstmode << 8;
	reg_data |= nway << 7;
	reg_data |= txpause << 6;
	reg_data |= rxpause << 5;
	reg_data |= link << 4;
	reg_data |= duplex << 2;
	reg_data |= speed;

	if ((retVal = rtl8367c_getAsicReg(RTL8367C_REG_REG_TO_ECO4, &regValue)) != 0)
		return retVal;

	if((regValue & (0x0001 << 5)) && (regValue & (0x0001 << 7)))
	{
		return 0;
	}

	if((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_FDUP_OFFSET, duplex)) != 0)
		return retVal;

	if((retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_SPD_MASK, speed)) != 0)
		return retVal;

	if((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_LINK_OFFSET, link)) != 0)
		return retVal;

	if((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_TXFC_OFFSET, txpause)) != 0)
		return retVal;

	if((retVal = rtl8367c_setAsicRegBit(RTL8367C_REG_SDS_MISC, RTL8367C_CFG_SGMII_RXFC_OFFSET, rxpause)) != 0)
		return retVal;

	printk("End %s\n", __func__);
	return rtl8367c_setAsicReg(RTL8367C_REG_DIGITAL_INTERFACE0_FORCE + ext_id, reg_data);
}

int gsw_reg_rd(u16 Offset, u16 Shift, u16 Size, uint *value)
{
	u16 ro, rv = 0, mask;

	if ((Offset & 0xD000) == 0xD000)
		ro = Offset;
	else
		ro = (Offset | 0xE000);

	sgmac_phy_write(g_priv->bus, 0, SMDIO_WRADDR, ro);
	rv = sgmac_phy_read(g_priv->bus, 0, SMDIO_RDADDR);

	mask = (1 << Size) - 1;
	rv = (rv >> Shift);
	*value = (rv & mask);
	return 0;
}

int gsw_reg_wr(u16 Offset, u16 Shift, u16 Size, u32 value)
{
	u16 ro, rv = 0, mask;

	if ((Offset & 0xD000) == 0xD000)
		ro = Offset;
	else
		ro = (Offset | 0xE000);

	sgmac_phy_write(g_priv->bus, 0, SMDIO_WRADDR, ro);

	if (Size != 16) {
		rv = sgmac_phy_read(g_priv->bus, 0, SMDIO_RDADDR);
		mask = (1 << Size) - 1;
		mask = (mask << Shift);
		value = ((value << Shift) & mask);
		value = ((rv & ~mask) | value);
		sgmac_phy_write(g_priv->bus, 0, SMDIO_WRADDR, ro);
	}

	sgmac_phy_write(g_priv->bus, 0, SMDIO_RDADDR, value);
	return 0;
}

typedef struct
{
   /** Device address on the MDIO interface */
   u8	nAddressDev;
   /** Register address inside the device. */
   u8	nAddressReg;
   /** Exchange data word with the device (read / write). */
   u16	nData;
} GSW_MDIO_data_t;

int GSW_MDIO_DataRead(GSW_MDIO_data_t *parm)
{
	int s;
	uint r;

	do {
		s = gsw_reg_rd(MMDIO_CTRL_MBUSY_OFFSET,
			MMDIO_CTRL_MBUSY_SHIFT,
			MMDIO_CTRL_MBUSY_SIZE, &r);
		if (s != 0)
			return s;
	} while (r);
	r = ((0x2 << 10) | ((parm->nAddressDev & 0x1F) << 5)
		| (parm->nAddressReg & 0x1F));
	s = gsw_reg_wr(MMDIO_CTRL_MBUSY_OFFSET,
		MMDIO_CTRL_REGAD_SHIFT,
		MMDIO_CTRL_REG_SIZE, r);
	if (s != 0)
		return s;
	do {
		s = gsw_reg_rd(MMDIO_CTRL_MBUSY_OFFSET,
			MMDIO_CTRL_MBUSY_SHIFT,
			MMDIO_CTRL_MBUSY_SIZE, &r);
		if (s != 0)
			return s;
	} while (r);
	s = gsw_reg_rd(MMDIO_READ_RDATA_OFFSET,
		MMDIO_READ_RDATA_SHIFT,
		MMDIO_READ_RDATA_SIZE, &r);
	if (s != 0)
		return s;
	parm->nData = (r & 0xFFFF);
	return 0;
}

int GSW_MDIO_DataWrite(GSW_MDIO_data_t *parm)
{
	int s;
	uint r;

	do {
		s = gsw_reg_rd(MMDIO_CTRL_MBUSY_OFFSET,
			MMDIO_CTRL_MBUSY_SHIFT,
			MMDIO_CTRL_MBUSY_SIZE, &r);
		if (s != 0)
			return s;
	} while (r);

	r = parm->nData & 0xFFFF;
	s = gsw_reg_wr(MMDIO_WRITE_WDATA_OFFSET,
		MMDIO_WRITE_WDATA_SHIFT,
		MMDIO_WRITE_WDATA_SIZE, r);
	if (s != 0)
		return s;
	r = ((0x1 << 10) | ((parm->nAddressDev & 0x1F) << 5)
		| (parm->nAddressReg & 0x1F));
	s = gsw_reg_wr(MMDIO_CTRL_MBUSY_OFFSET,
		MMDIO_CTRL_REGAD_SHIFT,
		MMDIO_CTRL_REG_SIZE, r);
	if (s != 0)
		return s;
	do {
		s = gsw_reg_rd(MMDIO_CTRL_MBUSY_OFFSET,
			MMDIO_CTRL_MBUSY_SHIFT,
			MMDIO_CTRL_MBUSY_SIZE, &r);
		if (s != 0)
			return s;
	} while (r);
	return 0;
}

int intel_rgmii_init(int port)
{
	int s;
	GSW_MDIO_data_t md;
	uint r;

	s = gsw_reg_rd((MAC_PSTAT_PACT_OFFSET + (0xC * port)),
			MAC_PSTAT_PACT_SHIFT,
			MAC_PSTAT_PACT_SIZE, &r);
	if (s != 0)
		return s;

	if (r) {
		uint pa, fd, pc;
		s = gsw_reg_rd((PHY_ADDR_0_ADDR_OFFSET - port),
				PHY_ADDR_0_ADDR_SHIFT,
				PHY_ADDR_0_ADDR_SIZE, &pa);
		if (s != 0)
			return s;

		fd = 1; //DUPLEX_FULL
		pc = PHY_AN_ADV_1000FDX;
		md.nAddressReg = 9;
		md.nAddressDev = pa;
		s = GSW_MDIO_DataRead(&md);
		if (s != 0)
			return s;
		md.nData &= ~(PHY_AN_ADV_1000HDX
				| PHY_AN_ADV_1000FDX);
		md.nData |= pc;
		md.nAddressDev = pa;
		md.nAddressReg = 9;
		s = GSW_MDIO_DataWrite(&md);
		if (s != 0)
			return s;
		md.nAddressReg = 4;
		md.nAddressDev = pa;
		s = GSW_MDIO_DataRead(&md);
		if (s != 0)
			return s;
		md.nData &= ~(PHY_AN_ADV_10HDX
				| PHY_AN_ADV_10FDX
				| PHY_AN_ADV_100HDX
				| PHY_AN_ADV_100FDX);
		md.nAddressReg = 4;
		md.nAddressDev = pa;
		s = GSW_MDIO_DataWrite(&md);
		if (s != 0)
			return s;

		md.nAddressDev = pa;
		md.nAddressReg = 0;
		s = GSW_MDIO_DataRead(&md);
		if (s != 0)
			return s;
		md.nData = 0x1200;
		s = GSW_MDIO_DataWrite(&md);
		if (s != 0)
			return s;
	} else {
		uint pr;
		s = gsw_reg_rd((PHY_ADDR_0_ADDR_OFFSET - port),
				PHY_ADDR_0_ADDR_SHIFT,
				PHY_ADDR_0_REG_SIZE,
				&pr);
		if (s != 0)
			return s;

		//set GSW_DUPLEX_FULL
		pr &= ~(3 << 9);
		pr |= (1 << 9);

		//set GSW_PORT_LINK_UP
		pr &= ~(3 << 13);
		pr |= (1 << 13);

		//set GSW_PORT_SPEED_1000
		pr &= ~(3 << 11);
		pr |= (2 << 11);
		s = gsw_reg_wr((PHY_ADDR_0_ADDR_OFFSET - port),
				PHY_ADDR_0_ADDR_SHIFT,
				PHY_ADDR_0_REG_SIZE,
				pr);
		if (s != 0)
			return s;
	}
	//'Low Power Idle' Support for 'Energy Efficient Ethernet'. default dsiable.
	s = gsw_reg_wr((MAC_CTRL_4_LPIEN_OFFSET + (0xC * port)),
		MAC_CTRL_4_LPIEN_SHIFT,
		MAC_CTRL_4_LPIEN_SIZE, 0);
	printk("End %s\n", __func__);
	return 0;
}


/**
 * sgmac_mdio_probe - MDIO probe function.
 * @priv:	Pointer to ARC EMAC private data structure.
 *
 * returns:	0 on success, -ENOMEM when mdiobus_alloc
 * (to allocate memory for MII bus structure) fails.
 *
 * Sets up and registers the MDIO interface.
 */
static int sgmac_mdio_probe(struct sgmac_priv *priv)
{
	struct mii_bus *bus;
	int error;

	bus = mdiobus_alloc();
	if (!bus)
		return -ENOMEM;

	g_priv = priv;
	priv->bus = bus;
	bus->priv = priv;
	bus->parent = priv->dev;
	bus->name = "Synopsys MII Bus",
	bus->read = &sgmac_phy_read;
	bus->write = &sgmac_phy_write;

	snprintf(bus->id, MII_BUS_ID_SIZE, "%s", bus->name);

	error = of_mdiobus_register(bus, priv->dev->of_node);
	if (error) {
		netdev_err(priv->ndev, "cannot register MDIO bus %s\n",
				bus->name);
		mdiobus_free(bus);
		return -error;
	}

	return 0;
}

#ifdef CONFIG_SFAX8_PTP
static int sgmac_ptp_init_systime(struct sgmac_priv *priv, u32 sec, u32 nsec)
{
	int limit;
	u32 value;

	writel(sec, priv->base + GMAC_TS_HIGH_UPDATE);
	writel(nsec, priv->base + GMAC_TS_LOW_UPDATE);
	/* issue command to initialize the system time value */
	value = readl(priv->base + GMAC_TS_CONTROL);
	value |= GMAC_TS_CONTROL_TSINIT;
	writel(value, priv->base + GMAC_TS_CONTROL);

	/* wait for present system time initialize to complete */
	limit = 10;
	while (limit--) {
		if (!(readl(priv->base + GMAC_TS_CONTROL) &
				    GMAC_TS_CONTROL_TSINIT))
			break;
		mdelay(10);
	}
	if (limit < 0)
		return -EBUSY;

	return 0;
}

static int sgmac_ptp_config_addend(struct sgmac_priv *priv, u32 addend)
{
	u32 value;
	int limit;

	writel(addend, priv->base + GMAC_TS_ADDEND);
	/* issue command to update the addend value */
	value = readl(priv->base + GMAC_TS_CONTROL);
	value |= GMAC_TS_CONTROL_TSADDREG;
	writel(value, priv->base + GMAC_TS_CONTROL);

	/* wait for present addend update to complete */
	limit = 10;
	while (limit--) {
		if (!(readl(priv->base + GMAC_TS_CONTROL) &
				    GMAC_TS_CONTROL_TSADDREG))
			break;
		mdelay(10);
	}
	if (limit < 0)
		return -EBUSY;

	return 0;
}

static int sgmac_ptp_adjust_systime(struct sgmac_priv *priv,
			u32 sec, u32 nsec, int add_sub)
{
	u32 value;
	int limit;

	writel(sec, priv->base + GMAC_TS_HIGH_UPDATE);
	writel(((add_sub << 31) | nsec), priv->base + GMAC_TS_LOW_UPDATE);
	/* issue command to initialize the system time value */
	value = readl(priv->base + GMAC_TS_CONTROL);
	value |= GMAC_TS_STATUS_TSTRGTERR;
	writel(value, priv->base + GMAC_TS_CONTROL);

	/* wait for present system time adjust/update to complete */
	limit = 10;
	while (limit--) {
		if (!(readl(priv->base + GMAC_TS_CONTROL) &
				    GMAC_TS_STATUS_TSTRGTERR))
			break;
		mdelay(10);
	}
	if (limit < 0)
		return -EBUSY;

	return 0;
}

static u64 sgmac_ptp_get_systime(struct sgmac_priv *priv)
{
	u64 ns;

	ns = readl(priv->base + GMAC_TS_LOW);
	/* convert sec time value to nanosecond */
	ns += readl(priv->base + GMAC_TS_HIGH) * 1000000000ULL;

	return ns;
}

/**
 * sgmac_adjust_freq
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ppb: desired period change in parts ber billion
 *
 * Description: this function will adjust the frequency of hardware clock.
 */
// austin: I have to reference other codes.
static int sgmac_ptp_adjust_freq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct sgmac_priv *priv =
			container_of(ptp, struct sgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u32 diff, addend;
	int neg_adj = 0;
	u64 adj;

	if (ppb < 0) {
		neg_adj = 1;
		ppb = -ppb;
	}

	addend = 50000000ULL;
	adj = addend;
	adj *= ppb;
	diff = div_u64(adj, 1000000000ULL);
	addend = neg_adj ? (addend - diff) : (addend + diff);

	spin_lock_irqsave(&priv->ptp_lock, flags);

	sgmac_ptp_config_addend(priv, addend);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

/**
 * sgmac_adjust_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @delta: desired change in nanoseconds
 *
 * Description: this function will shift/adjust the hardware clock time.
 */
static int sgmac_ptp_adjust_time(struct ptp_clock_info *ptp, s64 delta)
{
	struct sgmac_priv *priv =
			container_of(ptp, struct sgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u32 sec, nsec;
	u32 quotient, reminder;
	int neg_adj = 0;

	if (delta < 0) {
		neg_adj = 1;
		delta = -delta;
	}

	quotient = div_u64_rem(delta, 1000000000ULL, &reminder);
	sec = quotient;
	nsec = reminder;

	spin_lock_irqsave(&priv->ptp_lock, flags);

	sgmac_ptp_adjust_systime(priv, sec, nsec, neg_adj);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

/**
 * sgmac_get_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: pointer to hold time/result
 *
 * Description: this function will read the current time from the
 * hardware clock and store it in @ts.
 */
static int sgmac_ptp_get_time(struct ptp_clock_info *ptp, struct timespec *ts)
{
	struct sgmac_priv *priv =
			container_of(ptp, struct sgmac_priv, ptp_clock_ops);
	unsigned long flags;
	u64 ns;
	u32 reminder;

	spin_lock_irqsave(&priv->ptp_lock, flags);

	ns = sgmac_ptp_get_systime(priv);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	ts->tv_sec = div_u64_rem(ns, 1000000000ULL, &reminder);
	ts->tv_nsec = reminder;

	return 0;
}

/**
 * sgmac_set_time
 *
 * @ptp: pointer to ptp_clock_info structure
 * @ts: time value to set
 *
 * Description: this function will set the current time on the
 * hardware clock.
 */
static int sgmac_ptp_set_time(struct ptp_clock_info *ptp,
		const struct timespec *ts)
{
	struct sgmac_priv *priv =
			container_of(ptp, struct sgmac_priv, ptp_clock_ops);
	unsigned long flags;

	spin_lock_irqsave(&priv->ptp_lock, flags);

	sgmac_ptp_init_systime(priv, ts->tv_sec, ts->tv_nsec);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

static int sgmac_ptp_enable(struct ptp_clock_info *ptp,
		struct ptp_clock_request *rq,
		int on)
{
	return -EOPNOTSUPP;
}

/* structure describing a PTP hardware clock */
static struct ptp_clock_info sgmac_ptp_clock_ops = {
	.owner = THIS_MODULE,
	.name = "sgmac_ptp_clock",
	.max_adj = 50000000,
	.n_alarm = 0,
	.n_ext_ts = 1,
	.n_per_out = 0,
	.n_pins = 0,
	.pps = 0,
	.adjfreq = sgmac_ptp_adjust_freq,
	.adjtime = sgmac_ptp_adjust_time,
	.gettime = sgmac_ptp_get_time,
	.settime = sgmac_ptp_set_time,
	.enable = sgmac_ptp_enable,
};

/**
 * sgmac_ptp_register
 * @priv: driver private structure
 * Description: this function will register the ptp clock driver
 * to kernel. It also does some house keeping work.
 */
int sgmac_ptp_register(struct sgmac_priv *priv)
{
	spin_lock_init(&priv->ptp_lock);
	priv->ptp_clock_ops = sgmac_ptp_clock_ops;
	priv->hwts_tx_en = 0;
	priv->hwts_rx_en = 0;

	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_ops, priv->dev);
	if (IS_ERR(priv->ptp_clock)) {
		priv->ptp_clock = NULL;
		netdev_err(priv->ndev, "ptp_clock_register() failed on %s\n",
				priv->ndev->name);
		return -EINVAL;
	} else {
		netdev_dbg(priv->ndev, "Added PTP HW clock successfully on %s\n",
				priv->ndev->name);
	}
	return 0;
}

/**
 * sgmac_ptp_unregister
 * @priv: driver private structure
 * Description: this function will remove/unregister the ptp clock driver
 * from the kernel.
 */
void sgmac_ptp_unregister(struct sgmac_priv *priv)
{
	if (priv->ptp_clock) {
		ptp_clock_unregister(priv->ptp_clock);
		priv->ptp_clock = NULL;
		netdev_dbg(priv->ndev,
				"Removed PTP HW clock successfully on %s\n",
				priv->ndev->name);
	}
}

/* sgmac_ptp_get_tx_hwtstamp: get HW TX timestamps
 * @priv: driver private structure
 * @entry : descriptor index to be used.
 * @skb : the socket buffer
 * Description :
 * This function will read timestamp from the descriptor & pass it to stac
 * and also perform some sanity checks.
 */
static void sgmac_ptp_get_tx_hwtstamp(struct sgmac_priv *priv,
		struct sgmac_dma_desc *desc,
		struct sk_buff *skb)
{
	struct skb_shared_hwtstamps shhwtstamp;
	u64 ns;
	/* check tx tstamp status */
	if (!desc_get_tx_timestamp_status(desc))
		return;

	/* get the valid tstamp */
	ns = desc_get_timestamp(desc);

	memset(&shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
	shhwtstamp.hwtstamp = ns_to_ktime(ns);
	/* pass tstamp to stack */
	skb_tstamp_tx(skb, &shhwtstamp);

	return;
}

/* sgmac_ptp_get_rx_hwtstamp: get HW RX timestamps
 * @priv: driver private structure
 * @entry : descriptor index to be used.
 * @skb : the socket buffer
 * Description :
 * This function will read received packet's timestamp from the descriptor
 * and pass it to stack. It also perform some sanity checks.
 */
static void sgmac_ptp_get_rx_hwtstamp(struct sgmac_priv *priv,
		struct sgmac_dma_desc *desc,
		struct sk_buff *skb)
{
	struct skb_shared_hwtstamps *shhwtstamp = NULL;
	u64 ns;

	/* exit if rx tstamp is not valid */
	if (!desc_get_rx_timestamp_status(desc))
		return;

	/* get valid tstamp */
	ns = desc_get_timestamp(desc);
	shhwtstamp = skb_hwtstamps(skb);
	memset(shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
	shhwtstamp->hwtstamp = ns_to_ktime(ns);
}

/**
 *  sgmac_hwtstamp_ioctl - control hardware timestamping.
 *  @dev: device pointer.
 *  @ifr: An IOCTL specefic structure, that can contain a pointer to
 *  a proprietary structure used to pass information to the driver.
 *  Description:
 *  This function configures the MAC to enable/disable both outgoing(TX)
 *  and incoming(RX) packets time stamping based on user input.
 *  Return Value:
 *  0 on success and an appropriate -ve integer on failure.
 */
static int sgmac_ptp_hwtstamp_ioctl(struct net_device *dev, struct ifreq *ifr)
{
	struct sgmac_priv *priv = netdev_priv(dev);
	struct hwtstamp_config config;
	struct timespec now;
	u64 temp = 0;
	u32 ptp_v2 = 0;
	u32 tstamp_all = 0;
	u32 ptp_over_ipv4_udp = 0;
	u32 ptp_over_ipv6_udp = 0;
	u32 ptp_over_ethernet = 0;
	u32 snap_type_sel = 0;
	u32 ts_master_en = 0;
	u32 ts_event_en = 0;
	u32 value = 0;
	int ret;
	if (copy_from_user(&config, ifr->ifr_data,
			    sizeof(struct hwtstamp_config)))
		return -EFAULT;

	netdev_err(priv->ndev,
			"%s config flags:0x%x, tx_type:0x%x, rx_filter:0x%x\n",
			__func__, config.flags, config.tx_type,
			config.rx_filter);

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	if (config.tx_type != HWTSTAMP_TX_OFF &&
			config.tx_type != HWTSTAMP_TX_ON)
		return -ERANGE;

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		/* time stamp no incoming packet at all */
		config.rx_filter = HWTSTAMP_FILTER_NONE;
		break;

	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
		/* PTP v1, UDP, any kind of event packet */
		config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
		/* take time stamp for all event messages */
		snap_type_sel = GMAC_TS_CONTROL_BIT16;

		ptp_over_ipv4_udp = GMAC_TS_CONTROL_TSIPV4ENA;
		ptp_over_ipv6_udp = GMAC_TS_CONTROL_TSIPV6ENA;
		break;

	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
		/* PTP v1, UDP, Sync packet */
		config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_SYNC;
		/* take time stamp for SYNC messages only */
		ts_event_en = GMAC_TS_CONTROL_TSEVNTENA;

		ptp_over_ipv4_udp = GMAC_TS_CONTROL_TSIPV4ENA;
		ptp_over_ipv6_udp = GMAC_TS_CONTROL_TSIPV6ENA;
		break;

	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		/* PTP v1, UDP, Delay_req packet */
		config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ;
		/* take time stamp for Delay_Req messages only */
		ts_master_en = GMAC_TS_CONTROL_TSMSTRENA;
		ts_event_en = GMAC_TS_CONTROL_TSEVNTENA;

		ptp_over_ipv4_udp = GMAC_TS_CONTROL_TSIPV4ENA;
		ptp_over_ipv6_udp = GMAC_TS_CONTROL_TSIPV6ENA;
		break;

	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
		/* PTP v2, UDP, any kind of event packet */
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
		ptp_v2 = GMAC_TS_CONTROL_TSVER2ENA;
		/* take time stamp for all event messages */
		snap_type_sel = GMAC_TS_CONTROL_BIT16;

		ptp_over_ipv4_udp = GMAC_TS_CONTROL_TSIPV4ENA;
		ptp_over_ipv6_udp = GMAC_TS_CONTROL_TSIPV6ENA;
		break;

	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
		/* PTP v2, UDP, Sync packet */
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_SYNC;
		ptp_v2 = GMAC_TS_CONTROL_TSVER2ENA;
		/* take time stamp for SYNC messages only */
		ts_event_en = GMAC_TS_CONTROL_TSEVNTENA;

		ptp_over_ipv4_udp = GMAC_TS_CONTROL_TSIPV4ENA;
		ptp_over_ipv6_udp = GMAC_TS_CONTROL_TSIPV6ENA;
		break;

	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		/* PTP v2, UDP, Delay_req packet */
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ;
		ptp_v2 = GMAC_TS_CONTROL_TSVER2ENA;
		/* take time stamp for Delay_Req messages only */
		ts_master_en = GMAC_TS_CONTROL_TSMSTRENA;
		ts_event_en = GMAC_TS_CONTROL_TSEVNTENA;

		ptp_over_ipv4_udp = GMAC_TS_CONTROL_TSIPV4ENA;
		ptp_over_ipv6_udp = GMAC_TS_CONTROL_TSIPV6ENA;
		break;

	case HWTSTAMP_FILTER_PTP_V2_EVENT:
		/* PTP v2/802.AS1 any layer, any kind of event packet */
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		ptp_v2 = GMAC_TS_CONTROL_TSVER2ENA;
		/* take time stamp for all event messages */
		snap_type_sel = GMAC_TS_CONTROL_BIT16;

		ptp_over_ipv4_udp = GMAC_TS_CONTROL_TSIPV4ENA;
		ptp_over_ipv6_udp = GMAC_TS_CONTROL_TSIPV6ENA;
		ptp_over_ethernet = GMAC_TS_CONTROL_TSIPENA;
		break;

	case HWTSTAMP_FILTER_PTP_V2_SYNC:
		/* PTP v2/802.AS1, any layer, Sync packet */
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_SYNC;
		ptp_v2 = GMAC_TS_CONTROL_TSVER2ENA;
		/* take time stamp for SYNC messages only */
		ts_event_en = GMAC_TS_CONTROL_TSEVNTENA;

		ptp_over_ipv4_udp = GMAC_TS_CONTROL_TSIPV4ENA;
		ptp_over_ipv6_udp = GMAC_TS_CONTROL_TSIPV6ENA;
		ptp_over_ethernet = GMAC_TS_CONTROL_TSIPENA;
		break;

	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		/* PTP v2/802.AS1, any layer, Delay_req packet */
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_DELAY_REQ;
		ptp_v2 = GMAC_TS_CONTROL_TSVER2ENA;
		/* take time stamp for Delay_Req messages only */
		ts_master_en = GMAC_TS_CONTROL_TSMSTRENA;
		ts_event_en = GMAC_TS_CONTROL_TSEVNTENA;

		ptp_over_ipv4_udp = GMAC_TS_CONTROL_TSIPV4ENA;
		ptp_over_ipv6_udp = GMAC_TS_CONTROL_TSIPV6ENA;
		ptp_over_ethernet = GMAC_TS_CONTROL_TSIPENA;
		break;

	case HWTSTAMP_FILTER_ALL:
		/* time stamp any incoming packet */
		config.rx_filter = HWTSTAMP_FILTER_ALL;
		tstamp_all = GMAC_TS_CONTROL_TSENALL;
		break;

	default:
		return -ERANGE;
	}
	priv->hwts_rx_en = ((config.rx_filter == HWTSTAMP_FILTER_NONE) ? 0 : 1);
	priv->hwts_tx_en = config.tx_type == HWTSTAMP_TX_ON;

	if (!priv->hwts_tx_en && !priv->hwts_rx_en)
		writel(0, priv->base + GMAC_TS_CONTROL);
	else {
		value = (GMAC_TS_CONTROL_TSENA | GMAC_TS_CONTROL_TSCFUPDT |
				GMAC_TS_CONTROL_TSCTRLSSR | tstamp_all |
				ptp_v2 | ptp_over_ethernet | ptp_over_ipv6_udp |
				ptp_over_ipv4_udp | ts_event_en | ts_master_en |
				snap_type_sel);
		writel(value, priv->base + GMAC_TS_CONTROL);

		/* program Sub Second Increment reg */
		// 1s / 50MHz = 20ns
		value = (1000000000ULL / 50000000);
		writel(value, priv->base + GMAC_TS_SUBSEC_INCR);

		/* calculate default added value:
		 * formula is :
		 * addend = (2^32)/freq_div_ratio;
		 * where, freq_div_ratio = clk_ptp_ref_i/50MHz
		 * hence, addend = ((2^32) * 50MHz)/clk_ptp_ref_i;
		 * NOTE: clk_ptp_ref_i should be >= 50MHz to
		 *       achive 20ns accuracy.
		 *
		 * 2^x * y == (y << x), hence
		 * 2^32 * 50000000 ==> (50000000 << 32)
		 */
		temp = (u64)(50000000ULL << 32);
		value = 0;
		sgmac_ptp_config_addend(priv, value);

		/* initialize system time */
		getnstimeofday(&now);
		sgmac_ptp_init_systime(priv, now.tv_sec, now.tv_nsec);
	}

	ret = copy_to_user(
			ifr->ifr_data, &config, sizeof(struct hwtstamp_config));

	return ret ? -EFAULT : 0;
}
#endif

/**
 * sgmac_set_flow_ctrl:
 * @priv: private driver structure
 * @rx: enable rx flow ctrl or not
 * @tx: enable tx flow ctrl or not
 * Description: enable tx flow ctrl and rx flow ctrl
 */
static int sgmac_set_flow_ctrl(struct sgmac_priv *priv, int rx, int tx)
{
	u32 reg;
	unsigned int flow = 0;

	priv->rx_pause = rx;
	priv->tx_pause = tx;

	if (rx || tx) {
		if (rx)
			flow |= GMAC_FLOW_CTRL_RFE;
		if (tx)
			flow |= GMAC_FLOW_CTRL_TFE;

		flow |= GMAC_FLOW_CTRL_PLT | GMAC_FLOW_CTRL_UP;
		flow |= (PAUSE_TIME << GMAC_FLOW_CTRL_PT_SHIFT);

		writel(flow, priv->base + GMAC_FLOW_CTRL);

		reg = readl(priv->base + GMAC_DMA_OPERATION);
		reg |= DMA_OPERATION_EFC;
		writel(reg, priv->base + GMAC_DMA_OPERATION);
	} else {
		writel(0, priv->base + GMAC_FLOW_CTRL);

		reg = readl(priv->base + GMAC_DMA_OPERATION);
		reg &= ~DMA_OPERATION_EFC;
		writel(reg, priv->base + GMAC_DMA_OPERATION);
	}

	return 0;
}

/**
 * sgmac_rx_refill:
 * @priv: private driver structure
 * Description: it refills all the rx descs which are owned by driver
 * with new sk_buf and sets these descs owned by dma.
 */
static void sgmac_rx_refill(struct sgmac_priv *priv)
{
	struct sgmac_dma_desc *p;
	dma_addr_t paddr;
	int bufsz = priv->ndev->mtu + ETH_HLEN + ETH_FCS_LEN;

	while (dma_ring_space(priv->rx_head, priv->rx_tail, DMA_RX_RING_SZ) >
			1) {
		int entry = priv->rx_head;
		struct sk_buff *skb;

		p = priv->dma_rx + entry;

		if (likely(priv->rx_skbuff[entry] == NULL)) {
			skb = netdev_alloc_skb_ip_align(priv->ndev, bufsz + EXTER_HEADROOM);
			if (unlikely(skb == NULL))
				break;

			skb_reserve(skb, EXTER_HEADROOM);
			paddr = dma_map_single(priv->dev, skb->data,
					priv->dma_buf_sz - NET_IP_ALIGN,
					DMA_FROM_DEVICE);
			if (dma_mapping_error(priv->dev, paddr)) {
				dev_kfree_skb_any(skb);
				break;
			}
			priv->rx_skbuff[entry] = skb;
			desc_set_buf_addr(p, paddr, priv->dma_buf_sz);
		}

		netdev_dbg(priv->ndev, "rx ring: head %d, tail %d\n",
				priv->rx_head, priv->rx_tail);

		priv->rx_head = dma_ring_incr(priv->rx_head, DMA_RX_RING_SZ);
		desc_set_rx_owner(p);
	}
}

/**
 * init_sgmac_dma_desc_rings - init the RX/TX descriptor rings
 * @dev: net device structure
 * Description:  this function initializes the DMA RX/TX descriptors
 * and allocates the socket buffers.
 */
static int sgmac_dma_desc_rings_init(struct net_device *ndev)
{
	struct sgmac_priv *priv = netdev_priv(ndev);
	unsigned int bfsize;

	/* Set the Buffer size according to the MTU;
	 * The total buffer size including any IP offset must be a multiple
	 * of 8 bytes.
	 */
	bfsize = ALIGN(ndev->mtu + ETH_HLEN + ETH_FCS_LEN + NET_IP_ALIGN, 8);

	netdev_dbg(priv->ndev, "mtu [%d] bfsize [%d]\n", ndev->mtu, bfsize);

	priv->rx_skbuff = kzalloc(
			sizeof(struct sk_buff *) * DMA_RX_RING_SZ, GFP_KERNEL);
	if (!priv->rx_skbuff)
		return -ENOMEM;
	priv->dma_rx = dma_alloc_coherent(priv->dev,
			DMA_RX_RING_SZ * sizeof(struct sgmac_dma_desc),
			&priv->dma_rx_phy, GFP_KERNEL);
	if (!priv->dma_rx)
		goto err_dma_rx;

	priv->tx_skbuff = kzalloc(
			sizeof(struct sk_buff *) * DMA_TX_RING_SZ, GFP_KERNEL);
	if (!priv->tx_skbuff)
		goto err_tx_skb;
	priv->dma_tx = dma_alloc_coherent(priv->dev,
			DMA_TX_RING_SZ * sizeof(struct sgmac_dma_desc),
			&priv->dma_tx_phy, GFP_KERNEL);
	if (!priv->dma_tx)
		goto err_dma_tx;

	netdev_dbg(priv->ndev,
			"DMA desc rings: virt addr (Rx %p, "
			"Tx %p)\n\tDMA phy addr (Rx 0x%08x, Tx 0x%08x)\n",
			priv->dma_rx, priv->dma_tx,
			(unsigned int)priv->dma_rx_phy,
			(unsigned int)priv->dma_tx_phy);

	priv->rx_tail = 0;
	priv->rx_head = 0;
	priv->dma_buf_sz = bfsize;
	desc_init_rx_desc(priv->dma_rx, DMA_RX_RING_SZ, priv->dma_buf_sz);
	//we should refill rx buffer immediately case hw need it when receive high thoughput stream
	sgmac_rx_refill(priv);

	priv->tx_tail = 0;
	priv->tx_head = 0;
	desc_init_tx_desc(priv->dma_tx, DMA_TX_RING_SZ);

	writel(priv->dma_tx_phy, priv->base + GMAC_DMA_TX_BASE_ADDR);
	writel(priv->dma_rx_phy, priv->base + GMAC_DMA_RX_BASE_ADDR);
	return 0;

err_dma_tx:
	kfree(priv->tx_skbuff);
err_tx_skb:
	dma_free_coherent(priv->dev,
			DMA_RX_RING_SZ * sizeof(struct sgmac_dma_desc),
			priv->dma_rx, priv->dma_rx_phy);
err_dma_rx:
	kfree(priv->rx_skbuff);
	return -ENOMEM;
}

static void sgmac_free_rx_skbufs(struct sgmac_priv *priv)
{
	int i;
	struct sgmac_dma_desc *p;

	if (!priv->rx_skbuff)
		return;

	for (i = 0; i < DMA_RX_RING_SZ; i++) {
		struct sk_buff *skb = priv->rx_skbuff[i];
		if (skb == NULL)
			continue;

		p = priv->dma_rx + i;
		dma_unmap_single(priv->dev, desc_get_buf_addr(p),
				priv->dma_buf_sz - NET_IP_ALIGN,
				DMA_FROM_DEVICE);
		dev_kfree_skb_any(skb);
		priv->rx_skbuff[i] = NULL;
	}
}

static void sgmac_free_tx_skbufs(struct sgmac_priv *priv)
{
	int i;
	struct sgmac_dma_desc *p;

	if (!priv->tx_skbuff)
		return;

	for (i = 0; i < DMA_TX_RING_SZ; i++) {
		if (priv->tx_skbuff[i] == NULL)
			continue;

		p = priv->dma_tx + i;
		if (desc_get_tx_fs(p))
			dma_unmap_single(priv->dev, desc_get_buf_addr(p),
					desc_get_buf_len(p), DMA_TO_DEVICE);
		else
			dma_unmap_page(priv->dev, desc_get_buf_addr(p),
					desc_get_buf_len(p), DMA_TO_DEVICE);

		if (desc_get_tx_ls(p))
			dev_kfree_skb_any(priv->tx_skbuff[i]);
		priv->tx_skbuff[i] = NULL;
	}
}

static void sgmac_free_dma_desc_rings(struct sgmac_priv *priv)
{
	/* Release the DMA TX/RX socket buffers */
	sgmac_free_rx_skbufs(priv);
	sgmac_free_tx_skbufs(priv);

	/* Free the consistent memory allocated for descriptor rings */
	if (priv->dma_tx) {
		dma_free_coherent(priv->dev,
				DMA_TX_RING_SZ * sizeof(struct sgmac_dma_desc),
				priv->dma_tx, priv->dma_tx_phy);
		priv->dma_tx = NULL;
	}
	if (priv->dma_rx) {
		dma_free_coherent(priv->dev,
				DMA_RX_RING_SZ * sizeof(struct sgmac_dma_desc),
				priv->dma_rx, priv->dma_rx_phy);
		priv->dma_rx = NULL;
	}
	kfree(priv->rx_skbuff);
	priv->rx_skbuff = NULL;
	kfree(priv->tx_skbuff);
	priv->tx_skbuff = NULL;
}

/**
 * gmac_tx:
 * @priv: private driver structure
 * Description: it reclaims resources after transmission completes.
 */
static void sgmac_tx_complete(struct sgmac_priv *priv)
{
	while (dma_ring_cnt(priv->tx_head, priv->tx_tail, DMA_TX_RING_SZ)) {
		unsigned int entry = priv->tx_tail;
		struct sk_buff *skb = priv->tx_skbuff[entry];
		struct sgmac_dma_desc *p = priv->dma_tx + entry;

		/* Check if the descriptor is owned by the DMA. */
		if (desc_get_owner(p))
			break;

		netdev_dbg(priv->ndev, "tx ring: curr %d, dirty %d\n",
				priv->tx_head, priv->tx_tail);

		if (desc_get_tx_fs(p))
			dma_unmap_single(priv->dev, desc_get_buf_addr(p),
					desc_get_buf_len(p), DMA_TO_DEVICE);
		else
			dma_unmap_page(priv->dev, desc_get_buf_addr(p),
					desc_get_buf_len(p), DMA_TO_DEVICE);

		/* Check tx error on the last segment */
		if (desc_get_tx_ls(p)) {
			desc_get_tx_status(priv, p);
			priv->netstats.tx_packets++;
			priv->netstats.tx_bytes += skb->len;
			dev_consume_skb_any(skb);
		}
#ifdef CONFIG_SFAX8_PTP
		if(priv->hwts_tx_en)
			sgmac_ptp_get_tx_hwtstamp(priv, p, skb);
#endif
		priv->tx_skbuff[entry] = NULL;
		priv->tx_tail = dma_ring_incr(entry, DMA_TX_RING_SZ);
	}

	/* Ensure tx_tail is visible to sgmac_xmit */
	smp_mb();
	if (unlikely(netif_queue_stopped(priv->ndev) &&
			    (tx_dma_ring_space(priv) > MAX_SKB_FRAGS)))
		netif_wake_queue(priv->ndev);
}

static void sgmac_tx_timeout_work(struct work_struct *work)
{
	u32 reg, value;
	struct sgmac_priv *priv =
			container_of(work, struct sgmac_priv, tx_timeout_work);

	napi_disable(&priv->napi);

	writel(0, priv->base + GMAC_DMA_INTR_ENA);

	netif_tx_lock(priv->ndev);

	reg = readl(priv->base + GMAC_DMA_OPERATION);
	writel(reg & ~DMA_OPERATION_ST, priv->base + GMAC_DMA_OPERATION);
	do {
		value = readl(priv->base + GMAC_DMA_STATUS) & 0x700000;
	} while (value && (value != 0x600000));

	sgmac_free_tx_skbufs(priv);
	desc_init_tx_desc(priv->dma_tx, DMA_TX_RING_SZ);
	priv->tx_tail = 0;
	priv->tx_head = 0;
	writel(priv->dma_tx_phy, priv->base + GMAC_DMA_TX_BASE_ADDR);
	writel(reg | DMA_OPERATION_ST, priv->base + GMAC_DMA_OPERATION);

	writel(DMA_STATUS_TU | DMA_STATUS_TPS | DMA_STATUS_NIS | DMA_STATUS_AIS,
			priv->base + GMAC_DMA_STATUS);

	netif_tx_unlock(priv->ndev);
	netif_wake_queue(priv->ndev);

	napi_enable(&priv->napi);

	/* Enable interrupts */
	writel(DMA_INTR_DEFAULT_MASK, priv->base + GMAC_DMA_STATUS);
	writel(DMA_INTR_DEFAULT_MASK, priv->base + GMAC_DMA_INTR_ENA);
}

static int sgmac_hw_init(struct net_device *ndev)
{
	u32 value, ctrl;
	int limit;
	struct sgmac_priv *priv = netdev_priv(ndev);
	/* Save the ctrl register value */
	ctrl = readl(priv->base + GMAC_CONTROL) & GMAC_CONTROL_SPD_MASK;
	/* SW reset */
	value = DMA_BUS_MODE_SFT_RESET;
	writel(value, priv->base + GMAC_DMA_BUS_MODE);
	limit = 15000;
	while (limit-- && (readl(priv->base + GMAC_DMA_BUS_MODE) &
					  DMA_BUS_MODE_SFT_RESET))
		cpu_relax();
	if (limit < 0){
		netdev_err(ndev,"%s reset fail bus mode 0x%08x\n",__func__, readl(priv->base + GMAC_DMA_BUS_MODE));
		return -EBUSY;
	}

	value = (0x10 << DMA_BUS_MODE_PBL_SHIFT) |
		(0x10 << DMA_BUS_MODE_RPBL_SHIFT) | DMA_BUS_MODE_FB |
		DMA_BUS_MODE_ATDS | DMA_BUS_MODE_AAL;
	writel(value, priv->base + GMAC_DMA_BUS_MODE);

	writel(0, priv->base + GMAC_DMA_INTR_ENA);

	/* Mask power mgt interrupt */
	writel(GMAC_INT_MASK_PMTIM, priv->base + GMAC_INT_STAT);

	/* GMAC requires AXI bus init. This is a 'magic number' for now */
	writel(0x0077000E, priv->base + GMAC_DMA_AXI_BUS);
	/* set default link, speed and duplex here */
	priv->link = 0;
	priv->duplex = DUPLEX_FULL;
#ifdef CONFIG_SFAX8_RGMII_GMAC
	priv->speed = SPEED_1000;
	ctrl = GMAC_CONTROL_CST | GMAC_CONTROL_JE | GMAC_CONTROL_ACS |
	       GMAC_SPEED_1000M | GMAC_CONTROL_DM;
#else
	priv->speed = SPEED_100;
	ctrl = GMAC_CONTROL_CST | GMAC_CONTROL_JE | GMAC_CONTROL_ACS |
	       GMAC_SPEED_100M | GMAC_CONTROL_DM;
#endif

	if (ndev->features & NETIF_F_RXCSUM)
		ctrl |= GMAC_CONTROL_IPC;
	writel(ctrl, priv->base + GMAC_CONTROL);
	/* Set the HW DMA mode and the COE */
	writel(DMA_OPERATION_TSF | DMA_OPERATION_RFD | DMA_OPERATION_RFA |
					DMA_OPERATION_RTC_256 |
					DMA_OPERATION_OSF,
			priv->base + GMAC_DMA_OPERATION);

	return 0;
}

/**
 *  sgmac_open - open entry point of the driver
 *  @dev : pointer to the device structure.
 *  Description:
 *  This function is the open entry point of the driver.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int sgmac_open(struct net_device *ndev)
{
	int ret, phy_value;
	struct sgmac_priv *priv = netdev_priv(ndev);

	memset(&priv->xstats, 0, sizeof(struct sgmac_extra_stats));
	// if not connect to phy , reset would fail with rtl 82111f RM#2248
	/* austin:
	 * In mpw0, only RMII can work well.
	 * In mpw1, RGMII can work but it need extern phy rx delay function, and
	 * RMII can also work well.
	 * In later mpw version, maybe we will fix RGMII timing bug and can
	 * use PHY_INTERFACE_MODE_RGMII instead of PHY_INTERFACE_MODE_RGMII_RXID.
	 */
	/* should init gswitch before gmac reset */
	if (priv->gswitch && !strncmp(priv->gswitch, "realtek", 7))
	{
		rtk_extPort_rgmii_init(16);
	}
	else if (priv->gswitch && !strncmp(priv->gswitch, "intel", 5))
	{
		intel_rgmii_init(5);
	}
	else{
#ifdef CONFIG_SFAX8_RGMII_GMAC
#ifdef CONFIG_SF16A18_MPW1
		priv->phydev = of_phy_connect(ndev, priv->phy_node, sgmac_adjust_link,
				0, PHY_INTERFACE_MODE_RGMII_RXID);
#else
		priv->phydev = of_phy_connect(ndev, priv->phy_node, sgmac_adjust_link,
				0, PHY_INTERFACE_MODE_RGMII);
#endif
#else
		priv->phydev = of_phy_connect(ndev, priv->phy_node, sgmac_adjust_link,
				0, PHY_INTERFACE_MODE_RMII);
#endif

		if (!priv->phydev) {
			netdev_err(ndev, "of_phy_connect() failed\n");
			// when phy connect fail, should to release all resource
			sgmac_stop(ndev);
			return -ENODEV;
		}
		dev_info(priv->dev, "connected to %s with id 0x%x\n",
				priv->phydev->drv->name, priv->phydev->phy_id);

		/* phy_id 0x1cc916 means Realtek phy */
		if (priv->phydev->phy_id == 0x1cc916){
			/* set 10/100M led link */
			sgmac_phy_write(priv->bus, priv->phydev->mdio.addr, 31, 0x0d04);
			phy_value = sgmac_phy_read(priv->bus, priv->phydev->mdio.addr, 16);
			//bit 10 for LED2_LINK_10, bit 11 for LED2_LINK_100
			phy_value |= (1 << 10) | (1 << 11);
			sgmac_phy_write(priv->bus, priv->phydev->mdio.addr, 16, phy_value);
			sgmac_phy_write(priv->bus, priv->phydev->mdio.addr, 31, 0x0);
		}
	}
	/* Initialize the GMAC and descriptors */
	sgmac_hw_init(ndev);
	sgmac_set_mac_addr(priv, ndev->dev_addr, 0);
	sgmac_set_flow_ctrl(priv, priv->rx_pause, priv->tx_pause);

	ret = sgmac_dma_desc_rings_init(ndev);
	if (ret < 0)
		return ret;

	napi_enable(&priv->napi);
	netif_start_queue(ndev);

	/* Enable interrupts */
	// austin:why we have to write GMAC_DMA_STATUS here?
	// TODO: we should test whether writing GMAC_DMA_STATUS will effect our
	// driver.
	//	writel(DMA_INTR_DEFAULT_MASK, ioaddr + GMAC_DMA_STATUS);
	writel(DMA_INTR_DEFAULT_MASK, priv->base + GMAC_DMA_INTR_ENA);

	/* Enable the MAC Rx/Tx */
	sgmac_mac_enable(priv);

	if (!priv->gswitch)
		phy_start_aneg(priv->phydev);

	return 0;
}

/**
 *  sgmac_stop - close entry point of the driver
 *  @dev : device pointer.
 *  Description:
 *  This is the stop entry point of the driver.
 */
static int sgmac_stop(struct net_device *ndev)
{
	struct sgmac_priv *priv = netdev_priv(ndev);
	if (priv->phydev)
		phy_disconnect(priv->phydev);

	if (readl(priv->base + GMAC_DMA_INTR_ENA))
		napi_disable(&priv->napi);

	writel(0, priv->base + GMAC_DMA_INTR_ENA);

	netif_tx_disable(ndev);

	/* Disable the MAC core */
	sgmac_mac_disable(priv);

	/* Release and free the Rx/Tx resources */
	sgmac_free_dma_desc_rings(priv);
	return 0;
}

/**
 *  sgmac_xmit:
 *  @skb : the socket buffer
 *  @ndev : device pointer
 *  Description : Tx entry point of the driver.
 */
static netdev_tx_t sgmac_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct sgmac_priv *priv = netdev_priv(ndev);
	unsigned int entry;
	int i;
	u32 irq_flag;
	int nfrags = skb_shinfo(skb)->nr_frags;
	struct sgmac_dma_desc *desc, *first;
	unsigned int desc_flags;
	unsigned int len;
	dma_addr_t paddr;

	/* We do a little when we receive tx irq, so we tx 32 frames and then
	 * set tx irq. */
	priv->tx_irq_cnt = (priv->tx_irq_cnt + 1) & (DMA_TX_RING_SZ / 4 - 1);
	irq_flag = priv->tx_irq_cnt ? 0 : TXDESC_INTERRUPT;

	desc_flags = (skb->ip_summed == CHECKSUM_PARTIAL) ? TXDESC_CSUM_ALL : 0;
	entry = priv->tx_head;
	desc = priv->dma_tx + entry;
	first = desc;

	len = skb_headlen(skb);
	paddr = dma_map_single(priv->dev, skb->data, len, DMA_TO_DEVICE);
	if (dma_mapping_error(priv->dev, paddr)) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	priv->tx_skbuff[entry] = skb;
	desc_set_buf_addr_and_size(desc, paddr, len);

	for (i = 0; i < nfrags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		len = frag->size;

		paddr = skb_frag_dma_map(
				priv->dev, frag, 0, len, DMA_TO_DEVICE);
		if (dma_mapping_error(priv->dev, paddr))
			goto dma_err;

		entry = dma_ring_incr(entry, DMA_TX_RING_SZ);
		desc = priv->dma_tx + entry;
		priv->tx_skbuff[entry] = skb;

		desc_set_buf_addr_and_size(desc, paddr, len);
		if (i < (nfrags - 1))
			desc_set_tx_owner(desc, desc_flags);
	}

	/* Interrupt on completition only for the latest segment */
	if (desc != first)
		desc_set_tx_owner(
				desc, desc_flags | TXDESC_LAST_SEG | irq_flag);
	else
		desc_flags |= TXDESC_LAST_SEG | irq_flag;

	/* Set owner on first desc last to avoid race condition */
	wmb();
	desc_set_tx_owner(first, desc_flags | TXDESC_FIRST_SEG);
	cpu_relax();

	writel(1, priv->base + GMAC_DMA_TX_POLL);

	priv->tx_head = dma_ring_incr(entry, DMA_TX_RING_SZ);

	/* Ensure tx_head update is visible to tx completion */
	smp_mb();
	if (unlikely(tx_dma_ring_space(priv) <= MAX_SKB_FRAGS)) {
		netif_stop_queue(ndev);
		/* Ensure netif_stop_queue is visible to tx completion */
		smp_mb();
		if (tx_dma_ring_space(priv) > MAX_SKB_FRAGS)
			netif_start_queue(ndev);
	}

	return NETDEV_TX_OK;

dma_err:
	entry = priv->tx_head;
	for (; i > 0; i--) {
		entry = dma_ring_incr(entry, DMA_TX_RING_SZ);
		desc = priv->dma_tx + entry;
		priv->tx_skbuff[entry] = NULL;
		dma_unmap_page(priv->dev, desc_get_buf_addr(desc),
				desc_get_buf_len(desc), DMA_TO_DEVICE);
		desc_clear_tx_owner(desc);
	}
	desc = first;
	dma_unmap_single(priv->dev, desc_get_buf_addr(desc),
			desc_get_buf_len(desc), DMA_TO_DEVICE);
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static int sgmac_rx(struct sgmac_priv *priv, int limit)
{
	unsigned int entry;
	unsigned int count = 0;
	struct sgmac_dma_desc *p;
	while (count < limit) {
		int ip_checksum;
		struct sk_buff *skb;
		int frame_len;

		if (!dma_ring_cnt(priv->rx_head, priv->rx_tail, DMA_RX_RING_SZ)) {
			break;
		}

		entry = priv->rx_tail;
		p = priv->dma_rx + entry;
		if (desc_get_owner(p))
			break;

		count++;
		priv->rx_tail = dma_ring_incr(priv->rx_tail, DMA_RX_RING_SZ);

		/* read the status of the incoming frame */
		ip_checksum = desc_get_rx_status(priv, p);
		if (ip_checksum < 0)
			continue;

		skb = priv->rx_skbuff[entry];
		if (unlikely(!skb)) {
			netdev_err(priv->ndev,
					"Inconsistent Rx descriptor chain\n");
			break;
		}
		priv->rx_skbuff[entry] = NULL;
		sgmac_rx_refill(priv);

		frame_len = desc_get_rx_frame_len(p);
		netdev_dbg(priv->ndev, "RX frame size %d, COE status: %d\n",
				frame_len, ip_checksum);

#ifdef CONFIG_SFAX8_PTP
		if (priv->hwts_rx_en)
			sgmac_ptp_get_rx_hwtstamp(priv, p, skb);
#endif
		skb_put(skb, frame_len);
		dma_unmap_single(priv->dev, desc_get_buf_addr(p),
				priv->dma_buf_sz - NET_IP_ALIGN,
				DMA_FROM_DEVICE);

		skb->protocol = eth_type_trans(skb, priv->ndev);
		skb->ip_summed = ip_checksum;
		priv->netstats.rx_packets++;
		priv->netstats.rx_bytes += frame_len;
		if (ip_checksum == CHECKSUM_NONE)
			netif_receive_skb(skb);
		else
			napi_gro_receive(&priv->napi, skb);
	}

	return count;
}

/**
 *  sgmac_poll - gmac poll method (NAPI)
 *  @napi : pointer to the napi structure.
 *  @budget : maximum number of packets that the current CPU can receive from
 *	      all interfaces.
 *  Description :
 *   This function implements the the reception process.
 *   Also it runs the TX completion thread
 */
static int sgmac_poll(struct napi_struct *napi, int budget)
{
	struct sgmac_priv *priv = container_of(napi, struct sgmac_priv, napi);
	int work_done = 0;

	sgmac_tx_complete(priv);
	work_done = sgmac_rx(priv, budget);

	if (work_done < budget) {
		napi_complete(napi);
		__raw_writel(DMA_INTR_DEFAULT_MASK,
				priv->base + GMAC_DMA_INTR_ENA);
	}
	return work_done;
}

/**
 *  sgmac_tx_timeout
 *  @ndev : Pointer to net device structure
 *  Description: this function is called when a packet transmission fails to
 *   complete within a reasonable tmrate. The driver will mark the error in the
 *   netdev structure and arrange for the device to be reset to a sane state
 *   in order to transmit a new packet.
 */
static void sgmac_tx_timeout(struct net_device *ndev)
{
	struct sgmac_priv *priv = netdev_priv(ndev);
	schedule_work(&priv->tx_timeout_work);
}

/**
 *  sgmac_set_rx_mode - entry point for multicast addressing
 *  @ndev : pointer to the device structure
 *  Description:
 *  This function is a driver entry point which gets called by the kernel
 *  whenever multicast addresses must be enabled/disabled.
 *  Return value: void.
 */
// austin:
// according to netdev_uc_count(dev),set unicast addressing(perfect or hash)
// according to netdev_mc_count(dev),set multicast addressing(perfect or hash)
static void sgmac_set_rx_mode(struct net_device *ndev)
{
	int i;
	struct sgmac_priv *priv = netdev_priv(ndev);
	unsigned int value = 0;
	u32 hash_filter[GMAC_NUM_HASH];
	int reg = 1;
	struct netdev_hw_addr *ha;
	bool use_hash = false;

	netdev_dbg(priv->ndev, "# mcasts %d, # unicast %d\n",
			netdev_mc_count(ndev), netdev_uc_count(ndev));

	if (ndev->flags & IFF_PROMISC)
		value |= GMAC_FRAME_FILTER_PR;

	memset(hash_filter, 0, sizeof(hash_filter));

	if (netdev_uc_count(ndev) > priv->max_macs) {
		use_hash = true;
		value |= GMAC_FRAME_FILTER_HUC | GMAC_FRAME_FILTER_HPF;
	}
// austin: we can use first to save some time.
#if 0
	if (use_hash) {
		netdev_for_each_uc_addr(ha, ndev) {
			//austin:we use 8 bits hash value, not 9 bits
			//we have to test the value of bit_nr
			//u32 bit_nr = ether_crc(ETH_ALEN, ha->addr) >> 24;
			u32 bit_nr = ~ether_crc(ETH_ALEN, ha->addr) >> 23;
			/* The most significant 4 bits determine the register to
			 * use (H/L) while the other 5 bits determine the bit
			 * within the register. */
			hash_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
		}
	}else {
	netdev_for_each_uc_addr(ha, ndev) {
			sgmac_set_mac_addr(priv, ha->addr, reg);
			reg++;
		}
	}
#else
	netdev_for_each_uc_addr(ha, ndev)
	{
		if (use_hash) {
			// austin:we use 8 bits
			// we have to test the value of bit_nr
			// u32 bit_nr = ether_crc(ETH_ALEN, ha->addr) >> 24;
			u32 bit_nr = ~ether_crc(ETH_ALEN, ha->addr) >> 23;
			/* The most significant 4 bits determine the register to
			 * use (H/L) while the other 5 bits determine the bit
			 * within the register. */
			hash_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
		} else {
			sgmac_set_mac_addr(priv, ha->addr, reg);
			reg++;
		}
	}
#endif
	if (ndev->flags & IFF_ALLMULTI) {
		value |= GMAC_FRAME_FILTER_PM;
		goto out;
	}

	if ((netdev_mc_count(ndev) + reg - 1) > priv->max_macs) {
		use_hash = true;
		value |= GMAC_FRAME_FILTER_HMC | GMAC_FRAME_FILTER_HPF;
	} else {
		use_hash = false;
	}
	netdev_for_each_mc_addr(ha, ndev)
	{
		if (use_hash) {
			u32 bit_nr = ~ether_crc(ETH_ALEN, ha->addr) >> 23;

			/* The most significant 4 bits determine the register to
			 * use (H/L) while the other 5 bits determine the bit
			 * within the register. */
			hash_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
		} else {
			sgmac_set_mac_addr(priv, ha->addr, reg);
			reg++;
		}
	}

out:
	for (i = reg; i <= priv->max_macs; i++)
		sgmac_set_mac_addr(priv, NULL, i);
	for (i = 0; i < GMAC_NUM_HASH; i++)
		writel(hash_filter[i], priv->base + GMAC_HASH(i));

	writel(value, priv->base + GMAC_FRAME_FILTER);
}

/**
 *  sgmac_change_mtu - entry point to change MTU size for the device.
 *  @ndev : device pointer.
 *  @new_mtu : the new MTU size for the device.
 *  Description: the Maximum Transfer Unit (MTU) is used by the network layer
 *  to drive packet transmission. Ethernet has an MTU of 1500 octets
 *  (ETH_DATA_LEN). This value can be changed with ifconfig.
 *  Return value:
 *  0 on success and an appropriate (-)ve integer as defined in errno.h
 *  file on failure.
 */
static int sgmac_change_mtu(struct net_device *ndev, int new_mtu)
{
	struct sgmac_priv *priv = netdev_priv(ndev);
	int old_mtu;

	if ((new_mtu < 46) || (new_mtu > MAX_MTU)) {
		netdev_err(priv->ndev, "invalid MTU, max MTU is: %d\n",
				MAX_MTU);
		return -EINVAL;
	}

	old_mtu = ndev->mtu;

	/* return early if the buffer sizes will not change */
	if (old_mtu == new_mtu)
		return 0;

	ndev->mtu = new_mtu;

	/* if device has been brouht down, return here */
	if (unlikely(!netif_running(ndev)))
		return 0;

	/* if device up, reload the device */
	sgmac_stop(ndev);
	return sgmac_open(ndev);
}

static irqreturn_t sgmac_pmt_interrupt(int irq, void *dev_id)
{
	u32 intr_status;
	struct net_device *ndev = (struct net_device *)dev_id;
	struct sgmac_priv *priv = netdev_priv(ndev);

	intr_status = readl(priv->base + GMAC_INT_STAT);
	if (intr_status & GMAC_INT_STAT_PMTIS) {
		netdev_dbg(priv->ndev, "received Magic frame\n");
		/* clear the PMT bits 5 and 6 by reading the PMT */
		// austin: we should add pmt handle here.
		// this handle should also add in sgmac_interrupt.
		// pm_wakeup_event(priv->dev, 0);
		readl(priv->base + GMAC_PMT);
	}
	return IRQ_HANDLED;
}

static irqreturn_t sgmac_interrupt(int irq, void *dev_id)
{
	u32 intr_status;
	struct net_device *ndev = (struct net_device *)dev_id;
	struct sgmac_priv *priv = netdev_priv(ndev);
	struct sgmac_extra_stats *extra_stats = &priv->xstats;
	/* read the status register (CSR5) */
	intr_status = __raw_readl(priv->base + GMAC_DMA_STATUS);
#ifdef CONFIG_SFAX8_RGMII_GMAC
	// handle RGMII link status by reading link status reg.
	if (unlikely(intr_status & DMA_STATUS_GLI))
		__raw_readl(priv->base + GMAC_LINK_STATUS);
#endif

	if (unlikely(intr_status & DMA_STATUS_GLPII)){
		__raw_readl(priv->base + GMAC_LPI_CTRL);
		__raw_writel(0x400, priv->base + GMAC_INT_MASK);
	}

	intr_status &= __raw_readl(priv->base + GMAC_DMA_INTR_ENA);
	__raw_writel(intr_status, priv->base + GMAC_DMA_STATUS);

	/* ABNORMAL interrupts */
	if (unlikely(intr_status & DMA_STATUS_AIS)) {
		if (intr_status & DMA_STATUS_TJT) {
			netdev_err(priv->ndev, "transmit jabber\n");
			extra_stats->tx_jabber++;
		}
		if (intr_status & DMA_STATUS_RU)
			extra_stats->rx_buf_unav++;
		if (intr_status & DMA_STATUS_RPS) {
			netdev_err(priv->ndev, "receive process stopped\n");
			extra_stats->rx_process_stopped++;
		}
		if (intr_status & DMA_STATUS_ETI) {
			netdev_err(priv->ndev, "transmit early interrupt\n");
			extra_stats->tx_early++;
		}
		if (intr_status & DMA_STATUS_TPS) {
			netdev_err(priv->ndev, "transmit process stopped\n");
			extra_stats->tx_process_stopped++;
			schedule_work(&priv->tx_timeout_work);
		}
		if (intr_status & DMA_STATUS_FBI) {
			netdev_err(priv->ndev, "fatal bus error\n");
			extra_stats->fatal_bus_error++;
		}
	}

	/* TX/RX NORMAL interrupts */
	if (intr_status & (DMA_STATUS_RI | DMA_STATUS_TU | DMA_STATUS_TI)) {
		__raw_writel(DMA_INTR_ABNORMAL, priv->base + GMAC_DMA_INTR_ENA);
		napi_schedule(&priv->napi);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/* Polling receive - used by NETCONSOLE and other diagnostic tools
 * to allow network I/O with interrupts disabled. */
static void sgmac_poll_controller(struct net_device *ndev)
{
	disable_irq(ndev->irq);
	sgmac_interrupt(ndev->irq, ndev);
	enable_irq(ndev->irq);
}
#endif

static int sgmac_set_mac_address(struct net_device *ndev, void *p)
{
	struct sgmac_priv *priv = netdev_priv(ndev);
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(ndev->dev_addr, addr->sa_data, ndev->addr_len);

	sgmac_set_mac_addr(priv, ndev->dev_addr, 0);

	return 0;
}

static int sgmac_set_features(struct net_device *ndev,
		netdev_features_t features)
{
	u32 ctrl;
	struct sgmac_priv *priv = netdev_priv(ndev);
	netdev_features_t changed = ndev->features ^ features;

	if (!(changed & NETIF_F_RXCSUM))
		return 0;

	ctrl = readl(priv->base + GMAC_CONTROL);
	if (features & NETIF_F_RXCSUM)
		ctrl |= GMAC_CONTROL_IPC;
	else
		ctrl &= ~GMAC_CONTROL_IPC;
	writel(ctrl, priv->base + GMAC_CONTROL);

	return 0;
}

static void sgmac_set_vid(struct sgmac_priv *priv)
{
	int i = 0;
	u16 vid_le;
	unsigned char vid[2];
	// for hash vlan filter
	u16 vlan_hash_table = 0;
	u32 crc;

	writel(0, priv->base + GMAC_VLAN_TAG);
	writel(0, priv->base + GMAC_VLAN_HASH);
	if (priv->vids_num == 0)
		return;

	/* We use perfect vlan filter if there is just one vid */
	if (priv->vids_num == 1) {
		vid_le = find_first_bit(priv->vids, VLAN_N_VID);
		vid_le = cpu_to_le16(i);
		writel(vid_le, priv->base + GMAC_VLAN_TAG);
		return;
	}

	/* We use hash vlan filter if there are more than one vids */
	for_each_set_bit(i, priv->vids, VLAN_N_VID)
	{
		vid_le = cpu_to_le16(i);
		// we change little ending to big ending here because the IP of
		// GMAC have some question.
		//		vid_le = htons(vid_le);
		vid[0] = vid_le & 0xff;
		vid[1] = vid_le >> 8;
		crc = ~ether_crc(VLAN_HLEN, vid) >> 28;
		vlan_hash_table |= (1 << crc);
	}
	writel(GMAC_VLAN_HASH_VALUE, priv->base + GMAC_VLAN_TAG);
	writel(vlan_hash_table, priv->base + GMAC_VLAN_HASH);
}

static int sgmac_vlan_rx_add_vid(struct net_device *ndev, __be16 proto, u16 vid)
{
	struct sgmac_priv *priv = netdev_priv(ndev);

	/* Packets with VID 0 are always received by Lancer by default */
	if (vid == 0 || vid > VLAN_N_VID)
		return 0;

	if (test_bit(vid, priv->vids))
		return 0;

	set_bit(vid, priv->vids);
	priv->vids_num++;

	sgmac_set_vid(priv);
	return 0;
}

static int
sgmac_vlan_rx_kill_vid(struct net_device *ndev, __be16 proto, u16 vid)
{
	struct sgmac_priv *priv = netdev_priv(ndev);

	/* Packets with VID 0 are always received by Lancer by default */
	if (vid == 0 || vid > VLAN_N_VID)
		return 0;

	if (!test_bit(vid, priv->vids))
		return 0;

	clear_bit(vid, priv->vids);
	priv->vids_num--;

	sgmac_set_vid(priv);
	return 0;
}

static void sgmac_get_stats64(struct net_device *dev,
                           struct rtnl_link_stats64 *storage)
{
	struct sgmac_priv *priv = netdev_priv(dev);
	struct rtnl_link_stats64 *netstats = &priv->netstats;
	storage->tx_packets = netstats->tx_packets;
	storage->tx_bytes = netstats->tx_bytes;
	storage->rx_packets = netstats->rx_packets;
	storage->rx_bytes = netstats->rx_bytes;
}

static int sgmac_do_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	struct sgmac_priv *priv = netdev_priv(ndev);
	int ret = -EOPNOTSUPP;

	if (!netif_running(ndev))
		return -EINVAL;

	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		if (!priv->phydev)
			return -EINVAL;
		ret = phy_mii_ioctl(priv->phydev, ifr, cmd);
		break;
#ifdef CONFIG_SFAX8_PTP
	case SIOCSHWTSTAMP:
		ret = sgmac_ptp_hwtstamp_ioctl(ndev, ifr);
		break;
#endif
	default:
		break;
	}

	return ret;
}

static const struct net_device_ops sgmac_netdev_ops = {
	.ndo_open = sgmac_open,
	.ndo_start_xmit = sgmac_xmit,
	.ndo_stop = sgmac_stop,
	.ndo_change_mtu = sgmac_change_mtu,
	.ndo_set_rx_mode = sgmac_set_rx_mode,
	.ndo_tx_timeout = sgmac_tx_timeout,
	.ndo_get_stats64 = sgmac_get_stats64,
	.ndo_do_ioctl = sgmac_do_ioctl,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = sgmac_poll_controller,
#endif
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = sgmac_set_mac_address,
	.ndo_vlan_rx_add_vid = sgmac_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid = sgmac_vlan_rx_kill_vid,
	.ndo_set_features = sgmac_set_features,
};

static void sgmac_ethtool_get_drvinfo(struct net_device *ndev,
		struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, "sgmac", sizeof(info->driver));
	strlcpy(info->version, "1.0", sizeof(info->version));
	strlcpy(info->fw_version, "DWC_gmac 3.73a", sizeof(info->fw_version));
}

static void sgmac_ethtool_get_pauseparam(struct net_device *ndev,
		struct ethtool_pauseparam *pause)
{
	struct sgmac_priv *priv = netdev_priv(ndev);
	pause->rx_pause = priv->rx_pause;
	pause->tx_pause = priv->tx_pause;
}

static int sgmac_ethtool_set_pauseparam(struct net_device *ndev,
		struct ethtool_pauseparam *pause)
{
	struct sgmac_priv *priv = netdev_priv(ndev);

	if (pause->autoneg)
		return -EINVAL;
	return sgmac_set_flow_ctrl(priv, pause->rx_pause, pause->tx_pause);
}

static int sgmac_ethtool_get_regs_len(struct net_device *ndev)
{
	return SGMAC_REG_SPACE_SIZE;
}

static void sgmac_ethtool_gregs(struct net_device *ndev,
		struct ethtool_regs *regs,
		void *regbuf)
{
	int i;
	u32 *reg_space = regbuf;
	struct sgmac_priv *priv = netdev_priv(ndev);
	u32 __iomem *ioaddr = priv->base;

	memset(reg_space, 0x0, SGMAC_REG_SPACE_SIZE);
	for (i = 0; i < SGMAC_REG_SPACE_SIZE; i += 4)
		memcpy_fromio(reg_space++, ioaddr++, 4);
}

static int sgmac_nway_reset(struct net_device *ndev)
{
	struct sgmac_priv *priv = netdev_priv(ndev);
	return phy_start_aneg(priv->phydev);
}

struct sgmac_stats {
	char stat_string[ETH_GSTRING_LEN];
	int stat_offset;
	bool is_reg;
};

// this is in struct sgmac_priv *priv.
#define SGMAC_STAT(m)                                                          \
	{                                                                      \
		#m, offsetof(struct sgmac_priv, xstats.m), false               \
	}
static const struct sgmac_stats sgmac_gstrings_stats[] = {
	SGMAC_STAT(tx_frame_flushed), SGMAC_STAT(tx_payload_error),
	SGMAC_STAT(tx_ip_header_error), SGMAC_STAT(tx_local_fault),
	SGMAC_STAT(tx_remote_fault), SGMAC_STAT(tx_early),
	SGMAC_STAT(tx_process_stopped), SGMAC_STAT(tx_jabber),
	SGMAC_STAT(rx_buf_unav), SGMAC_STAT(rx_process_stopped),
	SGMAC_STAT(rx_payload_error), SGMAC_STAT(rx_ip_header_error),
	SGMAC_STAT(rx_da_filter_fail), SGMAC_STAT(fatal_bus_error),
};

#define SGMAC_STATS_LEN ARRAY_SIZE(sgmac_gstrings_stats)

static void sgmac_ethtool_get_ethtool_stats(struct net_device *ndev,
		struct ethtool_stats *dummy,
		u64 *data)
{
	struct sgmac_priv *priv = netdev_priv(ndev);
	void *p = priv;
	int i;

	for (i = 0; i < SGMAC_STATS_LEN; i++) {
		if (sgmac_gstrings_stats[i].is_reg)
			*data++ = readl(priv->base +
					sgmac_gstrings_stats[i].stat_offset);
		else
			*data++ = *(u32 *)(p +
					   sgmac_gstrings_stats[i].stat_offset);
	}
}

static int sgmac_ethtool_get_sset_count(struct net_device *ndev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return SGMAC_STATS_LEN;
	default:
		return -EINVAL;
	}
}

static void
sgmac_ethtool_get_strings(struct net_device *ndev, u32 stringset, u8 *data)
{
	int i;
	u8 *p = data;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < SGMAC_STATS_LEN; i++) {
			memcpy(p, sgmac_gstrings_stats[i].stat_string,
					ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		break;
	default:
		WARN_ON(1);
		break;
	}
}

static void sgmac_ethtool_get_wol(struct net_device *ndev,
		struct ethtool_wolinfo *wol)
{
	struct sgmac_priv *priv = netdev_priv(ndev);

	if (device_can_wakeup(priv->dev)) {
		wol->supported = WAKE_MAGIC | WAKE_UCAST | WAKE_BCAST;
		wol->wolopts = priv->wolopts;
	}
}

static int sgmac_ethtool_set_wol(struct net_device *ndev,
		struct ethtool_wolinfo *wol)
{
	struct sgmac_priv *priv = netdev_priv(ndev);
	u32 support = WAKE_MAGIC | WAKE_UCAST | WAKE_BCAST;

	if (!device_can_wakeup(priv->dev))
		return -ENOTSUPP;

	if (wol->wolopts & ~support)
		return -EINVAL;

	priv->wolopts = wol->wolopts;

	if (wol->wolopts) {
		device_set_wakeup_enable(priv->dev, 1);
		enable_irq_wake(ndev->irq);
	} else {
		device_set_wakeup_enable(priv->dev, 0);
		disable_irq_wake(ndev->irq);
	}

	return 0;
}

static const struct ethtool_ops sgmac_ethtool_ops = {
	.get_drvinfo = sgmac_ethtool_get_drvinfo,
	.get_link = ethtool_op_get_link,
	.get_pauseparam = sgmac_ethtool_get_pauseparam,
	.set_pauseparam = sgmac_ethtool_set_pauseparam,
	.get_ethtool_stats = sgmac_ethtool_get_ethtool_stats,
	.get_strings = sgmac_ethtool_get_strings,
	.get_wol = sgmac_ethtool_get_wol,
	.set_wol = sgmac_ethtool_set_wol,
	.get_sset_count = sgmac_ethtool_get_sset_count,
	.get_regs_len = sgmac_ethtool_get_regs_len,
	.get_regs = sgmac_ethtool_gregs,
	.nway_reset = sgmac_nway_reset,
    .get_link_ksettings = phy_ethtool_get_link_ksettings,
    .set_link_ksettings = phy_ethtool_set_link_ksettings,
};

/**
 * sgmac_probe
 * @pdev: platform device pointer
 * Description: the driver is initialized through platform_device.
 */
static int set_sf_address(char * mac) {
	//10:16:88
	mac[0] = 0x10;
	mac[1] = 0x16;
	mac[2] = 0x88;
	return 0;
}

static int sgmac_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct net_device *ndev = NULL;
	struct sgmac_priv *priv = NULL;
	u32 uid, clkgate;
	struct device * p_dev = &pdev->dev;
#ifdef CONFIG_SMP
	struct cpumask irq_affi;
	unsigned int cpu = 0;
#endif

#ifdef CONFIG_SFAX8_RGMII_GMAC
	if(release_reset_with_value(SF_EMAC_SOFT_RESET, 1))
#else
	if(release_reset_with_value(SF_EMAC_SOFT_RESET, 0))
#endif
	  return -EFAULT;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	if (!request_mem_region(res->start, resource_size(res), pdev->name))
		return -EBUSY;

	ndev = alloc_etherdev(sizeof(struct sgmac_priv));
	if (!ndev) {
		ret = -ENOMEM;
		goto err_alloc;
	}
	ether_setup(ndev);

	//get mac addres from disk
#ifdef CONFIG_SFAX8_FACTORY_READ
	if(sf_get_value_from_factory(READ_WAN_MAC_ADDRESS, ndev->dev_addr, 6)){
#endif
		if(eth_platform_get_mac_address(p_dev, ndev->dev_addr) < 0){
			eth_hw_addr_random(ndev);
			set_sf_address(ndev->dev_addr);
		}
#ifdef CONFIG_SFAX8_FACTORY_READ
	}
#endif

	SET_NETDEV_DEV(ndev, &pdev->dev);
	priv = netdev_priv(ndev);
	memset(priv, 0, sizeof(struct sgmac_priv));
	platform_set_drvdata(pdev, ndev);
	ndev->netdev_ops = &sgmac_netdev_ops;
	spin_lock_init(&priv->stats_lock);
	INIT_WORK(&priv->tx_timeout_work, sgmac_tx_timeout_work);

	priv->dev = &pdev->dev;
	priv->ndev = ndev;
	priv->rx_pause = 0;
	priv->tx_pause = 0;

	of_property_read_string(p_dev->of_node, "gswitch", &priv->gswitch);
	if (!priv->gswitch)
		ndev->ethtool_ops = &sgmac_ethtool_ops;

	priv->base = ioremap(res->start, resource_size(res));
	if (!priv->base) {
		netdev_err(ndev, "ioremap failed\n");
		ret = -ENOMEM;
		goto err_io;
	}

#ifdef CONFIG_SFAX8_RGMII_GMAC
	priv->eth_byp_ref_clk = of_clk_get(priv->dev->of_node, 0);
	if (IS_ERR(priv->eth_byp_ref_clk)) {
		netdev_err(ndev, "unable to get eth_byp_ref_clk\n");
		ret = -EINVAL;
		goto err_byp_ref_clk;
	}
	ret = clk_prepare_enable(priv->eth_byp_ref_clk);
	if (ret) {
		netdev_err(ndev, "unable to enable eth_byp_ref_clk\n");
		goto err_byp_ref_clk;
	}
#else
	/* gating unused eth_byp_ref_clk */
	clkgate = get_module_clk_gate(SF_EMAC_SOFT_RESET, false);
	set_module_clk_gate(SF_EMAC_SOFT_RESET, clkgate & (~EMAC_ETH_BYPREF_CLK), false);
#endif

#ifdef CONFIG_SFAX8_PTP
	priv->eth_tsu_clk = of_clk_get(priv->dev->of_node, 1);
	if (IS_ERR(priv->eth_tsu_clk)) {
		netdev_err(ndev, "unable to get eth_tsu_clk\n");
		ret = -EINVAL;
		goto err_tsu_clk;
	}
	ret = clk_prepare_enable(priv->eth_tsu_clk);
	if (ret) {
		netdev_err(ndev, "unable to enable eth_tsu_clk\n");
		goto err_tsu_clk;
	}
#else
	/* gating unused eth_tsu_clk */
	clkgate = get_module_clk_gate(SF_EMAC_SOFT_RESET, false);
	set_module_clk_gate(SF_EMAC_SOFT_RESET, clkgate & (~EMAC_ETH_TSU_CLK), false);
#endif

	priv->eth_bus_clk = of_clk_get(priv->dev->of_node, 2);
	if (IS_ERR(priv->eth_bus_clk)) {
		netdev_err(ndev, "unable to get eth_bus_clk\n");
		ret = -EINVAL;
		goto err_bus_clk;
	}
	ret = clk_prepare_enable(priv->eth_bus_clk);
	if (ret) {
		netdev_err(ndev, "unable to enable eth_bus_clk\n");
		goto err_bus_clk;
	}

	uid = readl(priv->base + GMAC_VERSION);
	netdev_info(ndev, "h/w version is 0x%x\n", uid);

	/* Figure out how many valid mac address filter registers we
	 * have */
	// austin:we have 32 addr filter regs in A18
	// if we have different configs of GAMC, maybe we need it here.
	writel(1, priv->base + GMAC_ADDR_HIGH(31));
	if (readl(priv->base + GMAC_ADDR_HIGH(31)) == 1)
		priv->max_macs = 31;
	else
		priv->max_macs = 7;

	/* Get PHY from device tree */
	priv->phy_node = of_parse_phandle(priv->dev->of_node, "phy", 0);
	if (!priv->phy_node) {
		netdev_err(ndev, "failed to retrieve phy description "
				 "from device tree\n");
		ret = -ENODEV;
		goto err_phy;
	}

	writel(0, priv->base + GMAC_DMA_INTR_ENA);

	ndev->irq = platform_get_irq(pdev, 0);
	if (ndev->irq == -ENXIO) {
		netdev_err(ndev, "No irq resource\n");
		ret = ndev->irq;
		goto err_irq;
	}

	ret = request_irq(ndev->irq, sgmac_interrupt, 0, dev_name(&pdev->dev),
			ndev);
    netdev_info(ndev, "gmac request irq=%d ret=%d\n",ndev->irq, ret);
	if (ret < 0) {
		netdev_err(ndev, "Could not request irq %d - ret %d)\n",
				ndev->irq, ret);
		goto err_irq;
	}

	priv->pmt_irq = platform_get_irq(pdev, 1);
	ret = request_irq(priv->pmt_irq, sgmac_pmt_interrupt, 0,
			dev_name(&pdev->dev), ndev);
    netdev_info(ndev, "gmac request pmt irq=%d ret=%d\n", priv->pmt_irq, ret);
	if (ret < 0) {
		netdev_err(ndev, "Could not request irq %d - ret %d)\n",
				ndev->irq + 2, ret);
		goto err_pmt_irq;
	}

#ifdef CONFIG_SMP
	if (of_property_read_u32(p_dev->of_node, "smp-affinity", &cpu) == 0) {
		if (cpu < NR_CPUS) {
			memset(&irq_affi, 0, sizeof(struct cpumask));
			cpumask_set_cpu(cpu, &irq_affi);
			if (irq_set_affinity_hint(ndev->irq, &irq_affi))
				netdev_err(ndev, "can not set the affinity for irq : %d\n", ndev->irq);
			if (irq_set_affinity_hint(ndev->irq + 2, &irq_affi))
				netdev_err(ndev, "can not set the affinity for irq : %d\n", ndev->irq);
		}
	}
#endif

	device_set_wakeup_capable(&pdev->dev, 1);
	if (device_can_wakeup(priv->dev))
		priv->wolopts = WAKE_MAGIC; /* Magic Frame as default */

	// austin: we can change ndev->hw_features, ndev->features and
	// ndev->priv_flags to control sth.
	ndev->hw_features =
			NETIF_F_SG | NETIF_F_HIGHDMA;
	// austin: we have these features in A18.
	// if we have different configs of GMAC, maybe we need it here.
	if (readl(priv->base + GMAC_DMA_HW_FEATURE) & DMA_HW_FEAT_TXCOESEL)
		ndev->hw_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
				     NETIF_F_RXCSUM;
	ndev->features |= ndev->hw_features;
	ndev->vlan_features |= ndev->hw_features;
	ndev->priv_flags |= IFF_UNICAST_FLT;

	sgmac_set_mdc_clk_div(priv);
	ret = sgmac_mdio_probe(priv);
	if (ret < 0) {
		netdev_err(ndev, "failed to probe MII bus\n");
		goto err_mdio;
	}

#ifdef CONFIG_SFAX8_PTP
	ret = sgmac_ptp_register(priv);
	if (ret < 0) {
		netdev_err(ndev, "failed to ptp register\n");
		goto err_ptp;
	}
#endif
	netif_napi_add(ndev, &priv->napi, sgmac_poll, 64);
	ret = register_netdev(ndev);
	if (ret)
		goto err_reg;

	return 0;

err_reg:
	netif_napi_del(&priv->napi);
#ifdef CONFIG_SFAX8_PTP
	sgmac_ptp_unregister(priv);
err_ptp:
#endif
	sgmac_mdio_remove(priv);
err_mdio:
#ifdef CONFIG_SMP
    irq_set_affinity_hint(ndev->irq + 2, NULL);
#endif
	free_irq(priv->pmt_irq, ndev);
err_pmt_irq:
#ifdef CONFIG_SMP
    irq_set_affinity_hint(ndev->irq, NULL);
#endif
	free_irq(ndev->irq, ndev);
err_irq:
	of_node_put(priv->phy_node);
err_phy:
	clk_disable_unprepare(priv->eth_bus_clk);
err_bus_clk:
#ifdef CONFIG_SFAX8_PTP
	clk_disable_unprepare(priv->eth_tsu_clk);
err_tsu_clk:
#endif
#ifdef CONFIG_SFAX8_RGMII_GMAC
	clk_disable_unprepare(priv->eth_byp_ref_clk);
err_byp_ref_clk:
#endif
	iounmap(priv->base);
err_io:
	free_netdev(ndev);
err_alloc:
	release_mem_region(res->start, resource_size(res));
	return ret;
}

/**
 * sgmac_remove
 * @pdev: platform device pointer
 * Description: this function resets the TX/RX processes, disables the
 * MAC RX/TX
 * changes the link status, releases the DMA descriptor rings,
 * unregisters the MDIO bus and unmaps the allocated memory.
 */
static int sgmac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct sgmac_priv *priv = netdev_priv(ndev);
	struct resource *res;

	sgmac_mac_disable(priv);
#ifdef CONFIG_SFAX8_PTP
	sgmac_ptp_unregister(priv);
#endif

	/* Free the IRQ lines */
#ifdef CONFIG_SMP
    irq_set_affinity_hint(ndev->irq, NULL);
    irq_set_affinity_hint(ndev->irq + 2, NULL);
#endif
	free_irq(ndev->irq, ndev);
	free_irq(priv->pmt_irq, ndev);

	unregister_netdev(ndev);
	//make sure remove mdio after sgmac_stop case we should not access phydev->drv after mdiobus_unregister case it will NULL phydev->drv after that
	sgmac_mdio_remove(priv);
	netif_napi_del(&priv->napi);

	of_node_put(priv->phy_node);
#ifdef CONFIG_SFAX8_RGMII_GMAC
	clk_disable_unprepare(priv->eth_byp_ref_clk);
#endif
#ifdef CONFIG_SFAX8_PTP
	clk_disable_unprepare(priv->eth_tsu_clk);
#endif
	clk_disable_unprepare(priv->eth_bus_clk);

	iounmap(priv->base);
	free_netdev(ndev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static void sgmac_pmt(struct sgmac_priv *priv, unsigned long mode)
{
	unsigned int pmt = 0;

	if (mode & WAKE_MAGIC)
		pmt |= GMAC_PMT_PWRDWN | GMAC_PMT_MGKPKTEN;
	if (mode & WAKE_UCAST)
		pmt |= GMAC_PMT_PWRDWN | GMAC_PMT_GLBLUCAST;

	writel(pmt, priv->base + GMAC_PMT);
}

static int sgmac_suspend(struct device *dev)
{
	struct net_device *ndev = platform_get_drvdata(to_platform_device(dev));
	struct sgmac_priv *priv = netdev_priv(ndev);
	u32 value;

	if (!ndev || !netif_running(ndev))
		return 0;

	netif_device_detach(ndev);
	napi_disable(&priv->napi);
	writel(0, priv->base + GMAC_DMA_INTR_ENA);

	if (device_may_wakeup(priv->dev)) {
		/* Stop TX/RX DMA Only */
		value = readl(priv->base + GMAC_DMA_OPERATION);
		value &= ~(DMA_OPERATION_ST | DMA_OPERATION_SR);
		writel(value, priv->base + GMAC_DMA_OPERATION);

		sgmac_pmt(priv, priv->wolopts);
	} else
		sgmac_mac_disable(priv);

#ifdef CONFIG_SFAX8_RGMII_GMAC
	if (!IS_ERR(priv->eth_byp_ref_clk))
		clk_disable_unprepare(priv->eth_byp_ref_clk);
#endif
#ifdef CONFIG_SFAX8_PTP
	if (!IS_ERR(priv->eth_tsu_clk))
		clk_disable_unprepare(priv->eth_tsu_clk);
#endif

	return 0;
}

static int sgmac_resume(struct device *dev)
{
	struct net_device *ndev = platform_get_drvdata(to_platform_device(dev));
	struct sgmac_priv *priv = netdev_priv(ndev);
	int ret = 0;

	if (!netif_running(ndev))
		return 0;

#ifdef CONFIG_SFAX8_RGMII_GMAC
	ret = clk_prepare_enable(priv->eth_byp_ref_clk);
	if (ret)
		return ret;
#endif
#ifdef CONFIG_SFAX8_PTP
	ret = clk_prepare_enable(priv->eth_tsu_clk);
	if (ret)
		return ret;
#endif

	sgmac_pmt(priv, 0);

	/* Enable the MAC and DMA */
	sgmac_mac_enable(priv);
	writel(DMA_INTR_DEFAULT_MASK, priv->base + GMAC_DMA_STATUS);
	writel(DMA_INTR_DEFAULT_MASK, priv->base + GMAC_DMA_INTR_ENA);

	netif_device_attach(ndev);
	napi_enable(&priv->napi);


	return ret;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(sgmac_pm_ops, sgmac_suspend, sgmac_resume);

static const struct of_device_id sgmac_of_match[] = {
	{
		.compatible = "siflower,sfax8-gmac",
	},
	{},
};
MODULE_DEVICE_TABLE(of, sgmac_of_match);

static struct platform_driver sgmac_driver = {
	.driver = {
		.name = "sf_gmac",
		.of_match_table = sgmac_of_match,
	},
	.probe = sgmac_probe,
	.remove = sgmac_remove,
	.driver.pm = &sgmac_pm_ops,
};

static void __exit exit_sgmac(void)
{
	platform_driver_unregister(&sgmac_driver);
	hold_reset(SF_EMAC_SOFT_RESET);
	return;
}

static int __init init_sgmac(void)
{
	int ret = platform_driver_register(&sgmac_driver);
	return ret;
}

module_init(init_sgmac);
module_exit(exit_sgmac);

MODULE_AUTHOR("austin, <austin.xu@siflower.com.cn>");
#ifdef CONFIG_SFAX8_RGMII_GMAC
MODULE_DESCRIPTION("Siflower 1000M RGMII GMAC driver");
#else
MODULE_DESCRIPTION("Siflower 100M RMII GMAC driver");
#endif
MODULE_LICENSE("GPL v2");
