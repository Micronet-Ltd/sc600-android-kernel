/*
 * CAN bus driver for Microchip 251x/25625 CAN Controller with SPI Interface
 *
 * MCP2510 support and bug fixes by Christian Pellegrin
 * <chripell@evolware.org>
 *
 * Copyright 2009 Christian Pellegrin EVOL S.r.l.
 *
 * Copyright 2007 Raymarine UK, Ltd. All Rights Reserved.
 * Written under contract by:
 *   Chris Elston, Katalix Systems, Ltd.
 *
 * Based on Microchip MCP251x CAN controller driver written by
 * David Vrabel, Copyright 2006 Arcom Control Systems Ltd.
 *
 * Based on CAN bus driver for the CCAN controller written by
 * - Sascha Hauer, Marc Kleine-Budde, Pengutronix
 * - Simon Kallweit, intefo AG
 * Copyright 2007
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 *
 *
 * Your platform definition file should specify something like:
 *
 * static struct mcp251x_platform_data mcp251x_info = {
 *         .oscillator_frequency = 8000000,
 * };
 *
 * static struct spi_board_info spi_board_info[] = {
 *         {
 *                 .modalias = "mcp2510",
 *			// "mcp2515" or "mcp25625" depending on your controller
 *                 .platform_data = &mcp251x_info,
 *                 .irq = IRQ_EINT13,
 *                 .max_speed_hz = 2*1000*1000,
 *                 .chip_select = 2,
 *         },
 * };
 *
 * Please see mcp251x.h for a description of the fields in
 * struct mcp251x_platform_data.
 *
 */

//#define pr_fmt(fmt) "%s: " fmt, __func__
#include <linux/can/core.h>
#include <linux/can/dev.h>
#include <linux/can/led.h>
#include <linux/can/platform/mcp251x.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>

/* SPI interface instruction set */
#define INSTRUCTION_WRITE	0x02
#define INSTRUCTION_READ	0x03
#define INSTRUCTION_BIT_MODIFY	0x05
#define INSTRUCTION_LOAD_TXB(n)	(0x40 + 2 * (n))
#define INSTRUCTION_READ_RXB(n)	(((n) == 0) ? 0x90 : 0x94)
#define INSTRUCTION_RESET	0xC0
#define RTS_TXB0		0x01
#define RTS_TXB1		0x02
#define RTS_TXB2		0x04
#define INSTRUCTION_RTS(n)	(0x80 | ((n) & 0x07))


/* MPC251x registers */
#define CANSTAT	      0x0e
#define CANCTRL	      0x0f
#  define CANCTRL_REQOP_MASK	    0xe0
#  define CANCTRL_REQOP_CONF	    0x80
#  define CANCTRL_REQOP_LISTEN_ONLY 0x60
#  define CANCTRL_REQOP_LOOPBACK    0x40
#  define CANCTRL_REQOP_SLEEP	    0x20
#  define CANCTRL_REQOP_NORMAL	    0x00
#  define CANCTRL_OSM		    0x08
#  define CANCTRL_ABAT		    0x10
#define TEC	      0x1c
#define REC	      0x1d
#define CNF1	      0x2a
#  define CNF1_SJW_SHIFT   6
#define CNF2	      0x29
#  define CNF2_BTLMODE	   0x80
#  define CNF2_SAM         0x40
#  define CNF2_PS1_SHIFT   3
#define CNF3	      0x28
#  define CNF3_SOF	   0x08
#  define CNF3_WAKFIL	   0x04
#  define CNF3_PHSEG2_MASK 0x07
#define CANINTE	      0x2b
#  define CANINTE_MERRE 0x80
#  define CANINTE_WAKIE 0x40
#  define CANINTE_ERRIE 0x20
#  define CANINTE_TX2IE 0x10
#  define CANINTE_TX1IE 0x08
#  define CANINTE_TX0IE 0x04
#  define CANINTE_RX1IE 0x02
#  define CANINTE_RX0IE 0x01
#define CANINTF	      0x2c
#  define CANINTF_MERRF 0x80
#  define CANINTF_WAKIF 0x40
#  define CANINTF_ERRIF 0x20
#  define CANINTF_TX2IF 0x10
#  define CANINTF_TX1IF 0x08
#  define CANINTF_TX0IF 0x04
#  define CANINTF_RX1IF 0x02
#  define CANINTF_RX0IF 0x01
#  define CANINTF_RX (CANINTF_RX0IF | CANINTF_RX1IF)
#  define CANINTF_TX (CANINTF_TX2IF | CANINTF_TX1IF | CANINTF_TX0IF)
#  define CANINTF_ERR (CANINTF_ERRIF)
#define EFLG	      0x2d
#  define EFLG_EWARN	0x01
#  define EFLG_RXWAR	0x02
#  define EFLG_TXWAR	0x04
#  define EFLG_RXEP	0x08
#  define EFLG_TXEP	0x10
#  define EFLG_TXBO	0x20
#  define EFLG_RX0OVR	0x40
#  define EFLG_RX1OVR	0x80
#define TXBCTRL(n)  (((n) * 0x10) + 0x30 + TXBCTRL_OFF)
#  define TXBCTRL_ABTF	0x40
#  define TXBCTRL_MLOA	0x20
#  define TXBCTRL_TXERR 0x10
#  define TXBCTRL_TXREQ 0x08
#define TXBSIDH(n)  (((n) * 0x10) + 0x30 + TXBSIDH_OFF)
#  define SIDH_SHIFT    3
#define TXBSIDL(n)  (((n) * 0x10) + 0x30 + TXBSIDL_OFF)
#  define SIDL_SID_MASK    7
#  define SIDL_SID_SHIFT   5
#  define SIDL_EXIDE_SHIFT 3
#  define SIDL_EID_SHIFT   16
#  define SIDL_EID_MASK    3
#define TXBEID8(n)  (((n) * 0x10) + 0x30 + TXBEID8_OFF)
#define TXBEID0(n)  (((n) * 0x10) + 0x30 + TXBEID0_OFF)
#define TXBDLC(n)   (((n) * 0x10) + 0x30 + TXBDLC_OFF)
#  define DLC_RTR_SHIFT    6
#define TXBCTRL_OFF 0
#define TXBSIDH_OFF 1
#define TXBSIDL_OFF 2
#define TXBEID8_OFF 3
#define TXBEID0_OFF 4
#define TXBDLC_OFF  5
#define TXBDAT_OFF  6
#define RXBCTRL(n)  (((n) * 0x10) + 0x60 + RXBCTRL_OFF)
#  define RXBCTRL_BUKT	0x04
#  define RXBCTRL_RXM0	0x20
#  define RXBCTRL_RXM1	0x40
#define RXBSIDH(n)  (((n) * 0x10) + 0x60 + RXBSIDH_OFF)
#  define RXBSIDH_SHIFT 3
#define RXBSIDL(n)  (((n) * 0x10) + 0x60 + RXBSIDL_OFF)
#  define RXBSIDL_IDE   0x08
#  define RXBSIDL_SRR   0x10
#  define RXBSIDL_EID   3
#  define RXBSIDL_SHIFT 5
#define RXBEID8(n)  (((n) * 0x10) + 0x60 + RXBEID8_OFF)
#define RXBEID0(n)  (((n) * 0x10) + 0x60 + RXBEID0_OFF)
#define RXBDLC(n)   (((n) * 0x10) + 0x60 + RXBDLC_OFF)
#  define RXBDLC_LEN_MASK  0x0f
#  define RXBDLC_RTR       0x40
#define RXBCTRL_OFF 0
#define RXBSIDH_OFF 1
#define RXBSIDL_OFF 2
#define RXBEID8_OFF 3
#define RXBEID0_OFF 4
#define RXBDLC_OFF  5
#define RXBDAT_OFF  6
#define RXFSID(n) ((n < 3) ? 0 : 4)
#define RXFSIDH(n) ((n) * 4 + RXFSID(n))
#define RXFSIDL(n) ((n) * 4 + 1 + RXFSID(n))
#define RXFEID8(n) ((n) * 4 + 2 + RXFSID(n))
#define RXFEID0(n) ((n) * 4 + 3 + RXFSID(n))
#define RXMSIDH(n) ((n) * 4 + 0x20)
#define RXMSIDL(n) ((n) * 4 + 0x21)
#define RXMEID8(n) ((n) * 4 + 0x22)
#define RXMEID0(n) ((n) * 4 + 0x23)

#define GET_BYTE(val, byte)			\
	(((val) >> ((byte) * 8)) & 0xff)
#define SET_BYTE(val, byte)			\
	(((val) & 0xff) << ((byte) * 8))

/*
 * Buffer size required for the largest SPI transfer (i.e., reading a
 * frame)
 */
#define CAN_FRAME_MAX_DATA_LEN	8
#define SPI_TRANSFER_BUF_LEN	(6 + CAN_FRAME_MAX_DATA_LEN)
#define CAN_FRAME_MAX_BITS	128

#define TX_ECHO_SKB_MAX	1

#define MCP251X_OST_DELAY_MS	(5)

#define DEVICE_NAME "mcp251x"

static int mcp251x_enable_dma = 0; /* Enable SPI DMA. Default: 0 (Off) */
module_param(mcp251x_enable_dma, int, S_IRUGO);
MODULE_PARM_DESC(mcp251x_enable_dma, "Enable SPI DMA. Default: 0 (Off)");

static const struct can_bittiming_const mcp251x_bittiming_const = {
	.name = DEVICE_NAME,
	.tseg1_min = 3,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,
};

enum mcp251x_model {
	CAN_MCP251X_MCP2510	= 0x2510,
	CAN_MCP251X_MCP2515	= 0x2515,
	CAN_MCP251X_MCP25625	= 0x25625,
};

//#define MAX_QUE 0
#define MAX_QUE 2056
#if MAX_QUE
struct frame_queue {
    int head;
    int tail;
    spinlock_t frame_queue_lock;
    struct mutex que_lock;
    struct can_frame rx_frame[MAX_QUE];
};
#endif

struct mcp251x_priv {
	struct can_priv	   can;
	struct net_device *net;
	struct spi_device *spi;
	enum mcp251x_model model;

	struct mutex mcp_lock; /* SPI device lock */

	u8 *spi_tx_buf;
	u8 *spi_rx_buf;
	dma_addr_t spi_tx_dma;
	dma_addr_t spi_rx_dma;

	struct sk_buff *tx_skb;
	int tx_len;

	struct workqueue_struct *wq;
	struct work_struct tx_work;
	struct work_struct restart_work;

	int force_quit;
	int after_suspend;
#define AFTER_SUSPEND_UP 1
#define AFTER_SUSPEND_DOWN 2
#define AFTER_SUSPEND_POWER 4
#define AFTER_SUSPEND_RESTART 8
	int restart_tx;
	struct regulator *power;
	struct regulator *transceiver;
	struct clk *clk;
    struct mcp251x_platform_data *pdata;
#if MAX_QUE
    struct frame_queue rx_frames_q;
    struct work_struct q_work;
#endif
};

#define MCP251X_IS(_model) \
static inline int mcp251x_is_##_model(struct spi_device *spi) \
{ \
	struct mcp251x_priv *priv = spi_get_drvdata(spi); \
	return priv->model == CAN_MCP251X_MCP##_model; \
}

MCP251X_IS(2510);

static void mcp251x_clean(struct net_device *net)
{
	struct mcp251x_priv *priv = netdev_priv(net);

	if (priv->tx_skb || priv->tx_len)
		net->stats.tx_errors++;
	if (priv->tx_skb)
		dev_kfree_skb(priv->tx_skb);
	if (priv->tx_len)
		can_free_echo_skb(priv->net, 0);
	priv->tx_skb = NULL;
	priv->tx_len = 0;
}

/*
 * Note about handling of error return of mcp251x_spi_trans: accessing
 * registers via SPI is not really different conceptually than using
 * normal I/O assembler instructions, although it's much more
 * complicated from a practical POV. So it's not advisable to always
 * check the return value of this function. Imagine that every
 * read{b,l}, write{b,l} and friends would be bracketed in "if ( < 0)
 * error();", it would be a great mess (well there are some situation
 * when exception handling C++ like could be useful after all). So we
 * just check that transfers are OK at the beginning of our
 * conversation with the chip and to avoid doing really nasty things
 * (like injecting bogus packets in the network stack).
 */
static int mcp251x_spi_trans(struct spi_device *spi, int len)
{
	struct mcp251x_priv *priv = spi_get_drvdata(spi);
	struct spi_transfer t = {
		.tx_buf = priv->spi_tx_buf,
		.rx_buf = priv->spi_rx_buf,
		.len = len,
		.cs_change = 0,
	};
	struct spi_message m;
	int ret;

	spi_message_init(&m);

	if (mcp251x_enable_dma) {
		t.tx_dma = priv->spi_tx_dma;
		t.rx_dma = priv->spi_rx_dma;
		m.is_dma_mapped = 1;
	}

	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);
	if (ret)
		dev_err(&spi->dev, "spi transfer failed: ret = %d\n", ret);
	return ret;
}

static u8 mcp251x_read_reg(struct spi_device *spi, uint8_t reg)
{
	struct mcp251x_priv *priv = spi_get_drvdata(spi);
	u8 val = 0;

	priv->spi_tx_buf[0] = INSTRUCTION_READ;
	priv->spi_tx_buf[1] = reg;

	mcp251x_spi_trans(spi, 3);
	val = priv->spi_rx_buf[2];

	return val;
}

static void mcp251x_read_2regs(struct spi_device *spi, uint8_t reg,
		uint8_t *v1, uint8_t *v2)
{
	struct mcp251x_priv *priv = spi_get_drvdata(spi);

	priv->spi_tx_buf[0] = INSTRUCTION_READ;
	priv->spi_tx_buf[1] = reg;

	mcp251x_spi_trans(spi, 4);

	*v1 = priv->spi_rx_buf[2];
	*v2 = priv->spi_rx_buf[3];
}

static void mcp251x_write_reg(struct spi_device *spi, u8 reg, uint8_t val)
{
	struct mcp251x_priv *priv = spi_get_drvdata(spi);

	priv->spi_tx_buf[0] = INSTRUCTION_WRITE;
	priv->spi_tx_buf[1] = reg;
	priv->spi_tx_buf[2] = val;

	mcp251x_spi_trans(spi, 3);
}

static void mcp251x_write_bits(struct spi_device *spi, u8 reg,
			       u8 mask, uint8_t val)
{
	struct mcp251x_priv *priv = spi_get_drvdata(spi);

	priv->spi_tx_buf[0] = INSTRUCTION_BIT_MODIFY;
	priv->spi_tx_buf[1] = reg;
	priv->spi_tx_buf[2] = mask;
	priv->spi_tx_buf[3] = val;

	mcp251x_spi_trans(spi, 4);
}

static void mcp251x_hw_tx_frame(struct spi_device *spi, u8 *buf,
				int len, int tx_buf_idx)
{
	struct mcp251x_priv *priv = spi_get_drvdata(spi);

	if (mcp251x_is_2510(spi)) {
		int i;

		for (i = 1; i < TXBDAT_OFF + len; i++)
			mcp251x_write_reg(spi, TXBCTRL(tx_buf_idx) + i,
					  buf[i]);
	} else {
		memcpy(priv->spi_tx_buf, buf, TXBDAT_OFF + len);
		mcp251x_spi_trans(spi, TXBDAT_OFF + len);
	}
}

static void mcp251x_hw_tx(struct spi_device *spi, struct can_frame *frame,
			  int tx_buf_idx)
{
	struct mcp251x_priv *priv = spi_get_drvdata(spi);
	u32 sid, eid, exide, rtr;
	u8 buf[SPI_TRANSFER_BUF_LEN];

	exide = (frame->can_id & CAN_EFF_FLAG) ? 1 : 0; /* Extended ID Enable */
	if (exide)
		sid = (frame->can_id & CAN_EFF_MASK) >> 18;
	else
		sid = frame->can_id & CAN_SFF_MASK; /* Standard ID */
	eid = frame->can_id & CAN_EFF_MASK; /* Extended ID */
	rtr = (frame->can_id & CAN_RTR_FLAG) ? 1 : 0; /* Remote transmission */

	buf[TXBCTRL_OFF] = INSTRUCTION_LOAD_TXB(tx_buf_idx);
	buf[TXBSIDH_OFF] = sid >> SIDH_SHIFT;
	buf[TXBSIDL_OFF] = ((sid & SIDL_SID_MASK) << SIDL_SID_SHIFT) |
		(exide << SIDL_EXIDE_SHIFT) |
		((eid >> SIDL_EID_SHIFT) & SIDL_EID_MASK);
	buf[TXBEID8_OFF] = GET_BYTE(eid, 1);
	buf[TXBEID0_OFF] = GET_BYTE(eid, 0);
	buf[TXBDLC_OFF] = (rtr << DLC_RTR_SHIFT) | frame->can_dlc;
	memcpy(buf + TXBDAT_OFF, frame->data, frame->can_dlc);
	mcp251x_hw_tx_frame(spi, buf, frame->can_dlc, tx_buf_idx);

	/* use INSTRUCTION_RTS, to avoid "repeated frame problem" */
	priv->spi_tx_buf[0] = INSTRUCTION_RTS(1 << tx_buf_idx);
	mcp251x_spi_trans(priv->spi, 1);
}

static void mcp251x_hw_rx_frame(struct spi_device *spi, u8 *buf,
				int buf_idx)
{
	struct mcp251x_priv *priv = spi_get_drvdata(spi);

	if (mcp251x_is_2510(spi)) {
		int i, len;

		for (i = 1; i < RXBDAT_OFF; i++)
			buf[i] = mcp251x_read_reg(spi, RXBCTRL(buf_idx) + i);

		len = get_can_dlc(buf[RXBDLC_OFF] & RXBDLC_LEN_MASK);
		for (; i < (RXBDAT_OFF + len); i++)
			buf[i] = mcp251x_read_reg(spi, RXBCTRL(buf_idx) + i);
	} else {
		priv->spi_tx_buf[RXBCTRL_OFF] = INSTRUCTION_READ_RXB(buf_idx);
		mcp251x_spi_trans(spi, SPI_TRANSFER_BUF_LEN);
		memcpy(buf, priv->spi_rx_buf, SPI_TRANSFER_BUF_LEN);
	}
}

#if MAX_QUE
int frames_queue_empty(struct frame_queue *queue)
{
    return queue->head == queue->tail;
}

struct can_frame *frames_queue_get(struct frame_queue *queue)
{
    queue->tail = (queue->tail + 1) % MAX_QUE;
    return &(queue->rx_frame[queue->tail]);
}
 
int frames_queue_put(struct frame_queue *queue, struct can_frame *frame)
{
    int full = 0;

    if (queue->head >= queue->tail) {
        full = queue->head - queue->tail;
    } else {
        full = sizeof(queue->rx_frame)/sizeof(queue->rx_frame[0]) - queue->tail + queue->head;
    }

    if (full < 2*MAX_QUE/3) {
        full = 0; 
    }

    queue->head = (queue->head + 1) % MAX_QUE; 
    if(queue->head == queue->tail) {
        queue->tail = (queue->tail + 1) % MAX_QUE;
        full = MAX_QUE;
    }
 
    if (frame) {
        memset(&queue->rx_frame[queue->head], 0, sizeof(*frame));
        memcpy(&queue->rx_frame[queue->head], frame, sizeof(*frame));
    }

    return full;
}

static int mcp251x_hw_rx(struct spi_device *spi, int buf_idx)
{
    unsigned long flags;
    int full;
	struct mcp251x_priv *priv = spi_get_drvdata(spi);
	struct can_frame frame;
	u8 buf[SPI_TRANSFER_BUF_LEN];

	mcp251x_hw_rx_frame(spi, buf, buf_idx);
	if (buf[RXBSIDL_OFF] & RXBSIDL_IDE) {
		/* Extended ID format */
		frame.can_id = CAN_EFF_FLAG;
		frame.can_id |=
			/* Extended ID part */
			SET_BYTE(buf[RXBSIDL_OFF] & RXBSIDL_EID, 2) |
			SET_BYTE(buf[RXBEID8_OFF], 1) |
			SET_BYTE(buf[RXBEID0_OFF], 0) |
			/* Standard ID part */
			(((buf[RXBSIDH_OFF] << RXBSIDH_SHIFT) |
			  (buf[RXBSIDL_OFF] >> RXBSIDL_SHIFT)) << 18);
		/* Remote transmission request */
		if (buf[RXBDLC_OFF] & RXBDLC_RTR)
			frame.can_id |= CAN_RTR_FLAG;
	} else {
		/* Standard ID format */
		frame.can_id =
			(buf[RXBSIDH_OFF] << RXBSIDH_SHIFT) |
			(buf[RXBSIDL_OFF] >> RXBSIDL_SHIFT);
		if (buf[RXBSIDL_OFF] & RXBSIDL_SRR)
			frame.can_id |= CAN_RTR_FLAG;
	}
	/* Data length */
	frame.can_dlc = get_can_dlc(buf[RXBDLC_OFF] & RXBDLC_LEN_MASK);
	memcpy(frame.data, buf + RXBDAT_OFF, frame.can_dlc);

	can_led_event(priv->net, CAN_LED_EVENT_RX);

    spin_lock_irqsave(&priv->rx_frames_q.frame_queue_lock, flags);
    full = frames_queue_put(&priv->rx_frames_q, &frame);
    if (MAX_QUE == full) {
        priv->net->stats.rx_dropped++;
    }
    spin_unlock_irqrestore(&priv->rx_frames_q.frame_queue_lock, flags);
    if (full) {
        queue_work(priv->wq, &priv->q_work);
        //dev_notice(&spi->dev, "frame que is mostly full [%d] %lld\n", full, ktime_to_us(ktime_get()));
    }

    return full; 
}

static void mcp251x_hw_rx_skb(struct spi_device *spi)
{
    unsigned long flags;
	struct mcp251x_priv *priv = spi_get_drvdata(spi);
	struct sk_buff *skb;
	struct can_frame *frame, *frame_q;

    while (!frames_queue_empty(&priv->rx_frames_q)) {
        spin_lock_irqsave(&priv->rx_frames_q.frame_queue_lock, flags);
        frame_q = frames_queue_get(&priv->rx_frames_q);
        spin_unlock_irqrestore(&priv->rx_frames_q.frame_queue_lock, flags);

        skb = alloc_can_skb(priv->net, &frame);
        if (!skb) {
            dev_err(&spi->dev, "cannot allocate RX skb\n");
            priv->net->stats.rx_dropped++;
            return;
        }
        frame->can_id = frame_q->can_id;
        /* Data length */
        frame->can_dlc = frame_q->can_dlc;
        memcpy(frame->data, frame_q->data, frame_q->can_dlc);

        priv->net->stats.rx_packets++;
        priv->net->stats.rx_bytes += frame->can_dlc;

        netif_rx_ni(skb);
    }
}
#else

static void mcp251x_hw_rx(struct spi_device *spi, int buf_idx)
{
	struct mcp251x_priv *priv = spi_get_drvdata(spi);
	struct sk_buff *skb;
	struct can_frame *frame;
	u8 buf[SPI_TRANSFER_BUF_LEN];

	skb = alloc_can_skb(priv->net, &frame);
	if (!skb) {
		dev_err(&spi->dev, "cannot allocate RX skb\n");
		priv->net->stats.rx_dropped++;
		return;
	}

	mcp251x_hw_rx_frame(spi, buf, buf_idx);
	if (buf[RXBSIDL_OFF] & RXBSIDL_IDE) {
		/* Extended ID format */
		frame->can_id = CAN_EFF_FLAG;
		frame->can_id |=
			/* Extended ID part */
			SET_BYTE(buf[RXBSIDL_OFF] & RXBSIDL_EID, 2) |
			SET_BYTE(buf[RXBEID8_OFF], 1) |
			SET_BYTE(buf[RXBEID0_OFF], 0) |
			/* Standard ID part */
			(((buf[RXBSIDH_OFF] << RXBSIDH_SHIFT) |
			  (buf[RXBSIDL_OFF] >> RXBSIDL_SHIFT)) << 18);
		/* Remote transmission request */
		if (buf[RXBDLC_OFF] & RXBDLC_RTR)
			frame->can_id |= CAN_RTR_FLAG;
	} else {
		/* Standard ID format */
		frame->can_id =
			(buf[RXBSIDH_OFF] << RXBSIDH_SHIFT) |
			(buf[RXBSIDL_OFF] >> RXBSIDL_SHIFT);
		if (buf[RXBSIDL_OFF] & RXBSIDL_SRR)
			frame->can_id |= CAN_RTR_FLAG;
	}
	/* Data length */
	frame->can_dlc = get_can_dlc(buf[RXBDLC_OFF] & RXBDLC_LEN_MASK);
	memcpy(frame->data, buf + RXBDAT_OFF, frame->can_dlc);

	priv->net->stats.rx_packets++;
	priv->net->stats.rx_bytes += frame->can_dlc;

	can_led_event(priv->net, CAN_LED_EVENT_RX);

	netif_rx_ni(skb);
}
#endif

static void mcp251x_hw_sleep(struct spi_device *spi)
{
	mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_SLEEP);
}

static netdev_tx_t mcp251x_hard_start_xmit(struct sk_buff *skb,
					   struct net_device *net)
{
	struct mcp251x_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;

	if (priv->tx_skb || priv->tx_len) {
		dev_warn(&spi->dev, "hard_xmit called while tx busy\n");
		return NETDEV_TX_BUSY;
	}

	if (can_dropped_invalid_skb(net, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(net);
	priv->tx_skb = skb;
	queue_work(priv->wq, &priv->tx_work);

	return NETDEV_TX_OK;
}

static int mcp251x_do_set_mode(struct net_device *net, enum can_mode mode)
{
	struct mcp251x_priv *priv = netdev_priv(net);

	switch (mode) {
	case CAN_MODE_START:
		mcp251x_clean(net);
		/* We have to delay work since SPI I/O may sleep */
		priv->can.state = CAN_STATE_ERROR_ACTIVE;
		priv->restart_tx = 1;
		if (priv->can.restart_ms == 0)
			priv->after_suspend = AFTER_SUSPEND_RESTART;
		queue_work(priv->wq, &priv->restart_work);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int mcp251x_set_normal_mode(struct spi_device *spi)
{
	struct mcp251x_priv *priv = spi_get_drvdata(spi);
	unsigned long timeout;

	/* Enable interrupts */
	mcp251x_write_reg(spi, CANINTE,
			  CANINTE_ERRIE | CANINTE_TX2IE | CANINTE_TX1IE |
			  CANINTE_TX0IE | CANINTE_RX1IE | CANINTE_RX0IE);

	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) {
		/* Put device into loopback mode */
		mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_LOOPBACK);
	} else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY) {
		/* Put device into listen-only mode */
		mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_LISTEN_ONLY);
	} else {
		/* Put device into normal mode */
		mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_NORMAL);

		/* Wait for the device to enter normal mode */
		timeout = jiffies + HZ;
		while (mcp251x_read_reg(spi, CANSTAT) & CANCTRL_REQOP_MASK) {
			schedule();
			if (time_after(jiffies, timeout)) {
				dev_err(&spi->dev, "MCP251x didn't"
					" enter in normal mode\n");
				return -EBUSY;
			}
		}
	}
	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	return 0;
}

static int mcp251x_do_set_bittiming(struct net_device *net)
{
	struct mcp251x_priv *priv = netdev_priv(net);
	struct can_bittiming *bt = &priv->can.bittiming;
	struct spi_device *spi = priv->spi;

	mcp251x_write_reg(spi, CNF1, ((bt->sjw - 1) << CNF1_SJW_SHIFT) |
			  (bt->brp - 1));
	mcp251x_write_reg(spi, CNF2, CNF2_BTLMODE |
			  (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES ?
			   CNF2_SAM : 0) |
			  ((bt->phase_seg1 - 1) << CNF2_PS1_SHIFT) |
			  (bt->prop_seg - 1));
	mcp251x_write_bits(spi, CNF3, CNF3_PHSEG2_MASK,
			   (bt->phase_seg2 - 1));
	dev_notice(&spi->dev, "CNF: 0x%02x 0x%02x 0x%02x\n",
		mcp251x_read_reg(spi, CNF1),
		mcp251x_read_reg(spi, CNF2),
		mcp251x_read_reg(spi, CNF3));

	return 0;
}

static int mcp251x_setup(struct net_device *net, struct mcp251x_priv *priv, struct spi_device *spi)
{
    int set_mask = sizeof(priv->pdata->masks)/sizeof(priv->pdata->masks[0]) - 1;
	mcp251x_do_set_bittiming(net);

    do {
        if (priv->pdata->masks[set_mask].sid || priv->pdata->masks[set_mask].eid) {
            break;
        }
        set_mask--;
    } while (set_mask >= 0);

    if (set_mask < 0) {
        dev_notice(&spi->dev, "Accept all ids\n");
        mcp251x_write_reg(spi, RXBCTRL(0), RXBCTRL_BUKT | RXBCTRL_RXM0 | RXBCTRL_RXM1); 
        mcp251x_write_reg(spi, RXBCTRL(1), RXBCTRL_RXM0 | RXBCTRL_RXM1);
    } else {
        unsigned char val;
        mcp251x_write_reg(spi, RXBCTRL(0), RXBCTRL_BUKT); 
        mcp251x_write_reg(spi, RXBCTRL(1), 0);

        for (set_mask = 0; set_mask < sizeof(priv->pdata->masks)/sizeof(priv->pdata->masks[0]); set_mask++) {
            // masking standard ids bits 10:3
            val = (priv->pdata->masks[set_mask].sid & 0x7F8) >> 3;
            dev_notice(&spi->dev, "RXM%dSIDH[%x]\n", set_mask, val);
            mcp251x_write_reg(spi, RXMSIDH(set_mask), val);

            // masking standard ids bits 2:0
            val = (priv->pdata->masks[set_mask].sid & 0x7) << 5;
            // masking extended ids bits 17:16
            val |= (priv->pdata->masks[set_mask].eid & 0x30000) >> 16;
            dev_notice(&spi->dev, "RXM%dSIDL[%x]\n", set_mask, val);
            mcp251x_write_reg(spi, RXMSIDL(set_mask), val);

            // masking extended ids bits 15:8
            val = (priv->pdata->masks[set_mask].eid & 0xFF00) >> 8;
            dev_notice(&spi->dev, "RXM%dEID8 [%x]\n", set_mask, val);
            mcp251x_write_reg(spi, RXMEID8(set_mask), val);

            // masking extended ids bits 7:0
            val = priv->pdata->masks[set_mask].eid & 0xFF;
            dev_notice(&spi->dev, "RXM%dEID0 [%x]\n", set_mask, val);
            mcp251x_write_reg(spi, RXMEID0(set_mask), val);
        }

        for (set_mask = 0; set_mask < sizeof(priv->pdata->filters)/sizeof(priv->pdata->filters[0]); set_mask++) {
            // filtering standard ids bits 10:3
            val = (priv->pdata->filters[set_mask].fid.sid & 0x7F8) >> 3;
            dev_notice(&spi->dev, "RXF%dSIDH[%x]\n", set_mask, val);
            mcp251x_write_reg(spi, RXFSIDH(set_mask), val);

            // filtering standard ids bits 2:0
            val = (priv->pdata->filters[set_mask].fid.sid & 0x7) << 5;
            // masking extended ids bits 17:16
            val |= (priv->pdata->filters[set_mask].fid.eid & 0x30000) >> 16;
            // enable extended ids filtering
            val |= priv->pdata->filters[set_mask].exide << 3;
            dev_notice(&spi->dev, "RXF%dSIDL[%x]\n", set_mask, val);
            mcp251x_write_reg(spi, RXFSIDL(set_mask), val);

            // filtering extended ids bits 15:8
            val = (priv->pdata->filters[set_mask].fid.eid & 0xFF00) >> 8;
            dev_notice(&spi->dev, "RXF%dEID8 [%x]\n", set_mask, val);
            mcp251x_write_reg(spi, RXFEID8(set_mask), val);

            // filtering extended ids bits 7:0
            val = priv->pdata->filters[set_mask].fid.eid & 0xFF;
            dev_notice(&spi->dev, "RXF%dEID0 [%x]\n", set_mask, val);
            mcp251x_write_reg(spi, RXFEID0(set_mask), val);
        }
    }

	return 0;
}

static int mcp251x_hw_reset(struct spi_device *spi)
{
	struct mcp251x_priv *priv = spi_get_drvdata(spi);
	unsigned long timeout;
	int ret;
    unsigned err = 0, prev_err = 0xFF;

	/* Wait for oscillator startup timer after power up */
	mdelay(MCP251X_OST_DELAY_MS);

	priv->spi_tx_buf[0] = INSTRUCTION_RESET;
	ret = mcp251x_spi_trans(spi, 1);
	if (ret)
		return ret;

	dev_notice(&spi->dev, "%s: transfer %x\n", __func__, (unsigned)INSTRUCTION_RESET);

	/* Wait for oscillator startup timer after reset */
	mdelay(MCP251X_OST_DELAY_MS);

    //dev_notice(&spi->dev, "set CANCTRL_REQOP_CONF\n");
    //mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_CONF);

	/* Wait for reset to finish */
	timeout = jiffies + HZ;
	while (((err = mcp251x_read_reg(spi, CANSTAT)) & CANCTRL_REQOP_MASK) !=
	       CANCTRL_REQOP_CONF) {
		usleep_range(MCP251X_OST_DELAY_MS * 1000,
			     MCP251X_OST_DELAY_MS * 1000 * 2);

		if (time_after(jiffies, timeout)) {
			dev_err(&spi->dev,
				"MCP251x didn't enter in conf mode after reset\n");
			return -EBUSY;
		}
        if (err != prev_err) {
            dev_notice(&spi->dev, "CANSTAT [%x]\n", err); 
            prev_err = err;
        }
	}
	return 0;
}

static int mcp251x_hw_probe(struct spi_device *spi)
{
	u8 ctrl;
	int ret;

	ret = mcp251x_hw_reset(spi);
	if (ret)
		return ret;

	ctrl = mcp251x_read_reg(spi, CANCTRL);

	dev_notice(&spi->dev, "CANCTRL 0x%02x\n", ctrl);

	/* Check for power up default value */
	if ((ctrl & 0x17) != 0x07)
		return -ENODEV;

	return 0;
}

static int mcp251x_power_enable(struct regulator *reg, int enable)
{
	if (IS_ERR_OR_NULL(reg))
		return 0;

	if (enable)
		return regulator_enable(reg);
	else
		return regulator_disable(reg);
}

static void mcp251x_open_clean(struct net_device *net)
{
	struct mcp251x_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;

    dev_notice(&spi->dev, "clean\n");
//	free_irq(spi->irq, priv);
    free_irq(priv->pdata->irq, priv);
	mcp251x_hw_sleep(spi);
    if (gpio_is_valid(priv->pdata->standby_pin)) {
        gpio_set_value(priv->pdata->standby_pin, priv->pdata->standby_l);
    }
	mcp251x_power_enable(priv->transceiver, 0);
	close_candev(net);
}

static int mcp251x_stop(struct net_device *net)
{
	struct mcp251x_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;

    dev_notice(&spi->dev, "close\n");
	close_candev(net);

	priv->force_quit = 1;
//	free_irq(spi->irq, priv);
    free_irq(priv->pdata->irq, priv);

	destroy_workqueue(priv->wq);
	priv->wq = NULL;

	mutex_lock(&priv->mcp_lock);

	/* Disable and clear pending interrupts */
	mcp251x_write_reg(spi, CANINTE, 0x00);
	mcp251x_write_reg(spi, CANINTF, 0x00);

	mcp251x_write_reg(spi, TXBCTRL(0), 0);
	mcp251x_clean(net);

	mcp251x_hw_sleep(spi);

    if (gpio_is_valid(priv->pdata->standby_pin)) {
        gpio_set_value(priv->pdata->standby_pin, priv->pdata->standby_l);
    }
	mcp251x_power_enable(priv->transceiver, 0);

	priv->can.state = CAN_STATE_STOPPED;

	mutex_unlock(&priv->mcp_lock);

	can_led_event(net, CAN_LED_EVENT_STOP);

	return 0;
}

static void mcp251x_error_skb(struct net_device *net, int can_id, int data1)
{
	struct sk_buff *skb;
	struct can_frame *frame;
    struct mcp251x_priv *priv = netdev_priv(net);
    unsigned long flags;
    int full = 0;

	skb = alloc_can_err_skb(net, &frame);
	if (skb) {
		frame->can_id |= can_id;
		frame->data[1] = data1;

        spin_lock_irqsave(&priv->rx_frames_q.frame_queue_lock, flags);
        full = frames_queue_put(&priv->rx_frames_q, frame);
        if (MAX_QUE == full) {
            priv->net->stats.rx_dropped++;
        }
        spin_unlock_irqrestore(&priv->rx_frames_q.frame_queue_lock, flags);
        if (full) {
            queue_work(priv->wq, &priv->q_work);
            //dev_notice(&spi->dev, "frame que is mostly full [%d] %lld\n", full, ktime_to_us(ktime_get()));
        }
//		netif_rx_ni(skb);
	} else {
		netdev_err(net, "cannot allocate error skb\n");
	}
}

static void mcp251x_tx_work_handler(struct work_struct *ws)
{
	struct mcp251x_priv *priv = container_of(ws, struct mcp251x_priv,
						 tx_work);
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;
	struct can_frame *frame;

	mutex_lock(&priv->mcp_lock);
	if (priv->tx_skb) {
		if (priv->can.state == CAN_STATE_BUS_OFF) {
			mcp251x_clean(net);
		} else {
			frame = (struct can_frame *)priv->tx_skb->data;

			if (frame->can_dlc > CAN_FRAME_MAX_DATA_LEN)
				frame->can_dlc = CAN_FRAME_MAX_DATA_LEN;
			mcp251x_hw_tx(spi, frame, 0);
			priv->tx_len = 1 + frame->can_dlc;
			can_put_echo_skb(priv->tx_skb, net, 0);
			priv->tx_skb = NULL;
		}
	}
	mutex_unlock(&priv->mcp_lock);
}

static void mcp251x_restart_work_handler(struct work_struct *ws)
{
	struct mcp251x_priv *priv = container_of(ws, struct mcp251x_priv,
						 restart_work);
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;

	mutex_lock(&priv->mcp_lock);
	if (priv->after_suspend) {
		mcp251x_hw_reset(spi);
		mcp251x_setup(net, priv, spi);
		if (priv->after_suspend & AFTER_SUSPEND_RESTART) {
			mcp251x_set_normal_mode(spi);
		} else if (priv->after_suspend & AFTER_SUSPEND_UP) {
			netif_device_attach(net);
			mcp251x_clean(net);
			mcp251x_set_normal_mode(spi);
			netif_wake_queue(net);
		} else {
			mcp251x_hw_sleep(spi);
		}
		priv->after_suspend = 0;
		priv->force_quit = 0;
	}

	if (priv->restart_tx) {
		priv->restart_tx = 0;
		mcp251x_write_reg(spi, TXBCTRL(0), 0);
		mcp251x_clean(net);
		netif_wake_queue(net);
		mcp251x_error_skb(net, CAN_ERR_RESTARTED, 0);
	}
	mutex_unlock(&priv->mcp_lock);
}

#if MAX_QUE
static void mcp251x_q_work(struct work_struct *ws)
{
	struct mcp251x_priv *priv = container_of(ws, struct mcp251x_priv, q_work);
	struct spi_device *spi = priv->spi;

	mutex_lock(&priv->rx_frames_q.que_lock);
    mcp251x_hw_rx_skb(spi);
	mutex_unlock(&priv->rx_frames_q.que_lock);
}
#endif

#define DEBUG_OVERRUN 0
#if DEBUG_OVERRUN
struct frames_loss_dbg {
    s64 ts[64];
    int c;
};

struct frames_loss_dbg loss_frames;
#endif

static irqreturn_t mcp251x_can_ist(int irq, void *dev_id)
{
	struct mcp251x_priv *priv = dev_id;
	struct spi_device *spi = priv->spi;
	struct net_device *net = priv->net;

#if MAX_QUE
    disable_irq_nosync(irq);
#endif

    mutex_lock(&priv->mcp_lock);
    while (!priv->force_quit) {
		enum can_state new_state;
		u8 intf, eflag;
		u8 clear_intf = 0;
		int can_id = 0, data1 = 0;
#if MAX_QUE
        int full = 0;
#endif

#if DEBUG_OVERRUN
        if (loss_frames.c > sizeof(loss_frames.ts)/sizeof(loss_frames.ts[0]) - 1) {
            loss_frames.c = 0;
        }
        loss_frames.ts[loss_frames.c] = ktime_to_us(ktime_get());
#endif
		mcp251x_read_2regs(spi, CANINTF, &intf, &eflag);

		/* mask out flags we don't care about */
		intf &= CANINTF_RX | CANINTF_TX | CANINTF_ERR;

		/* receive buffer 0 */
		if (intf & CANINTF_RX0IF) {
#if MAX_QUE
            full +=
#endif
            mcp251x_hw_rx(spi, 0); 
			/* Free one buffer ASAP
			 * (The MCP2515/25625 does this automatically.)
			 */
			if (mcp251x_is_2510(spi))
				mcp251x_write_bits(spi, CANINTF, CANINTF_RX0IF, 0x00);
            else
                clear_intf |= CANINTF_RX0IF;
		}

		/* receive buffer 1 */
		if (intf & CANINTF_RX1IF) {
#if MAX_QUE
			full +=
#endif
            mcp251x_hw_rx(spi, 1);
			/* The MCP2515/25625 does this automatically. */
			//if (mcp251x_is_2510(spi))
				clear_intf |= CANINTF_RX1IF;
		}

        //if (full) {
        //    queue_work(priv->wq, &priv->q_work);
        //}
        /* any error or tx interrupt we need to clear? */
		if (intf & (CANINTF_ERR | CANINTF_TX))
			clear_intf |= intf & (CANINTF_ERR | CANINTF_TX);
		if (clear_intf)
			mcp251x_write_bits(spi, CANINTF, clear_intf, 0x00);

		if (eflag & (EFLG_RX0OVR | EFLG_RX1OVR))
			mcp251x_write_bits(spi, EFLG, eflag, 0x00);

		/* Update can state */
		if (eflag & EFLG_TXBO) {
			new_state = CAN_STATE_BUS_OFF;
			can_id |= CAN_ERR_BUSOFF;
		} else if (eflag & EFLG_TXEP) {
			new_state = CAN_STATE_ERROR_PASSIVE;
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_TX_PASSIVE;
		} else if (eflag & EFLG_RXEP) {
			new_state = CAN_STATE_ERROR_PASSIVE;
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_RX_PASSIVE;
		} else if (eflag & EFLG_TXWAR) {
			new_state = CAN_STATE_ERROR_WARNING;
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_TX_WARNING;
		} else if (eflag & EFLG_RXWAR) {
			new_state = CAN_STATE_ERROR_WARNING;
			can_id |= CAN_ERR_CRTL;
			data1 |= CAN_ERR_CRTL_RX_WARNING;
		} else {
			new_state = CAN_STATE_ERROR_ACTIVE;
		}

		/* Update can state statistics */
		switch (priv->can.state) {
		case CAN_STATE_ERROR_ACTIVE:
			if (new_state >= CAN_STATE_ERROR_WARNING &&
			    new_state <= CAN_STATE_BUS_OFF)
				priv->can.can_stats.error_warning++;
		case CAN_STATE_ERROR_WARNING:	/* fallthrough */
			if (new_state >= CAN_STATE_ERROR_PASSIVE &&
			    new_state <= CAN_STATE_BUS_OFF)
				priv->can.can_stats.error_passive++;
			break;
		default:
			break;
		}
		priv->can.state = new_state;

		if (intf & CANINTF_ERRIF) {
			/* Handle overflow counters */
			if (eflag & (EFLG_RX0OVR | EFLG_RX1OVR)) {
				if (eflag & EFLG_RX0OVR) {
					net->stats.rx_over_errors++;
					net->stats.rx_errors++;
				}
				if (eflag & EFLG_RX1OVR) {
					net->stats.rx_over_errors++;
					net->stats.rx_errors++;
				}
#if DEBUG_OVERRUN
                dev_notice(&spi->dev, "overrun on %lld\n", ktime_to_us(ktime_get()));
                while (loss_frames.c >= 0) {
                    dev_notice(&spi->dev, "previous   %lld\n", loss_frames.ts[loss_frames.c--]);
                }
                loss_frames.c = 0;
#endif
				can_id |= CAN_ERR_CRTL;
				data1 |= CAN_ERR_CRTL_RX_OVERFLOW;
			}
			mcp251x_error_skb(net, can_id, data1);
		}

		if (priv->can.state == CAN_STATE_BUS_OFF) {
			if (priv->can.restart_ms == 0) {
				priv->force_quit = 1;
				priv->can.can_stats.bus_off++;
				can_bus_off(net);
				mcp251x_hw_sleep(spi);
				break;
			}
		}

		if (intf == 0) {
#if MAX_QUE
            if (gpio_is_valid(priv->pdata->irq_pin)) {
                if (priv->pdata->irq_l == gpio_get_value(priv->pdata->irq_pin)) {
                    continue;
                }
            }
            queue_work(priv->wq, &priv->q_work);
#endif
			break;
        }

		if (intf & CANINTF_TX) {
			net->stats.tx_packets++;
			net->stats.tx_bytes += priv->tx_len - 1;
			can_led_event(net, CAN_LED_EVENT_TX);
			if (priv->tx_len) {
				can_get_echo_skb(net, 0);
				priv->tx_len = 0;
			}
			netif_wake_queue(net);
		}

	}
    mutex_unlock(&priv->mcp_lock);

#if MAX_QUE
    enable_irq(priv->pdata->irq);
#endif

	return IRQ_HANDLED;
}

static int mcp251x_open(struct net_device *net)
{
	struct mcp251x_priv *priv = netdev_priv(net);
	struct spi_device *spi = priv->spi;
	unsigned long flags = IRQF_ONESHOT |
#if MAX_QUE
        IRQF_TRIGGER_LOW;
#else
        IRQF_TRIGGER_FALLING;
#endif

	int ret;

    dev_notice(&spi->dev, "open\n");
	ret = open_candev(net);
	if (ret) {
		dev_err(&spi->dev, "unable to set initial baudrate!\n");
		return ret;
	}

	mutex_lock(&priv->mcp_lock);
    if (gpio_is_valid(priv->pdata->standby_pin)) {
        gpio_set_value(priv->pdata->standby_pin, !priv->pdata->standby_l);
    }
	mcp251x_power_enable(priv->transceiver, 1);

	priv->force_quit = 0;
	priv->tx_skb = NULL;
	priv->tx_len = 0;
#if MAX_QUE
    priv->rx_frames_q.head = priv->rx_frames_q.tail = 0;
#endif
#if DEBUG_OVERRUN
    loss_frames.c = 0;
#endif

//	ret = request_threaded_irq(spi->irq, NULL, mcp251x_can_ist,
//				   flags | IRQF_ONESHOT, DEVICE_NAME, priv);
    ret = request_threaded_irq(priv->pdata->irq, 0, mcp251x_can_ist, flags, DEVICE_NAME, priv);
	if (ret) {
		//dev_err(&spi->dev, "failed to acquire irq %d\n", spi->irq);
        dev_err(&spi->dev, "failed to acquire irq %d\n", priv->pdata->irq);
        if (gpio_is_valid(priv->pdata->standby_pin)) {
            gpio_set_value(priv->pdata->standby_pin, priv->pdata->standby_l);
        }
		mcp251x_power_enable(priv->transceiver, 0);
		close_candev(net);
		goto open_unlock;
	}

	priv->wq = alloc_workqueue("mcp251x_wq", WQ_FREEZABLE | WQ_MEM_RECLAIM, 0);
	INIT_WORK(&priv->tx_work, mcp251x_tx_work_handler);
	INIT_WORK(&priv->restart_work, mcp251x_restart_work_handler);
#if MAX_QUE
    INIT_WORK(&priv->q_work, mcp251x_q_work);
#endif

	ret = mcp251x_hw_reset(spi);
	if (ret) {
		mcp251x_open_clean(net);
		goto open_unlock;
	}
	ret = mcp251x_setup(net, priv, spi);
	if (ret) {
		mcp251x_open_clean(net);
		goto open_unlock;
	}
	ret = mcp251x_set_normal_mode(spi);
	if (ret) {
		mcp251x_open_clean(net);
		goto open_unlock;
	}

	can_led_event(net, CAN_LED_EVENT_OPEN);

	netif_wake_queue(net);

open_unlock:
	mutex_unlock(&priv->mcp_lock);
	return ret;
}

static const struct net_device_ops mcp251x_netdev_ops = {
	.ndo_open = mcp251x_open,
	.ndo_stop = mcp251x_stop,
	.ndo_start_xmit = mcp251x_hard_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};

static const struct of_device_id mcp251x_of_match[] = {
	{
		.compatible	= "microchip,mcp2510",
		.data		= (void *)CAN_MCP251X_MCP2510,
	},
	{
		.compatible	= "microchip,mcp2515",
		.data		= (void *)CAN_MCP251X_MCP2515,
	},
	{
		.compatible	= "microchip,mcp25625",
		.data		= (void *)CAN_MCP251X_MCP25625,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, mcp251x_of_match);

static const struct spi_device_id mcp251x_id_table[] = {
	{
		.name		= "mcp2510",
		.driver_data	= (kernel_ulong_t)CAN_MCP251X_MCP2510,
	},
	{
		.name		= "mcp2515",
		.driver_data	= (kernel_ulong_t)CAN_MCP251X_MCP2515,
	},
	{
		.name		= "mcp25625",
		.driver_data	= (kernel_ulong_t)CAN_MCP251X_MCP25625,
	},
	{ }
};
MODULE_DEVICE_TABLE(spi, mcp251x_id_table);

static ssize_t mcp251x_mask_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct net_device *net = dev_get_platdata(dev);
    struct mcp251x_priv *priv = netdev_priv(net);

    if (sscanf(buf, "%x:%x,%x:%x",
                    &priv->pdata->masks[0].sid, &priv->pdata->masks[0].eid,
                    &priv->pdata->masks[1].sid, &priv->pdata->masks[1].eid) == 4) {
        priv->pdata->masks[0].sid &= 0x7FF;
        priv->pdata->masks[1].sid &= 0x7FF;
        priv->pdata->masks[0].eid &= 0x3FFFF;
        priv->pdata->masks[1].eid &= 0x3FFFF;

        return count;
    }

    return -EINVAL;
}

static ssize_t mcp251x_mask_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct net_device *net = dev_get_platdata(dev);
    struct mcp251x_priv *priv = netdev_priv(net);

	return sprintf(buf, "%x:%x, %x:%x\n", priv->pdata->masks[0].sid, priv->pdata->masks[0].eid,
                                          priv->pdata->masks[1].sid, priv->pdata->masks[1].eid);
}

static ssize_t mcp251x_filter_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct net_device *net = dev_get_platdata(dev);
    struct mcp251x_priv *priv = netdev_priv(net);
    int i;

    if (strlen(buf) < 35) {
        dev_notice(&priv->spi->dev, "input [%s] invalid\n", buf);
        return -EINVAL;
    }

    if (sscanf(buf, "%x:%x-%d,%x:%x-%d,%x:%x-%d,%x:%x-%d,%x:%x-%d,%x:%x-%d",
                    &priv->pdata->filters[0].fid.sid, &priv->pdata->filters[0].fid.eid, &priv->pdata->filters[0].exide,
                    &priv->pdata->filters[1].fid.sid, &priv->pdata->filters[1].fid.eid, &priv->pdata->filters[1].exide,
                    &priv->pdata->filters[2].fid.sid, &priv->pdata->filters[2].fid.eid, &priv->pdata->filters[2].exide,
                    &priv->pdata->filters[3].fid.sid, &priv->pdata->filters[3].fid.eid, &priv->pdata->filters[3].exide,
                    &priv->pdata->filters[4].fid.sid, &priv->pdata->filters[4].fid.eid, &priv->pdata->filters[4].exide,
                    &priv->pdata->filters[5].fid.sid, &priv->pdata->filters[5].fid.eid, &priv->pdata->filters[5].exide) == 18) {
        for (i = 0; i < sizeof(priv->pdata->filters)/sizeof(priv->pdata->filters[0]); i++) {
            priv->pdata->filters[i].fid.sid &= 0x7FF; 
        }
        for (i = 0; i < sizeof(priv->pdata->filters)/sizeof(priv->pdata->filters[0]); i++) {
            priv->pdata->filters[i].fid.eid &= 0x3FFFF;
        }

        return count;
    }

    return -EINVAL;
}

static ssize_t mcp251x_filter_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct net_device *net = dev_get_platdata(dev);
    struct mcp251x_priv *priv = netdev_priv(net);

	return sprintf(buf, "%x:%x-%d, %x:%x-%d, %x:%x-%d, %x:%x-%d, %x:%x-%d, %x:%x-%d\n",
                   priv->pdata->filters[0].fid.sid, priv->pdata->filters[0].fid.eid, priv->pdata->filters[0].exide,
                   priv->pdata->filters[1].fid.sid, priv->pdata->filters[1].fid.eid, priv->pdata->filters[1].exide,
                   priv->pdata->filters[2].fid.sid, priv->pdata->filters[2].fid.eid, priv->pdata->filters[2].exide,
                   priv->pdata->filters[3].fid.sid, priv->pdata->filters[3].fid.eid, priv->pdata->filters[3].exide,
                   priv->pdata->filters[4].fid.sid, priv->pdata->filters[4].fid.eid, priv->pdata->filters[4].exide,
                   priv->pdata->filters[5].fid.sid, priv->pdata->filters[5].fid.eid, priv->pdata->filters[5].exide);
}

static int mcp251x_can_probe(struct spi_device *spi)
{
	const struct of_device_id *of_id = of_match_device(mcp251x_of_match, &spi->dev);
	struct mcp251x_platform_data *pdata = dev_get_platdata(&spi->dev);
	struct net_device *net;
	struct mcp251x_priv *priv;
	struct clk *clk;
	int freq, err;
    struct device_node *np;
    struct pinctrl_state *pctls;

    np = spi->dev.of_node;
    if (!np) {
        dev_notice(&spi->dev, "failure to find device tree\n");
        return -EINVAL;
    }

    if (!pdata) {
        pdata = devm_kzalloc(&spi->dev, sizeof(struct mcp251x_platform_data), GFP_KERNEL);
        if (!pdata) {
            dev_err(&spi->dev, "Add some memory\n");
            return -ENOMEM;
        }
    }
    err = of_property_read_u32(np, "oscillator-frequency", &freq);
    if (err < 0) {
        dev_err(&spi->dev, "failure to get osc freq, using default 20MHz\n");
        freq = 20000000;
    }
    pdata->oscillator_frequency = freq;

    clk = devm_clk_get(&spi->dev, 0); 
	if (IS_ERR(clk)) {
		if (pdata)
			freq = pdata->oscillator_frequency;
		else
			return PTR_ERR(clk);
	} else {
		freq = clk_get_rate(clk);
	}
    dev_notice(&spi->dev, "spi clk[%lu], osc freq[%d] \n", (clk)?clk_get_rate(clk):0, freq);

    pdata->pctl = devm_pinctrl_get(&spi->dev);
    if (IS_ERR(pdata->pctl)) {
        if (PTR_ERR(pdata->pctl) == -EPROBE_DEFER) {
            dev_err(&spi->dev, "pin ctl critical error!\n");
            err = -EPROBE_DEFER;
            goto out_free_loc;
        }

        dev_err(&spi->dev, "pin control isn't used\n");
        pdata->pctl = 0;
    }

    if (pdata->pctl) {
        pctls = pinctrl_lookup_state(pdata->pctl, "mcp25625_standby_active");
        if (IS_ERR(pctls)) {
            dev_err(&spi->dev, "failure to get pinctrl mcp25625_standby_active state\n");
            err = PTR_ERR(pctls);
            dev_err(&spi->dev, "pin control isn't used\n");
            devm_pinctrl_put(pdata->pctl);
            pdata->pctl = 0;
        } else {
            err = pinctrl_select_state(pdata->pctl, pctls);
            if (err) {
                devm_pinctrl_put(pdata->pctl);
                dev_err(&spi->dev, "failure to set pinctrl active state\n");
                dev_err(&spi->dev, "pin control isn't used\n");
                pdata->pctl = 0;
            }
        }
    }

    err = of_get_named_gpio_flags(np, "mcp25625,irq-pin", 0, (enum of_gpio_flags *)&pdata->irq_l);
    if (!gpio_is_valid(err)) {
        dev_err(&spi->dev, "ivalid irq pin\n");
        err = -EINVAL;
        pdata->irq_pin = -1;
        pdata->irq_l = 1;
        goto out_free_loc;
    } else {
        pdata->irq_pin = err;
    }
    pdata->irq_l = !pdata->irq_l;

    err = of_get_named_gpio_flags(np, "mcp25625,reset-pin", 0, (enum of_gpio_flags *)&pdata->reset_l);
    if (!gpio_is_valid(err)) {
        dev_err(&spi->dev, "ivalid reset pin, won't use\n");
        pdata->reset_pin = -1;
        pdata->reset_l = 1;
    } else {
        pdata->reset_pin = err;
    }
    pdata->reset_l = !pdata->reset_l;

    err = of_get_named_gpio_flags(np, "mcp25625,standby-pin", 0, (enum of_gpio_flags *)&pdata->standby_l);
    if (!gpio_is_valid(err)) {
        dev_err(&spi->dev, "ivalid standby pin, won't use\n");
        pdata->standby_pin = -1;
        pdata->standby_l = 1;
    } else {
        pdata->standby_pin = err;
    }
    pdata->standby_l = pdata->standby_l;

	/* Sanity check */
	if (freq < 1000000 || freq > 25000000)
		return -ERANGE;

    dev_notice(&spi->dev, "irq-pin[%u], reset-pin[%d], standby-pin[%d] \n", pdata->irq_pin, pdata->reset_pin, pdata->standby_pin);
    dev_notice(&spi->dev, "spi clk[%u], osc freq[%d] \n", spi->max_speed_hz, freq);

    if (gpio_is_valid(pdata->irq_pin)) {
        err = devm_gpio_request(&spi->dev, pdata->irq_pin, "can0-irq");
        if (err < 0) {
            dev_err(&spi->dev, "failure to request irq pin[%d]\n", pdata->irq_pin);
            err = -EINVAL;
            goto out_free_loc;
        }
        err = gpio_direction_input(pdata->irq_pin);
        if (err < 0) {
            dev_err(&spi->dev, "failure to set direction of irq pin[%d]\n", pdata->irq_pin);
            devm_gpio_free(&spi->dev, pdata->irq_pin);
            err = -EINVAL;
            goto out_free_loc;
        }
        gpio_export(pdata->irq_pin, 0);

        pdata->irq = gpio_to_irq(pdata->irq_pin);
        if (pdata->irq < 0) {
            dev_err(&spi->dev, "failure to convert gpio[%d] to irq\n", pdata->irq_pin);
        }
    }

    if (gpio_is_valid(pdata->reset_pin)) {
        err = devm_gpio_request(&spi->dev, pdata->reset_pin, "can0-reset");
        if (err < 0) {
            dev_err(&spi->dev, "failure to request reset pin[%d]\n", pdata->reset_pin);
            devm_gpio_free(&spi->dev, pdata->irq_pin);
            err = -EINVAL;
            goto out_free_loc;
        }
        err = gpio_direction_output(pdata->reset_pin, pdata->reset_l);
        if (err < 0) {
            dev_err(&spi->dev, "failure to set direction of reset pin[%d]\n", pdata->reset_pin);
            devm_gpio_free(&spi->dev, pdata->irq_pin);
            devm_gpio_free(&spi->dev, pdata->reset_pin);
            err = -EINVAL;
            goto out_free_loc;
        }
        gpio_export(pdata->reset_pin, 0);
    }

    if (gpio_is_valid(pdata->standby_pin)) {
        err = devm_gpio_request(&spi->dev, pdata->standby_pin, "can0-standby");
        if (err < 0) {
            dev_err(&spi->dev, "failure to request standby pin[%d]\n", pdata->standby_pin);
            devm_gpio_free(&spi->dev, pdata->irq_pin);
            devm_gpio_free(&spi->dev, pdata->reset_pin);
            err = -EINVAL;
            goto out_free_loc;
        }
        err = gpio_direction_output(pdata->standby_pin, !pdata->standby_l);
        if (err < 0) {
            dev_err(&spi->dev, "failure to set direction of standby pin[%d]\n", pdata->standby_pin);
            devm_gpio_free(&spi->dev, pdata->irq_pin);
            devm_gpio_free(&spi->dev, pdata->reset_pin);
            devm_gpio_free(&spi->dev, pdata->standby_pin);
            err = -EINVAL;
            goto out_free_loc;
        }
        err = gpio_export(pdata->standby_pin, 0);
        if (err < 0) {
            dev_err(&spi->dev, "failure to export standby pin[%d, %d]\n", pdata->standby_pin, err);
        }
    }

    dev_notice(&spi->dev, "spi irq[%u], can irq[%d] \n", spi->irq, pdata->irq);

	/* Allocate can/net device */
	net = alloc_candev(sizeof(struct mcp251x_priv), TX_ECHO_SKB_MAX);
	if (!net)
		return -ENOMEM;

	if (!IS_ERR(clk)) {
		err = clk_prepare_enable(clk);
		if (err)
			goto out_free;
	}

	net->netdev_ops = &mcp251x_netdev_ops;
	net->flags |= IFF_ECHO;

	priv = netdev_priv(net);
#if MAX_QUE
    spin_lock_init(&priv->rx_frames_q.frame_queue_lock);
    mutex_init(&priv->rx_frames_q.que_lock);
#endif
    priv->pdata = pdata;
	priv->can.bittiming_const = &mcp251x_bittiming_const;
	//priv->can.do_set_bittiming = mcp251x_do_set_bittiming;
	priv->can.do_set_mode = mcp251x_do_set_mode;
	priv->can.clock.freq = freq / 2;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES |
		CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY;
	if (of_id)
		priv->model = (enum mcp251x_model)of_id->data;
	else
		priv->model = spi_get_device_id(spi)->driver_data;
	priv->net = net;
	priv->clk = clk;

	spi_set_drvdata(spi, priv);

	/* Configure the SPI bus */
	spi->bits_per_word = 8;
	if (mcp251x_is_2510(spi))
		spi->max_speed_hz = spi->max_speed_hz ? : 5 * 1000 * 1000;
	else
		spi->max_speed_hz = spi->max_speed_hz ? : 10 * 1000 * 1000;

    dev_notice(&spi->dev, "setup spi [%u]\n", spi->max_speed_hz);

    err = spi_setup(spi);
	if (err)
		goto out_clk;

	priv->power = devm_regulator_get(&spi->dev, "vdd");
	priv->transceiver = devm_regulator_get(&spi->dev, "xceiver");
	if ((PTR_ERR(priv->power) == -EPROBE_DEFER) ||
	    (PTR_ERR(priv->transceiver) == -EPROBE_DEFER)) {
		err = -EPROBE_DEFER;
		goto out_clk;
	}

    dev_notice(&spi->dev, "power up, wake and reamin reset inactive\n");
    if (gpio_is_valid(pdata->reset_pin)) {
        gpio_set_value(pdata->reset_pin, !pdata->reset_l);
    }

    if (gpio_is_valid(pdata->standby_pin)) {
        gpio_set_value(pdata->standby_pin, !pdata->standby_l);
    }

	err = mcp251x_power_enable(priv->power, 1);
	if (err)
		goto out_clk;

	priv->spi = spi;
	mutex_init(&priv->mcp_lock);

	/* If requested, allocate DMA buffers */
	if (mcp251x_enable_dma) {
		spi->dev.coherent_dma_mask = ~0;

		/*
		 * Minimum coherent DMA allocation is PAGE_SIZE, so allocate
		 * that much and share it between Tx and Rx DMA buffers.
		 */
		priv->spi_tx_buf = dmam_alloc_coherent(&spi->dev,
						       PAGE_SIZE,
						       &priv->spi_tx_dma,
						       GFP_DMA);

		if (priv->spi_tx_buf) {
			priv->spi_rx_buf = (priv->spi_tx_buf + (PAGE_SIZE / 2));
			priv->spi_rx_dma = (dma_addr_t)(priv->spi_tx_dma +
							(PAGE_SIZE / 2));
		} else {
			/* Fall back to non-DMA */
			mcp251x_enable_dma = 0;
		}
	}

	/* Allocate non-DMA buffers */
	if (!mcp251x_enable_dma) {
		priv->spi_tx_buf = devm_kzalloc(&spi->dev, SPI_TRANSFER_BUF_LEN,
						GFP_KERNEL);
		if (!priv->spi_tx_buf) {
			err = -ENOMEM;
			goto error_probe;
		}
		priv->spi_rx_buf = devm_kzalloc(&spi->dev, SPI_TRANSFER_BUF_LEN,
						GFP_KERNEL);
		if (!priv->spi_rx_buf) {
			err = -ENOMEM;
			goto error_probe;
		}
        dev_notice(&spi->dev, "tx/rx buffers allocated\n");
	}

	SET_NETDEV_DEV(net, &spi->dev);

	/* Here is OK to not lock the MCP, no one knows about it yet */
	err = mcp251x_hw_probe(spi);
	if (err) {
		if (err == -ENODEV)
			dev_err(&spi->dev, "Cannot initialize MCP%x. Wrong wiring?\n", priv->model);
        goto error_probe;
	}

	mcp251x_hw_sleep(spi);

	err = register_candev(net);
	if (err)
		goto error_probe;

    for (err = 0; err < sizeof(priv->pdata->masks)/sizeof(priv->pdata->masks[0]); err++) {
        priv->pdata->masks[err].sid = 0;
        priv->pdata->masks[err].eid = 0;
    }
    snprintf(priv->pdata->attr_mask.name, sizeof(priv->pdata->attr_mask.name) - 1, "masks"); 
    priv->pdata->attr_mask.attr.attr.name = priv->pdata->attr_mask.name;
    priv->pdata->attr_mask.attr.attr.mode = 0666;
    priv->pdata->attr_mask.attr.show = mcp251x_mask_show;
    priv->pdata->attr_mask.attr.store = mcp251x_mask_store;
    sysfs_attr_init(&priv->pdata->attr_mask.attr.attr);

    for (err = 0; err < sizeof(priv->pdata->filters)/sizeof(priv->pdata->filters[0]); err++) {
        priv->pdata->filters[err].fid.sid = 0;
        priv->pdata->filters[err].fid.eid = 0;
        priv->pdata->filters[err].exide = 0;
    }
    snprintf(priv->pdata->attr_filter.name, sizeof(priv->pdata->attr_filter.name) - 1, "filters");
    priv->pdata->attr_filter.attr.attr.name = priv->pdata->attr_filter.name;
    priv->pdata->attr_filter.attr.attr.mode = 0666;
    priv->pdata->attr_filter.attr.show = mcp251x_filter_show;
    priv->pdata->attr_filter.attr.store = mcp251x_filter_store;
    sysfs_attr_init(&priv->pdata->attr_filter.attr.attr);

    err = device_create_file(&priv->net->dev, &priv->pdata->attr_mask.attr);
    err = device_create_file(&priv->net->dev, &priv->pdata->attr_filter.attr);

	devm_can_led_init(net);

	netdev_info(net, "MCP%x successfully initialized.\n", priv->model);
	return 0;

error_probe:
	mcp251x_power_enable(priv->power, 0);

out_clk:
	if (!IS_ERR(clk))
		clk_disable_unprepare(clk);

out_free:
	free_candev(net);
out_free_loc:
    if (pdata) {
        if (pdata->pctl) {
            devm_pinctrl_put(pdata->pctl); 
        }
        devm_kfree(&spi->dev, pdata); 
    }

	dev_err(&spi->dev, "failure to probe, err=%d\n", -err);

	return err;
}

static int mcp251x_can_remove(struct spi_device *spi)
{
	struct mcp251x_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;

	unregister_candev(net);

    if (gpio_is_valid(priv->pdata->reset_pin)) {
        gpio_set_value(priv->pdata->reset_pin, priv->pdata->reset_l);
    }
	mcp251x_power_enable(priv->power, 0);

	if (!IS_ERR(priv->clk))
		clk_disable_unprepare(priv->clk);

    if (gpio_is_valid(priv->pdata->irq_pin)) {
        devm_gpio_free(&spi->dev, priv->pdata->irq_pin);
    }

    if (gpio_is_valid(priv->pdata->reset_pin)) {
        devm_gpio_free(&spi->dev, priv->pdata->reset_pin);
    }

    if (gpio_is_valid(priv->pdata->standby_pin)) {
        devm_gpio_free(&spi->dev, priv->pdata->standby_pin);
    }

    if (priv->pdata->pctl) {
        devm_pinctrl_put(priv->pdata->pctl); 
    }
    devm_kfree(&spi->dev, priv->pdata);
	free_candev(net);

	return 0;
}

static int __maybe_unused mcp251x_can_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct mcp251x_priv *priv = spi_get_drvdata(spi);
	struct net_device *net = priv->net;

	priv->force_quit = 1;
	//disable_irq(spi->irq);
    disable_irq_nosync(priv->pdata->irq);
	/*
	 * Note: at this point neither IST nor workqueues are running.
	 * open/stop cannot be called anyway so locking is not needed
	 */
	if (netif_running(net)) {
		netif_device_detach(net);

		mcp251x_hw_sleep(spi);
        if (gpio_is_valid(priv->pdata->standby_pin)) {
            gpio_set_value(priv->pdata->standby_pin, priv->pdata->standby_l);
        }
		mcp251x_power_enable(priv->transceiver, 0);
		priv->after_suspend = AFTER_SUSPEND_UP;
	} else {
		priv->after_suspend = AFTER_SUSPEND_DOWN;
	}

	if (!IS_ERR_OR_NULL(priv->power)) {
        if (gpio_is_valid(priv->pdata->reset_pin)) {
            gpio_set_value(priv->pdata->reset_pin, priv->pdata->reset_l);
        }
		regulator_disable(priv->power);
		priv->after_suspend |= AFTER_SUSPEND_POWER;
	}

	return 0;
}

static int __maybe_unused mcp251x_can_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct mcp251x_priv *priv = spi_get_drvdata(spi);

	if (priv->after_suspend & AFTER_SUSPEND_POWER){
        if (gpio_is_valid(priv->pdata->reset_pin)) {
            gpio_set_value(priv->pdata->reset_pin, !priv->pdata->reset_l);
        }
		mcp251x_power_enable(priv->power, 1);
    }

	if (priv->after_suspend & AFTER_SUSPEND_UP) {
        if (gpio_is_valid(priv->pdata->standby_pin)) {
            gpio_set_value(priv->pdata->standby_pin, !priv->pdata->standby_l);
        }
		mcp251x_power_enable(priv->transceiver, 1);
		queue_work(priv->wq, &priv->restart_work);
	} else {
		priv->after_suspend = 0;
	}

	priv->force_quit = 0;
	//enable_irq(spi->irq);
    enable_irq(priv->pdata->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(mcp251x_can_pm_ops, mcp251x_can_suspend,
	mcp251x_can_resume);

static struct spi_driver mcp251x_can_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = mcp251x_of_match,
		.pm = &mcp251x_can_pm_ops,
	},
	.id_table = mcp251x_id_table,
	.probe = mcp251x_can_probe,
	.remove = mcp251x_can_remove,
};
module_spi_driver(mcp251x_can_driver);

MODULE_AUTHOR("Chris Elston <celston@katalix.com>, "
	      "Christian Pellegrin <chripell@evolware.org>");
MODULE_DESCRIPTION("Microchip 251x/25625 CAN driver");
MODULE_LICENSE("GPL v2");
