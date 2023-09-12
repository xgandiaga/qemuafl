#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "net/net.h"
#include "net/eth.h"
#include "hw/hw.h"
#include "hw/irq.h"
#include "hw/net/lan9118.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "qemu/module.h"
/* For crc32 */
#include <zlib.h>
#include "qom/object.h"
#include "intermediateDriver/intermediateDriver.h"


/* The tx and rx fifo ports are a range of aliased 32-bit registers */
#define RX_DATA_FIFO_PORT_FIRST 0x00
#define RX_DATA_FIFO_PORT_LAST 0x1f
#define TX_DATA_FIFO_PORT_FIRST 0x20
#define TX_DATA_FIFO_PORT_LAST 0x3f

#define RX_STATUS_FIFO_PORT 0x40
#define RX_STATUS_FIFO_PEEK 0x44
#define TX_STATUS_FIFO_PORT 0x48
#define TX_STATUS_FIFO_PEEK 0x4c

#define CSR_ID_REV      0x50
#define CSR_IRQ_CFG     0x54
#define CSR_INT_STS     0x58
#define CSR_INT_EN      0x5c
#define CSR_BYTE_TEST   0x64
#define CSR_FIFO_INT    0x68
#define CSR_RX_CFG      0x6c
#define CSR_TX_CFG      0x70
#define CSR_HW_CFG      0x74
#define CSR_RX_DP_CTRL  0x78
#define CSR_RX_FIFO_INF 0x7c
#define CSR_TX_FIFO_INF 0x80
#define CSR_PMT_CTRL    0x84
#define CSR_GPIO_CFG    0x88
#define CSR_GPT_CFG     0x8c
#define CSR_GPT_CNT     0x90
#define CSR_WORD_SWAP   0x98
#define CSR_FREE_RUN    0x9c
#define CSR_RX_DROP     0xa0
#define CSR_MAC_CSR_CMD 0xa4
#define CSR_MAC_CSR_DATA 0xa8
#define CSR_AFC_CFG     0xac
#define CSR_E2P_CMD     0xb0
#define CSR_E2P_DATA    0xb4

#define E2P_CMD_MAC_ADDR_LOADED 0x100

/* IRQ_CFG */
#define IRQ_INT         0x00001000
#define IRQ_EN          0x00000100
#define IRQ_POL         0x00000010
#define IRQ_TYPE        0x00000001

/* INT_STS/INT_EN */
#define SW_INT          0x80000000
#define TXSTOP_INT      0x02000000
#define RXSTOP_INT      0x01000000
#define RXDFH_INT       0x00800000
#define TX_IOC_INT      0x00200000
#define RXD_INT         0x00100000
#define GPT_INT         0x00080000
#define PHY_INT         0x00040000
#define PME_INT         0x00020000
#define TXSO_INT        0x00010000
#define RWT_INT         0x00008000
#define RXE_INT         0x00004000
#define TXE_INT         0x00002000
#define TDFU_INT        0x00000800
#define TDFO_INT        0x00000400
#define TDFA_INT        0x00000200
#define TSFF_INT        0x00000100
#define TSFL_INT        0x00000080
#define RXDF_INT        0x00000040
#define RDFL_INT        0x00000020
#define RSFF_INT        0x00000010
#define RSFL_INT        0x00000008
#define GPIO2_INT       0x00000004
#define GPIO1_INT       0x00000002
#define GPIO0_INT       0x00000001
#define RESERVED_INT    0x7c001000

#define MAC_CR          1
#define MAC_ADDRH       2
#define MAC_ADDRL       3
#define MAC_HASHH       4
#define MAC_HASHL       5
#define MAC_MII_ACC     6
#define MAC_MII_DATA    7
#define MAC_FLOW        8
#define MAC_VLAN1       9 /* TODO */
#define MAC_VLAN2       10 /* TODO */
#define MAC_WUFF        11 /* TODO */
#define MAC_WUCSR       12 /* TODO */

#define MAC_CR_RXALL    0x80000000
#define MAC_CR_RCVOWN   0x00800000
#define MAC_CR_LOOPBK   0x00200000
#define MAC_CR_FDPX     0x00100000
#define MAC_CR_MCPAS    0x00080000
#define MAC_CR_PRMS     0x00040000
#define MAC_CR_INVFILT  0x00020000
#define MAC_CR_PASSBAD  0x00010000
#define MAC_CR_HO       0x00008000
#define MAC_CR_HPFILT   0x00002000
#define MAC_CR_LCOLL    0x00001000
#define MAC_CR_BCAST    0x00000800
#define MAC_CR_DISRTY   0x00000400
#define MAC_CR_PADSTR   0x00000100
#define MAC_CR_BOLMT    0x000000c0
#define MAC_CR_DFCHK    0x00000020
#define MAC_CR_TXEN     0x00000008
#define MAC_CR_RXEN     0x00000004
#define MAC_CR_RESERVED 0x7f404213

#define PHY_INT_ENERGYON            0x80
#define PHY_INT_AUTONEG_COMPLETE    0x40
#define PHY_INT_FAULT               0x20
#define PHY_INT_DOWN                0x10
#define PHY_INT_AUTONEG_LP          0x08
#define PHY_INT_PARFAULT            0x04
#define PHY_INT_AUTONEG_PAGE        0x02

#define GPT_TIMER_EN    0x20000000



/* The FPGA images have an odd combination of different RAMs,
     * because in hardware they are different implementations and
     * connected to different buses, giving varying performance/size
     * tradeoffs. For QEMU they're all just RAM, though. We arbitrarily
     * call the 16MB our "system memory", as it's the largest lump.
     *
     * AN385/AN386/AN511:
     *  0x21000000 .. 0x21ffffff : PSRAM (16MB)
     * AN385/AN386/AN500:
     *  0x00000000 .. 0x003fffff : ZBT SSRAM1
     *  0x00400000 .. 0x007fffff : mirror of ZBT SSRAM1
     *  0x20000000 .. 0x203fffff : ZBT SSRAM 2&3
     *  0x20400000 .. 0x207fffff : mirror of ZBT SSRAM 2&3
     * AN385/AN386 only:
     *  0x01000000 .. 0x01003fff : block RAM (16K)
     *  0x01004000 .. 0x01007fff : mirror of above
     *  0x01008000 .. 0x0100bfff : mirror of above
     *  0x0100c000 .. 0x0100ffff : mirror of above
     * AN511 only:
     *  0x00000000 .. 0x0003ffff : FPGA block RAM
     *  0x00400000 .. 0x007fffff : ZBT SSRAM1
     *  0x20000000 .. 0x2001ffff : SRAM
     *  0x20400000 .. 0x207fffff : ZBT SSRAM 2&3
     * AN500 only:
     *  0x60000000 .. 0x60ffffff : PSRAM (16MB)
     *
     * The AN385/AN386 has a feature where the lowest 16K can be mapped
     * either to the bottom of the ZBT SSRAM1 or to the block RAM.
     * This is of no use for QEMU so we don't implement it (as if
     * zbt_boot_ctrl is always zero).
     */

/*This means a new device can be mapped lower than 0x60ffffff without affecting the emulation overall*/
/*Also take into account, the ethernet base address for a MPS2 AN500 is 0xa0000000*/
/*We can assume mapping something below that is safe*/


void intermediateDriver_init(NICInfo *nd, uint32_t base, qemu_irq irq)
{   
    /**
     * DeviceState:
     * @realized: Indicates whether the device has been fully constructed.
     *            When accessed outside big qemu lock, must be accessed with
     *            qatomic_load_acquire()
     * @reset: ResettableState for the device; handled by Resettable interface.
     *
     * This structure should not be accessed directly.  We declare it here
     * so that it can be embedded in individual device state structures.
    */
    DeviceState *dev;
    SysBusDevice *s;

    /*Check that the target NIC model exists*/
    qemu_check_nic_model(nd, targetNIC);
    dev = qdev_new(targetNIC);

    /*Set the NIC properties, f.e. macaddress*/
    qdev_set_nic_properties(dev, nd);
    s = SYS_BUS_DEVICE(dev);
    
    /**Dunno, gotta investigate*/
    sysbus_realize_and_unref(s, &error_fatal);
    
    /*Maps device to base ethernet address*/
    sysbus_mmio_map(s, 0, base);

    /*Connects an interrupt to the device*/
    sysbus_connect_irq(s, 0, irq);

    /*Initiates another NIC to redirect stuff there*/
    /*Exchange this for a generic thatr uses targetNIC*/
    /*lan9118_init(nd, 0xf0000000, irq);*/

}

/*Working but not reliable*/

/*static const MemoryRegionOps lan9118_mem_ops = {
    .read = lan9118_readl,
    .write = lan9118_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};
*/

/*
static uint64_t lan9118_readl(void *opaque, hwaddr offset,
                              unsigned size)
{
    lan9118_state *s = (lan9118_state *)opaque;

    //DPRINTF("Read reg 0x%02x\n", (int)offset);
    if (offset <= RX_DATA_FIFO_PORT_LAST) {
        // RX FIFO
        return rx_fifo_pop(s);
    }
    switch (offset) {
    case RX_STATUS_FIFO_PORT:
        return rx_status_fifo_pop(s);
    case RX_STATUS_FIFO_PEEK:
        return s->rx_status_fifo[s->rx_status_fifo_head];
    case TX_STATUS_FIFO_PORT:
        return tx_status_fifo_pop(s);
    case TX_STATUS_FIFO_PEEK:
        return s->tx_status_fifo[s->tx_status_fifo_head];
    case CSR_ID_REV:
        return 0x01180001;
    case CSR_IRQ_CFG:
        return s->irq_cfg;
    case CSR_INT_STS:
        return s->int_sts;
    case CSR_INT_EN:
        return s->int_en;
    case CSR_BYTE_TEST:
        return 0x87654321;
    case CSR_FIFO_INT:
        return s->fifo_int;
    case CSR_RX_CFG:
        return s->rx_cfg;
    case CSR_TX_CFG:
        return s->tx_cfg;
    case CSR_HW_CFG:
        return s->hw_cfg;
    case CSR_RX_DP_CTRL:
        return 0;
    case CSR_RX_FIFO_INF:
        return (s->rx_status_fifo_used << 16) | (s->rx_fifo_used << 2);
    case CSR_TX_FIFO_INF:
        return (s->tx_status_fifo_used << 16)
               | (s->tx_fifo_size - s->txp->fifo_used);
    case CSR_PMT_CTRL:
        return s->pmt_ctrl;
    case CSR_GPIO_CFG:
        return s->gpio_cfg;
    case CSR_GPT_CFG:
        return s->gpt_cfg;
    case CSR_GPT_CNT:
        return ptimer_get_count(s->timer);
    case CSR_WORD_SWAP:
        return s->word_swap;
    case CSR_FREE_RUN:
        return (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) / 40) - s->free_timer_start;
    case CSR_RX_DROP:
        return 0;
    case CSR_MAC_CSR_CMD:
        return s->mac_cmd;
    case CSR_MAC_CSR_DATA:
        return s->mac_data;
    case CSR_AFC_CFG:
        return s->afc_cfg;
    case CSR_E2P_CMD:
        return s->e2p_cmd;
    case CSR_E2P_DATA:
        return s->e2p_data;
    }
    qemu_log_mask(LOG_GUEST_ERROR, "lan9118_read: Bad reg 0x%x\n", (int)offset);
    return 0;
}
*/



