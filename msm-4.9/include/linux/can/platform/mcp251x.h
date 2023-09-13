#ifndef _CAN_PLATFORM_MCP251X_H
#define _CAN_PLATFORM_MCP251X_H

/*
 *
 * CAN bus driver for Microchip 251x CAN Controller with SPI Interface
 *
 */

#include <linux/spi/spi.h>

/*
 * struct mcp251x_platform_data - MCP251X SPI CAN controller platform data
 * @oscillator_frequency:       - oscillator frequency in Hz
 */

struct mcp251x_attr {
    struct device_attribute attr;
    char name[32];
};

struct mcp251x_mask {
    uint32_t sid;
    uint32_t eid;
};

struct mcp251x_filter {
    struct mcp251x_mask fid;
    int32_t exide;
};

struct mcp251x_platform_data {
	unsigned long oscillator_frequency;
    struct pinctrl *pctl;
    int    irq;
    int    irq_pin;
    int    irq_l;
    int    reset_pin;
    int    reset_l;
    int    standby_pin;
    int    standby_l;
    struct mcp251x_attr attr_mask;
    struct mcp251x_attr attr_filter;
    struct mcp251x_mask masks[2];      // two  pairs (SID/EID) one per rxb0/1
    struct mcp251x_filter filters[6];  // six  pairs (SID/EID) two for rxb0 and four for fxb1
};

#endif /* !_CAN_PLATFORM_MCP251X_H */
