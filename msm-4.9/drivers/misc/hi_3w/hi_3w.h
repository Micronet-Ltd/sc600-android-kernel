/*
 * Copyright (C) 2019 Micronet Inc, All Right Reserved.
 *
 * Athor: vladimir.zatulovsky@micronet-inc.com
 *        marina.shaevich@micronet-inc.com
 *
 */

#include <linux/types.h>

#define GPIO_LOW            0
#define GPIO_HIGH           1
#define WAIT_DELAY_INTERVAL 2500 
#define WAIT_DATA_VALID     (WAIT_DELAY_INTERVAL >> 1)
#define WAIT_DATA_INVALID   (WAIT_DELAY_INTERVAL - WAIT_DATA_VALID)
#define RESPONSE_MSG_LEN    23
#define CMD_LEN             8


//Errors
#define NO_ERROR            0
#define GENERAL_ERROR       -1
#define NOT_AVAILABLE       -2
#define CMD_REJECTED        -3

extern int hi_3w_tx_cmd(uint32_t *, bool);

