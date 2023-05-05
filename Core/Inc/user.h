/*
 * user.h
 *
 *  Created on: May 2, 2023
 *      Author: dagl274982
 */

#ifndef INC_USER_H_
#define INC_USER_H_

#include "ringbuffer.h"

//#define CMDBUFSIZE 16
//#define TXBUFSIZE 16

//#define TX_RST   " ^RST\r\n"
//#define TX_ENT   " ^ENT\r\n"

int8_t processCdcRX(uint8_t *Buf, uint16_t len);
void processCommands(void);

#endif /* INC_USER_H_ */
