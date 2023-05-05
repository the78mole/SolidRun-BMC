/*
 * user.c
 *
 *  Created on: May 2, 2023
 *      Author: dagl274982
 */

#include "gpio.h"
#include "adc.h"
#include "usbd_cdc_if.h"
#include "user.h"

RingBuffer rb = { 0 };

typedef int (*pCmdFunc)(GPIO_PinState);


typedef struct {
	uint8_t cmd;
	pCmdFunc func;
	GPIO_PinState gpio;
} t_cmdEntry;

static const uint8_t introStr[]   = " Result:";
static const uint8_t measStr[]    = " Measurement: ";
static const uint8_t okStr[]      = " OK";
static const uint8_t failStr[]    = " FAILED";
static const uint8_t lineEndStr[] = "\r\n";

#ifdef FEATURE_ANALOG
static const uint8_t mvStr[]    = " mV";
static const uint8_t maStr[]    = " mA";
#else
static const uint8_t niyStr[]     = " NIY";
#endif

int cmd_reset(GPIO_PinState state) {
	HAL_GPIO_WritePin(MASTER_RESET_GPIO_Port, MASTER_RESET_Pin, state);
	return 1;
}

int cmd_powerBtn(GPIO_PinState state) {
	HAL_GPIO_WritePin(POWER_BUTTON_GPIO_Port, POWER_BUTTON_Pin, state);
	return 1;
}

int cmd_recoveryBoot(GPIO_PinState state) {
	HAL_GPIO_WritePin(FORCE_RECOVERY_BOOT_GPIO_Port, FORCE_RECOVERY_BOOT_Pin, state);
	return 1;
}

uint16_t uv_volts, uv_amps;

int cmd_getAnalog(GPIO_PinState state) {
//	MX_ADC_GetValues(&uv_volts, &uv_amps);
	return 1;
}

static const t_cmdEntry cmdTable[] = {
		{ 'R', cmd_reset, GPIO_PIN_RESET },
		{ 'r', cmd_reset, GPIO_PIN_SET },
		{ 'P', cmd_powerBtn, GPIO_PIN_RESET },
		{ 'p', cmd_powerBtn, GPIO_PIN_SET },
		{ 'B', cmd_recoveryBoot, GPIO_PIN_RESET },
		{ 'b', cmd_recoveryBoot, GPIO_PIN_SET },
		{ 'a', cmd_getAnalog, GPIO_PIN_SET },
		{ 0, NULL, GPIO_PIN_RESET }  // Keep this as last entry, it will break the processing
};

void printAnalog(uint16_t val, uint8_t* unitStr) {

	uint8_t valBuf[10];
	while(CDC_Check_State() != USBD_OK);
	memset(valBuf, 0, sizeof(valBuf));
	itoa(val, (char *) valBuf, 10);
	CDC_Transmit_FS(valBuf, sizeof(valBuf));

	while(CDC_Check_State() != USBD_OK);
	CDC_Transmit_FS((uint8_t*) unitStr, sizeof(unitStr));
}

void processCommands(void){
	uint8_t cmd;
	int retval;
	const t_cmdEntry* pCmd = &cmdTable[0];

	if(rb_avail(&rb)) {
		rb_pop(&rb, &cmd);
		CDC_Transmit_FS(&cmd, 1);
		while(pCmd->cmd) {
			if(pCmd->cmd == cmd) {
				retval = pCmd->func(pCmd->gpio);
				while(CDC_Check_State() != USBD_OK);

				if(pCmd->cmd == 'a') {
					CDC_Transmit_FS((uint8_t*) measStr, sizeof(introStr));
				} else {
					CDC_Transmit_FS((uint8_t*) introStr, sizeof(introStr));
				}


				if(retval) {
					if (pCmd->cmd == 'a') {

#ifdef FEATURE_ANALOG
						printAnalog(uv_volts, (uint8_t*) mvStr);
						printAnalog(uv_amps, (uint8_t*) maStr);
#else
						while(CDC_Check_State() != USBD_OK);
						CDC_Transmit_FS((uint8_t*) niyStr, sizeof(niyStr));
#endif

					}
					while(CDC_Check_State() != USBD_OK);
					CDC_Transmit_FS((uint8_t*) okStr, sizeof(okStr));
				} else {
					CDC_Transmit_FS((uint8_t*) failStr, sizeof(failStr));
				}

				while(CDC_Check_State() != USBD_OK);
				CDC_Transmit_FS((uint8_t*) lineEndStr, sizeof(lineEndStr));
				break;
			}
			pCmd++;
		}
	}
}

int8_t processCdcRX(uint8_t *Buf, uint16_t len) {

	uint8_t* pBuf = &Buf[0];
	uint8_t curChar;

	//HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

	if(!rb.isInit) rb_init(&rb);


	for(int i = 0; i < len; i++) {
		curChar = *pBuf++;
		rb_push(&rb, curChar);
	}
#if 0
		switch(curChar) {
			case '\r':
			case '\n':
				if(curCmdPos > 0) {
					parseCommand(cmdBuf, curCmdPos, txBuf, &curTxPos);
					curCmdPos = 0;
				}
				break;
			case '\e':
				memset(cmdBuf, 0, CMDBUFSIZE);
				curCmdPos = 0;
				memcpy(&txBuf[curTxPos], TX_RST, sizeof(TX_RST));
				curTxPos += sizeof(TX_RST);
				break;
			case '\0':
				break;
			default:
				txBuf[curTxPos++] = curChar;
				cmdBuf[curCmdPos++] = curChar;
				break;
		}
		curBufPos++;
		if(curTxPos > 0) {
			CDC_Transmit_FS(txBuf, curTxPos);
			curTxPos = 0;
		}
#endif

	return (USBD_OK);
}
