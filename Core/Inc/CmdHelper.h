/*
 * CmdHelper.h
 *
 *  Created on: Feb 17, 2022
 *      Author: ITry
 */

#ifndef INC_CMDHELPER_H_
#define INC_CMDHELPER_H_

#include "stm32g0xx_hal.h"
#include <stddef.h>

#define TYPE1_GET_VERSION 					0x104
#define TYPE1_GET_ADC	 							0x105
#define TYPE1_SET_VDD								0x201
#define TYPE1_SET_VPP							  0x202
#define TYPE1_SET_ADC								0x107
#define TYPE1_READ_ADC							0x108
#define TYPE1_CALIB									0x109
#define TYPE1_SAVE_ADC							0x200
#define TYPE1_GET_TASK_STATUS 0x400
#define TYPE1_GET_ALLTEST_DATA 0x401
#define TYPE1_WF3050_FT_DATA 0x567
#define TYPE1_WF280_M1 0x207
#define TYPE1_WF280_OUTDATA 0x208
#define TYPE1_WF280_COEFF 0x209
#define TYPE1_WF280_2V_OUTDATA 0x210
#define TYPE1_WF280_RAIL 0x211          /* 回读实际 DUT 轨压，out[0]=状态 out[1]=mV */
#define TYPE1_WF162A_TEST 9
#define TYPE1_WF100S_TEST 12
#define PRODUCT_WF100S TYPE1_WF100S_TEST
/* cmd[0] = 0x14 触发 WF280 整机异步测试。*/
#define TYPE1_WF280_TEST 0x14
#define PRODUCT_WF280 TYPE1_WF280_TEST

/* WF183/WF2200/WF3050 use 3.3 V; WF100S analog supply uses 5 V. */
#ifndef PRODUCT_TEST_VDD_MV
#define PRODUCT_TEST_VDD_MV 3300
#endif
#ifndef WF100S_TEST_VDD_MV
#define WF100S_TEST_VDD_MV 5000
#endif
#define WF183DE 0
#define WF2200 1
#define WF100S 2
#define WF162A 3
#define WF280 5
#define PRODUCT_RESULT_LENGTH 120
#define CMD_ERROR_BUSY 0xF1
#define CMD_ERROR_LENGTH 0xF2

extern volatile uint8_t IsTaskDone; /* 1: queued/running, 0: idle/completed */
extern volatile uint8_t IsTask_mub;
extern uint16_t sendBufBy[PRODUCT_RESULT_LENGTH];
void RunPendingProductTest(void);

void Parse(const uint16_t* cmd, uint8_t cmd_lens, uint16_t* out, uint16_t out_lens);
void ParseSystemCmd(const uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens);
void CopyAndClear(uint16_t* src, uint16_t* dest, size_t len);
void ParseI2CCmd(const uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens);
void ParseSerialCmd(const uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens);
void ParseOwiCmd(const uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens);
#endif /* INC_CMDHELPER_H_ */
