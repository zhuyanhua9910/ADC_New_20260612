/*
 * CmdHelper.c
 *
 *  Created on: Feb 17, 2022
 *      Author: ITry
 */

#include <string.h>
#include "main.h"
#include "CmdHelper.h"
#include "usart.h"
#include "ADS1220.h"
#include "flash.h"
#include "owi.h"
#include "tim.h"
#include "cmsis_os.h"
#include "SoftIIC.h"
#include "MCP4725.h"
#include "SoftUart.h"
#include "wf183.h"
#include "wf100s.h"
#include "wf2200.h"
#include "wf280.h"
#include "wf162a.h"
//一类：系统指令
//二类：串口通讯方式指令
//三类: IIC通讯方式指令
//四类: SPI通讯方式指令

const uint8_t FIRMWARE_VERSION[] = { 0x01, 0x00 };
#define DATA_LENTH					64

volatile uint8_t IsTaskDone;
volatile uint8_t IsTask_mub;
uint16_t sendBufBy[PRODUCT_RESULT_LENGTH];

void RunPendingProductTest(void)
{
    if (!IsTaskDone) return;
    memset(sendBufBy, 0, sizeof(sendBufBy));
    switch (IsTask_mub) {
    case WF183DE: AllTestFor183(sendBufBy, PRODUCT_RESULT_LENGTH); break;
    case WF100S: AllTestFor100S(sendBufBy, PRODUCT_RESULT_LENGTH); break;
    case WF2200: AllTestFor2200(sendBufBy, PRODUCT_RESULT_LENGTH); break;
    case WF162A: AllTestFor162A(sendBufBy, PRODUCT_RESULT_LENGTH); break;
    case WF280: AllTestFor280(sendBufBy, PRODUCT_RESULT_LENGTH); break;
    default: break;
    }
    IsTaskDone = 0;
}

void ParseSystemCmd(const uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens)
{
	uint16_t Vout = 0;
    uint64_t saved_adc[2] = {0};
	uint32_t ADC_Value = 0;
	uint16_t cmdHead;
    uint8_t required_cmd = 2;
    uint16_t required_out = 1;
    if (!cmd || !out || !out_lens) return;
    if (cmd_lens < 2) { out[0] = CMD_ERROR_LENGTH; return; }
    cmdHead = cmd[1];
    switch (cmdHead) {
    case TYPE1_GET_VERSION: required_out = 2; break;
    case TYPE1_GET_ADC: case TYPE1_READ_ADC: required_out = 4; break;
    case TYPE1_SET_ADC: case TYPE1_SAVE_ADC: required_cmd = 6; break;
    case TYPE1_SET_VDD: required_cmd = 4; break;
    case TYPE1_SET_VPP: required_cmd = 3; break;
    case TYPE1_WF280_M1: required_cmd = 3; break;
    case TYPE1_WF280_COEFF: required_cmd = 3; break;
    case TYPE1_WF280_OUTDATA: case TYPE1_WF280_2V_OUTDATA: required_cmd = 4; break;
    case TYPE1_WF280_RAIL: required_out = 2; break;
    default: break;
    }
    if (cmd_lens < required_cmd || out_lens < required_out) {
        out[0] = CMD_ERROR_LENGTH; return;
    }
	switch (cmdHead) 
	{
		
		case TYPE1_GET_VERSION://104:获取版本号
			out[0] = FIRMWARE_VERSION[0];
			out[1] = FIRMWARE_VERSION[1];
		break;
		
		case TYPE1_GET_ADC://105：获取adc值
			for(uint8_t i = 0;i<10;i++)
			{
				ADC_Value = GET_ADS1220_Val(&ADS1220);
				out[0] = (uint8_t)(ADC_Value); 
				out[1] = (uint8_t)(ADC_Value>>8);
				out[2] = (uint8_t)(ADC_Value>>16);
				out[3] = (uint8_t)(ADC_Value>>24);
				if(ADC_Value)break;
			}
			break;
		
		case TYPE1_SET_ADC://107：设置adc模式不保存参数
			ADS1220.config[0] = cmd[2];
			ADS1220.config[1] = cmd[3];
			ADS1220.config[2] = cmd[4];
			ADS1220.config[3] = cmd[5];
			ADS1220_Init(&ADS1220);
			out[0] = ADS1220.state;
		break;
		
		case TYPE1_SAVE_ADC://200：设置adc模式，保存参数至flash
			ADS1220.config[0] = cmd[2];
			ADS1220.config[1] = cmd[3];
			ADS1220.config[2] = cmd[4];
			ADS1220.config[3] = cmd[5];
			ADS1220_Init(&ADS1220);
			memcpy(saved_adc, &ADS1220, sizeof(ADS1220));
            Write_Flash(FLASH_USER_START_ADDR, saved_adc, 2);
        
			out[0] = ADS1220.state;
		break;
		
		case TYPE1_READ_ADC://108：读adc参数
			out[0] = ADS1220.config[0];
			out[1] = ADS1220.config[1];
			out[2] = ADS1220.config[2];
			out[3] = ADS1220.config[3];
		break;
		
		case TYPE1_SET_VDD://201：设置adc供电使能 
		  Vout = ((uint16_t)(cmd[3] << 8) | cmd[2]);
			MCP4725_WriteData_Voltage(Vout);
			WF162A_SetConfiguredSupply(Vout);
            out[0] = 0;
		break;
				
		case TYPE1_SET_VPP://202：设置VPP供电使能 
			HAL_GPIO_WritePin(VPP_EN_GPIO_Port, VPP_EN_Pin, cmd[2] ? GPIO_PIN_SET:GPIO_PIN_RESET);
			out[0] =HAL_GPIO_ReadPin(VPP_EN_GPIO_Port, VPP_EN_Pin);
		break;

        case TYPE1_GET_TASK_STATUS:
            out[0] = IsTaskDone;
            break;
        case TYPE1_GET_ALLTEST_DATA:
            if (IsTaskDone) out[0] = CMD_ERROR_BUSY;
            else 
						{
								memcpy(out, sendBufBy, (out_lens < PRODUCT_RESULT_LENGTH ? out_lens : PRODUCT_RESULT_LENGTH) * sizeof(*out));
								//out[0]=1;
						}
            break;
        case TYPE1_WF3050_FT_DATA:
            if (out_lens < 12) out[0] = CMD_ERROR_LENGTH;
            else WF3050FtData(out, out_lens);
            break;
        case TYPE1_WF280_M1:
            if (cmd[2] == 0 || cmd[2] > WF280_COEFF_MAX ||
                out_lens < (uint16_t)(5 + cmd[2])) out[0] = CMD_ERROR_LENGTH;
            else (void)WF280ReadM1((uint8_t)cmd[2], out, out_lens);
            break;
        case TYPE1_WF280_COEFF:
            if (cmd[2] == 0 || cmd[2] > WF280_COEFF_MAX ||
                out_lens < (uint16_t)(cmd[2] + 1)) out[0] = CMD_ERROR_LENGTH;
            else (void)WF280ReadCoefficients(0, (uint8_t)cmd[2], out, out_lens);
            break;
        case TYPE1_WF280_OUTDATA:
        case TYPE1_WF280_2V_OUTDATA:
            if (out_lens < WF280_OUTPUT_RESULT_LENGTH) out[0] = CMD_ERROR_LENGTH;
            else (void)WF280TestOutput(cmdHead == TYPE1_WF280_2V_OUTDATA ? WF280_SUPPLY_1V8_MV : WF280_SUPPLY_3V3_MV,
                                       cmd[2], cmd[3], out, out_lens);
            break;
        case TYPE1_WF280_RAIL: {
            /* 只做测量，不改变供电状态：out[0]=状态，out[1]=实际轨压 mV。 */
            uint16_t rail_mv = WF280ReadRailMv();
            out[0] = rail_mv ? WF280_STATUS_OK : WF280_STATUS_RAIL;
            out[1] = rail_mv;
            break;
        }
        default:
		break;
	}
}

void ParseSerialCmd(const uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens)
{
    uint16_t sendLens, recvLens, i, baud = 0;
    uint8_t SendBuff[DATA_LENTH] = {0};
    uint8_t RecvBuff[DATA_LENTH] = {0};
    if (!cmd || !out || !out_lens) return;
    if (cmd_lens < 6) { out[0] = CMD_ERROR_LENGTH; return; }
    sendLens = cmd[2]; recvLens = cmd[4];
    if (sendLens > DATA_LENTH || recvLens > DATA_LENTH ||
        cmd_lens < 6 + sendLens || out_lens < 2 + recvLens) {
        out[0] = CMD_ERROR_LENGTH; return;
    }
    memset(out, 0, out_lens * sizeof(*out));
    for (i = 0; i < sendLens; ++i) SendBuff[i] = (uint8_t)cmd[i + 6];
    UART_Init(&uart1, 9600, UART_TX_GPIO_Port, UART_TX_Pin, UART_RX_GPIO_Port, UART_RX_Pin);
    osDelay(10);
    Soft_Uart_Send(&uart1, SendBuff, sendLens, cmd[3]);
    if (recvLens) {
        Soft_Uart_Recv(&uart1, RecvBuff, recvLens, cmd[5]);
        /* Switching from I2C or a previous UART transaction can leave the
           first start bit missed. Re-arm the pins and retry one no-response
           transaction; a valid response is never sent twice. */
        if (uart1.Save_BuadRate == 0.0f) {
            UART_DeInit(&uart1);
            UART_Init(&uart1, 9600, UART_TX_GPIO_Port, UART_TX_Pin,
                      UART_RX_GPIO_Port, UART_RX_Pin);
            osDelay(2);
            Soft_Uart_Send(&uart1, SendBuff, sendLens, cmd[3]);
            Soft_Uart_Recv(&uart1, RecvBuff, recvLens, cmd[5]);
        }
    }
    if (uart1.Save_BuadRate > 0) baud = (uint16_t)(1000000.0f / uart1.Save_BuadRate);
    out[0] = (uint8_t)(baud >> 8); out[1] = (uint8_t)baud;
    for (i = 0; i < recvLens; ++i) out[i + 2] = RecvBuff[i];
    UART_DeInit(&uart1);
}

void ParseI2CCmd(const uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens)
{
    uint16_t sendLens, recvLens, i;
    uint8_t SendBuff[DATA_LENTH] = {0};
    uint8_t RecvBuff[DATA_LENTH] = {0};
    uint8_t legacy_data[DATA_LENTH + 1];
    if (!cmd || !out || !out_lens) return;

    /* Keep the original host protocol accepted by the WF test software:
       {3, channel, 0x11/0x22, address, reserved, length, payload...}.
       The current protocol below uses explicit send/receive lengths. */
    if (cmd_lens >= 6 && (cmd[2] == 0x11 || cmd[2] == 0x22)) {
        uint16_t length = cmd[5];
        uint8_t err;
        if (length > DATA_LENTH ||
            (cmd[2] == 0x22 && out_lens <= length) ||
            (cmd[2] == 0x11 && cmd_lens < 6 + length)) {
            out[0] = CMD_ERROR_LENGTH;
            return;
        }
        IIC_Init(&i2c1, I2C_SDA_GPIO_Port, I2C_SDA_Pin,
                 I2C_SCL_GPIO_Port, I2C_SCL_Pin);
        if (cmd[2] == 0x11) {
            legacy_data[0] = (uint8_t)(cmd[3] & 0xFE);
            for (i = 0; i < length; ++i) legacy_data[i + 1] = (uint8_t)cmd[i + 6];
            err = I2C_WriteNByte(&i2c1, legacy_data, length + 1);
            out[0] = err;
        } else {
            err = I2C_ReadNByte(&i2c1, (uint8_t)cmd[3], legacy_data, length);
            for (i = 0; i < length; ++i) out[i] = legacy_data[i];
            out[length] = err;
        }
        IIC_DeInit(&i2c1);
        return;
    }
    if (cmd_lens < 7) { out[0] = CMD_ERROR_LENGTH; return; }
    sendLens = cmd[2]; recvLens = cmd[4];
    if (!sendLens || sendLens > DATA_LENTH || recvLens > DATA_LENTH ||
        cmd_lens < 6 + sendLens || out_lens < 2 + recvLens) {
        out[0] = CMD_ERROR_LENGTH; return;
    }
    memset(out, 0, out_lens * sizeof(*out));
    for (i = 0; i < sendLens; ++i) SendBuff[i] = (uint8_t)cmd[i + 6];
    IIC_Init(&i2c1, I2C_SDA_GPIO_Port, I2C_SDA_Pin, I2C_SCL_GPIO_Port, I2C_SCL_Pin);
    osDelay(10);
    out[0] = I2C_WriteNByte(&i2c1, SendBuff, sendLens);
    if (!out[0] && recvLens) {
        out[1] = I2C_ReadNByte(&i2c1, SendBuff[0], RecvBuff, recvLens);
        for (i = 0; i < recvLens; ++i) out[i + 2] = RecvBuff[i];
    }
    IIC_DeInit(&i2c1);
}

void ParseOwiCmd(const uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens)
{
	uint8_t RecvBuff[DATA_LENTH] = {0};
	(void)cmd;
	(void)cmd_lens;
    if (!out || out_lens < 4) return;
	OWI_Device.cycle = 0;
	OWI(RecvBuff,200);
	osDelay(100);
	OWI(RecvBuff,200);
	out[0] = OWI_Device.cycle;
	out[1] = RecvBuff[0];
	out[2] = RecvBuff[1];
	out[3] = RecvBuff[2];
}
/*
 *
 */
void Parse(const uint16_t *cmd, uint8_t cmd_lens, uint16_t *out, uint16_t out_lens)
{
	uint16_t cmdType;
    if (!cmd || !cmd_lens || !out || !out_lens) return;
    cmdType = cmd[0];
    /* Only status/results may be queried while the product task owns the peripherals. */
    if (IsTaskDone && !(cmdType == 1 && cmd_lens >= 2 &&
        (cmd[1] == TYPE1_GET_TASK_STATUS || cmd[1] == TYPE1_GET_ALLTEST_DATA))) {
        out[0] = CMD_ERROR_BUSY; return;
    }
	switch (cmdType) 
	{
		
		case 1:
			ParseSystemCmd(cmd, cmd_lens, out, out_lens);
		break;
		
		case 2:
			ParseSerialCmd(cmd, cmd_lens, out, out_lens);
		break;
		
		case 3:
			ParseI2CCmd(cmd, cmd_lens, out, out_lens);
		break;
		
		case 5:
			ParseOwiCmd(cmd, cmd_lens, out, out_lens);
		break;
		
        case 8:
            IsTask_mub = WF183DE;
            IsTaskDone = 1;
            out[0] = 1;
            break;
        case TYPE1_WF162A_TEST:
            IsTask_mub = WF162A;
            IsTaskDone = 1;
            out[0] = 1;
            break;
        case 10:
            IsTask_mub = WF2200;
            IsTaskDone = 1;
            out[0] = 1;
            break;
        case TYPE1_WF100S_TEST:
            IsTask_mub = WF100S;
            IsTaskDone = 1;
            out[0] = 1;
            break;
        case 11:
            if (out_lens < WF280_RESULT_LENGTH) out[0] = CMD_ERROR_LENGTH;
            else WF280CheckOtpReg(out, out_lens);
            break;
        case TYPE1_WF280_TEST: // 14
            IsTask_mub = WF280;
            IsTaskDone = 1;
            out[0] = 1;
            break;
        default:
            break;
    }
}

// 拷贝 src 到 dest，然后清零 src
void CopyAndClear(uint16_t* src, uint16_t* dest, size_t len) 	
{
	if (!src || !dest || len == 0) return;
	size_t bytes = len * sizeof(uint16_t);
	memcpy(dest, src, bytes);
	memset(src, 0, bytes);
}
   







