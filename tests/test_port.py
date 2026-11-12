#!/usr/bin/env python3
"""Run product port regressions with simulated hardware; requires a host C compiler."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
HEADERS = {
    "stm32g0xx_hal.h": r'''
#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef struct { uint32_t BRR, BSRR; } GPIO_TypeDef;
extern GPIO_TypeDef fake_a, fake_b, fake_c;
#define GPIOA (&fake_a)
#define GPIOB (&fake_b)
#define GPIOC (&fake_c)
#define GPIO_PIN_0 1u
#define GPIO_PIN_1 2u
#define GPIO_PIN_3 8u
#define GPIO_PIN_4 16u
#define GPIO_PIN_5 32u
#define GPIO_PIN_6 64u
#define GPIO_PIN_7 128u
#define GPIO_PIN_8 256u
#define GPIO_PIN_11 2048u
#define GPIO_PIN_12 4096u
typedef enum {GPIO_PIN_RESET, GPIO_PIN_SET} GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_OUTPUT_OD 2
#define GPIO_MODE_INPUT 3
#define GPIO_MODE_ANALOG 4
#define GPIO_NOPULL 0
#define GPIO_PULLUP 1
#define GPIO_SPEED_FREQ_LOW 0
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
typedef enum { HAL_OK = 0u, HAL_ERROR = 1u, HAL_BUSY = 2u, HAL_TIMEOUT = 3u } HAL_StatusTypeDef;
typedef struct { uint32_t dummy; } ADC_HandleTypeDef;
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *, uint32_t);
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *);
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *);
#endif
''',
    "cmsis_os.h": "#include <stdint.h>\nvoid osDelay(uint32_t);\n",
    "usart.h": "",
    "tim.h": r'''
#include <stdint.h>
extern int htim2;
void test_counter_set(uint32_t);
uint32_t test_counter_get(void);
#define __HAL_TIM_SetCounter(t, v) test_counter_set(v)
#define __HAL_TIM_GET_COUNTER(t) test_counter_get()
''',
    "flash.h": "#include <stdint.h>\n#define FLASH_USER_START_ADDR 0x0800F800u\nvoid Write_Flash(uint32_t, uint64_t *, uint16_t);\n",
}
HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "CmdHelper.h"
#include "ADS1220.h"
#include "SoftIIC.h"
#include "SoftUart.h"
#include "I2CHelper.h"
#include "owi.h"
#include "wf280.h"
#include "wf162a.h"
#include "wf183.h"
#include "wf2200.h"
#include "wf100s.h"
GPIO_TypeDef fake_a, fake_b, fake_c;
ADS1220_TypeDef ADS1220;
ADC_HandleTypeDef hadc1;
OWI_TypeDef OWI_Device;
I2CInfo i2c1;
UartInfo uart1;
static unsigned voltage, last_power_on_mv, power_on, power_off, writes, reads, selected, last_command, mismatch, nack, adc_inits, adc_read_fail;
static unsigned tx_pin, rx_pin, uart_receives, uart_fail_once, flash_words;
static uint32_t last_gpio_mode;
static uint32_t adc_raw = 0x123456;
static uint32_t rail_code = 32760;
void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,GPIO_PinState s) {(void)p;(void)pin;(void)s;}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t pin) {(void)p;(void)pin;return GPIO_PIN_RESET;}
void HAL_GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *i) {(void)p;last_gpio_mode=i->Mode;}
void osDelay(uint32_t n) {(void)n;}
/* PB1 / ADC1_IN9 回读：满量程 65520 对应 VDDA*2。 */
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h) {(void)h;return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h) {(void)h;return HAL_OK;}
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *h,uint32_t t) {(void)h;(void)t;return HAL_OK;}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h) {(void)h;return rail_code;}
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *h) {(void)h;return HAL_OK;}
void delay_us(uint32_t n) {(void)n;}
void MCP4725_WriteData_Voltage(uint16_t mv) {voltage=mv;if(mv){last_power_on_mv=mv;++power_on;}else ++power_off;}
void ADS1220_Init(ADS1220_TypeDef *a) {a->state=1;++adc_inits;}
uint32_t GET_ADS1220_Val(ADS1220_TypeDef *a) {
 if(a->config[0]==0xD1u)return voltage*1024u;
 return adc_raw;
}
uint8_t ADS1220_ReadValue(ADS1220_TypeDef *a,uint32_t *value) {
 if(!a||!value)return 0;
 if(adc_read_fail)return 0;
 *value=(a->config[0]==0xD1u)?voltage*1024u:adc_raw;
 a->Data=*value;
 return 1;
}
void ADS1220_WriteRegister(uint32_t a,uint32_t n,uint8_t *p) {(void)a;(void)n;(void)p;}
void Write_Flash(uint32_t a,uint64_t *p,uint16_t n) {(void)a;flash_words=n;assert(!memcmp(p,&ADS1220,sizeof ADS1220));}
uint8_t OWI(uint8_t *p,uint32_t t) {(void)t;OWI_Device.cycle=42;p[0]=1;p[1]=2;p[2]=3;return 1;}
void UART_Init(UartInfo *u,uint32_t b,GPIO_TypeDef *tp,uint16_t t,GPIO_TypeDef *rp,uint16_t r) {
 assert(tp==UART_TX_GPIO_Port && rp==UART_RX_GPIO_Port);tx_pin=t;rx_pin=r;u->BuadRate=1000000/b;u->Save_BuadRate=0;
}
void UART_DeInit(UartInfo *u) {(void)u;}
void Soft_Uart_Send(UartInfo *u,uint8_t *b,uint16_t n,uint32_t t) {(void)u;(void)b;(void)n;(void)t;}
void Soft_Uart_Recv(UartInfo *u,uint8_t *b,uint16_t n,uint32_t t) {
 (void)t;++uart_receives;if(nack||uart_fail_once){uart_fail_once=0;return;}u->Save_BuadRate=104.f;memset(b,0,n);if(n>=2){b[0]=0xAA;b[1]=8;}
}
void IIC_Init(I2CInfo *i,GPIO_TypeDef *sd,uint16_t sdp,GPIO_TypeDef *sc,uint16_t scp) {
 assert(sd==I2C_SDA_GPIO_Port&&sdp==I2C_SDA_Pin&&sc==I2C_SCL_GPIO_Port&&scp==I2C_SCL_Pin);
 i->portSda=sd;i->portScl=sc;i->pinSda=sdp;i->pinScl=scp;i->error=0;
}
void IIC_DeInit(I2CInfo *i) {(void)i;}
uint8_t I2C_WriteNByte(I2CInfo *i,const uint8_t *b,uint32_t n) {
 (void)i;++writes;assert(n>=2&&!(b[0]&1));
 if(b[0]==0x70){last_command=b[1];selected=(n==2)?b[1]:b[2];}
 return nack;
}
uint8_t I2C_ReadNByte(I2CInfo *i,uint8_t a,uint8_t *b,uint32_t n) {
 (void)i;++reads;if(nack)return nack;
 for(unsigned j=0;j<n;++j)b[j]=j+1;
 if(a==0x70){
  if(n==1){b[0]=1;}
  else if(n==3){
   if(last_command==0xFE){b[0]=1;b[1]=0x0D;b[2]=1;}
   else {b[0]=selected;b[1]=0x12;b[2]=(selected&31)+(mismatch&&(selected&0x80));}
  }
  else if(n==4){assert(last_command==0xAD||last_command==0xAE);for(unsigned j=0;j<n;++j)b[j]=(uint8_t)(last_command+j);}
  else {assert(n==6);for(unsigned j=0;j<n;++j)b[j]=(uint8_t)(last_command+j);}
 }return 0;
}
void iic_start(I2CInfo *i){(void)i;}
void iic_stop(I2CInfo *i){(void)i;}
void iic_sendbyte(I2CInfo *i,uint8_t b){(void)i;(void)b;}
uint8_t iic_wait_ack(I2CInfo *i){(void)i;return nack;}
uint8_t iic_readbyte(I2CInfo *i,uint8_t a){(void)i;(void)a;return 0x33;}
void pp_iic_init(GPIO_TypeDef *sd,uint16_t sdp,GPIO_TypeDef *sc,uint16_t scp){assert(sd==GPIOB&&sdp==UART_RX_Pin&&sc==GPIOB&&scp==UART_TX_Pin);}
void pp_iic_start(void){}
void pp_iic_stop(void){}
void pp_iic_sendbyte(uint8_t b){(void)b;}
uint8_t pp_iic_wait_ack(void){return nack;}
uint8_t pp_iic_readbyte(uint8_t a){(void)a;return 0x44;}
static void reset(void){writes=reads=selected=last_command=mismatch=nack=power_on=power_off=uart_receives=adc_inits=adc_read_fail=0;uart_fail_once=0;last_power_on_mv=0;adc_raw=0x123456;IsTaskDone=0;}
int main(void){
 uint16_t out[122], c[70]={0};
 reset();c[0]=1;c[1]=TYPE1_SET_VDD;c[2]=0x88;c[3]=0x13;Parse(c,4,out,120);assert(voltage==5000);
 c[1]=TYPE1_SAVE_ADC;c[2]=1;c[3]=2;c[4]=3;c[5]=4;Parse(c,6,out,120);assert(flash_words==2);
 /* Generic latest-format I2C keeps status at 0/1 and data at 2. */
 uint16_t combined[]={3,0,2,0,4,0,0xDA,0x06};Parse(combined,8,out,120);assert(writes==1&&reads==1&&out[0]==0&&out[1]==0&&out[2]==1&&out[5]==4);
 /* The original WF host format remains accepted by the latest parser. */
 reset();uint16_t legacy_write[]={3,0,0x11,0xDA,0,2,0x0A,0x04};ParseI2CCmd(legacy_write,8,out,10);assert(writes==1&&out[0]==0);
 reset();uint16_t legacy_read[]={3,0,0x22,0xDA,0,3};ParseI2CCmd(legacy_read,6,out,10);assert(reads==1&&out[0]==1&&out[1]==2&&out[2]==3&&out[3]==0);
 c[0]=3;c[2]=65;Parse(c,70,out,120);assert(out[0]==CMD_ERROR_LENGTH);
 /* Product reads do not inject an extra address-only write. */
 reset();uint16_t readcmd[]={3,0,1,0,3,0,0xDA};assert(ProductI2CTransfer(readcmd,7,out,10)==0);assert(writes==0&&reads==1&&out[0]==1&&out[2]==3&&out[3]==0);
 nack=1;assert(ProductI2CTransfer(readcmd,7,out,10)==1&&out[0]==0&&out[3]==1);
 reset();WF280CheckOtpReg(out,120);assert(out[0]==1&&out[1]==0&&writes==64&&reads==128);
 reset();mismatch=1;WF280CheckOtpReg(out,120);assert(out[0]==0xFF&&out[1]==1);
 reset();nack=1;WF280CheckOtpReg(out,120);assert(out[0]==0xFF&&out[1]==1);
 reset();out[0]=0xABCD;WF280CheckOtpReg(out,1);assert(out[0]==0xABCD&&writes==0);
 /* WF280 command adapter: reset/poll/read, both output rails, coefficient
    reset, M1 current bytes and immediate short-circuit shutdown. */
 reset();uint16_t wf280_out[96]={0};
 assert(WF280ReadCoefficients(0,2,wf280_out,40)==0&&wf280_out[0]==0&&
        wf280_out[1]==0x1200&&wf280_out[2]==0x1201&&last_power_on_mv==3300&&voltage==0);
 reset();assert(WF280ReadM1(2,wf280_out,40)==0&&wf280_out[0]==0&&
        wf280_out[1]==0x56&&wf280_out[2]==0x34&&wf280_out[3]==0x56&&wf280_out[4]==0x34&&
        last_power_on_mv==2000&&voltage==0);
 reset();assert(WF280TestOutput(3300,0x1234,5,wf280_out,40)==0&&wf280_out[0]==0&&
        wf280_out[1]==0xAC&&wf280_out[6]==0xB1&&last_power_on_mv==3300&&voltage==0);
 reset();nack=1;assert(WF280TestOutput(3300,0x1234,5,wf280_out,40)==WF280_STATUS_I2C&&
        wf280_out[0]==WF280_STATUS_I2C&&power_off==1&&voltage==0);
reset();adc_raw=0xFFFFFF;assert(WF280TestOutput(2000,0x1234,5,wf280_out,40)==WF280_STATUS_SHORT&&
        wf280_out[0]==WF280_STATUS_SHORT&&writes==0&&power_off==1&&voltage==0);
/* cmd=14 整机测试（精简 10 字帧）：轨压 + 休眠/工作电流 + 0xAC 六字节原始转存 + 测量错误码，结束断电。
   rail_code=32760 对应 65520 的 50 %，即 VDDA*2/2 = 3300 mV。
   0xAC 读回 AC AD AE AF B0 B1（Status + 5 字节打包），按手册 4.3 拆包：
   电桥 = AD<<12|AE<<4|AF>>4 = 0xADAEF，温度 = (AF&0xF)<<16|B0<<8|B1 = 0xAF0B1。 */
reset();AllTestFor280(wf280_out,96);
assert(wf280_out[WF280_R_RAIL_MV]==3300&&
       wf280_out[WF280_R_WORK_UA]==3911&&wf280_out[WF280_R_SLEEP_UA]==3911&&
       wf280_out[WF280_R_RAW]==0xAC&&wf280_out[WF280_R_RAW+1]==0xAD&&
       wf280_out[WF280_R_RAW+2]==0xAE&&wf280_out[WF280_R_RAW+3]==0xAF&&
       wf280_out[WF280_R_RAW+4]==0xB0&&wf280_out[WF280_R_RAW+5]==0xB1&&
       wf280_out[WF280_R_MEAS_ERR]==0&&
       power_on==1&&power_off==1&&voltage==0);
/* 轨没起来就不去测芯片：轨压字保持 0，且不产生 I2C 事务。 */
reset();rail_code=0;AllTestFor280(wf280_out,96);
assert(wf280_out[WF280_R_RAIL_MV]==0&&writes==0&&power_off==1&&voltage==0);
rail_code=32760;
/* All product paths preserve existing result offsets and power down. */
 reset();AllTestFor183(out,120);assert(last_power_on_mv==3300&&power_on==1&&power_off==1&&voltage==0);assert(out[2]==0xAA&&out[3]==8&&out[10]==42&&out[90]==1&&out[100]==1);assert(tx_pin==UART_TX_Pin&&rx_pin==UART_RX_Pin);
 reset();AllTestFor2200(out,120);assert(last_power_on_mv==3300&&power_on==1&&power_off==1&&out[0]==0x56&&out[4]==1&&out[8]==1&&out[22]==1&&out[73]==0x56);
 reset();ADS1220.config[0]=0x39;AllTestFor100S(out,120);assert(last_power_on_mv==5000&&power_on==1&&power_off==1&&ADS1220.config[0]==0x39&&out[0]==0x56&&out[12]==0x56);
 reset();WF3050FtData(out,120);assert(last_power_on_mv==3300&&out[0]==0&&out[1]==0x44&&out[6]==0&&out[7]==1&&voltage==0&&last_gpio_mode==GPIO_MODE_ANALOG);
 reset();nack=1;uint16_t sleep[]={2,0,4,100,0,0,0x55,4,2,0xC9};ParseSerialCmd(sleep,10,out,10);assert(out[0]==0&&out[1]==0);
 reset();nack=1;uint16_t noresponse[]={2,0,4,100,8,20,0x55,4,0x24,0x37};ParseSerialCmd(noresponse,10,out,10);assert(out[0]==0&&out[1]==0);
 reset();nack=1;AllTestFor183(out,120);assert(power_off==1&&voltage==0);
 reset();uart_fail_once=1;uint16_t serial_retry[]={2,0,4,100,2,20,0x55,4,0x24,0x37};ParseSerialCmd(serial_retry,10,out,10);assert(uart_receives==2&&out[2]==0xAA&&out[3]==8);
 /* Queued requests, busy protection, completion and saved results. */
 const unsigned async_commands[] = {8u, 10u, PRODUCT_WF100S};
 for(unsigned index=0;index<3;++index){unsigned type=async_commands[index];reset();c[0]=type;Parse(c,1,out,120);assert(IsTaskDone==1&&out[0]==1);
  c[0]=1;c[1]=TYPE1_GET_TASK_STATUS;Parse(c,2,out,120);assert(out[0]==1);
  c[1]=TYPE1_SET_VDD;Parse(c,4,out,120);assert(out[0]==CMD_ERROR_BUSY&&power_on==0);
  c[1]=TYPE1_GET_ALLTEST_DATA;Parse(c,2,out,120);assert(out[0]==CMD_ERROR_BUSY);
  RunPendingProductTest();assert(IsTaskDone==0&&power_off==1&&last_power_on_mv==(type==PRODUCT_WF100S?5000:3300));Parse(c,2,out,120);assert(!memcmp(out,sendBufBy,240));
 }
 /* WF162A uses the same asynchronous product protocol as WF100S. The host
    selects the supply with 0x201; one cmd=9 captures one selected round. */
 reset();c[0]=1;c[1]=TYPE1_SET_VDD;c[2]=3300&0xFF;c[3]=3300>>8;Parse(c,4,out,120);
 reset();ADS1220.config[0]=0x39;c[0]=PRODUCT_WF162A;Parse(c,1,out,120);assert(IsTaskDone==1&&out[0]==1);
 c[0]=1;c[1]=TYPE1_GET_TASK_STATUS;Parse(c,2,out,120);assert(out[0]==1);
 c[1]=TYPE1_GET_ALLTEST_DATA;Parse(c,2,out,120);assert(out[0]==CMD_ERROR_BUSY);
 RunPendingProductTest();assert(IsTaskDone==0&&power_on==1&&power_off==0&&last_power_on_mv==3300&&voltage==3300&&ADS1220.config[0]==0x39&&adc_inits==0);
 assert(sendBufBy[0]==0x56&&sendBufBy[4]==0x56&&sendBufBy[12]==0x56&&sendBufBy[16]==0&&
        sendBufBy[32]==3300&&sendBufBy[33]==WF162A_STATUS_REFERENCE_MEASURED);
 /* A valid zero differential result must not be reported as an ADC timeout. */
 adc_raw=0;AllTestFor162A(out,120);
 assert(out[32]==3300&&out[33]==WF162A_STATUS_REFERENCE_MEASURED);
 adc_read_fail=1;AllTestFor162A(out,120);
 assert((out[33]&(WF162A_STATUS_REFERENCE_FAILED|WF162A_STATUS_ADC_TIMEOUT)) ==
        (WF162A_STATUS_REFERENCE_FAILED|WF162A_STATUS_ADC_TIMEOUT) && out[0]==0);
 adc_read_fail=0;
 c[1]=TYPE1_GET_ALLTEST_DATA;Parse(c,2,out,120);assert(!memcmp(out,sendBufBy,240));
 reset();c[0]=1;c[1]=TYPE1_SET_VDD;c[2]=5000&0xFF;c[3]=5000>>8;Parse(c,4,out,120);reset();
 c[0]=PRODUCT_WF162A;Parse(c,1,out,120);assert(IsTaskDone==1&&out[0]==1);
 RunPendingProductTest();assert(IsTaskDone==0&&power_on==1&&power_off==0&&last_power_on_mv==5000&&voltage==5000&&ADS1220.config[0]==0x39&&adc_inits==0);
 /* Task 9's upper computer reads only registers 0..15.  Keep that view
    populated for 5 V and retain the extended 16..31 copy.  The 5 V words
    are normalized to the task-9 host's established 3.3 V scale. */
 assert(sendBufBy[0]==0x1D&&sendBufBy[4]==0x1D&&sendBufBy[12]==0x1D&&
        sendBufBy[16]==0x1D&&sendBufBy[20]==0x1D&&sendBufBy[28]==0x1D&&
        sendBufBy[32]==5000&&sendBufBy[33]==WF162A_STATUS_REFERENCE_MEASURED);
 /* A second task-9 round reuses the selected rail without another 0x201
    command; an explicit 0x201=0 remains the shutdown path. */
 reset();c[0]=PRODUCT_WF162A;Parse(c,1,out,120);RunPendingProductTest();
 assert(power_on==1&&power_off==0&&last_power_on_mv==5000&&voltage==5000);
 c[0]=1;c[1]=TYPE1_SET_VDD;c[2]=0;c[3]=0;Parse(c,4,out,120);
 assert(power_off==1&&voltage==0);
 /* WF100S still forces 5 V even if the shared VDD command selected 3.3 V. */
 reset();c[0]=1;c[1]=TYPE1_SET_VDD;c[2]=3300&0xFF;c[3]=3300>>8;Parse(c,4,out,120);
 reset();c[0]=PRODUCT_WF100S;Parse(c,1,out,120);RunPendingProductTest();
 assert(last_power_on_mv==5000&&power_off==1&&voltage==0);
 /* Guard caller buffers and malformed command lengths. */
 reset();for(unsigned i=0;i<122;++i)out[i]=0xCAFE;
 c[0]=1;c[1]=TYPE1_GET_ADC;Parse(c,2,&out[1],1);assert(out[0]==0xCAFE&&out[1]==CMD_ERROR_LENGTH&&out[2]==0xCAFE);
 c[0]=2;Parse(c,5,out,120);assert(out[0]==CMD_ERROR_LENGTH);
 puts("PASS: latest commands, product layouts, OTP/NACK, power, ADC restore, task/busy, lengths");return 0;
}
'''
DRIVER_HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include "SoftIIC.h"
#include "SoftIIC_PP.h"
#include "SoftUart.h"
GPIO_TypeDef fake_a, fake_b, fake_c;
int htim2;
static uint32_t counter;
static GPIO_PinState level=GPIO_PIN_SET;
static unsigned ticks, sda_input, sda_output;
void test_counter_set(uint32_t v){counter=v;}
uint32_t test_counter_get(void){++ticks;return ++counter;}
void delay_us(uint32_t v){counter+=v;}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t pin){(void)p;(void)pin;return level;}
void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,GPIO_PinState s){(void)p;(void)pin;(void)s;}
void HAL_GPIO_Init(GPIO_TypeDef *p,GPIO_InitTypeDef *i){(void)p;if(i->Pin==UART_RX_Pin){if(i->Mode==GPIO_MODE_INPUT)++sda_input;else if(i->Mode==GPIO_MODE_OUTPUT_PP)++sda_output;}}
uint16_t Read_Level(UartInfo *,uint32_t);
uint8_t Soft_Uart_Read_Bit(UartInfo *,uint32_t);
int main(void){
 uint8_t data[4]={9,9,9,9};I2CInfo bus;
 IIC_Init(&bus,I2C_SDA_GPIO_Port,I2C_SDA_Pin,I2C_SCL_GPIO_Port,I2C_SCL_Pin);
 assert(I2C_ReadNByte(&bus,0xDA,data,4)==1&&data[0]==0); /* disconnected bus reports NACK */
 pp_iic_init(GPIOB,UART_RX_Pin,GPIOB,UART_TX_Pin);assert(pp_iic_wait_ack()==1&&sda_input>0);pp_iic_stop();assert(sda_output>0);
 UART_Init(&uart1,9600,UART_TX_GPIO_Port,UART_TX_Pin,UART_RX_GPIO_Port,UART_RX_Pin);
 ticks=0;assert(Soft_Uart_Read_Bit(&uart1,1000)==0&&uart1.Save_BuadRate==0&&ticks<1100);
 /* Sampling over 255 loop iterations must still detect a high bit. */
 assert(Read_Level(&uart1,1000)==1);level=GPIO_PIN_RESET;assert(Read_Level(&uart1,1000)==0);
 /* A stuck low line times out without zero-width arithmetic. */
 ticks=0;assert(Soft_Uart_Read_Bit(&uart1,200000)==0&&uart1.Save_BuadRate==0&&ticks<200100);
 puts("PASS: actual I2C NACK, push-pull SDA release, UART timeout and sample counter");return 0;
}
'''

DAC_HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#include "MCP4725.h"
#include "SoftIIC.h"
GPIO_TypeDef fake_a, fake_b, fake_c;
static uint8_t last_data[3];
void osDelay(uint32_t t){(void)t;}
void IIC_Init(I2CInfo *i,GPIO_TypeDef *sd,uint16_t sdp,GPIO_TypeDef *sc,uint16_t scp){
 assert(sd==DAC_SDA_GPIO_Port&&sdp==DAC_SDA_Pin&&sc==DAC_SCL_GPIO_Port&&scp==DAC_SCL_Pin);
 i->portSda=sd;i->portScl=sc;i->pinSda=sdp;i->pinScl=scp;i->error=0;
}
uint8_t I2C_WriteNByte(I2CInfo *i,const uint8_t *b,uint32_t n){
 (void)i;assert(n==3);for(unsigned j=0;j<n;++j)last_data[j]=b[j];return 0;
}
int main(void){
 MCP4725_Init();assert(last_data[0]==0xC0&&last_data[1]==0x00&&last_data[2]==0x00);
 MCP4725_WriteData_Voltage(5000);assert(last_data[1]==0x0C&&last_data[2]==0x1F);
 MCP4725_WriteData_Voltage(0);assert(last_data[1]==0&&last_data[2]==0);
 puts("PASS: actual DAC defaults to 3.3 V, selects 5 V, and powers down");return 0;
}
'''

def run():
    with tempfile.TemporaryDirectory(prefix="adc-port-test-") as tmp:
        tmp = Path(tmp)
        for name, text in HEADERS.items():
            (tmp / name).write_text(text)
        for name, harness, sources in [
            ("products", HARNESS, ["CmdHelper", "I2CHelper", "wf280", "wf162a", "wf183", "wf2200", "wf100s"]),
            ("drivers", DRIVER_HARNESS, ["SoftIIC", "SoftIIC_PP", "SoftUart"]),
            ("dac", DAC_HARNESS, ["MCP4725"]),
        ]:
            test = tmp / f"{name}.c"
            test.write_text(harness)
            executable = tmp / name
            subprocess.run([os.environ.get("CC", "cc"), "-std=c99", "-g", "-fsanitize=address,undefined",
                            "-I" + str(tmp), "-I" + str(ROOT / "Core/Inc"), str(test),
                            *(str(ROOT / "Core/Src" / (s + ".c")) for s in sources),
                            "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True)

if __name__ == "__main__":
    run()
