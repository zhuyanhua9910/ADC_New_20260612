
#ifndef __WF280_H__
#define __WF280_H__

#include "stm32g0xx_hal.h"
#include "CmdHelper.h"

#define WF280_RESULT_LENGTH 2
#define WF280_OUTPUT_RESULT_LENGTH 7
#define WF280_M1_RESULT_LENGTH 5
#define WF280_COEFF_MAX 32

/* AllTestFor280 (cmd = 14) 的结果帧：77 个输入寄存器，首字固定为状态，
   后面是真实数据，最后追加 M1/NOR 两 bank 的 32 路原始值。
   两档轨压各自在切压稳定后、读系数之前测：1.8 V 占 [1]，3.3 V 占 [2]。 */
#define WF280_TEST_RESULT_LENGTH 89

#define WF280_R_STATUS      0   /* 状态：0=OK，非 0=错误码 */
#define WF280_R_RAIL_1V8_MV 1   /* 1.8 V 实测轨压 mV（读 M1 之前测） */
#define WF280_R_RAIL_MV     2   /* 3.3 V 实测轨压 mV（读 NOR 之前测） */
#define WF280_R_WORK_RAW    3   /* 工作电流原始码，低 16 位在前 */
#define WF280_R_SLEEP_RAW   5   /* 休眠电流原始码，低 16 位在前 */
#define WF280_R_RAW         7   /* 3 次 0xAC 原始数据连续放：每次 6 字节 */
#define WF280_R_RAW_LEN     6
#define WF280_RAW_SAMPLES   3
#define WF280_R_M1         25   /* M1 bank 32 路系数原始值 */
#define WF280_R_NOR        57   /* NOR bank 32 路系数原始值 */




/* The WF280 adapter keeps the command/status convention used by the
   WF280_OS_BURN reference program.  A zero status is success; non-zero
   values identify a transport or protection failure. */
#define WF280_STATUS_OK 0
#define WF280_STATUS_I2C 1
#define WF280_STATUS_NOR_TIMEOUT 2
#define WF280_STATUS_CMD_TIMEOUT 3
#define WF280_STATUS_SHORT 0xE1
#define WF280_STATUS_RAIL 0xE2

#define WF280_SUPPLY_1V8_MV 1800
#define WF280_SUPPLY_3V3_MV 3300
/* 1.8 V 下满量程只有 15 mA，短路阈值必须低于它 */
#define WF280_SHORT_1V8_UA 10000


void WF280CheckOtpReg(uint16_t *out, uint16_t out_lens);

uint8_t WF280TestOutput(uint16_t supply_mv, uint16_t measurement,
                        uint16_t settle_ms, uint16_t *out,
                        uint16_t out_lens);
uint8_t WF280ReadCoefficients(uint8_t start, uint8_t count,
                              uint16_t *out, uint16_t out_lens);
uint8_t WF280ReadM1(uint8_t count, uint16_t *out, uint16_t out_lens);

/* 读取当前 DUT 轨压。PB1 / ADC1_IN9 经 R8(1K)/R9(1k) 分压接在 VDD 上，
   所以读到的是“实际输出”而不是 DAC 设定值。返回 mV，0 表示读取失败。
   该函数只做测量，不改变供电状态，任何时候都可以调用。 */
uint16_t WF280ReadRailMv(void);

/* cmd = 14：WF280 整机综合测试。轨压由上位机 {1, 0x201} 设置，
   固件只回读实际值并按它换算电流。
   1.8 V 上电，延时到点后回读 1.8 V 轨压，随即测一次电流判短路
   （10 mA，命中写 0xE1 并把原始码按字节拆到 [3..6]，直接返回）
   → 0xBA 复位 → 读 M1 → 切 3.3 V
   → 回读 3.3 V 轨压 → 读 NOR 并逐路比对 → 0xBA 复位 + 0xAC 测量
   → 测量中抓工作电流 → 一次读 6 字节原始数据（Status+电桥+温度）
   → 等 1 s 读休眠电流 → 恢复 ADC 配置 → 断电。 */
void AllTestFor280(uint16_t *out, uint16_t out_lens);

#endif //__WF280_H__
