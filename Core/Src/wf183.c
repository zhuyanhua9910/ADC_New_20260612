
#include "wf183.h"
#include <string.h>
#include "I2CHelper.h"
#include "SoftIIC.h"
#include "cmsis_os.h"
#include "ADS1220.h"

static const uint16_t sys_adc_183[] = { 0, TYPE1_GET_ADC };
static const uint16_t sys_vdd_off_183[] = { 1, TYPE1_SET_VDD, 0, 0 };
static const uint16_t sys_vdd_on_183[] = { 1, TYPE1_SET_VDD, (PRODUCT_TEST_VDD_MV & 0xFF), (PRODUCT_TEST_VDD_MV >> 8) };
static int32_t ADC_Value_183 = 0;

/************WF183DE****************/
 static const uint16_t uart_id[] = { 0, 0, 4, 100, 8, 20, 0x55, 0x04, 0x24, 0x37 };
 static const uint16_t uart_version[] = { 0, 0, 4, 20, 8, 100, 0x66, 0x04, 0x1D, 0x2F };
 static const uint16_t uart_tk[] = { 0, 0, 4, 100, 8, 20, 0x66, 0x04, 0x14, 0xB3 };
 static const uint16_t uart_tb[] = { 0, 0, 4, 100, 8, 20, 0x66, 0x04, 0x11, 0x8C };
 static const uint16_t uart_pk[] = { 0, 0, 4, 100, 8, 20, 0x66, 0x04, 0x38, 0x33 };
 static const uint16_t uart_pb[] = { 0, 0, 4, 100, 8, 20, 0x66, 0x04, 0x39, 0x6D };
 static const uint16_t uart_calT[] = { 0, 0, 4, 100, 6, 200, 0x55, 0x04, 0x0E, 0x6A };
 static const uint16_t uart_calP[] = { 0, 0, 4, 100, 8, 200, 0x55, 0x04, 0x0D, 0x88 };
 static const uint16_t uart_sleep02[] = { 0, 0, 4, 100, 0, 0, 0x55, 0x04, 0x02, 0xC9 };
 static const uint16_t iic_write_calT[] = { 3, 0, 3, 0, 0, 0, 0xDA, 0x0A, 0x04 };
 static const uint16_t iic_write_calT2[] = { 3, 0, 2, 0, 0, 0, 0xDA, 0x0F };
 static const uint16_t iic_read_calT[] = { 3, 0, 1, 0, 2, 0, 0xDA };
 static const uint16_t iic_write_calP[] = { 3, 0, 3, 0, 0, 0, 0xDA, 0x0A, 0x06 };
 static const uint16_t iic_write_calP2[] = { 3, 0, 2, 0, 0, 0, 0xDA, 0x0B };
 static const uint16_t iic_read_calP[] = { 3, 0, 1, 0, 4, 0, 0xDA };

/**********************************/

void AllTestFor183(uint16_t *out, uint16_t out_lens){
  if (out == NULL || out_lens < WF183_RESULT_LENGTH) return;
  memset(out, 0, out_lens * sizeof(*out));

    uint16_t _out[10] = {0};
    int32_t adc2;
    uint8_t i;
    volatile float curr;
    //oppen power
    ParseSystemCmd(sys_vdd_on_183, 4, _out, 4);
		ADS1220_Init(&ADS1220);
    osDelay(150);
    do {
        //short dect
        ADC_Value_183 = GET_ADS1220_Val(&ADS1220);
        curr = (float)ADC_Value_183/8388608.0f * 3.3f / 30.0f;
        if(curr>=0.02) {
            _out[0] = (uint8_t)(ADC_Value_183);
            _out[1] = (uint8_t)(ADC_Value_183>>8);
            _out[2] = (uint8_t)(ADC_Value_183>>16);
            _out[3] = (uint8_t)(ADC_Value_183>>24);
            CopyAndClear(_out, &out[110], 4);
            break;
        }
        {
            //sleep         ParseSerialCmd
            ParseSerialCmd(uart_sleep02, sizeof(uart_sleep02) / sizeof(uart_sleep02[0]), _out, 10);
            //sleep current     ParseSystemCmd
            ADC_Value_183 = GET_ADS1220_Val(&ADS1220);
            adc2 = GET_ADS1220_Val(&ADS1220);
            if(adc2<ADC_Value_183) ADC_Value_183 = adc2;
            out[114] = (uint8_t)(ADC_Value_183);
            out[115] = (uint8_t)(ADC_Value_183>>8);
            out[116] = (uint8_t)(ADC_Value_183>>16);
            out[117] = (uint8_t)(ADC_Value_183>>24);
        }
        {
            //id          ParseSerialCmd
            for(i=0; i<5; i++) {
                ParseSerialCmd(uart_id, sizeof(uart_id) / sizeof(uart_id[0]), _out, 10);
                if(_out[2]==0xAA && _out[3]==0x08) break;
            }
            if(i>4) break;
            CopyAndClear(_out, out, 10);
        }
        {
            //version     ParseSerialCmd
            ParseSerialCmd(uart_version, sizeof(uart_version) / sizeof(uart_version[0]), _out, 10);
            CopyAndClear(_out, &out[20], 10);
            //tk         ParseSerialCmd
            ParseSerialCmd(uart_tk, sizeof(uart_tk) / sizeof(uart_tk[0]), _out, 10);
            CopyAndClear(_out, &out[30], 10);
            //tb         ParseSerialCmd
            ParseSerialCmd(uart_tb, sizeof(uart_tb) / sizeof(uart_tb[0]), _out, 10);
            CopyAndClear(_out, &out[40], 10);
            //pk         ParseSerialCmd
            ParseSerialCmd(uart_pk, sizeof(uart_pk) / sizeof(uart_pk[0]), _out, 10);
            CopyAndClear(_out, &out[50], 10);
            //pb         ParseSerialCmd
            ParseSerialCmd(uart_pb, sizeof(uart_pb) / sizeof(uart_pb[0]), _out, 10);
            CopyAndClear(_out, &out[60], 10);
            //uart temp     ParseSerialCmd
            ParseSerialCmd(uart_calT, sizeof(uart_calT) / sizeof(uart_calT[0]), _out, 10);
            CopyAndClear(_out, &out[70], 10);
            //uart press     ParseSerialCmd
            ParseSerialCmd(uart_calP, sizeof(uart_calP) / sizeof(uart_calP[0]), _out, 10);
            CopyAndClear(_out, &out[80], 10);
        }

        {
            //iic  temp     ParseI2CCmd
            ProductI2CTransfer(iic_write_calT, sizeof(iic_write_calT) / sizeof(iic_write_calT[0]), _out, sizeof(_out) / sizeof(_out[0]));
            osDelay(100);
            ProductI2CTransfer(iic_write_calT2, sizeof(iic_write_calT2) / sizeof(iic_write_calT2[0]), _out, sizeof(_out) / sizeof(_out[0]));
            ProductI2CTransfer(iic_read_calT, sizeof(iic_read_calT) / sizeof(iic_read_calT[0]), _out, sizeof(_out) / sizeof(_out[0]));
            CopyAndClear(_out, &out[90], 10);
            osDelay(20);
            //iic  press     ParseI2CCmd
            ProductI2CTransfer(iic_write_calP, sizeof(iic_write_calP) / sizeof(iic_write_calP[0]), _out, sizeof(_out) / sizeof(_out[0]));
            osDelay(100);
            ProductI2CTransfer(iic_write_calP2, sizeof(iic_write_calP2) / sizeof(iic_write_calP2[0]), _out, sizeof(_out) / sizeof(_out[0]));
            ProductI2CTransfer(iic_read_calP, sizeof(iic_read_calP) / sizeof(iic_read_calP[0]), _out, sizeof(_out) / sizeof(_out[0]));
            CopyAndClear(_out, &out[100], 10);
        }
        {
            //owi press    ParseOwiCmd
            ParseOwiCmd(NULL,0,_out, 10);
            CopyAndClear(_out, &out[10], 10);
        }

        {
            //work current  ParseSystemCmd
            ParseSystemCmd(sys_adc_183, 2, _out, 4);
            CopyAndClear(_out, &out[110], 4);
        }


    } while(0);

    //Close power
    ParseSystemCmd(sys_vdd_off_183, 4, _out, 4);
}
