/**
******************************************************************************
* @file		: api_200x.c 
* @version	: v1.0
* @brief	: Parse Commands to operate JHM2000
* @author	: chenxk
******************************************************************************
*
* @par Copyright (c):  JiuHao Micro Co. Ltd
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

//#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_gpio.h"
#include "stm32f10x.h"
//#include "usbd_cdc_if.h"
//#include "board.h"
//#include "./jhm1200/sensor.h"
//#include "./jhm1200/ads1115.h"
#include "./jhm1200/api_711_internal.h"
#include <stdint.h>

// Define the upper and lower limits of the calibration pressure
#define CAL_L 30000  //30Kpa
#define CAL_H 110000 //110Kpa


//The 7-bit IIC address of the JHM1200 is 0x78
u8  Device_Address = 0xf0;	//0x78<<1;
//extern VIIC_Port vi2c1;

static unsigned char BSP_IIC_Write(unsigned char IIC_Address, unsigned char *buffer, unsigned short count)
{
#if 0
	Virtual_I2C_Master_Transmit(IIC_Address, buffer, count);
#else
	jhm1200_iic_write(buffer[0], buffer, 0);
#endif
	return 0;
}

static unsigned char BSP_IIC_Read(unsigned char IIC_Address, unsigned char *buffer, unsigned short count)
{
#if 0
	Virtual_I2C_Master_Receive(IIC_Address, buffer, count);
#else
	jhm1200_iic_read(buffer, count);
#endif
	return 0;
}

//Delay function needs to be defined
void DelayMs(unsigned char count)
{
	//HAL_Delay(1);
	int i,j;
	for(i=0;i<count;i++)
	{
		for(j=0;j<1000;j++)
		{
			;
		}
	}
}

//Read the status of IIC and judge whether IIC is busy
u8 JHM1200_IsBusy(void)
{
	u8 status;
	BSP_IIC_Read(Device_Address, &status, 1);
	status = (status >> 5) & 0x01;
	return status;
}

/**
  * @brief Using the 0xAC command to calculate the actual pressure and temperature using the JHM1200 internal algorithm
  * @note  Send 0xAC, read IIC status until IIC is not busy
  *	@note  The returned data is a total of six bytes, in order: status word, three-byte pressure value, two-byte temperature value
  * @note  The returned three-byte pressure value is proportional to the 24-bit maximum value 16777216. According to this ratio, 
           the actual pressure value is again converted according to the calibration range.
  * @note  The returned two-byte temperature value is proportional to the 16-bit maximum value 65536. According to this ratio, 
           the actual pressure value is again converted according to the calibration range.
  * @note  Zero pressure point and full pressure point of calibration pressure correspond to 20kpa and 120Kpa, respectively   
  * @note  The zero point of the calibration temperature is -40°C and the full point is 150°C
  * @note  The pressure actual value is calculated according to the span pressure unit is Pa, temperature actual value temp unit is 0.01°C
  */

void JHM1200_get_cal(void) 
{
	u8 buffer[6] = {0};
	u32 press_raw = 0;
	u16 temp_raw = 0;
	double press = 0.0, temp = 0.0;

	//Device_Address=0xf0;	//0x78  S->IIC_DevAddress;
	//Send 0xAC command and read the returned six-byte data
	buffer[0] = 0xAC;
	BSP_IIC_Write(Device_Address, buffer, 1);
	DelayMs(100);
	while(1)
	{
		if(JHM1200_IsBusy())
		{
			//printf("jhm1200 is busy! \n");
			DelayMs(1);
		}
		else
			break;
	}
	BSP_IIC_Read(Device_Address, buffer, 6);

	//The returned pressure and temperature values are converted into actual values according to the calibration range
	press_raw = ((u32)buffer[1] << 16) | ((u16)buffer[2] << 8) | buffer[3];
	temp_raw = ((u16)buffer[4] << 8) | (buffer[5] << 0);
	press = (double)press_raw / 16777216;
	press = press * (CAL_H - CAL_L) + CAL_L;
	temp = (double)temp_raw / 65536;
	temp = temp * 19000 - 4000;
	
	printf("raw: %d   ",press_raw);
	printf("buf[0]~buf[5]=0x:%x %x %x %x %x %x ",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5]);
	printf("press =: %d  temp*100 =: %d\n\r",(s32)press,(s32)temp);
	
}

unsigned char jhm1200_init(void)
{
	return 1;
}


