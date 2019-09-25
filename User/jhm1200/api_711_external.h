/**
******************************************************************************
* @file		: api_200x.h 
* @version	: v1.0
* @brief	: API Definition of JHM200X 
* @author	: Chenxk
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

#ifndef _API_711_EXTERNAL_H
#define _API_711_EXTERNAL_H

#include "sensor.h"
#include "stm32f1xx_hal.h"

//#define CPU_64Bits
#define CPU_32Bits

//#define Big_Endian
#define Small_Endian

#define ENABLE_FLOAT

typedef enum {
	success = 0,
	chip_busy,
	iic_error,
	null_pointer,
	patameter_error,
	set_az_failed
} JHM1200_BACK;

typedef enum {
	Filter_Off = 0,
	Filter_Two = 2,
	Filter_Four = 4,
	Filter_Eight = 8,
	Filter_Sixteen = 16
} Filter_Define;

typedef enum {
	Temp_Oversamp_2048 = 0,
	Temp_Oversamp_4096 = 1,
	Temp_Oversamp_8192 = 2,
	Temp_Oversamp_16384 = 3
} Temp_Oversamp;

typedef enum {
	Press_Oversamp_256 = 7,
	Press_Oversamp_512 = 6,
	Press_Oversamp_1024 = 5,
	Press_Oversamp_2048 = 4,
	Press_Oversamp_4096 = 3,
	Press_Oversamp_8192 = 2,
	Press_Oversamp_16384 = 1,
	Press_Oversamp_32768 = 0
} Press_Oversamp;

typedef signed int s32;
typedef unsigned int u32;
typedef signed short s16;
typedef unsigned short u16;
typedef signed char s8;
typedef unsigned char u8;
#ifdef CPU_64Bits
typedef long s64;
#elif defined(CPU_32Bits)
typedef long long s64;
#endif

typedef struct
{
	s32 co_p2;
	s32 co_p1;
	s32 co_p4;
	s32 co_p5;
	s32 co_p7;
	s32 co_p6;
	s32 co_p3;
	s32 co_t2;
	s32 co_t1;
	s32 co_t3;
	s32 co_p8;
	u16 version;
} Calib_Param_Define;

#ifdef Small_Endian
typedef struct
{
	u16 Gain_stage1 : 2;
	u16 Gain_stage2 : 3;
	u16 Gain_Polarity : 1;
	u16 Clk_divider : 2;
	u16 A2D_Offset : 3;
	u16 OSR_Pressure : 3;
	u16 OSR_Temp : 2;
} NVM14_Define;
#elif defined(Big_Endian)
typedef struct
{
	u16 OSR_Temp : 2;
	u16 OSR_Pressure : 3;
	u16 A2D_Offset : 3;
	u16 Clk_divider : 2;
	u16 Gain_Polarity : 1;
	u16 Gain_stage2 : 3;
	u16 Gain_stage1 : 2;
} NVM14_Define;
#else
#error "Please define your CPU Endian type"
#endif

typedef struct
{
	u32 ID;
	Filter_Define FilterFactor;
	NVM14_Define NVM14;
	Calib_Param_Define CalibParam;
	s32 baz;
	s32 taz;
	u8 (*IIC_Write)(u8 IIC_Address, u8 *buffer, u16 count);
	u8 (*IIC_Read)(u8 IIC_Address, u8 *buffer, u16 count);
	u8 Device_Address;
} JHM1200;

#define PowerOn (1 << 6)
#define DeviceBusy (1 << 5)
#define ModeStatus (1 << 3)
#define MemoryFlag (1 << 2)

/*!
 *	@brief This function is used for initialize jhm1200
 *         
 *
 *	@param p1200:JHM1200 structure pointer.
 *  @param Dev_Add:the IIC address
 *
 *	@note You must define a variable of JHM1200
 *	@note Then use API_JHM1200_Init function to initialize it
 *  @note with an address of IIC bus
 *  @note You'd better not to modify the variable
 *
 *	@return results of bus communication function
 *	@retval the result of the function,refer to the JHM1200_BACK definition
 *
 *
*/
s32 API_JHM1200_Init(JHM1200 *p1200, u8 Dev_Add);
/*!
 *	@brief this API is used to read the
 *	uncompensated temperature(ut)
 *
 *  @param  dataout:the value of the ut
 *
 *	@return the results of the function
 *
*/
JHM1200_BACK API_JHM1200_Read_unCompensated_Temperature(s32 *dataout);
/*!
 *	@brief this API is used to read the
 *	uncompensated pressure(up)
 *
 *  @param  dataout:the value of the up
 *
 *	@return the results of the function
 *
*/
JHM1200_BACK API_JHM1200_Read_unCompensated_Press(s32 *dataout);
/*!
 *	@brief this function used for read the calibration
 *	parameter from the NVM
 *
 *  @note When calculate the temperature and pressure,we
 *    need some calibration parameters,so do this
 *    when initializing
 *
 *	@return none
 *
 *
*/
void JHM1200_Get_Calib_Param(void);
/*!
 *	@brief This function is used for set the filter
 *         
 *
 *	@param Filter:the parameter of the filter to be set.
 *            Filter_Off->set the filter off
 *            Filter_Two->set the filter to be 2
 *            Filter_Four->set the filter to be 4
 *            Filter_Eight->set the filter to be 8
 *            Filter_Sixteen->set the filter to be 16      	
 *
 *	@return none
*/
void API_JHM1200_Set_Filter(Filter_Define Filter);
/*!
 *	@brief This function is used for get the filter
 *
 *	@param none    	
 *
 *	@return  parameter of the filter has been set.
 *            Filter_Off->off
 *            Filter_Two->2
 *            Filter_Four->4
 *            Filter_Eight->8
 *            Filter_Sixteen->16  
*/
Filter_Define API_JHM1200_Get_Filter(void);
/*!
 *	@brief This API is used to set
 *	the pressure oversampling
 *
 *  @param Oversamp:The value of temperature over sampling
 *
 *       000->Press_Oversamp_32768
 *       001->Press_Oversamp_16384
 *       010->Press_Oversamp_8192
 *       011->Press_Oversamp_4096
 *       100->Press_Oversamp_2048
 *       101->Press_Oversamp_1024
 *       110->Press_Oversamp_512
 *       111->Press_Oversamp_256
 *
 *  @return none
*/
void API_JHM1200_Set_Oversamp_Press(Press_Oversamp Oversamp);
/*!
 *	@brief This API is used to get
 *	the pressure oversampling setting
 *
 *       000->Press_Oversamp_32768
 *       001->Press_Oversamp_16384
 *       010->Press_Oversamp_8192
 *       011->Press_Oversamp_4096
 *       100->Press_Oversamp_2048
 *       101->Press_Oversamp_1024
 *       110->Press_Oversamp_512
 *       111->Press_Oversamp_256
 *
 *  @param none
 *
 *  @return The value of temperature over sampling
 *
 *
*/
Press_Oversamp API_JHM1200_Get_Oversamp_Press(void);
/*!
 *	@brief This API is used to set
 *	the temperature oversampling setting
 *
 *  @param Oversamp:value of temperature over sampling to be set
 *       00 -> Temp_Oversamp_2048
 *       01 -> Temp_Oversamp_4096
 *       02 -> Temp_Oversamp_8192
 *       03 -> Temp_Oversamp_16384
 *
 *
 *  @return none
*/
void API_JHM1200_Set_Oversamp_Temp(Temp_Oversamp Oversamp);
/*!
 *	@brief This API is used to get
 *	the temperature oversampling setting
 *
 *       00 -> Temp_Oversamp_2048
 *       01 -> Temp_Oversamp_4096
 *       02 -> Temp_Oversamp_8192
 *       03 -> Temp_Oversamp_16384
 *
 *  @param none
 *
 *  @return The value of temperature over sampling
*/
Temp_Oversamp API_JHM1200_Get_Oversamp_Temp(void);

#ifdef ENABLE_FLOAT
/*!
 *	@brief this API is used to calculate the true
 *	temperature and pressure,the raw data will be read
 *  int this function
 *
 *  @note in this function,we use plenty of double variable,
 *  so if there is no FPU(Float Point Unit)in your CPU,you'd
 *  better use API_JHM1200_Get_Data_Fixed_Press_Temp function with 
 *  the cost of accuracy decline
 *
 *	@param Press:the value of pressure in steps of 1.0 Pa
 *       eg: if the Press is equal to 991233.69,it means that the barometric pressure is 991233.69 Pa
 *	@param Temp:the value of temperature in steps of 1.0 ��
 *       eg: if the Temp is equal to 40.23,it means that the temperature is 40.23��
 *	@return Return the result of the function
 *
 *
*/
JHM1200_BACK API_JHM1200_Get_Data_Press_Temp(double *Press, double *Temp);
#endif
/*!
 *	@brief this API is used to calculate the true
 *	temperature and pressure,the raw data will be read
 *  int this function
 *
 *  @note This function is fit for the CPU with no FPU(Float Point Unit)
 *  if there is a FPU,we suggest that you can use API_JHM1200_Get_Data_Press_Temp
 *  to get a higher accuracy
 *
 *	@param Press:the value of pressure in steps of 1 Pa  
 *			eg: if the Press is equal to 991233,it means that the barometric pressure is 991233 Pa
 *	@param Temp:the value of temperature in steps of 0.01 deg Celsius
 *      eg: if the Temp is equal to 4023,it means that the temperature is 40.23��
 *
 *
 *
 *	@return Return the result of the function
 *
 *
*/
JHM1200_BACK API_JHM1200_Get_Data_Fixed_Press_Temp(s32 *Press, s32 *Temp);

uint8_t API_711_EXTERNAL_Parse_Command(uint8_t *Buf, uint8_t Len, Sensor *S);

void Read_data_every20ms(void);
#endif // API_711_EXTERNAL_H
