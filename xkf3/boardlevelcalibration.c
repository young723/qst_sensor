/*!
 * \file
 * \version 1.1.1 rev. 55984
 *
 * \copyright
 * Copyright (c) 2016 Fairchild Semiconductor Corporation or subsidiaries
 * worldwide. All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. 	Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 * 2. 	Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 * 3. 	Neither the names of the copyright holders nor the names of their
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "boardlevelcalibration.h"
#include "blconestep.h"
#include "sensorfusionconfig.h"
#include "xkf3.h"
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "fisimu_hal.h"

#include "./fis210x/fis210x.h"
#include "./i2c/bsp_i2c.h"
#include "./i2c/qst_sw_i2c.h"
#include "./spi/bsp_spi.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif
typedef void (*Int2Handler)(void);

static float g_gyrBias[3];	// Last estimated gyro bias (feedback)
static bool g_gyrBiasAvailable;	// Availability flag for the feedback gyro bias
static bool g_showEulerAngles; //Determines if orientation output is shown as Euler angles (true) or quaternion (false)
static uint32_t  g_cnt;
struct SensorFusionConfig g_sfc;
static struct Xkf3* g_xkf; // Pointer to the XKF3 filter

static Int2Handler g_int2Handler = 0;	// Pointer to the FIS INT2 handler
const uint8_t fisAddress = 0x6b << 1;	// FIS I2C address, SDO->GND(0x6a),  SDO->VDDIO(0x6b)

void setINT2Handler(const Int2Handler h);
void mcuWriteData(const uint8_t* data, uint8_t dataLength);
void mcuReadData(uint8_t address, uint8_t* data, uint8_t dataLength);
void mcuAssertReset(bool assert);
bool mcuInt1Asserted(void);
bool mcuInt2Asserted(void);
void mcuDelayMicroseconds(uint32_t delay);
void mcubusyWaitForEvent(enum FisImu_Interrupt interrupt, enum FisImu_InterruptEvent event);

struct FisImuHal g_fisHal =
{
	mcuWriteData,
	mcuReadData,
	mcuAssertReset,
	mcuInt1Asserted,
	mcuInt2Asserted,
	mcubusyWaitForEvent,
	mcuDelayMicroseconds,
#if defined(QST_USE_SPI)
	true,
#else
	false,
#endif
	FisImu_Fis2100
};


void setINT2Handler(const Int2Handler h)
{
	g_int2Handler = h;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(g_int2Handler)
		g_int2Handler();

}

void mcuWriteData(const uint8_t* data, uint8_t dataLength)
{
	if(g_fisHal.spiMode)
	{
#if defined(QST_USE_SPI)
		qst_fis210x_spi_write_bytes(data[0], (uint8_t*)(&data[1]),dataLength-1);
#endif
	}
	else
	{
#if defined(QST_USE_SW_I2C)
		qst_sw_writeregs(fisAddress, data[0], (uint8_t*)(&data[1]), dataLength-1);
#endif
	}
}

void mcuReadData(uint8_t address, uint8_t* data, uint8_t dataLength)
{
	if(g_fisHal.spiMode)
	{
#if defined(QST_USE_SPI)
		qst_fis210x_spi_read(address, data, dataLength);
#endif
	}
	else
	{
#if defined(QST_USE_SW_I2C)
		qst_sw_readreg(fisAddress, address, data, dataLength);
#endif
	}
}

void mcuAssertReset(bool flag)
{
	if(flag)
		GPIO_SetBits(GPIOA, GPIO_Pin_8);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_8);
}

/*!
* \brief Check if the FIS INT1 pin is asserted.
* \remark See FisImuHal documentation.
*/
bool mcuInt1Asserted(void)
{
	return (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == Bit_SET);
}

/*!
* \brief Check if the FIS INT2 pin is asserted.
* \remark See FisImuHal documentation.
*/
bool mcuInt2Asserted(void)
{
	return (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9) == Bit_SET);
}

/*!
* \brief Microseconds delay implementation.
* \remark See FisImuHal documentation.
*/
void mcuDelayMicroseconds(uint32_t delay)
{
	for (uint32_t i = 0; i < delay; ++i)
	{
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();

		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
		__NOP(); __NOP(); __NOP(); __NOP();
	}
}

void mcubusyWaitForEvent(enum FisImu_Interrupt interrupt, enum FisImu_InterruptEvent event)
{
	bool (*readIntState)(void) = (interrupt == Fis_Int1) ? mcuInt1Asserted : mcuInt2Asserted;

	switch(event)
	{
		case FisInt_low:
		case FisInt_high:
			{
				const bool requiredState = (event == FisInt_high);
				while (true)
				{
					if (readIntState() == requiredState)
						return;
				}
			}

		case FisInt_positiveEdge:
		case FisInt_negativeEdge:
			{
				const bool requiredState = (event == FisInt_positiveEdge);
				bool prevState = readIntState();
				while (true)
				{
					bool state = readIntState();
					if ((state != prevState) && (state == requiredState))
						return;

					prevState = state;
				}
			}
	}
}


struct FisImuHal const* getFISHal(void)
{
	return &g_fisHal;
}


void displayOffset(char const* description, float const* data, float conversion)
{
	printf("  %s:", description);
	for (int i = 0; i < 3; ++i)
	{
		printf(" %f", conversion * data[i]);
	}
	printf("\n");
}

bool runBoardLevelCalibration(void* blcMemory)
{
	float qst_acc[3],qst_gyr[3];
	struct FisImuConfig blcConfig;
	struct BLC* blc;
	struct FisImuRawSample sample;
	bool calSuccess;

	blc = BLC_initOneStep(blcMemory);
	// Initialize the FIS driver. It is necessary to reset the FIS hardware
	// to remove any previously applied calibration data.
	printf("runBoardLevelCalibration start... \n");
	// Configure FIS as required by board level calibration algorithm

	BLC_getSensorConfig(blc, &blcConfig);
#if 0
	printf("inputSelection 0x%x\n", blcConfig.inputSelection);
	printf("accRange 0x%x\n", blcConfig.accRange);
	printf("accOdr 0x%x\n", blcConfig.accOdr);
	printf("gyrRange 0x%x\n", blcConfig.gyrRange);
	printf("gyrOdr 0x%x\n", blcConfig.gyrOdr);
	printf("aeOdr 0x%x\n", blcConfig.aeOdr);
#endif
	fis210xConfig_apply(&blcConfig);
	qst_delay(1000);
	BLC_resetOneStep(blc);

	while(!BLC_isStepComplete(blc))
	{
		while(get_imu_drdy_flag() == 0)
		{
		}

		FisImu_read_rawsample(&sample);
		if(sample.accelerometerData)
		{
			FisImu_processAccelerometerData(sample.accelerometerData,qst_acc);
		}
		if(sample.gyroscopeData)
		{
			FisImu_processGyroscopeData(sample.gyroscopeData,qst_gyr);
		}
		printf("acc_gyro_cali,%f,%f,%f,%f,%f,%f\n",qst_acc[0],qst_acc[1],qst_acc[2],qst_gyr[0],qst_gyr[1],qst_gyr[2]);
		BLC_handleSample(blc, &sample);
	}
	
	fisImu_enableSensors(FISIMU_CTRL7_DISABLE_ALL);
	printf("runBoardLevelCalibration done...\n");

	// Check the result of the calibration
	calSuccess = BLC_calibrateOneStep(blc, true);

	if(calSuccess)
	{
		// Get the computed calibration parameters and apply them to the FIS
		struct FisImu_offsetCalibration cal;

		BLC_getOffsetCalibration(blc, &cal);
		FisImu_applyOffsetCalibration(&cal);

		printf("Board level calibration results:\n");
		displayOffset("Accelerometer offset (mg)", cal.accOffset, 1000.0 / 9.81);
		displayOffset("Gyroscope offset (dps)", cal.gyrOffset, 180.0 / M_PI);
	}

	// Clean up the BLC data structures
	BLC_destruct(blc);
	return calSuccess;
}


void SensorFusionConfig_default(struct SensorFusionConfig* config)
{
	config->fisConfig.inputSelection = FISIMU_CONFIG_AE_ENABLE;
	config->fisConfig.accRange = AccRange_8g;
	config->fisConfig.accOdr = AccOdr_256Hz;
	config->fisConfig.gyrRange = GyrRange_2048dps;
	config->fisConfig.gyrOdr = GyrOdr_256Hz;
	config->fisConfig.magOdr = MagOdr_32Hz;
	config->fisConfig.magDev = MagDev_AK8963;
	config->fisConfig.aeOdr = AeOdr_32Hz;

	config->fusionOdr = SensorFusion_32Hz;
	config->filterScenario = FusionScenario_Navigation;
	config->useExternalMagnetometer = false;
	config->calculateVelocity = true;
}
#if 0
void initXKFRun(void* xkfMemory, bool showEulerAngles)
{
	g_showEulerAngles = showEulerAngles;
	g_gyrBiasAvailable = false;
	memset(g_gyrBias, 0, sizeof(g_gyrBias));
	g_cnt = 0;

	// Initialize XKF3 data structures
	g_xkf = Xkf3_init(xkfMemory);
	// Initialize FIS driver and hardware
	SensorFusionConfig_default(&g_sfc);
	Xkf3_reinitialize(g_xkf, NULL, &g_sfc, XKF3_INIT_DEFAULT);
	// Configure FIS hardware and start sampling.
	fis210xConfig_apply(&g_sfc.fisConfig);
}
#endif
