#ifndef __LIS3DH_H
#define __LIS3DH_H
#include "stm32f10x.h"

#define LIS3DH_ACC_I2C_ADDRESS	(0x18<<1)

#define	LIS3DH_ACC_FS_MASK		0x30
#define LIS3DH_ACC_G_2G			0x00
#define LIS3DH_ACC_G_4G			0x10
#define LIS3DH_ACC_G_8G			0x20
#define LIS3DH_ACC_G_16G		0x30


void LIS3DHReadTemp(short *tempData);
void LIS3DHReadAcc(short *accData);
void LIS3DH_ReturnTemp(float *Temperature);
void LIS3DH_Init(void);
uint8_t LIS3DHReadID(void);
void LIS3DH_ReadData(u8 reg_add,unsigned char*Read,u8 num);
void LIS3DH_WriteReg(u8 reg_add,u8 reg_dat);

void MPU6050_PWR_MGMT_1_INIT(void);


#endif  /*LIS3DH*/
