/**
  ******************************************************************************
  * @file    bsp_mpu6050.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief    mpu6050驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 指南者 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "lis3dh.h"
#include "./usart/bsp_usart.h"
#include "./i2c/bsp_i2c.h"


// add by yangzhiqiang for SPI
// add by yangzhiqiang for SPI
#define SPI_CS_HIGH()	GPIO_SetBits(GPIOA, GPIO_Pin_4)  
#define SPI_CS_LOW()	GPIO_ResetBits(GPIOA, GPIO_Pin_4)  

void SPI_Configuration(void)
{

	//SPI_1串口参数部分定义，SPI_1参数为时钟平时为高，上升沿采样

	SPI_InitTypeDef  SPI_InitStructure;

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能SPI_1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);//使能SPI_1时钟

	//配置SPI1的MISO（PA6）为浮空输入
#if 0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//配置SPI1的MOSI（PA7）和SPI1的CLK（PA5）为复用推免输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//配置SPI1的NSS（PA4）为推免输出

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
#else
	// MISO PA6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// MOSI PA7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	/*  SCK PA5*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//配置SPI1的NSS（PA4）为推免输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	SPI_CS_HIGH();
#endif

	//SPI1同步参数初始化定义
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1,&SPI_InitStructure);
	SPI_Cmd(SPI1,ENABLE);
}


u8 SPI_SendByte(u8 byte)  
{  
  // 等待发送数据寄存器清空  
  while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);  
   
  SPI_I2S_SendData(SPI1, byte); // 向从机发送数据  
  while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET); // 等待接收数据寄存器非空  
   
  return SPI_I2S_ReceiveData(SPI1); // 获取接收寄存器中的数据  
} 


/**
  * @brief   写数据到LIS3DH寄存器
  * @param   
  * @retval  
  */
void LIS3DH_WriteReg(u8 reg_add,u8 reg_dat)
{
	u8 add, dat;
#if 0
	I2C_ByteWrite(reg_dat,reg_add); 
#else
	SPI_CS_LOW();

	add = SPI_SendByte(reg_add);
	dat = SPI_SendByte(reg_dat);
	printf("LIS3DH_WriteReg %x = %x \n", add, dat);
	SPI_CS_HIGH();
#endif
}

/**
  * @brief   从LIS3DH寄存器读取数据
  * @param   
  * @retval  
  */
void LIS3DH_ReadData(u8 reg_add,unsigned char* Read,u8 num)
{
#if 0
	reg_add = reg_add|0x80;
	I2C_BufferRead(Read,reg_add,num);
#else
	int i;
	SPI_CS_LOW();

	while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);	
	 
	SPI_I2S_SendData(SPI1, reg_add); // 向从机发送数据  
	while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET); // 等待接收数据寄存器非空  
	for(i=0;i<num;i++) 
	{
		Read[i] = SPI_I2S_ReceiveData(SPI1); // 获取接收寄存器中的数据  
		printf("LIS3DH_ReadData %x \n", Read[i]);
	}

	SPI_CS_HIGH();
#endif
}


/**
  * @brief   初始化LIS3DH芯片
  * @param   
  * @retval  
  */
void LIS3DH_Init(void)
{
  int i=0,j=0;
  //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错
#if 0  
  I2C_Bus_set_slave_addr(LIS3DH_ACC_I2C_ADDRESS);
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
      ;
    }
  }
	LIS3DH_WriteReg(0x20, 0x57);	     //解除休眠状态
	LIS3DH_WriteReg(0x23, 0x18);	     //解除休眠状态
#else
	SPI_Configuration();
	for(i=0;i<1000;i++)
	{
	  for(j=0;j<1000;j++)
	  {
		;
	  }
	}
	LIS3DH_WriteReg(0x20, 0x57);	     //解除休眠状态
	LIS3DH_WriteReg(0x23, 0x18);	     //解除休眠状态

#endif
}

/**
  * @brief   读取LIS3DH的ID
  * @param   
  * @retval  正常返回1，异常返回0
  */
uint8_t LIS3DHReadID(void)
{
	unsigned char Re = 0;
	LIS3DH_ReadData(0x0f,&Re,1);    //读器件地址

	if(Re != 0x33)
	{
		MPU_ERROR("LIS3DH dectected error!\r\n");
		return 0;
	}
	else
	{
		MPU_INFO("LIS3DH ID = %x\r\n",Re);
		return 1;
	}
}

/**
  * @brief   读取LIS3DH的加速度数据
  * @param   
  * @retval  
  */

void LIS3DHReadAcc(short *accData)
{
    u8 buf[6];
    LIS3DH_ReadData(0x28, buf, 6);
    accData[0] = (buf[1] << 8) | buf[0];
    accData[1] = (buf[3] << 8) | buf[2];
    accData[2] = (buf[5] << 8) | buf[4];
}

/**
  * @brief   读取LIS3DH的原始温度数据
  * @param   
  * @retval  
  */
void LIS3DHReadTemp(short *tempData)
{
		u8 buf[2];
    LIS3DH_ReadData(0x1f,buf,2);     //读取温度值
    *tempData = (buf[0] << 8) | buf[1];
}

/**
  * @brief   读取LIS3DH的温度数据，转化成摄氏度
  * @param   
  * @retval  
  */
void LIS3DH_ReturnTemp(float *Temperature)
{
	short temp3;
	u8 buf[2];
	
	LIS3DH_ReadData(0x1f,buf,2);     //读取温度值
	temp3= (buf[0] << 8) | buf[1];	
	*Temperature=((double) temp3/340.0)+36.53;

}

