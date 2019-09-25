/**
  ******************************************************************************
  * @file    bsp_mpu6050.c
  * @author  STMicroelectronics
  * @version V1.0
  * @date    2013-xx-xx
  * @brief    mpu6050����
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:���� ָ���� ������ 
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
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

	//SPI_1���ڲ������ֶ��壬SPI_1����Ϊʱ��ƽʱΪ�ߣ������ز���

	SPI_InitTypeDef  SPI_InitStructure;

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//ʹ��SPI_1ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);//ʹ��SPI_1ʱ��

	//����SPI1��MISO��PA6��Ϊ��������
#if 0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//����SPI1��MOSI��PA7����SPI1��CLK��PA5��Ϊ�����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//����SPI1��NSS��PA4��Ϊ�������

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

	//����SPI1��NSS��PA4��Ϊ�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	SPI_CS_HIGH();
#endif

	//SPI1ͬ��������ʼ������
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
  // �ȴ��������ݼĴ������  
  while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);  
   
  SPI_I2S_SendData(SPI1, byte); // ��ӻ���������  
  while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET); // �ȴ��������ݼĴ����ǿ�  
   
  return SPI_I2S_ReceiveData(SPI1); // ��ȡ���ռĴ����е�����  
} 


/**
  * @brief   д���ݵ�LIS3DH�Ĵ���
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
  * @brief   ��LIS3DH�Ĵ�����ȡ����
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
	 
	SPI_I2S_SendData(SPI1, reg_add); // ��ӻ���������  
	while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET); // �ȴ��������ݼĴ����ǿ�  
	for(i=0;i<num;i++) 
	{
		Read[i] = SPI_I2S_ReceiveData(SPI1); // ��ȡ���ռĴ����е�����  
		printf("LIS3DH_ReadData %x \n", Read[i]);
	}

	SPI_CS_HIGH();
#endif
}


/**
  * @brief   ��ʼ��LIS3DHоƬ
  * @param   
  * @retval  
  */
void LIS3DH_Init(void)
{
  int i=0,j=0;
  //�ڳ�ʼ��֮ǰҪ��ʱһ��ʱ�䣬��û����ʱ����ϵ�����ϵ����ݿ��ܻ����
#if 0  
  I2C_Bus_set_slave_addr(LIS3DH_ACC_I2C_ADDRESS);
  for(i=0;i<1000;i++)
  {
    for(j=0;j<1000;j++)
    {
      ;
    }
  }
	LIS3DH_WriteReg(0x20, 0x57);	     //�������״̬
	LIS3DH_WriteReg(0x23, 0x18);	     //�������״̬
#else
	SPI_Configuration();
	for(i=0;i<1000;i++)
	{
	  for(j=0;j<1000;j++)
	  {
		;
	  }
	}
	LIS3DH_WriteReg(0x20, 0x57);	     //�������״̬
	LIS3DH_WriteReg(0x23, 0x18);	     //�������״̬

#endif
}

/**
  * @brief   ��ȡLIS3DH��ID
  * @param   
  * @retval  ��������1���쳣����0
  */
uint8_t LIS3DHReadID(void)
{
	unsigned char Re = 0;
	LIS3DH_ReadData(0x0f,&Re,1);    //��������ַ

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
  * @brief   ��ȡLIS3DH�ļ��ٶ�����
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
  * @brief   ��ȡLIS3DH��ԭʼ�¶�����
  * @param   
  * @retval  
  */
void LIS3DHReadTemp(short *tempData)
{
		u8 buf[2];
    LIS3DH_ReadData(0x1f,buf,2);     //��ȡ�¶�ֵ
    *tempData = (buf[0] << 8) | buf[1];
}

/**
  * @brief   ��ȡLIS3DH���¶����ݣ�ת�������϶�
  * @param   
  * @retval  
  */
void LIS3DH_ReturnTemp(float *Temperature)
{
	short temp3;
	u8 buf[2];
	
	LIS3DH_ReadData(0x1f,buf,2);     //��ȡ�¶�ֵ
	temp3= (buf[0] << 8) | buf[1];	
	*Temperature=((double) temp3/340.0)+36.53;

}

