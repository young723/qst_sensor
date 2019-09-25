//#include "board.h"
//#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_gpio.h"
#include "stm32f10x.h"
#include "./jhm1200/virtual_iic.h"

#define REPEAT_CHK_ACK_NUM		8
//#define IIC_DEBUG

#define Set_SCL_H()		GPIO_SetBits(GPIO_PORT_I2C, I2C_SCL_PIN)
#define Set_SCL_L()		GPIO_ResetBits(GPIO_PORT_I2C, I2C_SCL_PIN)
#define Set_SDA_H()		GPIO_SetBits(GPIO_PORT_I2C, I2C_SDA_PIN)
#define Set_SDA_L()		GPIO_ResetBits(GPIO_PORT_I2C, I2C_SDA_PIN)
#define Read_SDA()		GPIO_ReadInputDataBit(GPIO_PORT_I2C, I2C_SDA_PIN)

VIIC_Port vi2c1;
/******************************************************************************/
/*     IIC master protocol emulation using GPIO								  */ 
/******************************************************************************/
/**
 * @brief Empty Loop for Delay
 * 
 * @param count 
 */
static void DelayUs(uint8_t count)
{
	uint8_t i;
	for (i = 0; i < count * 2; i++)
	{};
}
/**
 * @brief Initial virtual IIC Port and Pins
		SCL:P0.11   SDA:P0.14
	This function should modified according to the CPU type
	since the low level hardward drivers are required.
 * 
 * @param IIC 
 */
//void Virtual_I2C_Init(VIIC_Port *IIC)
void Virtual_I2C_Init(void)
{
#if 0
	GPIO_InitTypeDef GPIO_InitStruct;

	// Disable hard IIC, 
	CLEAR_BIT(I2C1->CR1, I2C_CR1_PE); 
	__HAL_RCC_I2C1_CLK_DISABLE();

	// Initial virtual IIC port and pin
	if (IIC->SCL_Port == IIC->SDA_Port)
	{
		HAL_GPIO_DeInit(IIC->SCL_Port, IIC->SCL_Pin | IIC->SDA_Pin);
		HAL_GPIO_WritePin(IIC->SCL_Port, IIC->SCL_Pin | IIC->SDA_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_DeInit(IIC->SCL_Port, IIC->SCL_Pin);
		HAL_GPIO_DeInit(IIC->SDA_Port, IIC->SDA_Pin);
		HAL_GPIO_WritePin(IIC->SDA_Port, IIC->SDA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IIC->SCL_Port, IIC->SCL_Pin, GPIO_PIN_SET);
	}

	/*Configure GPIO pin : SDA_s_Pin */
	GPIO_InitStruct.Pin   = IIC->SDA_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(IIC->SDA_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SCL_s_Pin */
	GPIO_InitStruct.Pin   = IIC->SCL_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(IIC->SCL_Port, &GPIO_InitStruct);
#else
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_I2C_PORT, ENABLE);	/* ��GPIOʱ�� */

	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	/* ��©��� */
	GPIO_Init(GPIO_PORT_I2C, &GPIO_InitStructure);
#endif
	Set_SCL_H();
	Set_SDA_H();
}

/**
 * @brief Disbale Initial virtual IIC Port and Pins
		SCL:P0.11   SDA:P0.14
	This function should modified according to the CPU type
	since the low level hardward drivers are required.
 * 
 * @param IIC 
 */
//void Virtual_I2C_DeInit(VIIC_Port *IIC)
void Virtual_I2C_DeInit(void)
{
#if 0
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/*Configure GPIO pin : SDA_s_Pin */
	GPIO_InitStruct.Pin   = IIC->SDA_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(IIC->SDA_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SCL_s_Pin */
	GPIO_InitStruct.Pin   = IIC->SCL_Pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(IIC->SCL_Port, &GPIO_InitStruct);
#else
#endif
}


/**
 * @brief Generate start symbol for IIC protocol
 * 
 * @param IIC 
 */
static void Start(VIIC_Port *IIC)
{
	Set_SDA_H(); // SDA = 1
	Set_SCL_H(); // SCL = 1
	DelayUs(5);
	Set_SDA_L(); // SDA = 0
	DelayUs(5);
	Set_SCL_L(); // SCL = 0
}
/**
 * @brief Generate stop symbol for IIC protocol
 * 
 * @param IIC 
 */
static void Stop(VIIC_Port *IIC)
{
	Set_SDA_L(); //SDA=0
	DelayUs(5);
	Set_SCL_H(); //SCL=1
	DelayUs(5);
	Set_SDA_H(); //SDA=1
	DelayUs(5);
}
/**
 * @brief IIC master check ACK from slave
 * 
 * @param IIC 
 * @return uint8_t 
 */
static uint8_t Check_ACK(VIIC_Port *IIC)
{
	uint8_t ack;
	uint8_t i;
	Set_SCL_H();
	for (i=0; i<REPEAT_CHK_ACK_NUM ; i++)
	{
		if (Read_SDA() == 0UL)
		{
			ack = 0;
			break;
		}
		else
		{
			ack = 1;
		}
	}
	DelayUs(5);
	Set_SCL_L();
	DelayUs(5);
	return ack;
}
/**
 * @brief IIC master send ACK
 * 
 * @param IIC 
 */
static void Send_ACK(VIIC_Port *IIC)
{
	Set_SDA_L();
	DelayUs(5);
	Set_SCL_H();
	DelayUs(5);
	Set_SCL_L();
	Set_SDA_H();
}
/**
 * @brief IIC master send NACK
 * 
 * @param IIC 
 */
static void Send_NOACK(VIIC_Port *IIC)
{
	Set_SDA_H();
	DelayUs(5);
	Set_SCL_H();
	DelayUs(5);
	Set_SCL_L();
}
/**
 * @brief IIC master send a byte of data
 * 
 * @param IIC 
 * @param byte 
 */
static void SendByte(VIIC_Port *IIC, uint8_t byte)
{
	uint8_t i = 0;
	for (i=0; i<8; i++)
	{
		if (byte & (0x80 >> i))
		{
			Set_SDA_H();
		}
		else
		{
			Set_SDA_L();
		}

		DelayUs(5);
		Set_SCL_H();
		DelayUs(5);
		Set_SCL_L();
		DelayUs(5); 	// this delay ensure enough hold time
	}
}
/**
 * @brief IIC master receive a byte of data
 * 
 * @param IIC 
 * @return uint8_t 
 */
static uint8_t ReceiveByte(VIIC_Port *IIC)
{
	uint8_t i   = 0;
	uint8_t rbyte = 0;
	for (i=0; i<8; i++)
	{
		DelayUs(5);
		Set_SCL_H();
		DelayUs(5);
		if ((Read_SDA()) != 0UL)
		{
			rbyte |= (0x80 >> i);
		}
		Set_SCL_L();
	}
	return rbyte;
}
/**
 * @brief IIC master perform write opertion at specific address
 * 
 * @param IIC 
 * @param address 
 * @param buf 
 * @param count 
 * @return uint8_t 
 */
uint8_t Virtual_I2C_Master_Transmit(uint8_t address, uint8_t *buf, uint8_t count)
{
	// Prepare IIC address with read bit 0
	address &= 0xFE;

	// IIC master send Start symbol
	Start(&vi2c1);
	DelayUs(1);

	// IIC master Send IIC address and read bit, check ACK
	SendByte(&vi2c1, address);
	DelayUs(5);
	if (Check_ACK(&vi2c1))
	{
#ifdef IIC_DEBUG
		printf("Tx���͵�ַ0x%Xû��Ӧ��,��ʱ\r\n",address);
#endif
		Stop(&vi2c1);
		return 1; //��ʱ
	}

	// IIC master Send data bytes & check ACK
	while (count)
	{
		SendByte(&vi2c1, *buf);
		DelayUs(5);
		if (Check_ACK(&vi2c1))
		{
#ifdef IIC_DEBUG
			printf("��������0x%Xû��Ӧ��ʱ\r\n",*buf);
#endif
			Stop(&vi2c1);
			return 2; //��ʱ
		}
		buf++;
		count--;
	}
	Stop(&vi2c1);
	return 0;
}
/**
 * @brief IIC master perform read operation at specific operation
 * 
 * @param IIC 
 * @param address 
 * @param buf 
 * @param count 
 * @return uint8_t 
 */
uint8_t Virtual_I2C_Master_Receive(uint8_t address, uint8_t *buf, uint8_t count)
{
	// Prepare IIC address with read bit 0
	address |= 0x01;

	// IIC master send Start symbol
	Start(&vi2c1);
	DelayUs(1);

	// IIC master Send IIC address and read bit, check ACK
	SendByte(&vi2c1, address);
	DelayUs(5);
	if (Check_ACK(&vi2c1))
	{
#ifdef IIC_DEBUG
		printf("Tx���͵�ַ0x%Xû��Ӧ��,��ʱ\r\n",address);
#endif
		Stop(&vi2c1);
		return 1; //��ʱ
	}

	// IIC master Send data bytes & check ACK
	while (count)
	{
		*buf = ReceiveByte(&vi2c1);
		if (count != 1)
		{
			Send_ACK(&vi2c1);
		}
		else
		{
			Send_NOACK(&vi2c1);
		}
		buf++;
		count--;
	}
	Stop(&vi2c1);
	return 0;
}
/************************ (C) COPYRIGHT JHM Co. Ltd  *****END OF FILE****/
