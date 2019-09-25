
#ifndef __VIRTUAL_IIC_H
#define __VIRTUAL_IIC_H

//#include "stm32f1xx_hal.h"
#include "stm32f10x.h"

// IIC Port definition, also used for SPI clk, SPI SDI
typedef struct{
    GPIO_TypeDef *SCL_Port;
    GPIO_TypeDef *SDA_Port;
    uint16_t      SCL_Pin;
    uint16_t      SDA_Pin;
}VIIC_Port; 

#define GPIO_PORT_I2C	GPIOB			/* GPIO端口 */
#define RCC_I2C_PORT 	RCC_APB2Periph_GPIOB		/* GPIO端口时钟 */
#define I2C_SCL_PIN		GPIO_Pin_6			/* 连接到SCL时钟线的GPIO */
#define I2C_SDA_PIN		GPIO_Pin_7			/* 连接到SDA数据线的GPIO */

//void Virtual_I2C_Init(VIIC_Port *IIC);
//void Virtual_I2C_DeInit(VIIC_Port *IIC);
void Virtual_I2C_Init(void);
void Virtual_I2C_DeInit(void);

//uint8_t Virtual_I2C_Master_Transmit(VIIC_Port *IIC, uint8_t address, uint8_t *buf, uint8_t count);
//uint8_t Virtual_I2C_Master_Receive(VIIC_Port *IIC, uint8_t address, uint8_t *buf, uint8_t count);
uint8_t Virtual_I2C_Master_Transmit(uint8_t address, uint8_t *buf, uint8_t count);
uint8_t Virtual_I2C_Master_Receive(uint8_t address, uint8_t *buf, uint8_t count);

#endif // __VIRTUAL_IIC_H
