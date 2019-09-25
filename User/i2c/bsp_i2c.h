#ifndef __BSP_I2C_H
#define	__BSP_I2C_H

#include "stm32f10x.h"
#include "./usart/bsp_usart.h"


/**************************I2C参数定义，I2C1或I2C2********************************/
#if defined(USE_I2C_1_PB6_7)
#define             SENSORS_I2Cx                                I2C1
#define             SENSORS_I2C_APBxClock_FUN                   RCC_APB1PeriphClockCmd
#define             SENSORS_I2C_CLK                             RCC_APB1Periph_I2C1
#define             SENSORS_I2C_GPIO_APBxClock_FUN              RCC_APB2PeriphClockCmd
#define             SENSORS_I2C_GPIO_CLK                        RCC_APB2Periph_GPIOB     
#define             SENSORS_I2C_SCL_PORT                        GPIOB   
#define             SENSORS_I2C_SCL_PIN                         GPIO_Pin_6
#define             SENSORS_I2C_SDA_PORT                        GPIOB 
#define             SENSORS_I2C_SDA_PIN                         GPIO_Pin_7
#endif

#if defined(USE_I2C_1_PB8_9)
#define             SENSORS_I2Cx                                I2C1
#define             SENSORS_I2C_APBxClock_FUN                   RCC_APB1PeriphClockCmd
#define             SENSORS_I2C_CLK                             RCC_APB1Periph_I2C1
#define             SENSORS_I2C_GPIO_APBxClock_FUN              RCC_APB2PeriphClockCmd
#define             SENSORS_I2C_GPIO_CLK                        RCC_APB2Periph_GPIOB     
#define             SENSORS_I2C_SCL_PORT                        GPIOB   
#define             SENSORS_I2C_SCL_PIN                         GPIO_Pin_8
#define             SENSORS_I2C_SDA_PORT                        GPIOB 
#define             SENSORS_I2C_SDA_PIN                         GPIO_Pin_9
#endif

/*等待超时时间*/
#define I2CT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT         ((uint32_t)(10 * I2CT_FLAG_TIMEOUT))

/*信息输出*/
#define BSP_I2C_DEBUG_ON         1

#define BSP_I2C_INFO(fmt,arg...)          printf("<<-QST-I2C-INFO->> "fmt"\n",##arg)
#define BSP_I2C_ERROR(fmt,arg...)         printf("<<-QST-I2C-ERROR->> "fmt"\n",##arg)
#define BSP_I2C_DEBUG(fmt,arg...)         do{\
                                          if(BSP_I2C_DEBUG_ON)\
                                          printf("<<-QST-I2C-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                          }while(0)


void I2C_Bus_Init(void);
uint8_t I2C_ByteWrite(u8 slave, u8 WriteAddr, u8 pBuffer);
uint8_t I2C_BufferRead(u8 slave, u8 ReadAddr, u8* pBuffer, u16 NumByteToRead);
void I2C_WaitStandbyState(void);
void I2C_Bus_set_slave_addr(uint8_t address);

#endif /* __BSP_I2C_H */
