#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x.h"
#include <stdio.h>


#define WIP_Flag                  0x01
#define Dummy_Byte                0xFF


/*SPI�ӿڶ���-��ͷ****************************/
#define      QST_SPIx                       SPI1
#define      QST_SPI_APBxClock_FUN          RCC_APB2PeriphClockCmd
#define      QST_SPI_CLK                    RCC_APB2Periph_SPI1

//CS(NSS)���� Ƭѡѡ��ͨGPIO����
#define      QST_SPI_CS_APBxClock_FUN       RCC_APB2PeriphClockCmd
#define      QST_SPI_CS_CLK                 RCC_APB2Periph_GPIOB	//RCC_APB2Periph_GPIOA    
#define      QST_SPI_CS_PORT                GPIOB	//GPIOA
#define      QST_SPI_CS_PIN                 GPIO_Pin_6	//GPIO_Pin_4

//SCK����
#define      QST_SPI_SCK_APBxClock_FUN      RCC_APB2PeriphClockCmd
#define      QST_SPI_SCK_CLK                RCC_APB2Periph_GPIOA   
#define      QST_SPI_SCK_PORT               GPIOA   
#define      QST_SPI_SCK_PIN                GPIO_Pin_5
//MISO����
#define      QST_SPI_MISO_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define      QST_SPI_MISO_CLK               RCC_APB2Periph_GPIOA    
#define      QST_SPI_MISO_PORT              GPIOA 
#define      QST_SPI_MISO_PIN               GPIO_Pin_6
//MOSI����
#define      QST_SPI_MOSI_APBxClock_FUN		RCC_APB2PeriphClockCmd
#define      QST_SPI_MOSI_CLK               RCC_APB2Periph_GPIOA    
#define      QST_SPI_MOSI_PORT              GPIOA 
#define      QST_SPI_MOSI_PIN               GPIO_Pin_7

#define  	 SPI_QST_CS_LOW()     			GPIO_ResetBits( QST_SPI_CS_PORT, QST_SPI_CS_PIN )
#define  	 SPI_QST_CS_HIGH()    			GPIO_SetBits( QST_SPI_CS_PORT, QST_SPI_CS_PIN )

/*SPI�ӿڶ���-��β****************************/

/*�ȴ���ʱʱ��*/
#define SPIT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT         ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))

extern u8 qmaX981_spi_read(u8 addr, u8* buff, u8 len);
extern u8 qmaX981_spi_write(u8 addr,u8 data);
extern unsigned char qmp6988_spi_write(unsigned char Addr, unsigned char Data);
extern unsigned char qmp6988_spi_read(unsigned char Addr, unsigned char *pData, unsigned char Length);
extern uint8_t qst_fis210x_spi_write(uint8_t Addr, uint8_t Data);
extern uint8_t qst_fis210x_spi_write_bytes(uint8_t Addr, uint8_t* Data, uint8_t Len);
extern uint8_t qst_fis210x_spi_read(uint8_t Addr, uint8_t *pData, uint16_t Length);
extern void Spi_Init(int mode);
#endif /* __SPI_QST_H */

