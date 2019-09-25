/**
  ******************************************************************************
  * @file    main.c
  * @author  LYC
  * @version V1.0
  * @date    2014-04-22
  * @brief   MPU6050 硬件IIC测试
  ******************************************************************************
  * @attention
  * 实验平台:秉火 指南者 STM32 开发板 
  ******************************************************************************
  */
  
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "stm32f10x_spi.h"
#include "./systick/bsp_SysTick.h"
#include "./led/bsp_led.h"
#include "./usart/bsp_usart.h"
#include "./lis3dh/lis3dh.h"
#include "./qmaX981/qmaX981.h"
#include "./qmcX983/qmcX983.h"
#include "./qmp6988/qmp6988.h"
#include "./fis210x/fis210x.h"
#include "./qmc6308/qmc6308.h"
#include "./i2c/bsp_i2c.h"
#include "./i2c/qst_sw_i2c.h"
#include "./spi/bsp_spi.h"
#include "./rtc/bsp_rtc.h"
#include "../../algo/imu/imualgo.h"
#include "../../algo/compass/ICAL.h"
#include "../../upper/qst_packet.h"
#include <math.h>

//#define QST_ADAPT_NIMING
//#define QST_IMU_INTEGRAL_USE_FIS_TIMESTAMP
#define QST_ACC_USE_INT
#define QST_IMU_USE_INT
#define QST_IMU_TEST


#define QST_PRINTF					printf
#define IMU_PRINTF					printf
#define OUT_PRINTF					//printf

#define ODR_TIM						0.004		// 0.032f(32hz) 0.00796f(128hz) 0.004f(256hz)

typedef void (*int_callback)(void);
enum 
{
	QST_REPORT_OFF,
	QST_REPORT_POLLING,
	QST_REPORT_DRI
};

typedef struct
{
	unsigned char int1_flag;
	unsigned char int2_flag;
	int_callback int1_func;
	int_callback int2_func;
} qst_callback_t;


typedef struct
{
	float	acc_out[3];
	float	mag_out[3];
	float	gyro_out[3];
	float	euler_ypr[3];
	float	acc_bis[3];
	float	gyro_bis[3];
	float	gyro_rms[3];
	qmp6988_data 	qmp6988_1;
} qst_sensor_out_t;

typedef struct
{
	int				report;
	unsigned char	cali_flag;
	
	unsigned char	int1_flag;
	unsigned char	int2_flag;
	int_callback	int1_func;
	int_callback	int2_func;

	unsigned char	t_init;
	unsigned char	timestamp_1;
	unsigned char	timestamp_0;
	unsigned long	long mcu_t0;
	unsigned long	long mcu_t1;
} qst_imu_t;

typedef struct
{
	int				report;
	unsigned char	cali_flag;
	unsigned char	int1_flag;
	unsigned char	int2_flag;
	int_callback	int1_func;
	int_callback	int2_func;
} qst_acc_t;


typedef struct
{
	u8 qmaX981_ok;
	u8 qmc7983_ok;
	u8 qmc6308_ok;
	u8 qmp6988_ok;
	u8 fis210x_ok;
} qst_chip_t;

static qst_chip_t g_chip =
{
	//0x00, 0x00, 0x00, 0x00, 0xff
	0xff, 0x00, 0x00, 0x00, 0x00
};
static u8 qst_mcu_sleep = 0;
static qst_sensor_out_t g_out;
static qst_imu_t g_imu;
static QMC_CALI_T qmc_cali;
static qst_acc_t g_acc;

static void imu_drdy_callback(void);
static void imu_drdy_callback_test(void);
static void imu_process_irq(void);
static void acc_int1_callback(void);
static void acc_int2_callback(void);
static void acc_process_irq(void);

static u8 mcu_sleep_check(float acc[3]);
static void mcu_sleep_exit(void);
void qst_sensor_read(void);
#if defined(QST_IMU_TEST)
static void imu_drdy_callback_test(void);
static void imu_polling_test(void);
#endif
 
extern unsigned int TickDelay;

void qst_delay(unsigned int delay)
{
#if 0
	int i,j;
	for(i=0;i<delay;i++)
	{
		for(j=0;j<1000;j++)
		{
			;
		}
	}
#else
	TickDelay = delay;
	while(TickDelay)
	{}
#endif
}

void bsp_led_set(uint8_t flag)
{
	if(flag)
		GPIO_ResetBits(GPIOA, GPIO_Pin_10);
	else
		GPIO_SetBits(GPIOA, GPIO_Pin_10);
}

void bsp_gpio_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	GPIO_SetBits(GPIOA, GPIO_Pin_10);
#if !defined(QST_USE_SPI)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_6);	// set i2c slave addr
#endif
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	GPIO_SetBits(GPIOB, GPIO_Pin_6);
}

// timer
void bsp_timer2_init(uint16_t ms)
{
    TIM_TimeBaseInitTypeDef  TIM2_base;
	NVIC_InitTypeDef 		NVIC_InitStructure; 

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	// 开启定时器时钟,即内部时钟CK_INT=72M
    TIM2_base.TIM_Period = ms*10;	// 自动重装载寄存器的值，累计TIM_Period+1个频率后产生一个更新或者中断
    TIM2_base.TIM_Prescaler= (uint16_t)(SystemCoreClock/10000)-1;	// 时钟预分频数为
	// 时钟分频因子 ，基本定时器没有，不用管
	//TIM2_base.TIM_ClockDivision=TIM_CKD_DIV1;
	// 计数器计数模式，基本定时器只能向上计数，没有计数模式的设置
	//TIM2_base.TIM_CounterMode=TIM_CounterMode_Up; 
	// 重复计数器的值，基本定时器没有，不用管
	//TIM2_base.TIM_RepetitionCounter=0;
	 
    TIM_TimeBaseInit(TIM2, &TIM2_base); 	// 初始化定时器
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);	// 清除计数器中断标志位
    TIM_ITConfig(TIM2,TIM_IT_Update, ENABLE);	// 开启计数器中断
    TIM_Cmd(TIM2, DISABLE);	// 使能计数器

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		// 设置中断组为0
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn ;	// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 // 设置主优先级为 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	// 设置抢占优先级为3
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void bsp_timer2_set(FunctionalState enable)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, enable);
	TIM_ITConfig(TIM2,TIM_IT_Update, enable);	// 开启计数器中断
	TIM_Cmd(TIM2, enable);	// 使能计数器
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET ) 
	{
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update); 
		qst_sensor_read();
	}
}
// timer

// clock
static void SYSCLK_Config_WakeUp(void)
{
#if defined(SYSCLK_USE_HSI)
	RCC_DeInit(); /*将外设RCC寄存器重设为缺省值 */ 
	RCC_HSICmd(ENABLE); 
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY)== RESET);//等待HSI就绪 
	RCC_HCLKConfig(RCC_SYSCLK_Div1);   /*设置AHB时钟（HCLK） RCC_SYSCLK_Div1——AHB时钟 = 系统时*/	
	RCC_PCLK2Config(RCC_HCLK_Div1);	 /* 设置高速AHB时钟（PCLK2)RCC_HCLK_Div1——APB2时钟 = HCLK*/	 
	RCC_PCLK1Config(RCC_HCLK_Div2); /*设置低速AHB时钟（PCLK1）RCC_HCLK_Div2——APB1时钟 = HCLK / 2*/ 	 
	FLASH_SetLatency(FLASH_Latency_2);	/*设置FLASH存储器延时时钟周期数FLASH_Latency_2	2延时周期*/   
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);  /*选择FLASH预取指缓存的模,预取指缓存使能*/ 
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);/*设置PLL时钟源及倍频系数，频率为8/2*16=64Mhz*/	 
	RCC_PLLCmd(ENABLE);	 /*使能PLL */ 
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) ; /*检查指定的RCC标志位(PLL准备好标志)设置与否*/    
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  /*设置系统时钟（SYSCLK） */	
	while(RCC_GetSYSCLKSource() != 0x08);	 /*0x08：PLL作为系统时钟 */	   
#else
	RCC_DeInit(); /*将外设RCC寄存器重设为缺省值 */ 
	RCC_HSEConfig(RCC_HSE_ON);		/* 使能 HSE */
	RCC_WaitForHSEStartUp();
	//while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
	//{
		/* 等待 HSE 准备就绪 */
	//}
	RCC_HCLKConfig(RCC_SYSCLK_Div1);
	RCC_PCLK2Config(RCC_HCLK_Div1);
	RCC_PCLK1Config(RCC_HCLK_Div2);
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

	RCC_PLLCmd(ENABLE);		/* 使能 PLL */ 
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
		/* 等待 PLL 准备就绪 */
	}
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);	/* 选择PLL作为系统时钟源 */
	while (RCC_GetSYSCLKSource() != 0x08)
	{	
		/* 等待PLL被选择为系统时钟源 */
	}
#endif
}
// clock


// irq
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
	{  
		  EXTI_ClearITPendingBit(EXTI_Line7);
		  g_imu.int1_flag = 1;
	}
	if(EXTI_GetITStatus(EXTI_Line9) != RESET)
	{  
		  EXTI_ClearITPendingBit(EXTI_Line9);
		  g_imu.int2_flag = 1;
	}	
}

void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4) != RESET)
	{  
		  EXTI_ClearITPendingBit(EXTI_Line4);
		  g_acc.int1_flag = 1;
	}
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line10) != RESET)
	{  
		  EXTI_ClearITPendingBit(EXTI_Line10);
		  g_acc.int2_flag = 1;
	}
}

static void imu_setup_irq(void)
{
#if defined(QST_IMU_USE_INT)
    NVIC_InitTypeDef NVIC_InitStructure;  
    EXTI_InitTypeDef EXTI_InitStructure;  
    GPIO_InitTypeDef GPIO_InitStructure;   
      
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);//打开GPIO AFIO的时钟  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	//GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;  
    GPIO_Init(GPIOC, &GPIO_InitStructure); 
          
    EXTI_ClearITPendingBit(EXTI_Line7);  
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7); 
    EXTI_InitStructure.EXTI_Line= EXTI_Line7;
    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
    EXTI_Init(&EXTI_InitStructure);  
          
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;          
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
    NVIC_Init(&NVIC_InitStructure); 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);//打开GPIO AFIO的时钟  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	//GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;  
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
          
    EXTI_ClearITPendingBit(EXTI_Line9);  
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource9); 
    EXTI_InitStructure.EXTI_Line= EXTI_Line9;
    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
    EXTI_Init(&EXTI_InitStructure);  
          
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;          
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
    NVIC_Init(&NVIC_InitStructure);
#endif
}

static void acc_setup_irq(void)
{
#if defined(QST_ACC_USE_INT)
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);//打开GPIO AFIO的时钟  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_10;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	//GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;  
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
          
    EXTI_ClearITPendingBit(EXTI_Line4);  
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4); 
    EXTI_InitStructure.EXTI_Line= EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
    EXTI_Init(&EXTI_InitStructure);  
          
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;          
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
    NVIC_Init(&NVIC_InitStructure); 

	EXTI_ClearITPendingBit(EXTI_Line10);  
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10); 
    EXTI_InitStructure.EXTI_Line= EXTI_Line10;
    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
    EXTI_Init(&EXTI_InitStructure);  
          
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;          
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
    NVIC_Init(&NVIC_InitStructure); 
#endif
	g_acc.int1_flag = 0;
	g_acc.int1_func = acc_int1_callback;
	g_acc.int2_flag = 0;
	g_acc.int2_func = acc_int2_callback;
}
// irq


static void imu_hw_reset(void)
{
	unsigned long long timerst = 0;

	memset(&g_imu, 0, sizeof(g_imu));
	g_imu.report = QST_REPORT_POLLING;

	GPIO_SetBits(GPIOA, GPIO_Pin_8);
	qst_delay(200);
	GPIO_ResetBits(GPIOA, GPIO_Pin_8);
	timerst = HAL_GetTick();
	QST_PRINTF("fis210x reset start %lld\n", timerst);
#if defined(QST_IMU_USE_INT)
	while(!g_imu.int1_flag)
	{
		if((HAL_GetTick()-timerst)>3000)
		{
			QST_PRINTF("fis210x wait for reset timeout time=%lld (ms)!!!\n",HAL_GetTick()-timerst);
			g_imu.report = QST_REPORT_POLLING;
			return;
		}
	}	
	g_imu.report = QST_REPORT_DRI;
	g_imu.int1_flag = 0;
#else
	qst_delay(2500);
#endif
	QST_PRINTF("fis210x reset done used time=%lld (ms)\n", HAL_GetTick()-timerst);
}

static int imu_sw_cali(void)
{
	while(g_imu.cali_flag == 0)
	{
		if(g_imu.report == QST_REPORT_DRI)
		{
			if(g_imu.int2_flag)
			{
				g_imu.int2_flag = 0;
				FisImu_read_xyz(g_out.acc_out, g_out.gyro_out, &g_imu.timestamp_0);
				g_imu.cali_flag = qst_algo_imu_cali(g_out.acc_out, g_out.gyro_out);
			}
		}
		else if(g_imu.report == QST_REPORT_POLLING)
		{
			u8 status;
			if((status=FisImu_readStatus0())&0x22)
			{				
				FisImu_read_xyz(g_out.acc_out, g_out.gyro_out, &g_imu.timestamp_0);
				g_imu.cali_flag = qst_algo_imu_cali(g_out.acc_out, g_out.gyro_out);
			}
			qst_delay(1);
		}
	}
	qst_algo_imu_get_offset(g_out.acc_bis, g_out.gyro_bis, g_out.gyro_rms);

	return 1;
}

static void imu_calc_euler(void)
{
	if(FisImu_readStatus0()&0x22)
	{
		float dt;

#if defined(QST_IMU_INTEGRAL_USE_FIS_TIMESTAMP)
		FisImu_read_xyz(g_out.acc_out, g_out.gyro_out, &g_imu.timestamp_1);
		dt = (float)(((g_imu.timestamp_1+256-g_imu.timestamp_0)%256)*ODR_TIM);
		g_imu.timestamp_0 = g_imu.timestamp_1;
#else
		FisImu_read_xyz(g_out.acc_out, g_out.gyro_out, NULL);
		g_imu.mcu_t1 = HAL_GetTick();
		dt = (float)(g_imu.mcu_t1-g_imu.mcu_t0)*0.001f;
		g_imu.mcu_t0 = g_imu.mcu_t1;
#endif
		if(g_imu.t_init == 0)
		{		
			g_imu.t_init = 1;
			dt = 0.0f;
		}
		qst_algo_imu_update(g_out.acc_out, g_out.gyro_out, dt);
		qst_algo_imu_angle(g_out.euler_ypr);		
		IMU_PRINTF("pol:%f	%f	%f\n",g_out.euler_ypr[0],g_out.euler_ypr[1],g_out.euler_ypr[2]);
	}
}


int main(void)
{
	s32 ret=0;
	u8 senosr_polling = 0;
#if 0
	SYSCLK_Config_WakeUp();
#endif
	SysTick_Init();
	SysTick_Enable(1);

	USART_Config();
	bsp_gpio_config();
#if defined(QST_USE_SW_I2C)
	i2c_sw_gpio_config();
#else
	I2C_Bus_Init();
#endif
#if defined(QST_USE_SPI)
	Spi_Init(3);
#endif
	//bsp_timer2_init(25);

	//RTC_NVIC_Config();
	//RTC_CheckAndConfig(&systmtime);
	QST_PRINTF("qst sensor entry!\n");
	qst_delay(500);

	if(g_chip.qmp6988_ok == 0xff)
	{
		g_chip.qmp6988_ok = qmp6988_init(&g_out.qmp6988_1);
	}
	if(g_chip.qmc7983_ok == 0xff)
	{
		g_chip.qmc7983_ok = qmcX983_init();
	}
	if(g_chip.qmc6308_ok == 0xff)
	{
		g_chip.qmc6308_ok = qmc6308_init();
	}
	if(g_chip.qmaX981_ok == 0xff)
	{
		acc_setup_irq();
		g_chip.qmaX981_ok = qmaX981_init();
		if(g_chip.qmaX981_ok)
		{			
			g_acc.cali_flag = 0;
			g_acc.report = QST_REPORT_POLLING;
		}
	}

	if(g_chip.fis210x_ok == 0xff)
	{
		imu_setup_irq();
		imu_hw_reset();
		g_chip.fis210x_ok = FisImu_init();
		if(g_chip.fis210x_ok)
		{
			if(g_imu.report == QST_REPORT_DRI)
			{
				// check int2 happen!
				g_imu.mcu_t0 = HAL_GetTick();
				QST_PRINTF("fis210x wait DRI start %lld\n", g_imu.mcu_t0);
				while(!g_imu.int2_flag)
				{
					if((HAL_GetTick()-g_imu.mcu_t0)>300)
					{
						g_imu.report = QST_REPORT_POLLING;
						break;
					}
				}
				QST_PRINTF("fis210x wait DRI done = %lld (ms) report=%d\n", (HAL_GetTick()-g_imu.mcu_t0), g_imu.report);
				qst_delay(100);
			}
			else
			{				
				qst_delay(500);
			}
			g_imu.int2_flag = 0;
		}
		else
		{
			g_imu.report = QST_REPORT_OFF;
		}
	}

	if(g_chip.fis210x_ok)
	{
		QST_PRINTF("fis210x found!\n");	
		imu_sw_cali();
		QST_PRINTF("fis210x cali done!\n");
		qst_algo_imu_init(g_out.acc_out, g_out.gyro_out);
#if defined(QST_IMU_TEST)
		g_imu.int2_func = imu_drdy_callback_test;
#else
		g_imu.int2_func = imu_drdy_callback;
#endif
	}
	else if(g_chip.qmaX981_ok)
	{
		qst_delay(200);
		while(g_acc.cali_flag == 0)
		{
			qmaX981_read_acc(g_out.acc_out);
			g_acc.cali_flag = qst_algo_imu_cali(g_out.acc_out, g_out.gyro_out);
			qst_delay(25);
		}
	}

	if(g_chip.qmaX981_ok||g_chip.qmc7983_ok||g_chip.qmc6308_ok||g_chip.qmp6988_ok)
	{
		senosr_polling = 1;
	}

	//bsp_timer2_set(ENABLE);
	while(1)
	{
		if(g_imu.report == QST_REPORT_POLLING)
		{
#if defined(QST_IMU_TEST)
			imu_polling_test();
#else
			imu_calc_euler();
			qst_delay(1);
#endif
		}
		else if(g_imu.report == QST_REPORT_DRI)
		{
			imu_process_irq();
		}

		if(g_chip.qmaX981_ok)
		{
			acc_process_irq();
		}

		if(senosr_polling)
		{
			if(!TickDelay)
			{
				qst_sensor_read();
				TickDelay = 25;			
				qst_mcu_sleep = mcu_sleep_check(g_out.acc_out);
				if(qst_mcu_sleep)
				{
					QST_PRINTF("Mcu enrty stop mode!\n");
					PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);
					//PWR_EnterSTANDBYMode();
					mcu_sleep_exit();
					qst_mcu_sleep = 0;
				}
			}
		}
	}
}

void qst_sensor_read(void)
{
#if defined(QST_ADAPT_NIMING)
	short acc_raw[3];
	short gyro_raw[3];
	short mag_raw[3];
#endif

	if(g_chip.qmaX981_ok && !g_chip.fis210x_ok)
	{
		qmaX981_read_acc(g_out.acc_out);
		qst_algo_imu_data_process(g_out.acc_out, g_out.gyro_out);
	}
	if(g_chip.qmp6988_ok)
	{
		qmp6988_calc_pressure(&g_out.qmp6988_1);
	}
	if(g_chip.qmc7983_ok||g_chip.qmc6308_ok)
	{
		if(g_chip.qmc7983_ok)
		{
			qmcX983_read_mag_xyz(g_out.mag_out);
		}
		else if(g_chip.qmc6308_ok)
		{
			int mag_int[3];

			qmc6308_read_mag_xyz(mag_int);
			g_out.mag_out[0] = (float)mag_int[0];
			g_out.mag_out[1] = (float)mag_int[1];
			g_out.mag_out[2] = (float)mag_int[2];
		}

		qmc_cali.acc[0] = g_out.acc_out[0];
		qmc_cali.acc[1] = g_out.acc_out[1];
		qmc_cali.acc[2] = g_out.acc_out[2];

		qmc_cali.dRawMag[0] = g_out.mag_out[0]*31.25;
		qmc_cali.dRawMag[1] = g_out.mag_out[1]*31.25;
		qmc_cali.dRawMag[2] = g_out.mag_out[2]*31.25;
		process(&qmc_cali.dRawMag[0], HAL_GetTick());
		qmc_cali.data_cali[0] = qmc_cali.dRawMag[0]/31.25;//uT
		qmc_cali.data_cali[1] = qmc_cali.dRawMag[1]/31.25;
		qmc_cali.data_cali[2] = qmc_cali.dRawMag[2]/31.25;
		push2mcal(&qmc_cali);
		//QST_PRINTF("%f	%f	%f	%f	%f	%f	%d\n",mag_out[0],mag_out[1],mag_out[2],qmc_cali.data_cali[0],qmc_cali.data_cali[1],qmc_cali.data_cali[2],get_mag_accuracy());
		if(get_mag_accuracy() == 3)
		{
			g_out.euler_ypr[2] = qmc_cali.yaw;
		}
	}
#if defined(QST_ADAPT_NIMING)
	acc_raw[0] = (short)(g_out.acc_out[0]*1000.0f);
	acc_raw[1] = (short)(g_out.acc_out[1]*1000.0f);
	acc_raw[2] = (short)(g_out.acc_out[2]*1000.0f);
	gyro_raw[0] = (short)(g_out.gyro_out[0]*57.2957796f);
	gyro_raw[1] = (short)(g_out.gyro_out[1]*57.2957796f);
	gyro_raw[2] = (short)(g_out.gyro_out[2]*57.2957796f);
	mag_raw[0] = (short)g_out.mag_out[0];
	mag_raw[1] = (short)g_out.mag_out[1];
	mag_raw[2] = (short)g_out.mag_out[2];
	qst_send_imu_euler(g_out.euler_ypr[0], g_out.euler_ypr[1], g_out.euler_ypr[2], g_out.qmp6988_1.altitude*100, 0x01, 1);
	qst_send_imu_rawdata(gyro_raw, acc_raw, mag_raw);
#else
	if(g_chip.fis210x_ok)
	{
		OUT_PRINTF("Euler:[%f	%f	%f]-mag[%d]-press[%f %f]\n",
						g_out.euler_ypr[0],g_out.euler_ypr[1],g_out.euler_ypr[2],
						get_mag_accuracy(),g_out.qmp6988_1.pressure,g_out.qmp6988_1.temperature);
	}
	else
	{
		OUT_PRINTF("Acc[%f	%f	%f]-mag[%f %d]-press[%f %f]\n",
						g_out.acc_out[0],g_out.acc_out[1],g_out.acc_out[2],
						qmc_cali.yaw,get_mag_accuracy(),g_out.qmp6988_1.pressure,g_out.qmp6988_1.temperature);
	}
#endif
}


static void imu_drdy_callback(void)
{
	float dt;
	
#if defined(QST_IMU_INTEGRAL_USE_FIS_TIMESTAMP)
	FisImu_read_xyz(g_out.acc_out, g_out.gyro_out, &g_imu.timestamp_1);
	dt = (float)(((g_imu.timestamp_1+256-g_imu.timestamp_0)%256)*ODR_TIM);
	g_imu.timestamp_0 = g_imu.timestamp_1;
#else
	FisImu_read_xyz(g_out.acc_out, g_out.gyro_out, NULL);
	g_imu.mcu_t1 = HAL_GetTick();
	dt = (float)(g_imu.mcu_t1-g_imu.mcu_t0)*0.001f;
	g_imu.mcu_t0 = g_imu.mcu_t1;
#endif
	if(g_imu.t_init == 0)
	{		
		g_imu.t_init = 1;
		dt = 0.0f;
	}
	qst_algo_imu_update(g_out.acc_out, g_out.gyro_out, dt);
	qst_algo_imu_angle(g_out.euler_ypr);
	
	IMU_PRINTF("dri:%f	%f	%f\n",g_out.euler_ypr[0],g_out.euler_ypr[1],g_out.euler_ypr[2]);
}

static void imu_process_irq(void)
{
	if(g_imu.int1_flag)
	{
		g_imu.int1_flag = 0;
		QST_PRINTF("imu_process_irq1\n");
		if(g_imu.int1_func)
			g_imu.int1_func();
	}
	
	if(g_imu.int2_flag)
	{
		qst_mcu_sleep = !qst_mcu_sleep;
		if(qst_mcu_sleep)
			GPIO_ResetBits(GPIOA, GPIO_Pin_10);
		else
			GPIO_SetBits(GPIOA, GPIO_Pin_10);
	
		g_imu.int2_flag = 0;		
		if(g_imu.int2_func)
			g_imu.int2_func();
	}
}

static void acc_int1_callback(void)
{
	//QST_PRINTF("acc_int1_callback \n");
	if(g_chip.qmaX981_ok)
	{
		qmaX981_irq_hdlr();
	}
}

static void acc_int2_callback(void)
{
	//QST_PRINTF("acc_int2_callback \n");
	if(g_chip.qmaX981_ok)
	{
		qmaX981_irq_hdlr();
	}
}

static void acc_process_irq(void)
{
	if(g_acc.int1_flag)
	{
		g_acc.int1_flag = 0;
		if(g_acc.int1_func)
			g_acc.int1_func();
	}
	
	if(g_acc.int2_flag)
	{
		g_acc.int2_flag = 0;
		if(g_acc.int2_func)
			g_acc.int2_func();
	}
}

static u8 mcu_sleep_check(float acc[3])
{
	static float sum_1, sum_2;
	static u32 check_count = 0;

	return 0;
	sum_2 = acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2];
	if(((sum_2 - sum_1) > -3.0) && ((sum_2 - sum_2) < 3.0))
	{
		sum_1 = sum_2;
		check_count++;
		if(check_count == 300)
		{
			check_count = 0;
			if(g_chip.fis210x_ok)
			{
				FisImu_enableWakeOnMotion();
				qst_delay(500);
			}
			return 1;
			//qst_delay(500);
		}
	}
	else
	{
		sum_1 = sum_2;
		check_count = 0;
	}

	return 0;
}

static void mcu_sleep_exit(void)
{
	SYSCLK_Config_WakeUp();
	USART_Config();
	QST_PRINTF("Exit stop mode！\n");
	if(g_chip.fis210x_ok)
	{
		u8 status1;

		status1 = FisImu_readStatus1();
		QST_PRINTF("status1[0x%x]\n", status1);
		FisImu_disableWakeOnMotion();
		FisImu_init();
	}
}


// add by yangzhiqiang for test
#if defined(QST_IMU_TEST)
#if 0
#define D_SIZE		150
#define IMU_ODR		256.0f
static sensor_data_t imu_array[2][D_SIZE];
static sensor_data_t imu_array_offset[2];
static sensor_data_t imu_array_rms[2];
static int imu_index = 0;
static int imu_temp = 0;
#endif
static short acc_out_raw[3];
static short gyro_out_raw[3];

static void imu_drdy_callback_test(void)
{	
	//FisImu_read_xyz(acc_out, gyro_out, NULL);
	FisImu_read_xyz_raw(acc_out_raw, gyro_out_raw, NULL);
#if 0
	if(imu_index < D_SIZE)
	{
		imu_array[0][imu_index].x = acc_out[0];
		imu_array[0][imu_index].y = acc_out[1];
		imu_array[0][imu_index].z = acc_out[2];

		imu_array[1][imu_index].x = gyro_out[0];
		imu_array[1][imu_index].y = gyro_out[1];
		imu_array[1][imu_index].z = gyro_out[2];
		imu_index++;
		if(imu_index >= D_SIZE)
		{
			int i;

			imu_index = 0;
			memset(imu_array_offset, 0 , sizeof(imu_array_offset));
			memset(imu_array_rms, 0 , sizeof(imu_array_rms));
			for(i=0;i<D_SIZE;i++)
			{
				imu_array_offset[0].x += imu_array[0][i].x;
				imu_array_offset[0].y += imu_array[0][i].y;
				imu_array_offset[0].z += imu_array[0][i].z;
				
				imu_array_offset[1].x += imu_array[1][i].x;
				imu_array_offset[1].y += imu_array[1][i].y;
				imu_array_offset[1].z += imu_array[1][i].z;
			}
			imu_array_offset[0].x /= (float)D_SIZE;
			imu_array_offset[0].y /= (float)D_SIZE;
			imu_array_offset[0].z /= (float)D_SIZE;
			imu_array_offset[1].x /= (float)D_SIZE;
			imu_array_offset[1].y /= (float)D_SIZE;
			imu_array_offset[1].z /= (float)D_SIZE;
			for(i=0;i<D_SIZE;i++)
			{
				imu_array_rms[0].x += (imu_array[0][i].x-imu_array_offset[0].x)*(imu_array[0][i].x-imu_array_offset[0].x);
				imu_array_rms[0].y += (imu_array[0][i].y-imu_array_offset[0].y)*(imu_array[0][i].y-imu_array_offset[0].y);
				imu_array_rms[0].z += (imu_array[0][i].z-imu_array_offset[0].z)*(imu_array[0][i].z-imu_array_offset[0].z);
				
				imu_array_rms[1].x += (imu_array[1][i].x-imu_array_offset[1].x)*(imu_array[1][i].x-imu_array_offset[1].x);
				imu_array_rms[1].y += (imu_array[1][i].y-imu_array_offset[1].y)*(imu_array[1][i].y-imu_array_offset[1].y);
				imu_array_rms[1].z += (imu_array[1][i].z-imu_array_offset[1].z)*(imu_array[1][i].z-imu_array_offset[1].z);
			}
			imu_array_rms[0].x = sqrtf(imu_array_rms[0].x/(float)D_SIZE)/sqrtf(IMU_ODR);
			imu_array_rms[0].y = sqrtf(imu_array_rms[0].y/(float)D_SIZE)/sqrtf(IMU_ODR);
			imu_array_rms[0].z = sqrtf(imu_array_rms[0].z/(float)D_SIZE)/sqrtf(IMU_ODR);
			
			imu_array_rms[1].x = sqrtf(imu_array_rms[1].x/(float)D_SIZE)/sqrtf(IMU_ODR);
			imu_array_rms[1].y = sqrtf(imu_array_rms[1].y/(float)D_SIZE)/sqrtf(IMU_ODR);
			imu_array_rms[1].z = sqrtf(imu_array_rms[1].z/(float)D_SIZE)/sqrtf(IMU_ODR);

			imu_array_offset[0].z -= 1000;
			imu_temp = (int)FisImu_readTemp();
		}
		QST_PRINTF("(%d) raw:%f %f %f %f %f %f offset:%f %f %f %f %f %f rms:%f %f %f %f %f %f\n",imu_temp, acc_out[0],acc_out[1],acc_out[2],gyro_out[0],gyro_out[1],gyro_out[2],
							imu_array_offset[0].x, imu_array_offset[0].y,imu_array_offset[0].z,imu_array_offset[1].x,imu_array_offset[1].y,imu_array_offset[1].z,
							imu_array_rms[0].x, imu_array_rms[0].y,imu_array_rms[0].z,imu_array_rms[1].x,imu_array_rms[1].y,imu_array_rms[1].z);
	}
	#else
	//QST_PRINTF("raw:%f %f %f %f %f %f\n",acc_out[0],acc_out[1],acc_out[2],gyro_out[0],gyro_out[1],gyro_out[2]);
	IMU_PRINTF("raw1:%d %d %d %d %d %d\n",acc_out_raw[0],acc_out_raw[1],acc_out_raw[2],gyro_out_raw[0],gyro_out_raw[1],gyro_out_raw[2]);
	#endif
}


static void imu_polling_test(void)
{
	if(FisImu_readStatus0()&0x22)

	{
		FisImu_read_xyz_raw(acc_out_raw, gyro_out_raw, NULL);
	}
	IMU_PRINTF("raw2:%d %d %d %d %d %d\n",acc_out_raw[0],acc_out_raw[1],acc_out_raw[2],gyro_out_raw[0],gyro_out_raw[1],gyro_out_raw[2]);
}
#endif

