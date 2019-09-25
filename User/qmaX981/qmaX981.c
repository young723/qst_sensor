/**
  ******************************************************************************
  * @file    qma7981.c
  * @author  Yangzhiqiang@qst
  * @version V1.0
  * @date    2017-12-15
  * @brief    qma6981驱动
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 指南者 开发板 
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */ 

#include "qmaX981.h"
#include <math.h>

#define QMAX981_LOG		printf
#define QMAX981_ERR		printf

#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
extern void qmaX981_step_debounce_reset(void);
extern s32 qmaX981_step_debounce_s32_work(s32 data, u8 irq_level);
extern s32 qmaX981_step_debounce_read_data(s32 result);
#endif
#if defined(QMAX981_CHECK_ABNORMAL_DATA)
extern s32 qmaX981_check_abnormal_data(s32 data_in, s32 *data_out);
#endif


typedef enum
{	
	CHIP_TYPE_QMA6981 = 0,
	CHIP_TYPE_QMA7981,
	CHIP_TYPE_QMA7981_V2,
	CHIP_TYPE_UNDEFINE,
	CHIP_TYPE_MAX
}qmaX981_type;

typedef struct
{
    s16 sign[3];
    u16 map[3];
}qst_convert;

typedef struct
{
	u8					chip_id;
	qmaX981_type		chip_type;
	s32					lsb_1g;
	u8					layout;
	qst_convert			cvt;
	u8					s32_level;
	u8					fifo_mode;
}qmaX981_data;

static const qst_convert qst_map[] = 
{
    { { 1, 1, 1}, {0, 1, 2} },
    { {-1, 1, 1}, {1, 0, 2} },
    { {-1,-1, 1}, {0, 1, 2} },
    { { 1,-1, 1}, {1, 0, 2} },

    { {-1, 1, -1}, {0, 1, 2} },
    { { 1, 1, -1}, {1, 0, 2} },
    { { 1,-1, -1}, {0, 1, 2} },
    { {-1,-1, -1}, {1, 0, 2} }
};


static qmaX981_data g_qmaX981;
static u8 QMAX981_I2C_ADDR	= QMAX981_I2C_SLAVE_ADDR<<1;

const u8 qma6981_init_tbl[][2] = 
{
#if defined(QMAX981_STEPCOUNTER)
	{0x11, 0x80},
	{0x36, 0xb6},
	{0xff, 5},
	{0x36, 0x00},
	{0x11, 0x80},
	{0x0f, QMAX981_RANGE_8G},
	{0x10, 0x2a},
	{0x12, 0x8f},
	{0x13, 0x10},
	{0x14, 0x14},
	{0x15, 0x10},	
	{0x16, 0x0c},
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	{0x19, 0x08},
#endif
	{0x32, 0x02},
	{0x27, QMA6981_OFFSET},
	{0x28, QMA6981_OFFSET},
	{0x29, QMA6981_OFFSET},
#else
	{0x11, 0x80},
	{0x36, 0xb6},
	{0xff, 5},
	{0x36, 0x00},
	{0x11, 0x80},
	//{0x36, 0xb6},
	{0xff, 5},
	//{0x36, 0x00},
	//{0x11, 0x80},
	{0x0f, QMAX981_RANGE_4G},
	{0x10, QMA6981_ODR_125HZ},
#endif
#if defined(QMAX981_FIFO_FUNC)
	{0x10, QMA6981_ODR_250HZ},
	//{0x11, 0x8b},
	{0x3E, 0x40},
//	{0x17, 0x40},
	
//	{0x3E, 0x40},
//	{0x17, 0x20},
	#if defined(QMAX981_FIFO_USE_INT)
//	{0x1a, 0x20},	// fifo s32 map to s321
	#endif
#endif
#if defined(QMAX981_TAP_FUNC)
	{0x10, QMA6981_ODR_250HZ},
	{0x11, 0x80},	// 0x85 {0x2a, 0x80},	
	{0x2b, 0x01},	//0x14	
	{0x16, 0x20},	
	{0x19, 0x20},
	//{0x1b, 0x20},
#endif
#if defined(QMAX981_DRDY_FUNC)
	{0x17, 0x10},
	{0x1a, 0x10},
#endif
#if defined(QMAX981_INT_LATCH_MODE)
	{0x21, 0x01},
#endif

	{0xff, 1}
};

/*	
qma7981 odr setting
0x10<2:0>		ODR(Hz)				Time(ms)	|	RANGE 0x0f<3:0>
000				43.3125				23.088		|	0001	2g  		244ug/LSB
001				86.4453				11.568		|	0010	4g  		488ug/LSB
002				172.1763			5.808		|	0100	8g  		977ug/LSB
003				341.5300			2.928		|	1000	16g  	1.95mg/LSB
004				672.0430			1.488		|	1111	32g  	3.91mg/LSB
005				32.5013				30.768		|	Others	2g  		244ug/LSB
006				129.3995			7.728		|
007				257.2016			3.888		|
*/

const u8 qma7981_init_tbl[][2] = 
{
	{0x11, 0x80},
	{0x36, 0xb6},
	{0xff, 5},
	{0x36, 0x00},
	{0x0f, QMAX981_RANGE_4G},
	{0x10, 0xe1},		// BW 128hz	
	{0x11, 0x80},
	{0x5f, 0x80},		// enable test mode,take control the FSM
	{0x5f, 0x00},		//normal mode

	{0xff, 1}
};


void qmaX981_delay(u32 delay)
{
	u32 i,j;
	for(i=0;i<delay;i++)
	{
		for(j=0;j<1000;j++)
		{
			;
		}
	}
}

u8 qmaX981_writereg(u8 reg_add,u8 reg_dat)
{
#if defined(QST_USE_SPI)
	return qmaX981_spi_write(reg_add, reg_dat);
#else
	#if defined(QST_USE_SW_I2C)
	return qst_sw_writereg(QMAX981_I2C_ADDR, reg_add, reg_dat);
	#else
	return I2C_ByteWrite(QMAX981_I2C_ADDR, reg_add, reg_dat);
	#endif
#endif
}

u8 qmaX981_readreg(u8 reg_add,u8 *buf,u16 num)
{
#if defined(QST_USE_SPI)
	return qmaX981_spi_read(reg_add, buf, num);
#else
	#if defined(QST_USE_SW_I2C)
	return qst_sw_readreg(QMAX981_I2C_ADDR, reg_add, buf, num);
	#else
	return I2C_BufferRead(QMAX981_I2C_ADDR, reg_add, buf, (u16)num);
	#endif
#endif
}


u8 qmaX981_chip_id()
{
	u8 chip_id = 0x00;
	//qmaX981_writereg(QMAX981_REG_POWER_CTL, 0x80);
	qmaX981_readreg(QMAX981_CHIP_ID, &chip_id, 1);
	QMAX981_LOG("qmaX981_chip_id id=0x%x \n", chip_id);

	return chip_id;
}


static s32 qma6981_initialize(void)
{
	s32 ret = 0;
	s32 index, total;
	u8 data[2] = {0};

	total = sizeof(qma6981_init_tbl)/sizeof(qma6981_init_tbl[0]);
	for(index=0; index<total; index++)
	{	
		data[0] = qma6981_init_tbl[index][0];
		data[1] = qma6981_init_tbl[index][1];
		if(data[0] == 0xff)
		{
			qmaX981_delay(data[1]);
		}
		else
		{
			if(data[0] == QMAX981_REG_RANGE)
			{
				if(data[1] == QMAX981_RANGE_4G)
					g_qmaX981.lsb_1g = 128;
				else if(data[1] == QMAX981_RANGE_8G)
					g_qmaX981.lsb_1g = 64;
				else					
					g_qmaX981.lsb_1g = 256;
			}

			ret = qmaX981_writereg(data[0],data[1]);
			if(ret == 0)
			{
				QMAX981_ERR("qma6981_initialize ret=%d reg_addr=%x \n", ret, data[0]);
				//return ret;
			}
			qmaX981_delay(2);
		}
	}

   	return ret;
}


static s32 qma7981_initialize(void)
{
	s32 ret = 0;
	s32 index, total;
	u8 data[4] = {0};
	u8 reg_0x10 = 0;
	u8 reg_0x16 = 0;
	u8 reg_0x18 = 0;
	u8 reg_0x19 = 0;
	u8 reg_0x1a = 0;
	u8 reg_0x2c = 0;

	total = sizeof(qma7981_init_tbl)/sizeof(qma7981_init_tbl[0]);
	for(index=0; index<total; index++)
	{	
		data[0] = qma7981_init_tbl[index][0];
		data[1] = qma7981_init_tbl[index][1];
		if(data[0] == 0xff)
		{
			qmaX981_delay(data[1]);
		}
		else
		{
			if(data[0] == QMAX981_REG_RANGE)
			{
				if(data[1] == QMAX981_RANGE_4G)
					g_qmaX981.lsb_1g = 2048;
				else if(data[1] == QMAX981_RANGE_8G)
					g_qmaX981.lsb_1g = 1024;
				else if(data[1] == QMAX981_RANGE_16G)
					g_qmaX981.lsb_1g = 512;
				else if(data[1] == QMAX981_RANGE_32G)
					g_qmaX981.lsb_1g = 256;
				else
					g_qmaX981.lsb_1g = 4096;
			}
			ret = qmaX981_writereg(data[0],data[1]);
			if(ret == 0)
			{
				QMAX981_ERR("qma7981_initialize ret=%d\n", ret);
				return ret;
			}
			qmaX981_delay(2);
		}
	}

	// read reg
	qmaX981_readreg(0x16, &reg_0x16, 1);
	qmaX981_readreg(0x18, &reg_0x18, 1);
	qmaX981_readreg(0x19, &reg_0x19, 1);
	qmaX981_readreg(0x1a, &reg_0x1a, 1);
	qmaX981_readreg(0x2c, &reg_0x2c, 1);
	// read reg
	reg_0x10 = 0xe0;
	qmaX981_writereg(0x10, reg_0x10);
#if defined(QMAX981_STEPCOUNTER)
	if(reg_0x10 == 0xe0)
	{
		// ODR: 65hz 15.48 ms
		qmaX981_writereg(0x12, 0x94);
		qmaX981_writereg(0x13, 0x80);		// clear step
		qmaX981_writereg(0x13, 0x00);		// 
		qmaX981_writereg(0x14, 0x12);		// STEP_TIME_LOW<7:0>*(1/ODR) 
		qmaX981_writereg(0x15, 0x10);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
	}
	else if(reg_0x10 == 0xe1)
	{
		// ODR: 130hz 7.74 ms
		qmaX981_writereg(0x12, 0x94);
		qmaX981_writereg(0x13, 0x80);		// clear step
		qmaX981_writereg(0x13, 0x00);		// 
		qmaX981_writereg(0x14, 0x24);		// STEP_TIME_LOW<7:0>*(1/ODR) 
		qmaX981_writereg(0x15, 0x20);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
	}
	else if(reg_0x10 == 0xe2)
	{
		// ODR: 258Hz 3.87 ms
		qmaX981_writereg(0x12, 0x94);
		qmaX981_writereg(0x13, 0x80);		// clear step
		qmaX981_writereg(0x13, 0x00);		// 
		qmaX981_writereg(0x14, 0x48);		// STEP_TIME_LOW<7:0>*(1/ODR) 
		qmaX981_writereg(0x15, 0x40);		// STEP_TIME_UP<7:0>*8*(1/ODR) 
	}

	//qmaX981_writereg(0x1f, 0x00);

	// step int
	#if defined(QMA7981_STEP_INT)
	reg_0x16 |= 0x08;
	reg_0x19 |= 0x08;
	qmaX981_writereg(0x16, reg_0x16);
	qmaX981_writereg(0x19, reg_0x19);
	#endif
	#if defined(QMA7981_SIGNIFICANT_STEP)
	qmaX981_writereg(0x1d, 0x26);		//every 30 step
	reg_0x16 |= 0x40;
	reg_0x19 |= 0x40;
	qmaX981_writereg(0x16, reg_0x16);
	qmaX981_writereg(0x19, reg_0x19);
	#endif
#endif

//RANGE<3:0> Acceleration range Resolution
//0001 2g 244ug/LSB
//0010 4g 488ug/LSB
//0100 8g 977ug/LSB
//1000 16g 1.95mg/LSB
//1111 32g 3.91mg/LSB
//Others 2g 244ug/LSB

//0x2c
//Duration = (NO_MOT_DUR<3:0> + 1) * 1s, if NO_MOT_DUR<5:4> =b00 
//Duration = (NO_MOT_DUR<3:0> + 4) * 5s, if NO_MOT_DUR<5:4> =b01 
//Duration = (NO_MOT_DUR<3:0> + 10) * 10s, if NO_MOT_DUR<5:4> =b1x 
//ANY_MOT_DUR<1:0>: any motion interrupt will be triggered when slope > ANY_MOT_TH for (ANY_MOT_DUR<1:0> + 1) samples 

//0x2e ANY MOTION MOT_CONF2
//TH= ANY_MOT_TH<7:0> * 16 * LSB 

#if defined(QMA7981_ANY_MOTION)
	reg_0x18 |= 0x07;
	reg_0x1a |= 0x01;
	reg_0x2c |= 0x00;
	
	qmaX981_writereg(0x18, reg_0x18);
	qmaX981_writereg(0x1a, reg_0x1a);
	qmaX981_writereg(0x2c, reg_0x2c);
	qmaX981_writereg(0x2e, 0x0a);		// 0.488*16*32 = 250mg
	
#if defined(QMA7981_SIGNIFICANT_MOTION)
	qmaX981_writereg(0x2f, 0x01|0x01);
	reg_0x19 |= 0x01;
	qmaX981_writereg(0x19, reg_0x19);
#endif
#endif

#if defined(QMA7981_NO_MOTION)
	reg_0x18 |= 0xe0;
	reg_0x1a |= 0x80;
	reg_0x2c |= 0x24;

	qmaX981_writereg(0x18, reg_0x18);
	qmaX981_writereg(0x1a, reg_0x1a);
	qmaX981_writereg(0x2c, reg_0x2c);
	qmaX981_writereg(0x2d, 0x14);
#endif

#if defined(QMA7981_HAND_UP_DOWN)
	reg_0x16 |= 0x02;
	reg_0x19 |= 0x02;
			
	qmaX981_writereg(0x16, reg_0x16);
	qmaX981_writereg(0x19, reg_0x19);
	// hand down
	reg_0x16 |= 0x04;
	reg_0x19 |= 0x04;
	qmaX981_writereg(0x16, reg_0x16);
	qmaX981_writereg(0x19, reg_0x19);
	// hand down	
#if 0	// swap xy
	read_reg(0x42, &reg_0x42, 1);
	reg_0x42 |= 0x80;		// 0x42 bit 7 swap x and y
	qmaX981_writereg(0x42, reg_0x42);
#endif
#endif

#if defined(QMA7981_DATA_READY)
	reg_0x1a |= 0x10;
	qmaX981_writereg(0x17, 0x10);
	qmaX981_writereg(0x1a, reg_0x1a);
#endif

#if defined(QMA7981_INT_LATCH)
	qmaX981_writereg(0x21, 0x1f);	// default 0x1c, step latch mode
	qmaX981_readreg(0x09, data, 3);
	QMAX981_LOG("read status=[0x%x 0x%x 0x%x] \n", data[0],data[1],data[2]);
#endif
/*
{
	u8 readdata = 0;
	qmaX981_readreg(0x18, &readdata, 1);
	QMAX981_LOG("read 0x18=0x%x\n", readdata);
	qmaX981_readreg(0x1a, &readdata, 1);
	QMAX981_LOG("read 0x1a=0x%x\n", readdata);
	qmaX981_readreg(0x2c, &readdata, 1);
	QMAX981_LOG("read 0x2c=0x%x\n", readdata);
	qmaX981_readreg(0x2d, &readdata, 1);
	QMAX981_LOG("read 0x2d=0x%x\n", readdata);
	qmaX981_readreg(0x2e, &readdata, 1);
	QMAX981_LOG("read 0x2e=0x%x\n", readdata);
}
*/
	return ret;
}


static s32 qma7981_v2_initialize(void)
{
	s32 ret = 0;
	s32 index, total;
	u8 data[4] = {0};
	u8 reg_0x10 = 0;
	u8 reg_0x16 = 0;
	u8 reg_0x18 = 0;
	u8 reg_0x19 = 0;
	u8 reg_0x1a = 0;
	u8 reg_0x2c = 0;
	#if defined(QMAX981_FIFO_FUNC)
	u8 reg_0x3e = 0x40;
	#endif

	total = sizeof(qma7981_init_tbl)/sizeof(qma7981_init_tbl[0]);
	for(index=0; index<total; index++)
	{	
		data[0] = qma7981_init_tbl[index][0];
		data[1] = qma7981_init_tbl[index][1];
		if(data[0] == 0xff)
		{
			qmaX981_delay(data[1]);
		}
		else
		{
			if(data[0] == QMAX981_REG_RANGE)
			{
				if(data[1] == QMAX981_RANGE_4G)
					g_qmaX981.lsb_1g = 2048;
				else if(data[1] == QMAX981_RANGE_8G)
					g_qmaX981.lsb_1g = 1024;
				else if(data[1] == QMAX981_RANGE_16G)
					g_qmaX981.lsb_1g = 512;
				else if(data[1] == QMAX981_RANGE_32G)
					g_qmaX981.lsb_1g = 256;
				else
					g_qmaX981.lsb_1g = 4096;
			}
			ret = qmaX981_writereg(data[0],data[1]);
			if(ret == 0)
			{
				QMAX981_ERR("qma7981_initialize ret=%d\n", ret);
				return ret;
			}
			qmaX981_delay(2);
		}
	}

	// read reg
	qmaX981_readreg(0x16, &reg_0x16, 1);
	qmaX981_readreg(0x18, &reg_0x18, 1);
	qmaX981_readreg(0x19, &reg_0x19, 1);
	qmaX981_readreg(0x1a, &reg_0x1a, 1);
	qmaX981_readreg(0x2c, &reg_0x2c, 1);
	// read reg

	//0xe0	[65 hz		15.48 ms]
	//0xe1	[129 hz 	7.74 ms]
	//0xe2	[258 hz 	3.87 ms]
	reg_0x10 = 0xe0;
	qmaX981_writereg(0x10, reg_0x10);
//RANGE<3:0> Acceleration range Resolution
//0001 2g 244ug/LSB
//0010 4g 488ug/LSB
//0100 8g 977ug/LSB
//1000 16g 1.95mg/LSB
//1111 32g 3.91mg/LSB
//Others 2g 244ug/LSB

//0x2c
//Duration = (NO_MOT_DUR<3:0> + 1) * 1s, if NO_MOT_DUR<5:4> =b00 
//Duration = (NO_MOT_DUR<3:0> + 4) * 5s, if NO_MOT_DUR<5:4> =b01 
//Duration = (NO_MOT_DUR<3:0> + 10) * 10s, if NO_MOT_DUR<5:4> =b1x 
//ANY_MOT_DUR<1:0>: any motion interrupt will be triggered when slope > ANY_MOT_TH for (ANY_MOT_DUR<1:0> + 1) samples 

//0x2e ANY MOTION MOT_CONF2
//TH= ANY_MOT_TH<7:0> * 16 * LSB 

#if defined(QMA7981_ANY_MOTION)
	reg_0x18 |= 0x07;
	reg_0x1a |= 0x01;
	reg_0x2c |= 0x00;
	
	qmaX981_writereg(0x1a, reg_0x1a);
	qmaX981_writereg(0x18, reg_0x18);
	qmaX981_writereg(0x2c, reg_0x2c);
	qmaX981_writereg(0x2e, 0x0a);		// 0.488*16*32 = 250mg
	
#if defined(QMA7981_SIGNIFICANT_MOTION)
	qmaX981_writereg(0x2f, 0x04|0x01);
	reg_0x19 |= 0x01;
	qmaX981_writereg(0x19, reg_0x19);
#endif
#endif

#if defined(QMA7981_NO_MOTION)
	reg_0x18 |= 0xe0;
	reg_0x1a |= 0x80;
	reg_0x2c |= 0x24;

	qmaX981_writereg(0x18, reg_0x18);
	qmaX981_writereg(0x1a, reg_0x1a);
	qmaX981_writereg(0x2c, reg_0x2c);
	qmaX981_writereg(0x2d, 0x14);
#endif

#if defined(QMA7981_DATA_READY)
	reg_0x1a |= 0x10;
	qmaX981_writereg(0x17, 0x10);
	qmaX981_writereg(0x1a, reg_0x1a);
#endif

#if defined(QMAX981_FIFO_FUNC)
	g_qmaX981.fifo_mode = 1;

	if(g_qmaX981.fifo_mode == 1)
	{
		//qmaX981_writereg(0x31, 0x40);
		qmaX981_writereg(0x3E, 0x40);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
		qmaX981_writereg(0x17, 0x20);
		qmaX981_writereg(0x1a, 0x20);
	}
	else if(g_qmaX981.fifo_mode == 2)
	{	
		qmaX981_writereg(0x31, 0x3f);
		qmaX981_writereg(0x3E, 0x80);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
		qmaX981_writereg(0x17, 0x40);
		qmaX981_writereg(0x1a, 0x40);
	}
	else 
	{
		qmaX981_writereg(0x3E, 0x00);	//bit[6:7] 0x00:BYPASS 0x40:FIFO 0x80:STREAM
		qmaX981_writereg(0x17, 0x20);
		qmaX981_writereg(0x1a, 0x20);
	}
#endif

#if defined(QMA7981_INT_LATCH)
	qmaX981_writereg(0x21, 0x1f);	// default 0x1c, step latch mode
	qmaX981_readreg(0x09, data, 3);
	QMAX981_LOG("read status=[0x%x 0x%x 0x%x] \n", data[0],data[1],data[2]);
#endif

	{
		u8 readdata = 0;
		qmaX981_readreg(0x18, &readdata, 1);
		QMAX981_LOG("read 0x18=0x%x\n", readdata);
		qmaX981_readreg(0x1a, &readdata, 1);
		QMAX981_LOG("read 0x1a=0x%x\n", readdata);
		qmaX981_readreg(0x2c, &readdata, 1);
		QMAX981_LOG("read 0x2c=0x%x\n", readdata);
		qmaX981_readreg(0x2d, &readdata, 1);
		QMAX981_LOG("read 0x2d=0x%x\n", readdata);
		qmaX981_readreg(0x2e, &readdata, 1);
		QMAX981_LOG("read 0x2e=0x%x\n", readdata);
		qmaX981_readreg(0x2f, &readdata, 1);
		QMAX981_LOG("read 0x2f=0x%x\n", readdata);
	}

	return 1;
}


#if defined(QMAX981_FIFO_FUNC)
static s32 qmaX981_fifo_data[64][3];
static u8 qmaX981_fifo_reg[64*6];

static s32 qma6981_read_fifo_raw(s32 *data)
{
	//s32 res;	
	u8 databuf[6] = {0};		
	u8 i;
	s32 ret;
	
	ret = qmaX981_readreg(0x3f, databuf, 6);
	qmaX981_delay(2);
	if(ret != 1)
	{
		QMAX981_LOG("qma6981_read_fifo_raw error \n");
		return ret;
	}

 	data[0]  = (short)((databuf[1]<<2) |( databuf[0]>>6));
	data[1]  = (short)((databuf[3]<<2) |( databuf[2]>>6));
	data[2]  = (short)((databuf[5]<<2) |( databuf[4]>>6));

	for(i=0;i<3;i++)				
	{	//because the data is store in binary complement number formation in computer system
		if ( data[i] == 0x0200 )	//so we want to calculate actual number here
			data[i]= -512;		//10bit resolution, 512= 2^(10-1)
		else if ( data[i] & 0x0200 )	//transfor format
		{					//prs32k("data 0 step %x \n",data[i]);
			data[i] -= 0x1;			//prs32k("data 1 step %x \n",data[i]);
			data[i] = ~data[i];		//prs32k("data 2 step %x \n",data[i]);
			data[i] &= 0x01ff;		//prs32k("data 3 step %x \n\n",data[i]);
			data[i] = -data[i];		
		}
#if defined(QMAX981_STEPCOUNTER)
		data[i] -= QMA6981_OFFSET;
#endif
	}
	//prs32k("qma6981 fifo raw: %d	%d	%d\n", data[0], data[1], data[2]);	

	return 1;	
}

static s32 qma7981_read_fifo_raw(s32 *data)
{
	s32 res;	
	u8 databuf[6] = {0};
	s32 ret;
	
	ret = qmaX981_readreg(0x3f, databuf, 6);
	qmaX981_delay(2);
	if(ret != 1)
	{
		QMAX981_LOG("qma7981_read_fifo_raw error \n");
		return ret;
	}

	data[0] = (short)((databuf[1]<<8)|(databuf[0]));
	data[1] = (short)((databuf[3]<<8)|(databuf[2]));
	data[2] = (short)((databuf[5]<<8)|(databuf[4]));
	data[0] = data[0]>>2;
	data[1] = data[1]>>2;
	data[2] = data[2]>>2;

	//prs32k("qma7981 fifo raw: %d	%d	%d\n", data[0], data[1], data[2]);	
	return 1;
}

static s32 qmaX981_read_fifo_acc(s32 *acc_data)
{
	s32 ret = 0;
	s32 raw_data[3];

	if(g_qmaX981.chip_type == CHIP_TYPE_QMA6981)
	{
		ret = qma6981_read_fifo_raw(raw_data);
	}
	else if(g_qmaX981.chip_type == CHIP_TYPE_QMA7981_V2)
	{
		ret = qma7981_read_fifo_raw(raw_data);
	}
	else
	{
		ret = 0;
	}
	
	if(1 != ret ){
		QMAX981_ERR("qmaX981_read_fifo_acc error\n");
		return ret;
	}
	
	//remap coordinate
	acc_data[g_qmaX981.cvt.map[0]] = g_qmaX981.cvt.sign[0]*raw_data[0];
	acc_data[g_qmaX981.cvt.map[1]] = g_qmaX981.cvt.sign[1]*raw_data[1];
	acc_data[g_qmaX981.cvt.map[2]] = g_qmaX981.cvt.sign[2]*raw_data[2];
	//QMAX981_LOG("qmaX981 AFTER x1:%d,y:%d,z:%d\n",data[0],data[1],data[2]);

	acc_data[0] = (acc_data[0]*9807)/(g_qmaX981.lsb_1g);
	acc_data[1] = (acc_data[1]*9807)/(g_qmaX981.lsb_1g);
	acc_data[2] = (acc_data[2]*9807)/(g_qmaX981.lsb_1g);

	return ret;
}

s32 qmaX981_read_fifo(u8 is_raw)
{
	s32 ret = 0;
	u8 databuf[2];
	s32 acc_data[3];
	s32 icount;
	s32 fifo_depth = 32;

	ret = qmaX981_readreg(QMAX981_FIFO_STATE, databuf, 1);
	//qmaX981_delay(2);
	fifo_depth = databuf[0]&0x7f;
#if 1
	qmaX981_readreg(0x3f, qmaX981_fifo_reg, fifo_depth*6);
#else
	for(icount=0; icount<fifo_depth; icount++)
	{
		qmaX981_readreg(0x3f, &qmaX981_fifo_reg[icount*6], 6);
	}
#endif

	if(g_qmaX981.fifo_mode == 0x01)
	{
		ret = qmaX981_readreg(QMAX981_INT_STAT1, databuf, 1);
		ret = qmaX981_writereg(0x3e, 0x40);
	}
// log fifo
	for(icount=0; icount<fifo_depth; icount++)
	{
		acc_data[0]  = (short)((qmaX981_fifo_reg[1+icount*6]<<8) |(qmaX981_fifo_reg[0+icount*6]));
		acc_data[1]  = (short)((qmaX981_fifo_reg[3+icount*6]<<8) |(qmaX981_fifo_reg[2+icount*6]));
		acc_data[2]  = (short)((qmaX981_fifo_reg[5+icount*6]<<8) |(qmaX981_fifo_reg[4+icount*6]));
		if(g_qmaX981.chip_type == CHIP_TYPE_QMA6981)
		{
			acc_data[0]  = acc_data[0]>>6;
			acc_data[1]  = acc_data[1]>>6;
			acc_data[2]  = acc_data[2]>>6;
		}
		else if(g_qmaX981.chip_type == CHIP_TYPE_QMA7981_V2)
		{
			acc_data[0]  = acc_data[0]>>2;
			acc_data[1]  = acc_data[1]>>2;
			acc_data[2]  = acc_data[2]>>2;
		}
		if(is_raw)
			QMAX981_LOG("fifo_data %d:%d	%d	%d\n", icount, acc_data[0],acc_data[1],acc_data[2]);
		else
			QMAX981_LOG("fifo_data %d:%f	%f	%f\n", icount, acc_data[0]*9.807f/g_qmaX981.lsb_1g, acc_data[1]*9.807f/g_qmaX981.lsb_1g, acc_data[2]*9.807f/g_qmaX981.lsb_1g);
	}
// log fifo
	return ret;
}
#endif


#define QMAX981_NO_MOTION_THRES		(g_qmaX981.lsb_1g/10)
#define QMAX981_NO_MOTION_TIMES		300

typedef struct
{
	int sum_last;
	int sum;
	int count;
}qst_no_motion_t;

static qst_no_motion_t g_nm;

unsigned char qmaX981_check_no_motion(void)
{
	s32 acc_raw[3];
	qma7981_read_raw_xyz(acc_raw);

	g_nm.sum = QMAX981_ABS(acc_raw[0])+QMAX981_ABS(acc_raw[1])+QMAX981_ABS(acc_raw[2]);
	//QMAX981_LOG("qmaX981_check_no_motion %d\n", g_nm.sum-g_nm.sum_last);
	if(QMAX981_ABS((g_nm.sum-g_nm.sum_last)) < QMAX981_NO_MOTION_THRES)
	{
		g_nm.count++;
	}
	else
	{
		g_nm.count = 0;
		QMAX981_LOG("any motion!\n");
	}

	g_nm.sum_last = g_nm.sum;
	if(g_nm.count >= QMAX981_NO_MOTION_TIMES)
	{
		if(!(g_nm.count%QMAX981_NO_MOTION_TIMES))
			QMAX981_LOG("no motion!\n");
		return 1;
	}
	else
	{
		return 0;
	}
}

void qmaX981_irq_hdlr(void)
{
	u8 ret = 0;
	u8 databuf[4];

	if(g_qmaX981.chip_type == CHIP_TYPE_QMA6981)
	{
		ret = qmaX981_readreg(QMAX981_INT_STAT0, databuf, 4);
		QMAX981_LOG("qma6981_irq_hdlr [0x%x 0x%x 0x%x 0x%x]\n", databuf[0],databuf[1],databuf[2],databuf[3]);
	}
	else if((g_qmaX981.chip_type == CHIP_TYPE_QMA7981)||(g_qmaX981.chip_type == CHIP_TYPE_QMA7981_V2))
	{
#if defined(QMA7981_DATA_READY)
		{
		//s32 acc_raw[3];
		//qma7981_read_raw_xyz(acc_raw);
		//QMAX981_LOG("qma798_drdy	%d	%d	%d\n", acc_raw[0],acc_raw[1],acc_raw[2]);
		qmaX981_check_no_motion();
		}
		return;
#endif
		ret = qmaX981_readreg(0x09, databuf, 3);
		QMAX981_LOG("qma7981_irq_hdlr [0x%x 0x%x 0x%x]\n", databuf[0],databuf[1],databuf[2]);
		if(databuf[0]&0x07)
		{
			QMAX981_LOG("qma7981_irq_hdlr any motion!\n");
		}
		else if(databuf[0]&0x80)
		{
			QMAX981_LOG("qma7981_irq_hdlr no motion!\n");
		}
#if defined(QMAX981_FIFO_FUNC)
		else if(databuf[2]&0x20)
		{
			//QMAX981_LOG("FIFO FULL\n");
			qmaX981_read_fifo(1);
		}
		else if(databuf[2]&0x40)
		{
			//QMAX981_LOG("FIFO WMK\n");
			qmaX981_read_fifo(1);
		}
#endif
	}
}


#if 0//defined(QMAX981_USE_IRQ1)
static void qmaX981_setup_irq1(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;  
    EXTI_InitTypeDef EXTI_InitStructure;  
    GPIO_InitTypeDef GPIO_InitStructure;   
      
    RCC_APB2PeriphClockCmd(QMAX981_IRQ1_RCC|RCC_APB2Periph_AFIO, ENABLE);//打开GPIO AFIO的时钟  
    GPIO_InitStructure.GPIO_Pin = QMAX981_IRQ1_PIN;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU;  
    GPIO_Init(QMAX981_IRQ1_PORT, &GPIO_InitStructure); 
          
    EXTI_ClearITPendingBit(EXTI_Line11);  
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource11);//PC11  为GPIOC的PIN11  
    EXTI_InitStructure.EXTI_Line= EXTI_Line11; //PC11，为：EXTI_Line11  
    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;   //中断方式为上升与下降沿  
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
    EXTI_Init(&EXTI_InitStructure);  
          
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;          
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
    NVIC_Init(&NVIC_InitStructure); 
}

void EXTI15_10_IRQHandler(void)         //这里为：EXTI15_10 (外部中断号的10~15都在这里实现）  
{
	u8 ret;
	u8 data[2];
	s32 step_num;
	
#if defined(QMAX981_TAP_FUNC)
	ret = qmaX981_readreg(QMAX981_INT_STAT0, data, 1);
	QMAX981_LOG("EXTI15_10_IRQHandler value_0a=%x \r\n", data[0]);
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line11);		 //清中断  
		QMAX981_LOG("EXTI_ClearITPendingBit\r\n");
	}
#endif
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)	
	if(EXTI_GetITStatus(EXTI_Line11) != RESET) //这里为判断相应的中断号是否进入中断，如果有多个中断的话。  
	{
		g_qmaX981.s32_level = GPIO_ReadInputDataBit(QMAX981_IRQ1_PORT,QMAX981_IRQ1_PIN);		
		ret = qmaX981_readreg(QMAX981_STEP_CNT_L, data, 2);
		if(ret)
		{
			step_num = (data[1]<<8)|data[0];
			QMAX981_LOG("gpio level = %d step_num=%d \r\n", g_qmaX981.s32_level, step_num);
			qmaX981_step_debounce_s32_work(step_num, g_qmaX981.s32_level);
		}
		EXTI_ClearITPendingBit(EXTI_Line11);		 //清中断  
		QMAX981_LOG("EXTI_ClearITPendingBit\r\n");
	}
#endif
#if defined(QMAX981_FIFO_USE_INT)
	if(EXTI_GetITStatus(EXTI_Line11) != RESET) //这里为判断相应的中断号是否进入中断，如果有多个中断的话。  
	{
		u8 reg_5b;
		
		ret = qmaX981_readreg(0x5b, &reg_5b, 1);
		QMAX981_LOG("reg_5b=0x%x \r\n", reg_5b);
		qmaX981_read_fifo(0);		
		EXTI_ClearITPendingBit(EXTI_Line11);		 //清中断  
	}
#endif
#if defined(QMA7981_ANY_MOTION)
	if(EXTI_GetITStatus(EXTI_Line11) != RESET) //这里为判断相应的中断号是否进入中断，如果有多个中断的话。  
	{
		u8 reg_data[3];
		
		EXTI_ClearITPendingBit(EXTI_Line11);		 //清中断  
		ret = qmaX981_readreg(0x09, reg_data, 3);
		QMAX981_LOG("reg_data=[0x%x 0x%x 0x%x]\n", reg_data[0],reg_data[1],reg_data[2]);
	}
#endif
}

#endif


#if 0//defined(QMAX981_USE_IRQ2)
static void qmaX981_setup_irq2(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;  
    EXTI_InitTypeDef EXTI_InitStructure;  
    GPIO_InitTypeDef GPIO_InitStructure;   
      
    RCC_APB2PeriphClockCmd(QMAX981_IRQ2_RCC|RCC_APB2Periph_AFIO, ENABLE);//打开GPIO AFIO的时钟  
    GPIO_InitStructure.GPIO_Pin = QMAX981_IRQ2_PIN;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(QMAX981_IRQ2_PORT, &GPIO_InitStructure); 
          
    EXTI_ClearITPendingBit(EXTI_Line5);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource5);//PC11  为GPIOC的PIN11  
    EXTI_InitStructure.EXTI_Line= EXTI_Line5; //PC11，为：EXTI_Line11  
    EXTI_InitStructure.EXTI_Mode= EXTI_Mode_Interrupt;   
    EXTI_InitStructure.EXTI_Trigger= EXTI_Trigger_Rising;   //中断方式为上升与下降沿  
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
    EXTI_Init(&EXTI_InitStructure);  
          
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);   //NVIC  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority= 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority= 2;          
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;     
    NVIC_Init(&NVIC_InitStructure); 
}


void EXTI9_5_IRQHandler(void)
{
	u8 ret;
	u8 value_0a;
	
	ret = qmaX981_readreg(QMAX981_INT_STAT0, &value_0a, 1);
	QMAX981_LOG("EXTI9_5_IRQHandler value_0a=%x\r\n", value_0a);
	
	if(EXTI_GetITStatus(EXTI_Line5) != RESET) //这里为判断相应的中断号是否进入中断，如果有多个中断的话。  
	{  
		  EXTI_ClearITPendingBit(EXTI_Line5);		 //清中断  
		  QMAX981_LOG("EXTI_ClearITPendingBit\r\n");
	}
}
#endif

static s32 qma6981_read_raw_xyz(s32 *data)
{
	//s32 res;	
	u8 databuf[6] = {0};		
	u8 i;
	s32 ret;

	ret = qmaX981_readreg(QMAX981_XOUTL, databuf, 6);
	if(ret == 0){
		QMAX981_ERR("read xyz error!!!");
		return 0;	
	}
#if 0
 	data[0]  = (short)((databuf[1]<<2) |( databuf[0]>>6));
	data[1]  = (short)((databuf[3]<<2) |( databuf[2]>>6));
	data[2]  = (short)((databuf[5]<<2) |( databuf[4]>>6));

	for(i=0;i<3;i++)				
	{	//because the data is store in binary complement number formation in computer system
		if ( data[i] == 0x0200 )	//so we want to calculate actual number here
			data[i]= -512;		//10bit resolution, 512= 2^(10-1)
		else if ( data[i] & 0x0200 )	//transfor format
		{					//prs32k("data 0 step %x \n",data[i]);
			data[i] -= 0x1;			//prs32k("data 1 step %x \n",data[i]);
			data[i] = ~data[i];		//prs32k("data 2 step %x \n",data[i]);
			data[i] &= 0x01ff;		//prs32k("data 3 step %x \n\n",data[i]);
			data[i] = -data[i];		
		}
#if defined(QMAX981_STEPCOUNTER)
		data[i] -= QMA6981_OFFSET;
#endif
	}
#else
	data[0] = (short)((databuf[1]<<8)|(databuf[0]));
	data[1] = (short)((databuf[3]<<8)|(databuf[2]));
	data[2] = (short)((databuf[5]<<8)|(databuf[4]));
	data[0] = data[0]>>6;
	data[1] = data[1]>>6;
	data[2] = data[2]>>6;
#endif

	return 1;
}

static s32 qma7981_read_raw_xyz(s32 *data)
{
	u8 databuf[6] = {0}; 	
	s32 ret;
	u32 retry = 0;
	u8 drdy = 0;
	//qma7981_acc_format data_14bit;

	if(0)
	{
		while(!drdy)
		{
			qmaX981_readreg(0x0b, &drdy, 1);
			if(retry++ > 20)
				break;
		}
	}
	//ret = qmaX981_readreg(0x09, databuf, 1);
	//QMAX981_ERR("0x09 = 0x%x \n", databuf[0]);
	ret = qmaX981_readreg(QMAX981_XOUTL, databuf, 6);
	if(ret == 0){
		QMAX981_ERR("read xyz error!!!");
		return 0;	
	}

	data[0] = (short)((databuf[1]<<8)|(databuf[0]));
	data[1] = (short)((databuf[3]<<8)|(databuf[2]));
	data[2] = (short)((databuf[5]<<8)|(databuf[4]));
	data[0] = data[0]>>2;
	data[1] = data[1]>>2;
	data[2] = data[2]>>2;
#if 0
	ret = qmaX981_readreg(0x09, databuf, 3);
	if(ret == 0){
		QMAX981_ERR("read xyz error!!!");
		return 0;	
	}
	if(databuf[0])
	QMAX981_LOG("0x09 = 0x%x \n", databuf[0]);
#endif
	return 1;
}

s32 qmaX981_read_raw(s32 *rawData)
{
	s32 ret;

	if(g_qmaX981.chip_type == CHIP_TYPE_QMA6981)
		ret = qma6981_read_raw_xyz(rawData);
	else if((g_qmaX981.chip_type == CHIP_TYPE_QMA7981)||(g_qmaX981.chip_type == CHIP_TYPE_QMA7981_V2))
		ret = qma7981_read_raw_xyz(rawData);
	else
		ret = 0;

	return ret;
}

u8 qmaX981_read_acc(float *accData)
{
	s32 ret;
	s32 rawData[3];

	qmaX981_read_raw(rawData);
	accData[g_qmaX981.cvt.map[0]] = g_qmaX981.cvt.sign[0]*rawData[0];
	accData[g_qmaX981.cvt.map[1]] = g_qmaX981.cvt.sign[1]*rawData[1];
	accData[g_qmaX981.cvt.map[2]] = g_qmaX981.cvt.sign[2]*rawData[2];

	accData[0] = (accData[0]*9.807f)/(g_qmaX981.lsb_1g);			//GRAVITY_EARTH_1000
	accData[1] = (accData[1]*9.807f)/(g_qmaX981.lsb_1g);
	accData[2] = (accData[2]*9.807f)/(g_qmaX981.lsb_1g);

	return ret;

}

#if defined(QMAX981_STEPCOUNTER)
u32 qmaX981_read_stepcounter(void)
{
	u8 data[3];
	s32 ret;
	u32 step_num;

	if(g_qmaX981.chip_type == CHIP_TYPE_QMA6981)
	{
		ret = qmaX981_readreg(QMAX981_STEP_CNT_L, data, 2);
		step_num = (data[1]<<8)|data[0];
	}
	else if(g_qmaX981.chip_type == CHIP_TYPE_QMA7981)
	{	
		ret = qmaX981_readreg(QMAX981_STEP_CNT_L, data, 2);
		ret = qmaX981_readreg(QMA7981_STEP_CNT_M, &data[2], 1);
		step_num = (u32)(((u32)data[2]<<16)|((u32)data[1]<<8)|data[0]);
	}
#if defined(QMAX981_CHECK_ABNORMAL_DATA)
	ret=qmaX981_check_abnormal_data(step_num, &step_num);
	if(ret != 0)
	{
		return -1;
	}
#endif
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	step_num = qmaX981_step_debounce_read_data(step_num);
#endif

	return step_num;
}
#endif

u8 qmaX981_init(void)
{
	s32 ret = 0;
	u8 slave_addr[2] = {QMAX981_I2C_SLAVE_ADDR, QMAX981_I2C_SLAVE_ADDR2};
	u8 index=0;

	g_qmaX981.chip_type = CHIP_TYPE_UNDEFINE;
	for(index=0; index<2; index++)
	{
		QMAX981_I2C_ADDR = slave_addr[index]<<1;
		g_qmaX981.chip_id = qmaX981_chip_id();
		if((g_qmaX981.chip_id>=0xa9) && (g_qmaX981.chip_id<=0xb9))
		{
			QMAX981_LOG("qma6981 find \n");
			g_qmaX981.chip_type = CHIP_TYPE_QMA6981;
			break;
		}
		else if((g_qmaX981.chip_id>=0xe0) && (g_qmaX981.chip_id<=0xe7))
		{
			QMAX981_LOG("qma7981 find \n");
			g_qmaX981.chip_type = CHIP_TYPE_QMA7981;
			break;
		}
		else if((g_qmaX981.chip_id>=0xe8) && (g_qmaX981.chip_id<=0xe9))
		{		
			QMAX981_LOG("qma7981 v2 find \n");
			g_qmaX981.chip_type = CHIP_TYPE_QMA7981_V2;
			break;
		}
		else
		{		
			QMAX981_LOG("qma acc chip id not defined!!! \n");
			g_qmaX981.chip_type = CHIP_TYPE_UNDEFINE;
		}
	}

	if(g_qmaX981.chip_type == CHIP_TYPE_QMA6981)
		ret = qma6981_initialize();
	else if(g_qmaX981.chip_type == CHIP_TYPE_QMA7981)
		ret = qma7981_initialize();
	else if(g_qmaX981.chip_type == CHIP_TYPE_QMA7981_V2)
		ret = qma7981_v2_initialize();
	else
		ret = 0;
	
	//g_qmaX981.layout = 3;
	g_qmaX981.layout = 0;
	g_qmaX981.cvt.map[0] = qst_map[g_qmaX981.layout].map[0];
	g_qmaX981.cvt.map[1] = qst_map[g_qmaX981.layout].map[1];
	g_qmaX981.cvt.map[2] = qst_map[g_qmaX981.layout].map[2];
	g_qmaX981.cvt.sign[0] = qst_map[g_qmaX981.layout].sign[0];
	g_qmaX981.cvt.sign[1] = qst_map[g_qmaX981.layout].sign[1];
	g_qmaX981.cvt.sign[2] = qst_map[g_qmaX981.layout].sign[2];
#if defined(QMAX981_STEP_DEBOUNCE_IN_INT)
	qmaX981_step_debounce_reset();
#endif

#if 0//defined(QMAX981_USE_IRQ1)
	qmaX981_setup_irq1();
#endif
	return ret;
}

