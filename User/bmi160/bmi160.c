
#include "./usart/bsp_usart.h"
#include "./i2c/bsp_i2c.h"
#include "./i2c/qst_sw_i2c.h"

#include "./bmi160/bmi160.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

extern void qst_delay(unsigned int delay);
extern unsigned char bmi160_spi_write(unsigned char reg_addr, unsigned char reg_data);
unsigned char bmi160_spi_read(unsigned char reg_addr, unsigned char* buff, unsigned char len);

void bmi160_config_int(void);

typedef struct
{
	float x;
	float y;
	float z;
}qst_sensor_data;

#define QST_PRINTF		printf
#define CALI_DATA_NUM		80
#define GYRO_AVERAGE_NUM	5
qst_sensor_data qst_gyro_array[CALI_DATA_NUM];
static unsigned char bmi160_index=0;
static unsigned int bmi160_nm_count=0;

static unsigned long bmi601_timestamp = 0;
static unsigned long bmi601_timestamp_old = 0;

static short bmi160_motion_type = 0;
static unsigned char cali_done_flag = 0;
static unsigned char cali_flag = 0;
static unsigned short cali_count = 0;

static unsigned short acc_lsb_div = 0;
static unsigned short gyro_lsb_div = 0;
static short raw_acc_xyz[3];
static short raw_gyro_xyz[3];

static float offset_acc[3];
static float offset_gyro[3];
static float acc_xyz[3];
static float gyro_xyz[3];
static float rms_gyro[3];
float yaw, roll, pitch;

unsigned char read_reg(unsigned char reg_addr, unsigned char* buf, unsigned char len)
{
	return bmi160_spi_read(reg_addr, buf, len);
}

unsigned char write_reg(unsigned char reg_addr, unsigned char reg_data)
{
	unsigned char read_out=0;
	short count =0;

	while(read_out != reg_data)
	{
		bmi160_spi_write(reg_addr, reg_data);
		qst_delay(50);
		read_reg(reg_addr, &read_out, 1);
		qst_delay(50);
		QST_PRINTF("reg:0x%x   w:%x r:%x \n", reg_addr, reg_data, read_out);
		count++;
		if(count>10)
			break;
	}

	return;
}



unsigned char bmi160_init(void)
{
	unsigned char acc_range=0;
	unsigned char gyro_range=0;
	unsigned char pmu_status=0;
	unsigned char chip_id = 0;

	memset(offset_acc, 0, sizeof(offset_acc));
	memset(offset_gyro, 0, sizeof(offset_gyro));
	memset(rms_gyro, 0, sizeof(rms_gyro));
	cali_flag = 1;
	cali_done_flag = 0;
	cali_count = 0;

	read_reg(BMI160_CHIP_ID_ADDR, &chip_id, 1);
	QST_PRINTF("bmi160  chip_id=0x%x \n", chip_id);
	if(chip_id == BMI160_CHIP_ID)
	{	
		QST_PRINTF("bmi160 find!!!\n");

		write_reg(BMI160_COMMAND_REG_ADDR, BMI160_SOFT_RESET_CMD);
		qst_delay(BMI160_SOFT_RESET_DELAY_MS*100);
		write_reg(BMI160_ACCEL_CONFIG_ADDR, (BMI160_ACCEL_BW_NORMAL_AVG4<<4)|BMI160_ACCEL_ODR_100HZ);	// odr 100hz
		acc_range = BMI160_ACCEL_RANGE_8G;
		if(acc_range == BMI160_ACCEL_RANGE_2G)
			acc_lsb_div = (1<<14);
		else if(acc_range == BMI160_ACCEL_RANGE_4G)
			acc_lsb_div = (1<<13);
		else if(acc_range == BMI160_ACCEL_RANGE_8G)
			acc_lsb_div = (1<<12);
		else if(acc_range == BMI160_ACCEL_RANGE_16G)
			acc_lsb_div = (1<<11);
			
		write_reg(BMI160_ACCEL_RANGE_ADDR, acc_range);	// odr 100hz
		read_reg(BMI160_ACCEL_RANGE_ADDR,&acc_range,1);
		QST_PRINTF("bmi160 acc_range=%x!!!\n", acc_range);

		write_reg(BMI160_GYRO_CONFIG_ADDR, (BMI160_GYRO_BW_NORMAL_MODE<<4)|BMI160_GYRO_ODR_400HZ);
		gyro_range = BMI160_GYRO_RANGE_500_DPS;
		if(gyro_range == BMI160_GYRO_RANGE_125_DPS)
			gyro_lsb_div = BMI160_FS_125_LSB;
		else if(gyro_range == BMI160_GYRO_RANGE_250_DPS)
			gyro_lsb_div = BMI160_FS_250_LSB;
		else if(gyro_range == BMI160_GYRO_RANGE_500_DPS)
			gyro_lsb_div = BMI160_FS_500_LSB;
		else if(gyro_range == BMI160_GYRO_RANGE_1000_DPS)
			gyro_lsb_div = BMI160_FS_1000_LSB;
		else if(gyro_range == BMI160_GYRO_RANGE_2000_DPS)
			gyro_lsb_div = BMI160_FS_2000_LSB;
			
		write_reg(BMI160_GYRO_RANGE_ADDR, gyro_range);
		

		write_reg(BMI160_COMMAND_REG_ADDR, BMI160_ACCEL_NORMAL_MODE);
		qst_delay(5*100);
		write_reg(BMI160_COMMAND_REG_ADDR, BMI160_GYRO_NORMAL_MODE);
		qst_delay(BMI160_GYRO_DELAY_MS*100);
		// check pmu status
		while(pmu_status != 0x14)
		{
			read_reg(BMI160_PMU_STAT_ADDR, &pmu_status, 1);
			qst_delay(1*100);
		}
		// check pmu status
		bmi160_config_int();

		bmi601_timestamp = 0;
		bmi601_timestamp_old = 0;

	}

	return chip_id;
}

void bmi160_calc_ypr(void)
{
	unsigned char	buf_reg[3];
	unsigned int	read_time=0;
	float	time_gap;
	int		index;
	float	xyz_sum[3];

	read_reg(0x18, buf_reg, 3);
	read_time = ((unsigned int)buf_reg[2]<<16)|((unsigned int)buf_reg[1]<<8)|buf_reg[0];
	bmi601_timestamp = (unsigned long)(read_time*39);
	if(bmi601_timestamp_old > 0)
	{
		time_gap = (bmi601_timestamp - bmi601_timestamp_old)/1000000.00f;
		if(cali_done_flag)
		{
		// average fliter
			qst_gyro_array[bmi160_index].x = gyro_xyz[0];
			qst_gyro_array[bmi160_index].y = gyro_xyz[1];
			qst_gyro_array[bmi160_index].z = gyro_xyz[2];
			bmi160_index = (bmi160_index+1)%GYRO_AVERAGE_NUM;
			xyz_sum[0] = xyz_sum[1] = xyz_sum[2] = 0.0;
			for(index=0; index<GYRO_AVERAGE_NUM; index++)
			{
				xyz_sum[0]+=qst_gyro_array[index].x;
				xyz_sum[1]+=qst_gyro_array[index].y;
				xyz_sum[2]+=qst_gyro_array[index].z;
			}
			gyro_xyz[0] = xyz_sum[0]/GYRO_AVERAGE_NUM;
			gyro_xyz[1] = xyz_sum[1]/GYRO_AVERAGE_NUM;
			gyro_xyz[2] = xyz_sum[2]/GYRO_AVERAGE_NUM;
		// average fliter
			if((fabs(gyro_xyz[0])<fabs(rms_gyro[0]))
				&&(fabs(gyro_xyz[1])<fabs(rms_gyro[1]))
				&&(fabs(gyro_xyz[2])<fabs(rms_gyro[2])))
			{
				if(bmi160_nm_count > 200)
				{
					if(bmi160_motion_type !=2)
					{
						bmi160_motion_type = 2;		// no motion
					}
				}
				else
				{			
					bmi160_nm_count++;
				}
			}
			else
			{
				bmi160_nm_count = 0;
				bmi160_motion_type = 1;		// any motion
			}
			
			if(bmi160_motion_type == 2)
			{
				if(fabs(gyro_xyz[0])>fabs(rms_gyro[0]))
					pitch += gyro_xyz[0]*time_gap;
				if(fabs(gyro_xyz[1])>fabs(rms_gyro[1]))
					roll += gyro_xyz[1]*time_gap;
				if(fabs(gyro_xyz[2])>fabs(rms_gyro[2]))
					yaw += gyro_xyz[2]*time_gap;
			}
			else
			{
				pitch += gyro_xyz[0]*time_gap;
				roll += gyro_xyz[1]*time_gap;
				yaw += gyro_xyz[2]*time_gap;
			}
			QST_PRINTF("bmi160:time_gap:%f yaw:%f pitch:%f roll:%f \n", time_gap, yaw, pitch, roll);
		}
	}
	else
	{
		for(index=0; index<GYRO_AVERAGE_NUM; index++)
		{
			qst_gyro_array[index].x = gyro_xyz[0];
			qst_gyro_array[index].y = gyro_xyz[1];
			qst_gyro_array[index].z = gyro_xyz[2];
		}		
	}
	
	bmi601_timestamp_old = bmi601_timestamp;
}


void bmi160_read_acc_xyz(void)
{
	unsigned char	buf_reg[6];
//	float			acc_xyz_tmp[3];	

	read_reg(BMI160_ACCEL_DATA_ADDR, buf_reg, 6);

	raw_acc_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_acc_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_acc_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));
#if 0
	if(cali_flag == 0)
	{
		raw_acc_xyz[0]+=offset[0];
		raw_acc_xyz[1]+=offset[1];
		raw_acc_xyz[2]+=offset[2];
	}
	acc_xyz_tmp[0] = (raw_acc_xyz[0]*9.807f)/acc_lsb_div;
	acc_xyz_tmp[1] = (raw_acc_xyz[1]*9.807f)/acc_lsb_div;
	acc_xyz_tmp[2] = (raw_acc_xyz[2]*9.807f)/acc_lsb_div;
	acc_layout = 0;
	acc_xyz[qst_map[acc_layout].map[0]] = qst_map[acc_layout].sign[0]*acc_xyz_tmp[0];
	acc_xyz[qst_map[acc_layout].map[1]] = qst_map[acc_layout].sign[1]*acc_xyz_tmp[1];
	acc_xyz[qst_map[acc_layout].map[2]] = qst_map[acc_layout].sign[2]*acc_xyz_tmp[2];
#else
	acc_xyz[0] = (raw_acc_xyz[0]*9.807f)/acc_lsb_div;
	acc_xyz[1] = (raw_acc_xyz[1]*9.807f)/acc_lsb_div;
	acc_xyz[2] = (raw_acc_xyz[2]*9.807f)/acc_lsb_div;
	
	if(cali_flag == 0)
	{
		acc_xyz[0]+=offset_acc[0];
		acc_xyz[1]+=offset_acc[1];
		acc_xyz[2]+=offset_acc[2];
	}
	//QST_PRINTF("bmi160 acc{%f %f %f} \n", acc_xyz[0],acc_xyz[1],acc_xyz[2]);
#endif
}

void bmi160_read_gyro_xyz(void)
{
//	float			gyro_xyz_tmp[3];	
	unsigned char	buf_reg[6];

	read_reg(BMI160_GYRO_DATA_ADDR, buf_reg, 6);

	raw_gyro_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_gyro_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_gyro_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));	
#if 0
	gyro_xyz_tmp[0] = (raw_gyro_xyz[0]*10.0f)/gyro_lsb_div;
	gyro_xyz_tmp[1] = (raw_gyro_xyz[1]*10.0f)/gyro_lsb_div;
	gyro_xyz_tmp[2] = (raw_gyro_xyz[2]*10.0f)/gyro_lsb_div;
	acc_layout = 0;
	gyro_xyz[qst_map[acc_layout].map[0]] = qst_map[acc_layout].sign[0]*gyro_xyz_tmp[0];
	gyro_xyz[qst_map[acc_layout].map[1]] = qst_map[acc_layout].sign[1]*gyro_xyz_tmp[1];
	gyro_xyz[qst_map[acc_layout].map[2]] = qst_map[acc_layout].sign[2]*gyro_xyz_tmp[2];
#else
	gyro_xyz[0] = (raw_gyro_xyz[0]*10.0f)/gyro_lsb_div;
	gyro_xyz[1] = (raw_gyro_xyz[1]*10.0f)/gyro_lsb_div;
	gyro_xyz[2] = (raw_gyro_xyz[2]*10.0f)/gyro_lsb_div;
#endif
	if(cali_flag == 0)
	{
		gyro_xyz[0] += offset_gyro[0];
		gyro_xyz[1] += offset_gyro[1];
		gyro_xyz[2] += offset_gyro[2];
	}
	
	//QST_PRINTF("bmi160 gyro{%f %f %f} \n", gyro_xyz[0],gyro_xyz[1],gyro_xyz[2]);
	bmi160_calc_ypr();
}

void bmi160_read_sensor_time(void)
{
	unsigned char	buf_reg[3];
	unsigned int	read_time=0;

	read_reg(0x18, buf_reg, 3);
	read_time = ((unsigned int)buf_reg[2]<<16)|((unsigned int)buf_reg[1]<<8)|buf_reg[0];
	bmi601_timestamp = (unsigned long)(read_time*39.00);
	QST_PRINTF("bmi160 sensor_time=%ld \n", bmi601_timestamp);
}

//#define BMI160_DRDY_INT
//#define BMI160_ANY_MOTION_INT
//#define BMI160_NO_MOTION_INT

void bmi160_config_int(void)
{
	write_reg(BMI160_INT_OUT_CTRL_ADDR, 0xbb);	// int 1,2  edge, high, pp
	write_reg(BMI160_INT_LATCH_ADDR, 0x00);	// non-latched
#if defined(MASTER_STM32_INT)
	if(master_type==MASTER_STM32F103)
	{
		write_reg(BMI160_INT_LATCH_ADDR, 0x0b); // temporary, 320ms
	}
#endif
#if defined(BMI160_ANY_MOTION_INT)
	// any motion
	write_reg(BMI160_INT_ENABLE_0_ADDR, 0x07);	// any motion enable xyz
	write_reg(BMI160_INT_MAP_0_ADDR, 0x04);	// any motion map to int1
	write_reg(BMI160_INT_DATA_1_ADDR, 0x00);	//selects filtered (pre-filtered)
	//threshold of any-motion:3.91mg@2g, 7.81mg@4g, 15.63mg@8g, 31.25mg@16g
	write_reg(BMI160_INT_MOTION_1_ADDR, 0x02);
#endif
#if defined(BMI160_NO_MOTION_INT)
	// no/slow motion
	write_reg(BMI160_INT_ENABLE_2_ADDR, 0x07);	// no/slow motion enable xyz
	write_reg(BMI160_INT_MAP_0_ADDR, 0x0c);	// any/no motion map to int1	
	// slow/no motion duration
	write_reg(BMI160_INT_MOTION_0_ADDR, 0x0c);		// 0x1c
	//threshold of no/slow-motion:3.91mg@2g, 7.81mg@4g, 15.63mg@8g, 31.25mg@16g
	write_reg(BMI160_INT_MOTION_2_ADDR, 0x0b);
	//bit 0--> 1:select no-motion 0:select slow-motion
	write_reg(BMI160_INT_MOTION_3_ADDR, 0x01);
#endif
#if defined(BMI160_DRDY_INT)
	write_reg(BMI160_INT_ENABLE_1_ADDR, 0x10);	// drdy enable
	write_reg(BMI160_INT_MAP_1_ADDR, 0x80);	// any motion map to int1
#endif
}

void bmi160_set_cali(void)
{
	memset(offset_acc, 0, sizeof(offset_acc));
	memset(offset_gyro, 0, sizeof(offset_gyro));
	memset(rms_gyro, 0, sizeof(rms_gyro));
	cali_flag = 1;
	cali_done_flag = 0;
	cali_count = 0;
}

void bmi160_cali_process(void)
{
	if(cali_flag)
	{
// acc
		cali_count++;
		offset_acc[0] += acc_xyz[0];
		offset_acc[1] += acc_xyz[1];
		offset_acc[2] += acc_xyz[2];

		if(cali_count>=CALI_DATA_NUM)
		{
			offset_acc[0] = offset_acc[0]/CALI_DATA_NUM;
			offset_acc[1] = offset_acc[1]/CALI_DATA_NUM;
			offset_acc[2] = offset_acc[2]/CALI_DATA_NUM;
			offset_acc[0] = 0-offset_acc[0];
			offset_acc[1] = 0-offset_acc[1];
			offset_acc[2] = 9.807-offset_acc[2];
			QST_PRINTF("Cali result: offset%d %d %d \n", offset_acc[0],offset_acc[1],offset_acc[2]);
		}
// acc

// gyro	
		offset_gyro[0] += gyro_xyz[0];
		offset_gyro[1] += gyro_xyz[1];
		offset_gyro[2] += gyro_xyz[2];
		// save data
		qst_gyro_array[cali_count-1].x = gyro_xyz[0];
		qst_gyro_array[cali_count-1].y = gyro_xyz[1];
		qst_gyro_array[cali_count-1].z = gyro_xyz[2];
		// save data
		if(cali_count>=CALI_DATA_NUM)
		{				
			int index;
			float average_gyro[3];
	
			average_gyro[0] = offset_gyro[0]/CALI_DATA_NUM;
			average_gyro[1] = offset_gyro[1]/CALI_DATA_NUM;
			average_gyro[2] = offset_gyro[2]/CALI_DATA_NUM;
			offset_gyro[0] = 0.0-average_gyro[0];
			offset_gyro[1] = 0.0-average_gyro[1];
			offset_gyro[2] = 0.0-average_gyro[2];
			QST_PRINTF("Cali result: offset_gyro:%f %f %f \n", offset_gyro[0],offset_gyro[1],offset_gyro[2]);
			rms_gyro[0] = 0.0;
			rms_gyro[1] = 0.0;
			rms_gyro[2] = 0.0;
			for(index=0; index<CALI_DATA_NUM; index++)
			{
				rms_gyro[0] += (qst_gyro_array [index].x-average_gyro[0])*(qst_gyro_array [index].x-average_gyro[0]);
				rms_gyro[1] += (qst_gyro_array [index].y-average_gyro[1])*(qst_gyro_array [index].y-average_gyro[1]);
				rms_gyro[2] += (qst_gyro_array [index].z-average_gyro[2])*(qst_gyro_array [index].z-average_gyro[2]);
			}
			rms_gyro[0] = (sqrt(rms_gyro[0]/CALI_DATA_NUM))*2;	// 3*
			rms_gyro[1] = (sqrt(rms_gyro[1]/CALI_DATA_NUM))*2;
			rms_gyro[2] = (sqrt(rms_gyro[2]/CALI_DATA_NUM))*2;				
			QST_PRINTF("Cali result: rms_gyro:%f %f %f \n", rms_gyro[0],rms_gyro[1],rms_gyro[2]);
		}
// gyro
		if(cali_count >= CALI_DATA_NUM)
		{
			cali_flag = 0;
			cali_done_flag = 1;
			cali_count = 0;
		}
	}
}

