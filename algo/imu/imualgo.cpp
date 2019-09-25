
#include <stdio.h>
#include <string>
#include <math.h>
#include "FastMath.h"
#include "imualgo.h"

#if defined(BUILD_ON_ANDROID)
#include <jni.h>
#include<android/log.h>
#define TAG "ImuAlgo"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG,TAG ,__VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO,TAG ,__VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN,TAG ,__VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR,TAG ,__VA_ARGS__)
#define LOGF(...) __android_log_print(ANDROID_LOG_FATAL,TAG ,__VA_ARGS__)
#else
#define LOGD 	printf
#define LOGI	printf
#define LOGW	printf
#define LOGE	printf
#define LOGF	printf
#endif

#define GYRO_LOG			printf
#define LPF_ALPHA           0.8f
#define MAX_CALI_COUNT      50
#define QST_FABS(v)			((v<0)?(-1.0f*v):(v))	
#define QST_ABS(v)			((v<0)?(-1*v):(v))	
#define EKF_HALFPI			1.5707963267948966192313216916398f
#define EKF_PI				3.1415926535897932384626433832795f
#define EKF_TWOPI			6.283185307179586476925286766559f
#define EKF_TODEG(x)		((x) * 57.2957796f)
#define ANDROID_CoordSystem

static qfloat gravity[3];
static qint  cali_count = 0;
//static qchar cali_done_flag = 0;


static qfloat offset_acc[3] = {0.0, 0.0, 0.0};
static qfloat offset_gyro[3] = {0.0, 0.0, 0.0};
static quaternion_t offset_dq;

static sensor_data_t accel_data[MAX_CALI_COUNT];
static sensor_data_t gyro_data[MAX_CALI_COUNT];
static quaternion_t dq_data[MAX_CALI_COUNT];

static qfloat gyro_x_thres = 0.0f;
static qfloat gyro_y_thres = 0.0f;
static qfloat gyro_z_thres = 0.0f;
static quaternion_t rms_dq;


static qst_imu_data_t qstimu;
static quaternion_t qstdq;

qvoid qmemset(qvoid *src, qchar c , qint len)
{
	char *tmpsrc=(char*)src;
	while(len--)
	{
		*tmpsrc++ = c;
	}
}

qvoid qst_algo_imu_get_offset(qfloat bias_acc[3], qfloat bias_gyro[3], qfloat rms_gyro[3])
{
	if(bias_acc)
	{
	    bias_acc[0] = offset_acc[0];
	    bias_acc[1] = offset_acc[1];
	    bias_acc[2] = offset_acc[2];
	}
	if(bias_gyro)
	{
	    bias_gyro[0] = offset_gyro[0];
	    bias_gyro[1] = offset_gyro[1];
	    bias_gyro[2] = offset_gyro[2];
	}
	if(rms_gyro)
	{
	    rms_gyro[0] = gyro_x_thres;
	    rms_gyro[1] = gyro_y_thres;
	    rms_gyro[2] = gyro_z_thres;
	}
}

qvoid qst_algo_imu_set_offset(qfloat bias_acc[3], qfloat bias_gyro[3], qfloat rms_gyro[3])
{
	if(bias_acc)
	{
	    offset_acc[0] = bias_acc[0];
	    offset_acc[1] = bias_acc[0];
	    offset_acc[2] = bias_acc[0];
	}
	if(bias_gyro)
	{
	    offset_gyro[0] = bias_gyro[0];
	    offset_gyro[1] = bias_gyro[1];
	    offset_gyro[2] = bias_gyro[2];
	}
	if(rms_gyro)
	{
	    gyro_x_thres = rms_gyro[0];
	    gyro_y_thres = rms_gyro[1];
	    gyro_z_thres = rms_gyro[2];
	}
}

qint qst_algo_imu_cali(qfloat acc[3],qfloat gyro[3])
{
	LOGD("qst_algo_imu_cali: acc:%f %f %f gyro:%f %f %f\n", acc[0], acc[1], acc[2], gyro[0],gyro[1],gyro[2]);
    if(cali_count == 0)
    {
        qmemset((qvoid *)offset_acc, 0, sizeof(offset_acc));
        qmemset((qvoid *)offset_gyro, 0, sizeof(offset_gyro));
        cali_count++;
    }
    else if(cali_count < MAX_CALI_COUNT)
    {
        accel_data[cali_count].x =  acc[0];
        accel_data[cali_count].y =  acc[1];
        accel_data[cali_count].z =  acc[2];
        gyro_data[cali_count].x =  gyro[0];
        gyro_data[cali_count].y =  gyro[1];
        gyro_data[cali_count].z =  gyro[2];

        offset_acc[0] += acc[0];
        offset_acc[1] += acc[1];
        offset_acc[2] += acc[2];
        offset_gyro[0] += gyro[0];
        offset_gyro[1] += gyro[1];
        offset_gyro[2] += gyro[2];

        cali_count++;
    }
    else if(cali_count == MAX_CALI_COUNT)
    {
        qint i = 0;
		qfloat avg_acc[3] = {0.0, 0.0, 0.0};
		qfloat avg_gyro[3] = {0.0, 0.0, 0.0};
		qfloat rms_acc[3] = {0.0, 0.0, 0.0};
		qfloat rms_gyro[3] = {0.0, 0.0, 0.0};

        avg_acc[0] = offset_acc[0] / (MAX_CALI_COUNT-1);
        avg_acc[1] = offset_acc[1] / (MAX_CALI_COUNT-1);
        avg_acc[2] = offset_acc[2] / (MAX_CALI_COUNT-1);

        avg_gyro[0] = offset_gyro[0] / (MAX_CALI_COUNT-1);
        avg_gyro[1] = offset_gyro[1] / (MAX_CALI_COUNT-1);
        avg_gyro[2] = offset_gyro[2] / (MAX_CALI_COUNT-1);

        offset_acc[0] = (0.0f-avg_acc[0]);
        offset_acc[1] = (0.0f-avg_acc[1]);
        offset_acc[2] = (9.807f-avg_acc[2]);

        offset_gyro[0] = (0.0f-avg_gyro[0]);
        offset_gyro[1] = (0.0f-avg_gyro[1]);
        offset_gyro[2] = (0.0f-avg_gyro[2]);
		LOGD("cali offset: acc:%f %f %f gyro:%f %f %f\n", offset_acc[0], offset_acc[1], offset_acc[2], offset_gyro[0],offset_gyro[1],offset_gyro[2]);

		rms_acc[0] = rms_acc[1] = rms_acc[2] = 0.0f;
		rms_gyro[0] = rms_gyro[1] = rms_gyro[2] = 0.0f;
        for(i = 1; i<MAX_CALI_COUNT; i++)
        {
            rms_acc[0] += (accel_data[i].x - avg_acc[0])*(accel_data[i].x - avg_acc[0]);
            rms_acc[1] += (accel_data[i].y - avg_acc[1])*(accel_data[i].y - avg_acc[1]);
            rms_acc[2] += (accel_data[i].z - avg_acc[2])*(accel_data[i].z - avg_acc[2]);

            rms_gyro[0] += (gyro_data[i].x - avg_gyro[0])*(gyro_data[i].x - avg_gyro[0]);
            rms_gyro[1] += (gyro_data[i].y - avg_gyro[1])*(gyro_data[i].y - avg_gyro[1]);
            rms_gyro[2] += (gyro_data[i].z - avg_gyro[2])*(gyro_data[i].z - avg_gyro[2]);
        }
        rms_acc[0] = sqrtf(rms_acc[0]/(MAX_CALI_COUNT-1));
        rms_acc[1] = sqrtf(rms_acc[1]/(MAX_CALI_COUNT-1));
        rms_acc[2] = sqrtf(rms_acc[2]/(MAX_CALI_COUNT-1));
        rms_gyro[0] = sqrtf(rms_gyro[0]/(MAX_CALI_COUNT-1));
        rms_gyro[1] = sqrtf(rms_gyro[1]/(MAX_CALI_COUNT-1));
        rms_gyro[2] = sqrtf(rms_gyro[2]/(MAX_CALI_COUNT-1));
		LOGD("cali rms: acc:%f %f %f gyro:%f %f %f\n", rms_acc[0], rms_acc[1], rms_acc[2], rms_gyro[0],rms_gyro[1],rms_gyro[2]);

        gyro_x_thres = 6*rms_gyro[0];
        gyro_y_thres = 6*rms_gyro[1];
        gyro_z_thres = 6*rms_gyro[2];

        cali_count = 0;
        return 1;
    }

    return 0;
}


#if 0
float accel_R[3] = { 0 };  
float accel_Q[3] = { 0 };
float accel_mid[3] = {0};
float accel_last[3] = {0,0,9.8f};
float accel_now[3]= {0};
float accel_p_mid[3] = {0} ;
float accel_p_last[3] = {0};
float accel_p_now[3] = {0};
float accel_kg[3] = {0};


float gyro_R[3] = { 0 };  
float gyro_Q[3] = { 0 };
float gyro_mid[3] = {0};
float gyro_last[3] = {0};
float gyro_now[3]= {0};
float gyro_p_mid[3] = {0} ;
float gyro_p_last[3] = {0};
float gyro_p_now[3] = {0};
float gyro_kg[3] = {0};

void EFK_filter(float *data, int type){
	
	if(0 == type)//accel
	{
		int i = 0;
		for(i = 0; i < 3; i++)
		{
			accel_mid[i] = accel_last[i]; //x_last=x(k-1|k-1),x_mid=x(k|k-1) 
			accel_p_mid[i] = accel_p_last[i] + accel_Q[i]; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=??
			accel_kg[i] = accel_p_mid[i] / (accel_p_mid[i] + accel_R[i]); //kg?kalman filter,R???
			accel_now[i] = accel_mid[i] + accel_kg[i] * (data[i] - accel_mid[i]); 

			accel_p_now[i] = (1 - accel_kg[i] ) * accel_p_mid[i];       

			accel_p_last[i] = accel_p_now[i]; 
			accel_last[i] = accel_now[i]; 
			//qst_pf("accel_p_last = %f %f %f\r\n",accel_p_last[0],accel_p_last[1],accel_p_last[2]);
		}
	}
	else if(1 == type)//gyro
	{
		int j;
		for(j = 0; j < 3; j++)
		{
			gyro_mid[j] = gyro_last[j]; //x_last=x(k-1|k-1),x_mid=x(k|k-1) 
			gyro_p_mid[j] = gyro_p_last[j] + gyro_Q[j]; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=??
			gyro_kg[j] = gyro_p_mid[j] / (gyro_p_mid[j] + gyro_R[j]); //kg?kalman filter,R???
			gyro_now[j] = gyro_mid[j] + gyro_kg[j] * (data[j] - gyro_mid[j]); //???????

			gyro_p_now[j] = (1 - gyro_kg[j] ) * gyro_p_mid[j];//??????covariance      

			gyro_p_last[j] = gyro_p_now[j]; //??covariance?
			gyro_last[j] = gyro_now[j]; //??????? 			
		} 
	}
}
#endif

qvoid qst_algo_linear_acc(qfloat acc[3],qfloat linear_acc[3]) 
{
	static qchar linear_acc_init = 0;

	if(linear_acc_init)
	{
		gravity[0] = gravity[0]*LPF_ALPHA + acc[0]*(1-LPF_ALPHA);
		gravity[1] = gravity[1]*LPF_ALPHA + acc[1]*(1-LPF_ALPHA);
		gravity[2] = gravity[2]*LPF_ALPHA + acc[2]*(1-LPF_ALPHA);
	}
	else
	{
		gravity[0] = acc[0];
		gravity[1] = acc[1];
		gravity[2] = acc[2];
		linear_acc_init = 1;
	}

	linear_acc[0] = acc[0]-gravity[0];
	linear_acc[1] = acc[1]-gravity[1];
	linear_acc[2] = acc[2]-gravity[2];
}

qvoid qst_algo_gravity_acc(qfloat acc[3]) 
{
	static qchar gravity_acc_init = 0;

	if(gravity_acc_init)
	{
		gravity[0] = gravity[0]*LPF_ALPHA + acc[0]*(1-LPF_ALPHA);
		gravity[1] = gravity[1]*LPF_ALPHA + acc[1]*(1-LPF_ALPHA);
		gravity[2] = gravity[2]*LPF_ALPHA + acc[2]*(1-LPF_ALPHA);
	}
	else
	{
		gravity[0] = acc[0];
		gravity[1] = acc[1];
		gravity[2] = acc[2];
		gravity_acc_init = 1;
	}
	acc[0] = gravity[0];
	acc[1] = gravity[1];
	acc[2] = gravity[2];
}

qint qst_algo_imu_data_process(qfloat acc[3],qfloat gyro[3])
{
	//static qchar gravity_init = 0;

// add offset
	acc[0] = acc[0]+offset_acc[0];
	acc[1] = acc[1]+offset_acc[1];
	acc[2] = acc[2]+offset_acc[2];
	gyro[0] = gyro[0]+offset_gyro[0];
	gyro[1] = gyro[1]+offset_gyro[1];
	gyro[2] = gyro[2]+offset_gyro[2];
// add offset	
// convert axis
#if 0
#endif
// convert axis
// data filter
	//EFK_filter(acc, 0);
	//EFK_filter(gyro, 1);
	qst_algo_gravity_acc(acc);
// data filter
	if((gyro[0]>-gyro_x_thres) && (gyro[0]<gyro_x_thres))
	{
		gyro[0] = 0.0f;
	}
	if((gyro[1]>-gyro_y_thres) && (gyro[1]<gyro_y_thres))
	{
		gyro[1] = 0.0f;
	}
	if((gyro[2]>-gyro_z_thres) && (gyro[2]<gyro_z_thres))
	{
		gyro[2] = 0.0f;
	}
	return 1;
}


qvoid qst_algo_imu_init(qfloat accel[3], qfloat gyro[3])
{
	//NED coordinate system unit vector
	qfloat t_acc[3];
	qfloat nedVector[3] = {0, 0 , 1.0f};
	qfloat accelVector[3] = {0, 0 , 0};
	qfloat norm;
	qfloat crossVector[3];
	qfloat sinwi, cosw, sinhalfw, coshalfw;

	qst_algo_imu_data_process(accel, gyro);
	t_acc[0] = accel[0];
	t_acc[1] = accel[1];
	t_acc[2] = accel[2];
	//unit accel
	norm = FastSqrtI(t_acc[0] * t_acc[0] + t_acc[1] * t_acc[1] + t_acc[2] * t_acc[2]);
	accelVector[0] = t_acc[0] * norm;
	accelVector[1] = t_acc[1] * norm;
	accelVector[2] = t_acc[2] * norm;
	
	//cross product between accel and reference
	crossVector[0] = accelVector[1] * nedVector[2] - accelVector[2] * nedVector[1];
	crossVector[1] = accelVector[2] * nedVector[0] - accelVector[0] * nedVector[2];
	crossVector[2] = accelVector[0] * nedVector[1] - accelVector[1] * nedVector[0];
	sinwi = FastSqrtI(crossVector[0] * crossVector[0] + crossVector[1] * crossVector[1] + crossVector[2] * crossVector[2]);
	crossVector[0] *= sinwi;
	crossVector[1] *= sinwi;
	crossVector[2] *= sinwi;

	//the angle between accel and reference is the dot product of the two vectors
	cosw = accelVector[0] * nedVector[0] + accelVector[1] * nedVector[1] + accelVector[2] * nedVector[2];
	coshalfw = FastSqrt((1.0f + cosw) * 0.5f);
	sinhalfw = FastSqrt((1.0f - cosw) * 0.5f);
	
	qstimu.X[0] = coshalfw;
	qstimu.X[1] = crossVector[0] * sinhalfw;
	qstimu.X[2] = crossVector[1] * sinhalfw;
	qstimu.X[3] = crossVector[2] * sinhalfw;

	norm = FastSqrtI(qstimu.X[0] * qstimu.X[0] + qstimu.X[1] * qstimu.X[1] + qstimu.X[2] * qstimu.X[2] + qstimu.X[3] * qstimu.X[3]);
	
	qstimu.X[0]  *= norm;
	qstimu.X[1]  *= norm;
	qstimu.X[2]  *= norm;
	qstimu.X[3]  *= norm;
}


#define Kp 2.0f
#define Ki 0.005f
qfloat exInt = 0, eyInt = 0, ezInt = 0;
static quchar gyro_stable_count = 0;
static quchar anti_gyro_stable_count = 0;
qvoid qst_algo_imu_update(qfloat accel[3], qfloat gyro[3], qfloat dt)
{
	qfloat t_acc[3];
	qfloat norm;
	qfloat vx, vy, vz;
	qfloat ex, ey, ez;
	qfloat q0 = 1, q1 = 0, q2 = 0, q3 = 0;

	qst_algo_imu_data_process(accel, gyro);

	if(gyro[0] == 0 && gyro[1] == 0 && gyro[2]== 0){
		gyro_stable_count ++;
		if(gyro_stable_count >= 100){	
			anti_gyro_stable_count = 0;
			return;
		}
	}else{
		anti_gyro_stable_count ++;
		if(anti_gyro_stable_count >= 50)
			gyro_stable_count = 0;
	}

	q0 = qstimu.X[0];
	q1 = qstimu.X[1];
	q2 = qstimu.X[2];
	q3 = qstimu.X[3];
	
	t_acc[0] = accel[0];
	t_acc[1] = accel[1];
	t_acc[2] = accel[2];
	norm = FastSqrtI(t_acc[0]*t_acc[0] + t_acc[1]*t_acc[1] + t_acc[2]*t_acc[2]);
	t_acc[0] = t_acc[0] * norm;
	t_acc[1] = t_acc[1] * norm;
	t_acc[2] = t_acc[2] * norm;
	
	vx = 2*(q1*q3-q0*q2);
	vy = 2*(q0*q1+q2*q3);
	vz = q0*q0+ q3*q3 - q1*q1 - q2*q2;
	
	ex = (t_acc[1]*vz - t_acc[2]*vy);
	ey = (t_acc[2]*vx - t_acc[0]*vz);
	ez = (t_acc[0]*vy - t_acc[1]*vx);
	
	exInt += ex*Ki*dt*0.5f;
	eyInt += ey*Ki*dt*0.5f;
	ezInt += ez*Ki*dt*0.5f;
	
	gyro[0] = gyro[0] + Kp*ex + exInt;
	gyro[1] = gyro[1] + Kp*ey + eyInt;
	gyro[2] = gyro[2] + Kp*ez + ezInt;
		
	q0 += (-q1*gyro[0]-q2*gyro[1] -q3*gyro[2])*dt*0.5f;
	q1 += (q0*gyro[0] + q2*gyro[2]-q3*gyro[1])*dt*0.5f;
	q2 += (q0*gyro[1]-q1*gyro[2] + q3*gyro[0])*dt*0.5f;
	q3 += (q0*gyro[2] + q1*gyro[1] -q2*gyro[0])*dt*0.5f;
	
	norm = FastSqrtI(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	qstimu.X[0] = q0 * norm;
	qstimu.X[1] = q1 * norm;
	qstimu.X[2] = q2 * norm;
	qstimu.X[3] = q3 * norm;
}

/*
volatile qfloat beta = 0.05f;
void qst_algo_imu_update2(qfloat *gyro, qfloat *accel, qfloat dt)
{
	qfloat recipNorm;
	qfloat s0, s1, s2, s3;
	qfloat qDot1, qDot2, qDot3, qDot4;
	qfloat _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	qfloat q0 = 1, q1 = 0, q2 = 0, q3 = 0;

	qst_algo_imu_data_process(accel, gyro);
	if(gyro[0] == 0 && gyro[1] == 0 && gyro[2]== 0){
		gyro_stable_count ++;
		if(gyro_stable_count >= 100){	
			anti_gyro_stable_count = 0;
			return;
		}
	}else{
		anti_gyro_stable_count ++;
		if(anti_gyro_stable_count >= 50)
			gyro_stable_count = 0;
	}
	q0 = qstimu.X[0];
	q1 = qstimu.X[1];
	q2 = qstimu.X[2];
	q3 = qstimu.X[3];
	
	qDot1 = 0.5f * (-q1 * gyro[0] - q2 * gyro[1] - q3 * gyro[2]);
	qDot2 = 0.5f * (q0 * gyro[0] + q2 * gyro[2] - q3 * gyro[1]);
	qDot3 = 0.5f * (q0 * gyro[1] - q1 * gyro[2] + q3 * gyro[0]);
	qDot4 = 0.5f * (q0 * gyro[2] + q1 * gyro[1] - q2 * gyro[0]);
	
	recipNorm = FastSqrtI(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
	accel[0] *= recipNorm;
	accel[1] *= recipNorm;
	accel[2] *= recipNorm;
	
	_2q0 = 2.0f * q0;
	_2q1 = 2.0f * q1;
	_2q2 = 2.0f * q2;
	_2q3 = 2.0f * q3;
	_4q0 = 4.0f * q0;
	_4q1 = 4.0f * q1;
	_4q2 = 4.0f * q2;
	_8q1 = 8.0f * q1;
	_8q2 = 8.0f * q2;
	q0q0 = q0 * q0;
	q1q1 = q1 * q1;
	q2q2 = q2 * q2;
	q3q3 = q3 * q3;

	s0 = _4q0 * q2q2 + _2q2 * accel[0] + _4q0 * q1q1 - _2q1 * accel[1];
	s1 = _4q1 * q3q3 - _2q3 * accel[0] + 4.0f * q0q0 * q1 - _2q0 * accel[1] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * accel[2];
	s2 = 4.0f * q0q0 * q2 + _2q0 * accel[0] + _4q2 * q3q3 - _2q3 * accel[1] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * accel[2];
	s3 = 4.0f * q1q1 * q3 - _2q1 * accel[0] + 4.0f * q2q2 * q3 - _2q2 * accel[1];
	
	recipNorm = FastSqrtI(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
	s0 *= recipNorm;
	s1 *= recipNorm;
	s2 *= recipNorm;
	s3 *= recipNorm;
	
	qDot1 -= beta * s0;
	qDot2 -= beta * s1;
	qDot3 -= beta * s2;
	qDot4 -= beta * s3;
	
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;
	
	recipNorm = FastSqrtI(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	qstimu.X[0] = q0 * recipNorm;
	qstimu.X[1] = q1 * recipNorm;
	qstimu.X[2] = q2 * recipNorm;
	qstimu.X[3] = q3 * recipNorm;			
}
*/

qvoid qst_algo_imu_angle(qfloat* rpy)
{
	qfloat CBn[9];

	CBn[0] = 2.0f * (qstimu.X[0] * qstimu.X[0] + qstimu.X[1]  * qstimu.X[1] ) - 1.0f;
	CBn[1] = 2.0f * (qstimu.X[1]  * qstimu.X[2]  + qstimu.X[0] * qstimu.X[3] );
	CBn[2] = 2.0f * (qstimu.X[1]  * qstimu.X[3]  - qstimu.X[0]  * qstimu.X[2]);
	CBn[3] = 2.0f * (qstimu.X[1] * qstimu.X[2] - qstimu.X[0] * qstimu.X[3]);
	CBn[4] = 2.0f * (qstimu.X[0] * qstimu.X[0] + qstimu.X[2] * qstimu.X[2]) - 1.0f;
	CBn[5] = 2.0f * (qstimu.X[2] * qstimu.X[3] + qstimu.X[0] * qstimu.X[1]);
//  CBn[6] = 2.0f * (qstimu.X[1] * qstimu.X[3] + qstimu.X[0] * qstimu.X[2]);
//	CBn[7] = 2.0f * (qstimu.X[2] * qstimu.X[3] - qstimu.X[0] * qstimu.X[1]);
	CBn[8] = 2.0f * (qstimu.X[0] * qstimu.X[0] + qstimu.X[3] * qstimu.X[3]) - 1.0f;

#ifdef ANDROID_CoordSystem
	//calculate the roll angle -90.0 <= Phi <= 90.0 deg
	if(CBn[2] >= 1.0f)
		rpy[0] = EKF_HALFPI;
	else if(CBn[2] <= -1.0f)
		rpy[0] = -EKF_HALFPI;
	else
		rpy[0] = FastAsin(CBn[2]);
	
	// calculate the pitch angle -180.0 <= The < 180.0 deg
	//rpy[1] = FastAtan2(-CBn[5],CBn[8]);
	rpy[1] = FastAtan2(CBn[5],CBn[8]);
	if(rpy[1] == EKF_PI)
		rpy[1] = -EKF_PI;
	
	//calculate the yaw (compass) angle 0.0 <= Psi < 360.0 deg
	if (rpy[0] == EKF_HALFPI)
	{
		// vertical downwards gimbal lock case
		rpy[2] = FastAtan2(CBn[3], CBn[4]) - rpy[1];
	}
	else if (rpy[0] == -EKF_HALFPI)
	{
		// vertical upwards gimbal lock case
		rpy[2] = FastAtan2(CBn[3], CBn[4]) + rpy[1];
	}
	else
	{
		// // general case
		rpy[2] = FastAtan2(-CBn[1], CBn[0]);
	}

	if(rpy[2] < 0.0f)
		rpy[2] += EKF_TWOPI;
	if(rpy[2] >= EKF_TWOPI)
		rpy[2] = 0.0f;
	
#endif
	rpy[0] = EKF_TODEG(rpy[0]);
	rpy[1] = EKF_TODEG(rpy[1]);
	rpy[2] = EKF_TODEG(rpy[2]);	
	
	qstimu.YRP[0] = (qshort)rpy[0];
	qstimu.YRP[1] = (qshort)rpy[1];
	qstimu.YRP[2] = (qshort)rpy[2];	
}


qvoid qst_algo_imu_angle2(qfloat* rpy)
{
	qfloat q00,q01,q02,q03,q11,q12,q13,q22,q23,q33;
	euler_martix result;

	q00=qstimu.X[0]*qstimu.X[0];
	q01=qstimu.X[0]*qstimu.X[1];
	q02=qstimu.X[0]*qstimu.X[2];
	q03=qstimu.X[0]*qstimu.X[3];
	q11=qstimu.X[1]*qstimu.X[1];
	q12=qstimu.X[1]*qstimu.X[2];
	q13=qstimu.X[1]*qstimu.X[3];
	q22=qstimu.X[2]*qstimu.X[2];
	q23=qstimu.X[2]*qstimu.X[3];
	q33=qstimu.X[3]*qstimu.X[3];
	result.T11=q00+q11-q22-q33;
	result.T12=2*(q12+q03);
	result.T13=2*(q13-q02);
	result.T21=2*(q12-q03);
	result.T22=q22-q33+q00-q11;
	result.T23=2*(q23+q01);
	result.T31=2*(q13+q02);
	result.T32=2*(q23-q01);
	result.T33=q33-q22-q11+q00;

	if(result.T13 >= 1.0)
        rpy[0] = -90.0;
	else if(result.T13 <= -1.0)
        rpy[0] = 90.0;
	else
	    rpy[0] = -EKF_TODEG(asin(result.T13));
	rpy[1] = -EKF_TODEG(atan2(result.T23, result.T33));
	rpy[2] = EKF_TODEG(atan2(result.T12, result.T11));
}


#if defined(USE_ATTITUDE_ENGINE)
qint qst_algo_dq_cali(qfloat dq[4])
{
    if(cali_count == 0)
    {
		offset_dq.q0 = 0.0f;
		offset_dq.q1 = 0.0f;
		offset_dq.q2 = 0.0f;
		offset_dq.q3 = 0.0f;
        cali_count++;
    }
    else if(cali_count < MAX_CALI_COUNT)
    {
        dq_data[cali_count].q0 =  dq[0];
        dq_data[cali_count].q1 =  dq[1];
        dq_data[cali_count].q2 =  dq[2];
        dq_data[cali_count].q3 =  dq[3];

		offset_dq.q0 += dq[0];
		offset_dq.q1 += dq[1];
		offset_dq.q2 += dq[2];
		offset_dq.q3 += dq[3];

        cali_count++;
    }
    else if(cali_count == MAX_CALI_COUNT)
    {
		qint i=0;
		quaternion_t avg_dq;
		
        avg_dq.q0 = offset_dq.q0 / (MAX_CALI_COUNT-1);
        avg_dq.q1 = offset_dq.q1 / (MAX_CALI_COUNT-1);
        avg_dq.q2 = offset_dq.q2 / (MAX_CALI_COUNT-1);
        avg_dq.q3 = offset_dq.q3 / (MAX_CALI_COUNT-1);

        offset_dq.q0 = 0.0f;	//(1.0f-avg_dq.q0);
        offset_dq.q1 = (0.0f-avg_dq.q1);
        offset_dq.q2 = (0.0f-avg_dq.q2);
        offset_dq.q3 = (0.0f-avg_dq.q3);

        LOGD("cali offset: dq:%f %f %f %f\n", offset_dq.q0,offset_dq.q1,offset_dq.q2,offset_dq.q3);
		rms_dq.q0 = 0.0f;
		rms_dq.q1 = 0.0f;
		rms_dq.q2 = 0.0f;
		rms_dq.q3 = 0.0f;
        for(i = 1; i<MAX_CALI_COUNT; i++)
        {
            rms_dq.q0 += (dq_data[i].q0 - avg_dq.q0)*(dq_data[i].q0 - avg_dq.q0);
            rms_dq.q1 += (dq_data[i].q1 - avg_dq.q1)*(dq_data[i].q1 - avg_dq.q1);
            rms_dq.q2 += (dq_data[i].q2 - avg_dq.q2)*(dq_data[i].q2 - avg_dq.q2);
            rms_dq.q0 += (dq_data[i].q3 - avg_dq.q3)*(dq_data[i].q3 - avg_dq.q3);
        }
        rms_dq.q0 = 0;	//sqrtf(rms_dq.q0/(MAX_CALI_COUNT-1));
        rms_dq.q1 = sqrtf(rms_dq.q1/(MAX_CALI_COUNT-1));
        rms_dq.q2 = sqrtf(rms_dq.q2/(MAX_CALI_COUNT-1));
        rms_dq.q3 = sqrtf(rms_dq.q3/(MAX_CALI_COUNT-1));

        LOGD("cali rms: dp:%f %f %f %f\n", rms_dq.q0,rms_dq.q1,rms_dq.q2,rms_dq.q3);
		rms_dq.q0 = rms_dq.q0*6;
		rms_dq.q1 = rms_dq.q1*6;
		rms_dq.q2 = rms_dq.q2*6;
		rms_dq.q3 = rms_dq.q3*6;

        cali_count = 0;
        return 1;
    }

    return 0;
}


qvoid qst_algo_dq_init(qvoid)
{
	qstdq.q0 = 1.0f;
	qstdq.q1 = 0.0f;
	qstdq.q2 = 0.0f;
	qstdq.q3 = 0.0f;
}

static float q0q0, q0q1, q0q2, q0q3;
static float q1q1, q1q2, q1q3;
static float q2q2, q2q3;
static float q3q3;
static float dcm[9] = {1.f,0.0f,0.0f,0.0f,1.f,0.0f,0.0f,0.0f,1.f };
qvoid qst_algo_dq_update(qfloat dq[4], qfloat rpy[3])
{
	static qchar algo_dq_init = 0;
	qfloat q_k[4];
	qfloat norm;

	dq[0] = dq[0] + offset_dq.q0;
	if(dq[1] != 0.0f)
		dq[1] = dq[1] + offset_dq.q1;
	if(dq[2] != 0.0f)
		dq[2] = dq[2] + offset_dq.q2;
	if(dq[3] != 0.0f)
		dq[3] = dq[3] + offset_dq.q3;
	
	if(QST_FABS(dq[1]) < rms_dq.q1)
		dq[1] = 0.0f;
	if(QST_FABS(dq[2]) < rms_dq.q2)
		dq[2] = 0.0f;
	if(QST_FABS(dq[3]) < rms_dq.q3)
		dq[3] = 0.0f;

	q_k[0] = qstdq.q0*dq[0] - qstdq.q1*dq[1] - qstdq.q2*dq[2] - qstdq.q3*dq[3];
	q_k[1] = qstdq.q1*dq[0] + qstdq.q0*dq[1] - qstdq.q3*dq[2] + qstdq.q2*dq[3];
	q_k[2] = qstdq.q2*dq[0] + qstdq.q3*dq[1] + qstdq.q0*dq[2] - qstdq.q1*dq[3];
	q_k[3] = qstdq.q3*dq[0] - qstdq.q2*dq[1] + qstdq.q1*dq[2] + qstdq.q0*dq[3];
	norm = sqrtf(q_k[0]*q_k[0] + q_k[1]*q_k[1] + q_k[2]*q_k[2] + q_k[3]*q_k[3]);
	qstdq.q0 = q_k[0]/norm;
	qstdq.q1 = q_k[1]/norm;
	qstdq.q2 = q_k[2]/norm;
	qstdq.q3 = q_k[3]/norm;

	q0q0 = qstdq.q0 * qstdq.q0;
	q0q1 = qstdq.q0 * qstdq.q1;
	q0q2 = qstdq.q0 * qstdq.q2;
	q0q3 = qstdq.q0 * qstdq.q3;
	q1q1 = qstdq.q1 * qstdq.q1;
	q1q2 = qstdq.q1 * qstdq.q2;
	q1q3 = qstdq.q1 * qstdq.q3;
	q2q2 = qstdq.q2 * qstdq.q2;
	q2q3 = qstdq.q2 * qstdq.q3;
	q3q3 = qstdq.q3 * qstdq.q3;
//	dcm[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
//    dcm[1] = 2.f * (q1*q2 + q0*q3);	// 12
    dcm[2] = 2.f * (q1q3 - q0q2);	// 13
    dcm[3] = 2.f * (q1q2 - q0q3);	// 21
    dcm[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
    dcm[5] = 2.f * (q2q3 + q0q1);	// 23
//    dcm[6] = 2.f * (q1*q3 + q0*q2);	// 31
//    dcm[7] = 2.f * (q2*q3 - q0*q1);	// 32
    dcm[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33

	rpy[0] = atan2f(-dcm[2], dcm[8])*(57.2957796f);
	rpy[1] = asinf(dcm[5])*(57.2957796f);
	rpy[2] = atan2f(-dcm[3],dcm[4])*(57.2957796f);
}

#endif

