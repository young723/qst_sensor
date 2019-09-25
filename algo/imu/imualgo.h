

#ifndef GYRO_ALGO_H_
#define GYRO_ALGO_H_

typedef		void					qvoid; 	
typedef		char					qchar; 	
typedef		unsigned char			quchar; 
typedef		short					qshort; 	
typedef		unsigned short			qushort; 	
typedef		int						qint; 	
typedef		unsigned int			quint; 	
typedef		long long				qlong;
typedef		unsigned long long		qulog;
typedef		float					qfloat;
typedef		double					qdouble;



typedef struct
{
    qfloat x;
    qfloat y;
    qfloat z;
}sensor_data_t;


typedef struct
{
    qfloat q0;
    qfloat q1;
    qfloat q2;
    qfloat q3;
}quaternion_t;


typedef struct 		//欧拉（姿态）矩阵结构体
{
	qfloat T11,T12,T13,	 T21,T22,T23, T31,T32,T33;
}euler_martix;


typedef struct
{

	float				X[4];
	qshort				YRP[3];
	sensor_data_t		buf_data[2];
}qst_imu_data_t;


#ifdef __cplusplus
extern "C" {
#endif

qint qst_algo_imu_cali(qfloat acc[3],qfloat gyro[3]);
qvoid qst_algo_imu_get_offset(qfloat bias_acc[3], qfloat bias_gyro[3], qfloat rms_gyro[3]);
qvoid qst_algo_imu_set_offset(qfloat bias_acc[3], qfloat bias_gyro[3], qfloat rms_gyro[3]);
qint qst_algo_imu_data_process(qfloat acc[3],qfloat gyro[3]);
qvoid qst_algo_imu_init(qfloat accel[3], qfloat gyro[3]);
qvoid qst_algo_imu_update(qfloat accel[3], qfloat gyro[3], qfloat dt);
qvoid qst_algo_imu_angle(qfloat* rpy);
qvoid qst_algo_imu_angle2(qfloat* rpy);


#if defined(USE_ATTITUDE_ENGINE)
qint qst_algo_dq_cali(qfloat dq[4]);
qvoid qst_algo_dq_init(qvoid);
qvoid qst_algo_dq_update(qfloat dq[4], qfloat rpy[3]);
#endif
qvoid qst_algo_linear_acc(qfloat acc[3],qfloat linear_acc[3]) ;


#ifdef __cplusplus
}
#endif

#endif /* GYRO_ALGO_H_ */
