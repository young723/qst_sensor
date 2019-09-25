/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */
/*******************************************************************************
 * $Id: $
 *******************************************************************************/

/**
 *  @defgroup STM32L STM32L System Layer
 *  @brief  STM32L System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file   packet.h
 *      @brief  Defines needed for sending data/debug packets via USB.
 */

#ifndef __QST_PACKET_H__
#define __QST_PACKET_H__

//#include "mltypes.h"

typedef enum {
    PACKET_DATA_ACCEL = 0,
    PACKET_DATA_GYRO,
    PACKET_DATA_COMPASS,
    PACKET_DATA_QUAT,
    PACKET_DATA_EULER,
    PACKET_DATA_ROT,
    PACKET_DATA_HEADING,
    PACKET_DATA_LINEAR_ACCEL,
    NUM_DATA_PACKETS
} qst_packet_e;


#define QST_LOG_UNKNOWN		(0)
#define QST_LOG_DEFAULT		(1)
#define QST_LOG_VERBOSE		(2)
#define QST_LOG_DEBUG		(3)
#define QST_LOG_INFO		(4)
#define QST_LOG_WARN		(5)
#define QST_LOG_ERROR		(6)
#define QST_LOG_SILENT		(8)

#define BUF_SIZE					(256)
#define PACKET_LENGTH				(23)
#define PACKET_DEBUG				(1)
#define PACKET_QUAT					(2)
#define PACKET_DATA					(3)
#define min(a,b)					((a < b) ? a : b)
#define BYTE0(dwTemp)				(*(char *)(&dwTemp))
#define BYTE1(dwTemp)				(*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)				(*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)				(*((char *)(&dwTemp) + 3))

#ifdef __cplusplus
extern "C" {
#endif
void qst_send_data(unsigned char type, long *data);
void qst_send_imu_quat(long *quat);
void qst_send_imu_euler(float angle_rol, float angle_pit, float angle_yaw, int alt, unsigned char fly_model, unsigned char armed);
void qst_send_imu_rawdata(short *Gyro,short *Accel, short *Mag);
#ifdef __cplusplus
}
#endif


#endif /* __QST_PACKET_H__ */
