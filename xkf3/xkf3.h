/*!
 * \file
 * \version 1.1.1 rev. 55968
 * \addtogroup XKF3_INTERFACE
 * \copyright
 * Copyright (c) 2016 Fairchild Semiconductor Corporation or subsidiaries
 * worldwide. All rights reserved.
 * This file and the source code it contains (and/OR the binary code files
 * found in the same folder that contains THIS FILE) and all related software
 * (collectively, "Code") are subject to a Restricted License Agreement
 * ("Agreement") between Fairchild Semiconductor as licensor and the authorized
 * licensee under the Agreement.
 * THE CODE MUST BE USED SOLELY WITH FAIRCHILD PRODUCTS INCORPORATED INTO
 * LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY use, modification,
 * copying or distribution of the Code is strictly prohibited unless expressly
 * authorized by the Agreement. If you are not an authorized user of the Code
 * in accordance with the Agreement, you must stop using or viewing the Code
 * now, remove any copies of the Code from your computer and notify Fairchild
 * Semiconductor immediately at corporate.legal@fairchildsemi.com. ANY COPIES
 * OR DERIVATIVES OF THE CODE (IN WHOLE OR IN PART) IN SOURCE CODE FORM THAT
 * ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE COPYRIGHT NOTICE AND
 * THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
 */

#ifndef __XKF3_H
#define __XKF3_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
//#include "fisimu_hal.h"
#include "sensorfusionconfig.h"

/*!
 * \defgroup XKF3 XKF3 Sensor Fusion Library
 * \brief XKF3 Sensor Fusion.

 * XKF3 is a sensor fusion algorithm, based on Extended Kalman Filter theory
 * that fuses 3D inertial sensor data (orientation and velocity increments) and
 * 3D magnetometer, also known as ‘9D’, data to optimally estimate 3D
 * orientation with respect to an Earth fixed frame.
 *
 * A license to use XKF3 in a CMSIS compliant library form for Cortex M0+, M3,
 * M4, and M4F for commercial purposes is provided with the FIS Evaluation Kit.
 *
 * XKF3 is developed by Xsens, a pioneering company in inertial based 3D motion
 * tracking. The first generation 9D sensor fusion algorithms were developed by
 * Xsens more than 15 years ago and have been proven in demanding 24/7
 * continuous use for a broad range of applications; from unmanned underwater
 * robotics to accurate joint angle measurements for rehabilitation and sports.
 * The XKF3 algorithm is now wholly owned by Fairchild Semiconductor.
 *
 * The XKF3 algorithms are provided as a precompiled static library. Three
 * variants of the library are included in the SDK:
 *
 * Name           | Target
 * ---------------|-------------------------------------------------------
 * libxkf3_CM0.a  | Cortex M0 and M0+ cores
 * libxkf3_CM3.a  | Cortex M3, M4 and M7 cores without floating point unit
 * libxkf3_CM4.a  | Cortex M4F and M7F cores with floating point unit
 *
 * Each library variant is available for the arm-none-eabi-gcc, IAR EWARM and
 * Keil ARMCC toolchains. Floating point libraries for gcc are compiled using
 * the Soft Float ABI.
 */

/*!
 * \defgroup XKF3_INTERFACE XKF3 Sensor Fusion Library Interface
 * \ingroup XKF3
 * \brief XKF3 sensor fusion library interface.
 *
 * \{
 */

/*!
 * \struct Xkf3
 * \brief The XKF3 filter structure.
 */
struct Xkf3;
struct Xkf3Input;
struct Xkf3Results;
struct Xkf3CorrectedSensorData;
struct Xkf3FeedbackParameters;


#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \name Filter information
 * \{
 */
/*!
 * \brief Return the XKF3 version as (major, minor, revision).
 *
 * \param[out] major a pointer to the major version number.
 * \param[out] minor a pointer to the minor version number.
 * \param[out] revision a pointer to the revision number.
 */
void Xkf3_getVersion(int* major, int* minor, int* revision);
/*! \} */

/*!
 * \name Filter memory management
 */
/*!
 * \brief Get the amount of memory required to store the filter structure.
 *
 * The XKF3 filter structure is used both to store the filter state and as a
 * temporary scratch space. This avoids any dynamic memory allocation in run.
 */
size_t Xkf3_getMemSize(void);

/*!
 * \brief Initialize and return the XKF3 filter structure.
 * \param mem Pointer to memory area to use for the filter.
 *
 * The memory provided to the initialization function must be at least as large
 * as the value returned by Xkf3_getMemSize().
 *
 * This function only prepares the filter structure. It is necessary to call
 * Xkf3_reinitialize() to prepare the filter for operation with a particular
 * sensor configuration.
 */
struct Xkf3* Xkf3_init(void* mem);

/*!
 * \brief Destroy the XKF3 filter structure.
 * \param[in] xkf The filter instance to destroy.
 */
void Xkf3_destruct(struct Xkf3* xkf);
/*! \}  */

/*!
 * \name Filter initialization
 * \{
 */

/*! \brief Indicates no custom options, re-initialization will preserve both filter state and sensor calibration if available. */
#define XKF3_INIT_DEFAULT				(0)

/*! \brief Force the re-initialization to flush the filter state. */
#define XKF3_INIT_CLEAR_STATE			(1 << 0)

/*!
 * \brief Force the re-initialization to flush the calibration state.
 * \note Any calibration parameters available on the device will not be cleared.
 * \see FisImu_reset() to clear the device calibration.
 */
#define XKF3_INIT_CLEAR_CALIBRATION		(1 << 1)

/*!
 * \brief Reinitialize the filter for the passed configuration.
 * \param[in,out] xkf The XKF3 filter to initialize.
 * \param[in] hal FIS hardware abstraction layer used to communicate with FIS.
 * \param[in] configuration The configuration of the sensor fusion system.
 * \param[in] options. Initialization options. \sa XKF3_INIT_DEFAULT, XKF3_INIT_CLEAR_STATE, XKF3_INIT_CLEAR_CALIBRATION
 *
 * \note During initialization XKF requires exclusive access to the FIS hardware.
 */
void Xkf3_reinitialize(struct Xkf3* xkf, struct FisImuHal const* hal,
                         struct SensorFusionConfig const* configuration, uint8_t options);
/*! \} */

/*!
 * \name Filter update
 * \{
 */
/*! \brief Input structure contains accelerometer data. */
#define XKF3_DATA_TYPE_ACCELEROMETER 1
/*! \brief Input structure contains magnetometer data. */
#define XKF3_DATA_TYPE_MAGNETOMETER 2
/*! \brief Input structure contains gyroscope data. */
#define XKF3_DATA_TYPE_GYROSCOPE 4
/*! \brief Input structure contains AttitudeEngine data. */
#define XKF3_DATA_TYPE_ATTITUDE_ENGINE 8

/*!
 * \brief New output data is available from the filter.
 *
 * XKF3 processed data can be retrieved using the Xkf3_getResults() and
 * Xkf3_getCorrectedSensorData() functions.
 */
#define XKF3_OUTPUT_AVAILABLE (1)

/*!
 * \brief New feedback data is available from the filter.
 *
 * XKF3 can provide feedback to the FIS to correct for gyroscope bias errors
 * in AttitudeEngine mode. The feedback parameters should be retrieved by
 * calling Xkf3_getFeedbackParameters().
 *
 * Example:
 * \code{.c}
 * uint8_t status = Xkf3_userFuser(xkf, &input);
 * if (status & XKF3_FEEDBACK_AVAILABLE)
 * {
 *     struct Xkf3FeedbackParameters params;
 *     Xkf3_getFeedbackParameters(xkf, &params);
 *     FisImu_updateAttitudeEngineGyroBias(params.gyr_b);
 * }
 * \endcode
 */
#define XKF3_FEEDBACK_AVAILABLE (2)

/*!
 * \brief XKF3 low frequency operations are required.
 *
 * XKF3 background filter operations need to be executed and therefore
 * Xkf3_runFilter() needs to called.
 *
 */
#define XKF3_MUST_RUN_FILTER	(4)

/*!
 * \brief XKF3 input data structure.
 */
struct Xkf3Input
{
	/*!
	 * \brief Bitfield indicating which data fields are valid.
	 * \see XKF3_DATA_TYPE_ACCELEROMETER
	 * \see XKF3_DATA_TYPE_GYROSCOPE
	 * \see XKF3_DATA_TYPE_MAGNETOMETER
	 * \see XKF3_DATA_TYPE_ATTITUDE_ENGINE
	 */
	uint8_t availableData;
	/*!
	 * \brief Sample counter of the current input sample.
	 */
	uint8_t sampleCounter;
	/*! \brief IMU (accelerometer & gyroscope) data. */
	union
	{
		/*!
		 * \brief Raw IMU data.
		 */
		struct
		{
			/*!
			 * \brief Acceleration vector [ax, ay, az] in m/s^2.
			 */
			float acceleration[3];
			/*!
			 * \brief Rotational rate vector [gx, gy, gz] in rad/s.
			 */
			float rotationalRate[3];
		} raw;
		/*!
		 * \brief AttitudeEngine data.
		 */
		struct
		{
			/*!
			 * \brief AttitudeEngine orientation increment [dq0, dq1, dq2, dq3].
			 */
			float orientationIncrement[4];
			/*!
			 * \brief AttitudeEngine velocity increment [dvx, dvy, dvz].
			 */
			float velocityIncrement[3];
			/*!
			 * \brief AttitudeEngine status flags.
			 */
			uint16_t flags;
		} attitudeEngine;
	} imu;
	/*!
	 * \brief Magnetic field data [mx, my, mz] in micro-Tesla.
	 */
	float magneticField[3];
	/*!
	 * \brief Contents of the FIS status 1 register.
	 */
	uint8_t status1;
};

/*!
 * \brief Pass timestamped data into the filter to update its state.
 * \param[in, out] xkf An XKF3 instance.
 * \param[in] input Input data structure containing the new sensor data.
 * \returns Status byte.
 *
 * Runs a filter update step. This function is intended for use when a single
 * thread of control is used for XKF3 processing.
 *
 * This function is roughly equivalent to
 * \code{.c}
 * uint8_t status = Xkf3_userFuser(xkf, input);
 * if (status & XKF3_MUST_RUN_FILTER)
 * {
 *     Xkf3_runFilter(xkf)
 * }
 * \endcode
 *
 * \see XKF3_OUTPUT_AVAILABLE
 * \see XKF3_FEEDBACK_AVAILABLE
 */
uint8_t Xkf3_update(struct Xkf3* xkf, struct Xkf3Input const* input);

/*!
 * \brief Run the high frequent output generating part of the filter.
 * \param[in, out] xkf An XKF3 instance.
 * \param[in] input Input data structure containing the new sensor data.
 * \returns Status byte.
 * \see XKF3_OUTPUT_AVAILABLE
 * \see XKF3_FEEDBACK_AVAILABLE
 * \see XKF3_MUST_RUN_FILTER
 *
 * This part of the filter is responsible for handling the input data and
 * producing user output.
 */
uint8_t Xkf3_userFuser(struct Xkf3* xkf, struct Xkf3Input const* input);

/*!
 * \brief Run the low frequent tracking part of the filter.
 * \param[in, out] xkf An XKF3 instance.
 * \return true if additional Xkf3_runFilter calls are required.
 *
 * This part of the filter tracks slowly changing sensor and environmental
 * parameters. This part of the filter can be run as a background task.
 */
bool Xkf3_runFilter(struct Xkf3* xkf);
/*! \} */

/*!
 * \name Filter output
 * \{
 */
/*! \brief Indicates no alarms. */
#define XKF3_STATUS_OK                  (0)

/*!
 * \brief Overall status alarm bit.
 *
 * LOW (0) is OK, if HIGH (1) XKF3 indicates that the accuracy of XKF3
 * should not be trusted (NOK). Details of possible causes can be found
 * in the subsequent bit-fields.

 * If the user is only interested in the overall status of the output, it is
 * enough to read the first bit (bit 0/zero).  Subsequent bits will only
 * provide more detail.
 */
#define XKF3_STATUS_WARNING             (1 << 0)

/*!
 * \brief Alarm bit to indicate clipping of gyroscopes and accelerometers.
 *
 * Clipping is defined as exceeding the dynamical range of the sensor.

 * If HIGH (1) one of the sensors is clipping. If LOW (0) all
 * sensors are operating within their specified dynamic range.
 */
#define XKF3_STATUS_CLIPPING_DETECTED   (1 << 1)

/*!
 * \brief Alarm bit to indicate that a magnetic distortion is detected.
 *
 * This bit indicates that the magnetic field mapping (MFM) is active due to a
 * detected magnetic distortion for which hard-iron calibration is needed.
 *
 * HIGH (1) if MFM is operating. LOW (0) if MFM is not active and converged,
 * i.e. the magnetic field sensors are calibrated.
 */
#define XKF3_STATUS_MFM_ACTIVE          (1 << 2)

/*!
 * \brief Alarm bit to indicate problems in heading accuracy.
 *
 * HIGH (1) if the heading cannot be estimated reliably. See
 * #XKF3_STATUS_VERTICAL_MAGFIELD and #XKF3_STATUS_MAGNETIC_DISTORTION for
 * possible causes.
 *
 * This bit can be ignored for the gaming/UI scenario.
 */
#define XKF3_STATUS_HEADING_UNRELIABLE  (1 << 3)

/*!
 * \brief Indicates that the local magnetic field is close to vertical.
 *
 * If the local magnetic field is close to vertical, i.e. aligned with gravity,
 * the heading is not observable.
 *
 * HIGH (1) if magnetic field is close to vertical. LOW (0) if magnetic field is
 * closer to the expected declination value.
 */
#define XKF3_STATUS_VERTICAL_MAGFIELD   (1 << 4)

/*!
 * \brief Alarm bit to indicate that the local magnetic field is distorted
 * (temporal or spatial).
 *
 *  HIGH (1) if magnetic field is distorted. LOW (0) if magnetic field is
 *  closer to the expected value.
 */
#define XKF3_STATUS_MAGNETIC_DISTORTION (1 << 5)

/*!
 * \brief Indicates that the inclination (roll/pitch) cannot be accurately estimated.
 *
 * HIGH (1) if the inclination can NOT be estimated accurately. LOW (0) if the
 * inclination can be estimated accurately.
 */
#define XKF3_STATUS_INCLINATION_WARNING (1 << 6)

/*!
 * \brief Indicates that heading estimate of Gaming scenario is not representative of true North tracking.
 *
 * HIGH (1) if the heading estimate of Gaming scenario is not tracking true North.
 * LOW (0) if the heading estimate of Gaming scenario is equal to the best available North referenced heading estimate (i.e. Navigation scenario).
 */
#define XKF3_STATUS_TRUENORTH_HEADING_DIFFERENCE   (1 << 7)

/*!
 * \brief XKF3 output data structure.
 */
struct Xkf3Results
{
	/*! \brief Quaternion representing the orientation of the device.
	 *
	 * The orientation is expressed with respect to the Local coordinates.
	 * First element (index 0) is the real part, second to fourth elements the
	 * imaginary parts. The quaternion is normalized.
	 *
	 * The quaternion is a unit quaternion according to [W X Y Z].
	 */
	float q_ls[4];

	/*! \brief The velocity estimate in [m/s].
	 *
	 * The velocity is the velocity vector, expressed in the Local coordinates
	 * in [m/s]. Due to the fundamental inertial drift, the velocity estimation
	 * is high-pass filtered and constant velocities or initial velocities will
	 * therefore not be tracked. The Local frame is Earth-fixed and has Y along
	 * the magnetic north and Z upwards along gravity.
	 *
	 * The velocity estimate is expressed in the Local frame in [m/s] according
	 * to [vX vY vZ].
	 */
	float vel_l[3];

	/*! \brief The user acceleration.
	 *
	 * The user acceleration is the acceleration
	 * (second derivative of position, gravity subtracted) of the user expressed
	 *in
	 * [m/s2] in the Local frame.
	 *
	 * The user acceleration is expressed in the Local frame in [m/s2] according
	 *to [aX aY aZ].
	 */
	float acc_l[3];

	/*! \brief The filter status bitfield.
	 *
	 * If equal to XKFCE_STATUS_OK (0), everything is OK. If not, check the set
	 * bits for more detailed information.
	 *
	 * \see XKF3_STATUS_OK
	 * \see XKF3_STATUS_WARNING
	 * \see XKF3_STATUS_CLIPPING_DETECTED
	 * \see XKF3_STATUS_MFM_ACTIVE
	 * \see XKF3_STATUS_HEADING_UNRELIABLE
	 * \see XKF3_STATUS_VERTICAL_MAGFIELD
	 * \see XKF3_STATUS_MAGNETIC_DISTORTION
	 * \see XKF3_STATUS_INCLINATION_WARNING
	 * \see XKF3_STATUS_TRUENORTH_HEADING_DIFFERENCE
	 */
	uint8_t status;

	/*!
	 * \brief The sample counter of output data.
	 *
	 * The sample counter of the output data corresponds to the sample count
	 * of the input data.
	 */
	uint8_t sampleCounter;
};

/*!
 * \brief A structure containing sensor data corrected by XKF3 filter.
 *
 * The XKF3 filter can correct for various common sensor data errors, such as:
 *
 * * Accelerometer scale errors
 * * Accelerometer bias errors
 * * Gyroscope bias errors
 * * Magnetometer scale errors
 * * Magnetometer bias errors
 */
struct Xkf3CorrectedSensorData
{
	/*! \brief The corrected accelerometer data.
	 *
	 * The corrected accelerometer data is the input accelerometer data in
	 * Sensor coordinate frame.  Any calibration applied by the filter is
	 * reflected in this vector.
	 *
	 * The corrected accelerometer data is expressed in the Sensor frame in
	 * [m/s2] according to [aX aY aZ].
	 */
	float acc_s[3];

	/*! \brief The corrected gyroscope data.
	 *
	 * The corrected gyroscope data is the input gyroscope data in Sensor
	 * coordinate frame.  Any calibration applied by the filter is reflected in
	 * this vector.
	 *
	 * The corrected gyroscope data is expressed in the Sensor frame in [rad/s]
	 * according to [gX gY gZ].
	 */
	float gyr_s[3];

	/*! \brief The corrected magnetic field.
	 *
	 * The corrected magnetic field data is the input magnetic field data in
	 * Sensor coordinate frame.  Any calibration applied by the filter is
	 * reflected in this vector.
	 *
	 * The corrected magnetic field data is expressed in the Sensor frame in
	 * [uT] according to [mX mY mZ].
	 */
	float mag_s[3];

	/*!
	 * \brief The sample counter of output data.
	 *
	 * The sample counter of the output data corresponds to the sample count
	 * of the input data.
	 */
	uint8_t sampleCounter;
};

/*!
 * \brief A structure containg XKF3 feedback information for the FIS.
 */
struct Xkf3FeedbackParameters
{
	/*! \brief The estimated gyroscope bias. */
	float gyr_b[3];
};

/*!
 * \brief Get the latest results from the XKF3 filter.
 * \param[in,out] xkf An XKF3 instance.
 * \param[out] results Pointer to container to store the results to.
 *
 * This function returns a fully populated Xkf3Results structure. If
 * only some of the fields are required then you can request these
 * individually using the Xkf3_getOrientation, Xkf3_getFreeAcceleration, and
 * Xkf3_getVelocity function.
 *
 * \note This function should only be called if the status byte returned
 * by Xkf3_update() had the #XKF3_OUTPUT_AVAILABLE flag set.
 *
 * Example:
 * \code{.c}
 * uint8_t status = Xkf3_update(xkf, &input);
 * if (status & XKF3_OUTPUT_AVAILABLE)
 * {
 *     struct Xkf3Results results;
 *     Xkf3_getResults(xkf, &results);
 *     // do something with results...
 * }
 * \endcode
 */
void Xkf3_getResults(struct Xkf3* xkf, struct Xkf3Results* results);

/*!
 * \brief Get the latest orientation results from the XKF3 filter.
 * \param[in, out] xkf An XKF3 instance.
 * \param[out] results Pointer to container to store the results to.
 * \param[in] scenario indicates the type of scenario for which the results are requested.
 *
 * \note This function should only be called if the status byte returned
 * by Xkf3_userFuser() had the #XKF3_OUTPUT_AVAILABLE flag set.
 *
 */
void Xkf3_getOrientation(struct Xkf3* xkf, struct Xkf3Results* results, enum SensorFusionScenario scenario);

/*!
 * \brief Get the latest free acceleration results from the XKF3 filter.
 * \param[in, out] xkf An XKF3 instance.
 * \param[out] results Pointer to container to store the results to.
 * \param[in] scenario indicates the type of scenario for which the results are requested.
 *
 * \note This function should only be called if the status byte returned
 * by Xkf3_userFuser() had the #XKF3_OUTPUT_AVAILABLE flag set.
 *
 */
void Xkf3_getFreeAcceleration(struct Xkf3* xkf, struct Xkf3Results* results, enum SensorFusionScenario scenario);

/*!
 * \brief Get the latest velocity results from the XKF3 filter.
 * \param[in, out] xkf An XKF3 instance.
 * \param[out] results Pointer to container to store the results to.
 * \param[in] scenario indicates the type of scenario for which the results are requested.
 *
 * \note This function should only be called if the status byte returned
 * by Xkf3_userFuser() had the #XKF3_OUTPUT_AVAILABLE flag set.
 *
 */
void Xkf3_getVelocity(struct Xkf3* xkf, struct Xkf3Results* results, enum SensorFusionScenario scenario);

/*!
 * \brief Get the latest orientation results from the XKF3 filter in Euler form.
 * \param[in, out] xkf An XKF3 instance.
 * \param[out] euler Euler orientation in degrees (Yaw, Pitch, Roll).
 * \param[in] scenario indicates the type of scenario for which the results are requested.
 *
 * \note This function should only be called if the status byte returned
 * by Xkf3_userFuser() had the #XKF3_OUTPUT_AVAILABLE flag set.
 */
void Xkf3_getEulerOrientation(struct Xkf3* xkf, float* euler, enum SensorFusionScenario scenario);

/*!
 * \brief Get the latest corrected sensor data from the XKF3 filter.
 * \param[in, out] xkf An XKF3 instance.
 * \param[out] data Pointer to container to store the results to.

 * \note This function should only be called if the status byte returned
 * by Xkf3_update() had the #XKF3_OUTPUT_AVAILABLE flag set.
 *
 * Example:
 * \code{.c}
 * uint8_t status = Xkf3_update(xkf, &input);
 * if (status & XKF3_OUTPUT_AVAILABLE)
 * {
 *     struct Xkf3CorrectedSensorData data;
 *     Xkf3_getCorrectedSensorData(xkf, &data);
 *     // do something with data...
 * }
 * \endcode
 */
void Xkf3_getCorrectedSensorData(struct Xkf3* xkf,
                                 struct Xkf3CorrectedSensorData* data);

/*!
 * \brief Get the latest XKF feedback parameters.
 * \param[in, out] xkf An XKF3 instance.
 * \param[out] params Pointer to container to store the parameters to.
 *
 * \note This function should only be called if the status byte returned
 * by Xkf3_update() had the #XKF3_FEEDBACK_AVAILABLE flag set.
 *
 * Example:
 * \code{.c}
 * uint8_t status = Xkf3_update(xkf, &input);
 * if (status & XKF3_FEEDBACK_AVAILABLE)
 * {
 *     struct Xkf3FeedbackParameters params;
 *     Xkf3_getFeedbackParameters(xkf, &params);
 *     FisImu_updateAttitudeEngineGyroBias(params.gyr_b);
 * }
 * \endcode
 */
void Xkf3_getFeedbackParameters(struct Xkf3* xkf,
                                  struct Xkf3FeedbackParameters* params);
/*! \} */

/*!
 * \name Heading resets
 * \{
 */
/*! \brief Reset heading estimate of Gaming scenario.
 *
 * This function can be used to reset heading estimate of Gaming scenario and align it either to the
 * Navigation scenario or to an absolute value.
 *
 * \param[in] xkf An xkf3 instance.
 * \param[in] alignToNav \c true if Gaming heading should be aligned to Navigation heading.
 * \param[in] headingOffsetDeg Heading offset value expressed in degrees.
 *
 * \note When \a alignToNav is \c false, \a headingOffsetDeg is intended as
 * the absolute heading value after resetGamingHeading is applied.
 * Alternatively, when \a alignToNav is \c true, \a headingOffsetDeg is a
 * requested offset value with respect to the heading estimate of the
 * Navigation scenario.
 *
 */
void Xkf3_resetGamingHeading(struct Xkf3* xkf, bool alignToNav, float headingOffsetDeg);

/*! \brief Reset heading estimate of Navigation scenario.
 *
 * This function can be used to instantaneously reset the heading estimate of Navigation scenario and align it to the local magnetic field.
 *
 * \note The estimate of local North and therefore of the resulting heading after reset are affected from any present magnetic field distortion.
 *
 */
void Xkf3_resetNavigationHeading(struct Xkf3* xkf);

/*! \} */

/*!
 * \name State persistence
 * \{
 */
/*!
 * \brief Save the state of the XKF3 filter.
 *
 * The state is a set of parameters that can be kept persistent between
 * XKF runs.
 *
 * \returns the number of stored values.
 *
 * \param[in] xkf An xkf3 instance.
 * \param[out] state A pointer to an array containing the state, can be NULL.
 *
 * Example using dynamic allocation:
 * \code{.c}
 * int size;
 * float *state;
 *
 * size = Xkf3_saveState(xkf, NULL);
 * state = (float*)malloc(size * sizeof(float));
 * size = Xkf3_saveState(xkf, state);
 * store_persistently(state, size);
 * free(stateToSave);
 * \endcode
 *
 * Example using static allocation:
 * \code{.c}
 * int size;
 * float state[20]; // choose a safe value
 *
 * size = Xkf3_saveState(xkf, state);
 * store_persistently(state, size);
 * \endcode
 *
 * \see Xkf3_restoreState()
 */
size_t Xkf3_saveState(struct Xkf3 const* xkf, float* state);

/*! \brief Restore a previously saved state.
 *
 * The state is a set of parameters that can be kept persistent between
 * XKF runs. The Xkf3_restoreState() function can be used to provide
 * the filter with a 'warm' start.
 *
 * \param[in] xkf An xkf3 instance.
 * \param[in] state A pointer to an array containing the state.
 *
 * \code{.c}
 * float state[20];
 *
 * retrieve_state_from_persistent_memory(state);
 *
 * void* mem = malloc(Xkf3_getMemSize());
 * Xkf3 *xkf = Xkf3_init(mem);
 * Xkf3_restoreState(xkf, state);
 * Xkf3_reinitialize(xkf, hal, config);
 * \endcode
 *
 * \see Xkf3_saveState()
 */
void Xkf3_restoreState(struct Xkf3* xkf, float const* state);


/*!
 * \brief Reset the IMU (Acc/Gyr) bias states to zero.
 *
 *  This function should be called if the IMU biases are estimated externally
 *  to the filter, e.g. using the \ref BOARD_LEVEL_CAL procedure.
 */
void Xkf3_resetImuBiasStates(struct Xkf3* xkf);

/*! \} */

#ifdef __cplusplus
}
#endif // extern "C"
/*! \} */
#endif // __XKF3_H
