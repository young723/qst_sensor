/*!
 * \file boardlevelcal.h
 * \version 1.1.1 rev. 58809
 *
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

#ifndef __BOARDLEVELCAL_H
#define __BOARDLEVELCAL_H

#include <stddef.h>
#include <stdbool.h>
#include "fis1100_driver.h"

struct Fis1100RawSample;
struct Fis1100Config;

/*!
 * \struct Blc
 * \brief The Board Level Calibration (BLC) structure.
 */
struct BLC;

/*!
 * \struct BLC_ReferenceData
 * \brief The Reference Data structure
 */

#if defined (__ICCARM__)
#define ANONYMOUS_UNION
#elif defined (__GNUC__)
#define ANONYMOUS_UNION
#elif (defined(__arm__) && defined(__ARMCC_VERSION))
#define ANONYMOUS_UNION _Pragma("anon_unions")
#else
#define ANONYMOUS_UNION
#endif

ANONYMOUS_UNION
struct BLC_ReferenceData
{
	/*! \brief Acceleration value expressed in m/s2 */
	float acceleration[3];
	union
	{
		/*! \brief Angular velocity value expressed in rad/s */
		float angularVelocity[3];
		/*! \brief Angular displacement value expressed in rad */
		float angle[3];
	};
};

ANONYMOUS_UNION
struct BLC_StepStatistics
{
	float timeOutOfPlaneMillis; /*! \brief is the cumulative time spent in 'out-of-plane' condition [ms] (i.e. estimated roation axis signicantly differs from reference rotation axis)*/
	float maxOutOfPlaneDegrees; /*! \brief contains information about the worst experienced 'out-of-plane' condition ( =0 perfectly in plane, =90 completely out of plane ) [deg] */
	union /*! \brief Maximum difference between referenceData and deviceData */
	{
		float maxAngleDifferenceDegrees; /*! \brief Maximum angular difference between raw estimated rotation and reference data, estimated per channel. Dynamic steps only. */
		float maxAccelerationDifferenceMs2; /*! \brief Maximum acceleration difference between raw acceleration and reference data, estimated per channel. Static steps only. */
	};
	float stepTimeMillis; /*! \brief is the effective step duration [ms] */
	float accMeanMs2[3]; /*! \brief contains acceleromenter step mean for X, Y and Z [m/s^2], Static steps only */
	float accStdMs2[3]; /*! \brief contains acceleromenter step standard deviation for X, Y and Z [m/s^2], Static steps only */
	float gyrMeanDps[3]; /*! \brief contains gyroscope step mean for X, Y and Z [deg/s], Static and Dynamic steps */
	float gyrStdDps[3]; /*! \brief contains gyroscope step standard deviation for X, Y and Z [deg/s], Static and Dynamic steps */
	float accData[3]; /*! \brief contains relevant acceleromer statistics for both Static and Dynamic steps */
	float gyrData[3]; /*! \brief contains relevant gyroscope statistics for both Static and Dynamic steps */
	float gyrMeanEndDps[3]; /*! \brief contains gyroscope mean for X, Y and Z [deg/s] from the last static period of the step, Static and Dynamic steps */
	float gyrStdEndDps[3];  /*! \brief contains gyroscope step standard deviation for X, Y and Z [deg/s], Static and Dynamic steps */
};

struct Fis1100_gainCalibration
{
	/*! \brief Accelerometer full gain matrix. */
	float accGain[9];

	/*! \brief Gyroscope full gain matrix. */
	float gyrGain[9];
};

/*! \brief Full FIS1100 calibration structure with biases and gains */
struct Fis1100_fullCalibration
{
	/*! \brief Legacy struct. Accelerometer and Gyroscope offset calibration. */
	//struct Fis1100_offsetCalibration offsetCal;
	/*! \brief Accelerometer and Gyroscope gain calibration. */
	struct Fis1100_gainCalibration gainCal;
};

typedef enum  {
	CalType_OneStep = 0, //One step static
	CalType_TwoSteps = 1, //One step static and one dynamic
	CalType_FourSteps = 2, //One step static and Three steps dynamic
	CalType_SevenSteps = 3 //Four steps static and Three steps dynamic
} BLC_CalibrationType;

typedef enum  {
	StepType_Static = 0,
	StepType_DynamicAngle = 1
} BLC_CalibrationStepType;

/*! \brief ReferenceData with Z axis pointing up */
extern const struct BLC_ReferenceData Static_Z_Up;
/*! \brief ReferenceData with Z axis pointing down */
extern const struct BLC_ReferenceData Static_Z_Down;

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \defgroup BOARD_LEVEL_CAL Board level calibration
 * \ingroup XKF3
 * \brief Fast calibration algorithm for production support.
 *
 * The board level calibration algorithm estimates sensor calibration paramenters
 * (e.g. offset, sensitivity) based on a small number of reference measurements
 * (e.g. orientations and elementary rotations). In its simplest form to use the BLC
 * the PCB assembly should be held stationary such that the positive Z-axis
 * of the FIS1100 is pointing vertically up (or down).
 * In this case the alignment of the Z-axis is crucial to the algorithm performance, and
 * should be within 3 degrees of vertical for acceptable performance. More complex
 * calibration procedures would include multiple steps (e.g. four or seven steps)
 * PCB rotations around X, Y and Z-axis and the availability of reference measures of the
 * travelled angular displacement
 *
 * \note The board level calibration routine is intended for use a part of the
 * production process, and is not designed for use by end users.
 *
 */

/*!
 * \brief Reset BLC state.
  * \param[in, out] blc The BLC instance to initialize/reset
  *
  * This function should only be called to initialize or reset all the internal states and data buffers of the calibration algorithm.
 */
void BLC_reset(struct BLC* blc);

/*!
 * \brief Handle a new raw sample from the FIS1100.
 * \param [in, out] blc Pointer to a BLC instance
 * \param [in] sample Pointer to raw FIS1100 sample to handle
 *
 * This function should be called once for each new available sample
 */
void BLC_handleSample(struct BLC* blc, struct Fis1100RawSample const* sample);


/*!
 * \brief Check to see if the calibration process is complete and if all results are available.
 * \param [in, out] blc Pointer to BLC instance
 * \returns true if all configured results have been estimated
 */
bool BLC_isAvailable(struct BLC* blc);

/*!
 * \brief Check if the calibration is valid and if results can be finalized
  * \param [in, out] blc Pointer to BLC instance
  * \returns true is calibration can be finalized with the available data
  *
  * The calibration will not be valid until all the required steps for the provided
  * calibration type are executed correctly or if an unrecognized motion pattern
  * is performed during the calibration procedure.
 */

bool BLC_isValid(struct BLC* blc);

/*!
 * \name Calibration results
 * \{
 */
/*! \brief Minimum required size of the calibration results buffer, expressed in bytes */
#define BLC_CALIBRATION_PARAMETERS_SIZE_BYTES	144

/*!
 * \brief Get the calibration results.
 * \param [in, out] blc Pointer to a BLC instance
 * \param [out] cal Pointer to calibration buffer to put result into.
 * \returns size expressed in bytes of the copied data
 *
 * The full calibration results are copied into the passed buffer.
 * This buffer must be at least BLC_CALIBRATION_PARAMETER_SIZE_BYTES in length.
 * Once the calibration result is read, it can be stored persistently for later use with Xkf3_applyCalibration().
 * \note This function should only be called after all the steps are completed and finalized and BLC_isValid()
 * has been checked to ensure that processing is finished and that the resulting
 * data is valid.
 *
 * Example using static allocation:
 * \code{.c}
 * int size;
 * uint8_t calibrationResult[BLC_CALIBRATION_PARAMETERS_SIZE_BYTES];
 *
 * size = BLC_getCalibration(blc, calibrationResult);
 * store_persistently(calibrationResult, size);
 * \endcode
 *
 * Example using dynamic allocation:
 * \code{.c}
 * int size;
 * uint8_t *calibrationResult;
 *
 * size = BLC_getCalibration(blc, NULL);
 * calibrationResult = (uint8_t*)malloc(size);
 * size = BLC_getCalibration(blc, calibrationResult);
 * store_persistently(calibrationResult, size);
 * free(calibrationResult);
 * \endcode
 *
 * \see Xkf3_applyCalibration()
 * \see BLC_isStepComplete()
 * \see BLC_calibrateForStep()
 * \see BLC_isValid()
*/

size_t BLC_getCalibration(struct BLC* blc, uint8_t* calDataBuffer);

/*!
 * \brief Get BLC offset calibration results.
  * \param [in, out] blc Pointer to a BLC instance
  * \param [out] cal Pointer to FIS100 offset calibration structure to put result into.
 *
 * The offset calibration results are copied into the passed structure. This function
 * should only be called after BoardLevelCal_isStepComplete() and BoardLevelCal_isValid()
 * have been checked to ensure that processing is complete and that the resulting
 * data is valid.
 */
void BLC_getOffsetCalibration(struct BLC* blc, struct Fis1100_offsetCalibration* cal);

/*!
 * \brief Get BLC gain calibration results.
  * \param [in, out] blc Pointer to a BLC instance
  * \param [out] cal Pointer to FIS100 gain calibration structure to put result into.
 *
 * The gain calibration results are copied into the passed structure. This function
 * should only be called after BoardLevelCal_isStepComplete() and BoardLevelCal_isValid()
 * have been checked to ensure that processing is complete and that the resulting
 * data is valid.
 */
void BLC_getGainCalibration(struct BLC* blc, struct Fis1100_gainCalibration* cal);

/*! \} */

/*!
 * \brief Get the amount of memory required to store the BLC structure.
 * \returns required size expressed in number of bytes
 *
 * The BLC structure is used both to store the filter state and as a
 * temporary data buffer space.
 */
size_t BLC_getMemSize(void);

/*!
 * \brief Return the BoardLevelCal version as (major, minor, revision)
 *
 * \param[out] major a pointer to the major version number
 * \param[out] minor a pointer to the minor version number
 * \param[out] revision a pointer to the revision number
 */
void BLC_getVersion(int* major, int* minor, int* revision);

/*!
 * \brief Initialize and return the BLC calibration structure.
 * \param[in, out] mem Pointer to memory area to use for the filter.
 * \param[in] type Calibration procedure type
 * \returns an instance of the BLC calibration
 *
 * This function prepares the calibration structure. It is necessary to call
 * BoardLevelCal_reset() and BoardLevelCal_initStep() to prepare the calibration routines for specific operations.
 * The memory provided to this function must be at least as large
 * as the value returned by BoardLevelCalibration_getMemSize(). This memory will be used to
 * allocate the required structures and must be mantained for the whole BLC struct life.
 */
struct BLC* BLC_init(void* mem, BLC_CalibrationType type);

/*!
 * \brief Destroy the BLC calibration structure
 * \param[in] blc The BLC instance to destroy
 */
void BLC_destruct(struct BLC* blc);

/*!
 * \brief Starts a BLC step.
 * \param[in, out] blc a BLC instance
 * \param [in] type of the calibration step intended to start
 *
 * This function should be called to initialize each single calibration step
 */
void BLC_initStep(struct BLC* blc, BLC_CalibrationStepType type);

/*!
 * \brief Check to see if the last initied calibration step is complete.
 * \param [in, out] blc Pointer to a BLC instance
  * \returns indicates if enough data is available to finalize the step correctly
 *
 * This function must be used to make sure the step is completed before finalizing it
 * and starting a new one. Note that this condition is necessary but often not sufficient
* (e.g. motion should be completed in case of dynamic operations)
 */
bool BLC_isStepComplete(struct BLC* blc);

/*!
 * \name Step finalization
 * \{
 * All the calibration steps require to be finalized with BLC_calibrateForStep() to be effectively
 * added to the calibration buffer and to be used, together with the provided
 * BLC_ReferenceData, to estimate calibration parameters when calling BLC_getCalibration().
 * Validity of calibration assumptions as well as correcteness of user operations
 * are continuously checked during all BLC operations to anticipate possible
 * procedure issues, to ensure accurate calibration results and to facilitate error handling
 * from the host application (as for example, repeating a single failed calibration step).
 * For this purpouse, an error and warning status is returned as part of BLC_calibrateForStep().
 *
 * In particular, steps that reports an error flag ((status & BLC_STEPSTATUS_MASK_ERRORS) > 0) are immidiately
 * identified to formally violate BLC calibration assumptions, and therefore they are automatically discarded
 * and not added to the final calibration buffer (used during BLC_getCalibration).
 *
 * \note: In these cases, it is recommended to check specific reported error flags, correctness of
 * related operations and to eventually repeat the  specific step from the beginning (calling BLC_initStep())
 * until it can be correctly finalized ((status & BLC_STEPSTATUS_MASK_ERRORS) == BLC_STEPSTATUS_NOERROR).
 *
 * Differently, steps that only report one or multiple warning flags ((status & BLC_STEPSTATUS_MASK_WARNINGS)> 0)
 * don't explicitly violate BLC calibration assumptions but they are likely to result in large parameters correction
 * terms and therefore they might still require some attention (or possibly be used to trigger system quality checks).
 * Example of possible causes for this are: bad initial DUT factory trimming, large misalignement
 * beteween reference and DUT (even by design), wrong calibration procedure;
 *
 * \note: Though under these conditions, by default, the step is fully finalized and automatically
 * added to the used calibration buffer (\see BLC_calibrateForStepSafe for critical warning operations),
 * it is recomended to make sure of correctness of operations before moving forward
 * with the remaining calibration steps and to eventually restart the full calibration
 * procedure if problems are identified.
 */

/*! \brief No error detected during step finalization, the step is correctly added to the calibration buffer */
#define BLC_STEPSTATUS_NOERROR						0x0000

/*! \brief Calibration step errors mask */
#define BLC_STEPSTATUS_MASK_ERRORS					0x00FF

/*! \brief Calibration step warning mask */
#define BLC_STEPSTATUS_MASK_WARNINGS				0xFF00

/*! \brief Error! Motion detected duting a static calibration step, please check sensor stillness and try again*/
#define BLC_STEPSTATUS_MASK_NOTSTILL				0x0001

/*! \brief Error! Sensor out of range detected during a static or dynamic step, check your sensor configuration or try to reduce motion speed */
#define BLC_STEPSTATUS_MASK_CLIPPINGDETECTED		0x0002

/*! \brief Error! Anglular displacement traveled in the dynamic step is too small to estimate
 * dynamic calibration parameters, try to increase it
 */
#define BLC_STEPSTATUS_MASK_SMALLANGLE				0x0004

/*! \brief Error! Step time is too short to estimate calibration parameters, try to increase it */
#define BLC_STEPSTATUS_MASK_SHORTTIME				0x0008

/*!
 * \brief Error! Movement is too complex to estimate calibration parameters during a
 * dynamic calibration step, only single axis rotations are currently supported
 */
#define BLC_STEPSTATUS_MASK_OUTOFPLANEDETECTED		0x0010

/*!
 * \brief Error! The step is not correctly finalized nor added to the calibration buffer due to an error
 * or critical warning and therefore it should be repeated, please check correctness of step operations and try again
 */
#define BLC_STEPSTATUS_MASK_STEPNOTVALID			0x0080

/*!
 * \brief Warning! Large acc offset/gain corrections expected from the finalized static step,
 * please check correctness of step operations, sensor alignment and stillness
 */
#define BLC_STEPSTATUS_MASK_ACCBIASGAINALIGN_WARN	0x0100

/*!
 * \brief  Warning! Large gyro gain corrections expected from the finalized dynamic step,
 * please check correctness of step operations and sensor alignment
 */
#define BLC_STEPSTATUS_MASK_GYRGAINALIGN_WARN		0x0200

/*!
 * \brief Warning! Large gyro bias corrections expected from the finalized static step or 
 * large bias variation between beginning and end of a dynamic step
 * please check correctness of step operations and sensor stillness during no-motion assumptions
 */
#define BLC_STEPSTATUS_MASK_GYRBIAS_WARN			0x0400

/*!
 * \brief Finalize a calibration step providing the required reference value
 * \param [in, out] blc Pointer to a BLC instance
 * \param [in] referenceData Accelerometer and Gyroscope references used to estimate calibration parameters
 * \returns Status half-word that indicates if calibration errors have been identified or the step is finalized correctly (bit mask)
 *
 * \see BLC_calibrateForStepSafe()
 *
 * \see BLC_STEPSTATUS_NOERROR
 * \see BLC_STEPSTATUS_MASK_NOTSTILL
 * \see BLC_STEPSTATUS_MASK_CLIPPINGDETECTED
 * \see BLC_STEPSTATUS_MASK_SMALLANGLE
 * \see BLC_STEPSTATUS_MASK_SHORTTIME
 * \see BLC_STEPSTATUS_MASK_OUTOFPLANEDETECTED
 * \see BLC_STEPSTATUS_MASK_STEPNOTVALID
 * \see BLC_STEPSTATUS_MASK_ACCGAINALIGN_WARN
 * \see BLC_STEPSTATUS_MASK_GYRGAINALIGN_WARN
 * \see BLC_STEPSTATUS_MASK_GYRBIAS_WARN
 */
uint16_t BLC_calibrateForStep(struct BLC* blc, struct BLC_ReferenceData const* referenceData);

/*!
 * \brief Finalize a calibration step providing the required reference value with raised importance level of specific critical warnings
 * \param [in, out] blc Pointer to a BLC instance
 * \param [in] referenceData Accelerometer and Gyroscope references used to estimate calibration parameters
 * \param [in] criticalWarnings bit mask used to identify warnings with raised importance level that should not allow step finalization
 * \returns Status half-word that indicates if calibration errors have been identified or the step is finalized correctly (bit mask)
 *
 * Similarly to BLC_calibrateForStep(), this function finalizes the current step and
 * add the available information to the calibration data. Additionally, the function raises
 * the immportance level of provided critical warnings (in form of a bit mask) to report
 * a BLC_STEPSTATUS_MASK_STEPNOTVALID and to avoid automatically adding the step to the
 * final calibration buffer in case such warnings are identified.
 * \note: This for example enable the possibility of repeating such steps.
 *
 * \see BLC_calibrateForStep()
 *
 * \see BLC_STEPSTATUS_MASK_ACCGAINALIGN_WARN
 * \see BLC_STEPSTATUS_MASK_GYRGAINALIGN_WARN
 * \see BLC_STEPSTATUS_MASK_GYRBIAS_WARN
 * \see BLC_STEPSTATUS_MASK_STEPNOTVALID
 */
uint16_t BLC_calibrateForStepSafe(struct BLC* blc, struct BLC_ReferenceData const* referenceData, uint16_t criticalWarnings);

/*!
 * \brief Finalize a calibration step providing the required reference value.
 * \param [in, out] blc Pointer to a BLC instance
 * \param [in] referenceData Accelerometer and Gyroscope references used to estimate calibration parameters
 * \returns Status half-word that indicates if calibration errors have been identified or the step is finalized correctly (bit mask) 
 * and ready to be added to the used measurements.
 * \note Differently from BLC_calibrateForStep() this function does NOT add automatically the step to the used calibration data
 *
 * This function finalizes the current step and fills-in all the step statistics for later access. 
 * This function is REQUIRED to succeed (returned Status with only non-critical warnings and no errors) before
 * BLC_addStepToMeasurements() can be succeffully called. Non-finalized steps will report BLC_STEPSTATUS_MASK_STEPNOTVALID
 * Step statistics will be available after this function is called and can be retrieved using BLC_getStepStats()
 *
 * \see BLC_finalizeStepSafe()
 * \see BLC_getStepStats()
 * \see BLC_calibrateForStep()
 * \see BLC_calibrateForStepSafe()
 *
 * \see BLC_STEPSTATUS_NOERROR
 * \see BLC_STEPSTATUS_MASK_NOTSTILL
 * \see BLC_STEPSTATUS_MASK_CLIPPINGDETECTED
 * \see BLC_STEPSTATUS_MASK_SMALLANGLE
 * \see BLC_STEPSTATUS_MASK_SHORTTIME
 * \see BLC_STEPSTATUS_MASK_OUTOFPLANEDETECTED
 * \see BLC_STEPSTATUS_MASK_STEPNOTVALID
 * \see BLC_STEPSTATUS_MASK_ACCGAINALIGN_WARN
 * \see BLC_STEPSTATUS_MASK_GYRGAINALIGN_WARN
 * \see BLC_STEPSTATUS_MASK_GYRBIAS_WARN
 */
uint16_t BLC_finalizeStep(struct BLC* blc, struct BLC_ReferenceData const* referenceData);

/*!
 * \brief Finalize a calibration step providing the required reference value.
 * \param [in, out] blc Pointer to a BLC instance
 * \param [in] referenceData Accelerometer and Gyroscope references used to estimate calibration parameters
 * \returns Status half-word that indicates if calibration errors have been identified or the step is finalized correctly (bit mask) 
 * and ready to be added to the used measurements.
 *
 * Similarly to BLC_finalizeStep(), this function finalizes the current step and fills in all the step statistics. 
 * Additionally, the function raises the immportance level of provided critical warnings (in form of a bit mask) to report
 * a BLC_STEPSTATUS_MASK_STEPNOTVALID.
 *
 * \see BLC_finalizeStepSafe()
 * \see BLC_getStepStats()
 * \see BLC_calibrateForStepSafe()
 *
 * \see BLC_STEPSTATUS_NOERROR
 * \see BLC_STEPSTATUS_MASK_WARNINGS
 * \see BLC_STEPSTATUS_MASK_STEPNOTVALID
 */
uint16_t BLC_finalizeStepSafe(struct BLC* blc, struct BLC_ReferenceData const* referenceData, uint16_t criticalWarnings);

/*!
 * \brief Add a successfully finalized step (no errors or critical warkings identified) to the set of available 
 * measurements used from the calibration routine.
 * \param [in, out] blc Pointer to a BLC instance
 * \param [in] referenceData Accelerometer and Gyroscope references used to estimate calibration parameters
 * \returns Status half-word that indicates if calibration errors have been identified or the step is finalized correctly (bit mask) 
 * and ready to be added to the used measurements.
 * \note BLC_finalizeStep() MUST be called before this function
 * 
 * \see BLC_finalizeStep()
 * \see BLC_finalizeStepSafe()
 * \see BLC_getStepStats()
 * \see BLC_calibrateForStep()
 * \see BLC_calibrateForStepSafe()
 *
 * \see BLC_STEPSTATUS_NOERROR
 * \see BLC_STEPSTATUS_MASK_WARNINGS
 * \see BLC_STEPSTATUS_MASK_STEPNOTVALID
 */
uint16_t BLC_addStepToMeasurements(struct BLC* blc, struct BLC_ReferenceData const* referenceData);

/*! \}
*/

/*!
 * \brief Set the sample frequency used to derive accurate timing
 * \param [in, out] blc Pointer to a BLC instance
 * \param [in] rate Used sample frequency
 */
void BLC_setSampleRate(struct BLC* blc, float freq);

/*!
 * \brief Get the FIS1100 sensor configuration required for the board level calibration procedure
 * \param [in, out] blc Pointer to a BLC instance
 * \param [out] conf Pointer to the Fis1100Config structure in which to receive the configuration
 */
void BLC_getSensorConfig(struct BLC* blc, struct Fis1100Config* conf);

/*!
 * \brief Get statistics from the most recent calibration step
 * \param [in, out] blc Pointer to a BLC instance
 * \param [out] stats Pointer to the step statistics structure in which all the information is copied
 * \returns size expressed in bytes of the copied data
 *
 * Statistics information from the most recent calibration step are copied into the passed structure.
 */
size_t BLC_getStepStats(struct BLC* blc, struct BLC_StepStatistics* stats);

/*!
 * \brief Parse a calibration buffer for easy access (and use) of accelerometer and gyrososcope gain and offset information
 * \param [in] calDataBuffer Pointer to the calibration buffer to be parsed
 * \param [out] cal Pointer to the gain and offset calibration structure to be filled
 *
 * \see BLC_getCalibration()
*/
void BLC_getFullCalibrationFromBuffer(uint8_t const* calDataBuffer, struct Fis1100_fullCalibration* cal);

/*!
 * \brief Get small angle non-orthogonality estimates from gain matrix, assuming small angle misalignement.
 * \param [in] gain full 3x3 matrix, expressed in form of 9 elements single dimension array 
 * \param [out] angleDegrees 3 elements array containing non-orthogonality estimates for the given gain matrix
 *
 * \note Gain matrices can be parsed from calibration results buffer using BLC_getFullCalibrationFromBuffer()
 * \see BLC_getFullCalibrationFromBuffer()
*/
void BLC_getNonOrthogonalityFromGain(float* gain, float* angleDegrees);

#ifdef __cplusplus
}
#endif // extern "C"

#endif // __BOARDLEVELCAL_H
