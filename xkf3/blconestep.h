/*!
 * \file
 * \version 1.1.1 rev. 55932
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

#ifndef __BOARDLEVELCALONESTEP_H
#define __BOARDLEVELCALONESTEP_H

#include <stddef.h>
#include <stdbool.h>

/*!
 * \struct BLC
 * \brief The Board Level Calibration (BLC) structure.
 */
struct BLC;

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \defgroup BOARD_LEVEL_CAL Board level calibration
 * \ingroup XKF3
 * \brief Fast calibration algorithm for production support.
 *
 * The board level calibration algorithm estimates sensor offsets based on
 * measurements in a single static orientation. To use the board level
 * calibration routine the PCB assembly should be held stationary such that
 * the Z-axis of the FIS is aligned vertically. The alignment of the Z-axis is
 * crucial to the algorithm performance, and should be within 3 degrees of
 * vertical for acceptable performance. During the calibration procedure the
 * PCB must remain stationary.
 *
 * \note The board level calibration routine is intended for use a part of the
 * production process, and is not designed for use by end users.
 *
 * \{
 */

/*!
 * \name BLC information
 * \{
 */
/*!
 * \brief Return the BoardLevelCal version as (major, minor, revision).
 *
 * \param[out] major a pointer to the major version number.
 * \param[out] minor a pointer to the minor version number.
 * \param[out] revision a pointer to the revision number.
 */
void BLC_getVersion(int* major, int* minor, int* revision);
/*! \} */

/*!
 * \name BLC memory management
 * \{
 */
/*!
 * \brief Get the amount of memory required to store the BLC structure.
 * \returns The required size expressed in bytes.
 *
 * The BLC structure is used both to store the filter state and as a
 * temporary data buffer space.
 */
size_t BLC_getMemSize(void);

/*!
 * \brief Initialize and return the BLC calibration structure.
 * \param mem Pointer to memory area to use for BLC calibration structure.
 *
 * The memory provided to the initialization function must be at least as
 * large at the value returned by BLC_getMemSize().
 */
struct BLC* BLC_initOneStep(void* mem);

/*!
 * \brief Destroy the BLC calibration structure.
 * \param[in] blc The BLC instance to destroy.
 */
void BLC_destruct(struct BLC* blc);
/*! \} */

/*!
 * \name BLC sensor configuration
 * \{
 */
/*!
 * \brief Get the FIS sensor configuration required for the board level calibration procedure.
 * \param [in, out] blc Pointer to a BLC instance.
 * \param [out] conf Pointer to the FisImuConfig structure in which to receive
 *     the configuration.
 *
 * The resulting FisImuConfig should be applied to the FIS using the
 * FisImuConfig_apply() function to configure the FIS correctly for the
 * calibration procedure.
 */
//void BLC_getSensorConfig(struct BLC* blc, struct FisImuConfig* conf);
/*! \} */

/*!
 * \name BLC data handling
 * \{
 */
/*!
 * \brief Reset the internal state of the BLC calibration structure.
 * \param [in, out] blc Pointer to a BLC instance.
 */
void BLC_resetOneStep(struct BLC* blc);

/*!
 * \brief Handle a new raw sample from the FIS.
 * \param [in, out] blc Pointer to a BLC instance.
 * \param [in] sample Pointer to FisImuRawSample containing the sample data to
 *      handle.
 *
 * This function should be called once for each new available sample.
 */
//void BLC_handleSample(struct BLC* blc, struct FisImuRawSample const* sample);

/*!
 * \brief Check to see if the current calibration step is complete.
 * \param [in, out] blc Pointer to a BLC instance.
 * This function must be used to make sure the step is completed before starting a new step.
 */
bool BLC_isStepComplete(struct BLC* blc);

/*! \} */

/*!
 * \name BLC output generation
 * \{
 */
/*!
 * \brief Run calibration procedure to estimate calibration parameters.
 * \param [in, out] blc Pointer to a BLC instance.
 * \param zUp \c true if the positive Z-axis of the FIS was facing up, \c false if the positive Z-axis was facing down.
 * \returns \c true if the calibration is succesful, \c false if motion was detected.
 */
bool BLC_calibrateOneStep(struct BLC* blc, bool zUp);

/*!
 * \brief Get BLC offset calibration results.
 * \param [in, out] blc Pointer to a BLC instance.
 * \param [out] cal Pointer to FIS100 offset calibration structure to put result into.
 *
 * The offset calibration results are copied into the passed structure. This
 * function should only be called after BLC_isStepComplete() has returned \c
 * true to ensure that processing is complete and that the resulting data is
 * valid.
 */
//void BLC_getOffsetCalibration(struct BLC* blc, struct FisImu_offsetCalibration* cal);
/*! \} */

/*! \} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // __BOARDLEVELCAL_H
