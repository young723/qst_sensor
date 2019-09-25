/*!
 * \file
 * \version 1.1.1 rev. 55989
 *
 * \copyright
 * Copyright (c) 2016 Fairchild Semiconductor Corporation or subsidiaries
 * worldwide. All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. 	Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 * 2. 	Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 * 3. 	Neither the names of the copyright holders nor the names of their
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __SENSORFUSIONCONFIG_H
#define __SENSORFUSIONCONFIG_H

//#include "fisimu_config.h"
#include "./fis210x/fis210x.h"

/*!
 * \defgroup SENSOR_FUSION_CONFIG Sensor fusion configuration
 * \ingroup XKF3
 * \brief Sensor fusion configuration options.
 *
 * The sensor fusion configuration structure is used to configure the FIS
 * hardware in the [CMSIS-RTOS reference implementation](\ref SENSOR_FUSION),
 * and is also used to provide the [XKF3 library](\ref XKF3_INTERFACE) with
 * details about the FIS configuration.
 * \{
 */


/*!
 * \brief Sensor fusion output scenarios.
 *
 * The XKF3 sensor fusion library supports two different output scenarios with
 * respect to the way heading (yaw) is computed. Both use a magnetic field
 * model describing the short-term magnetic variation as a combination of
 * spatial and temporal disturbances on top of the earth magnetic field to
 * compute a measure of heading. This means a magnetic field changes as a
 * function of position (e.g. moving towards a source of magnetic distortion)
 * and/or with time (a source of magnetic distortion moving with respect to the
 * device). These models allow XKF3 to observe the heading under heavy magnetic
 * distortions, as well as auto-calibrate the gyroscope bias and other
 * parameters.
 */
enum SensorFusionScenario
{
	/*!
	 * \brief 3D orientation with respect to a North referenced coordinate
	 * system for navigation, mapping, augmented reality, etc.

	 * This scenario aims to track the true North where possible. In
	 * case of a non-initialized magnetometer auto-calibration in XKF3 at
	 * startup or varying magnetic disturbances, strap-down integration will
	 * keep the tracking solution stable as long as gyro integration allows
	 * for. However in case the magnetic field cannot be used for prolonged
	 * period of time, the tracking solution will converge to a solution based
	 * on the current local magnetic field. As soon as the magnetic North can
	 * be observed again with some accuracy, the heading estimate will converge
	 * as quickly as possible (with a jump) to true North.
	 */
	FusionScenario_Navigation,

	/*!
	 * \brief 3D orientation un-referenced to North for gaming, UI, camera
	 * stabilization, etc.
	 *
	 * This scenario aims to generate a 3D tracking solution with a very
	 * low heading drift, giving a smooth heading output, avoiding the fast
	 * heading adjustments that may occur in the navigation scenario. It is
	 * fully immune to magnetic disturbances. This user scenario works without
	 * magnetometer input but accuracy may improve if magnetometer data is
	 * available. The heading will exhibit slow but long term drift by design,
	 * but the drift will be much smaller than the drift obtained by simple
	 * gyroscope integration. The heading is initialized using the local
	 * magnetic field direction, if available.
	 */
	FusionScenario_Gaming,

	/*!
	 * \brief 3D orientation with respect to a North referenced coordinate
	 * system for virtual pointer applications.
	 *
	 * This scenario uses the same North tracking system as the
	 * FusionScenario_Navigation, however corrections to the output are
	 * proportional to the amount of user motion. This results in a stable
	 * orientation estimate when the user holds the sensor still to point
	 * at a target; and a natural feel for the user as small motions directly
	 * affect the output while larger, more rapid, motions will seamlessly
	 * correct for accumulated errors.
	 */
	FusionScenario_Pointer
};


/*!
 * \brief Sensor fusion output data rate configuration.
 *
 * The maximum output rate of the sensor fusion system is limited by the FIS
 * output data rate. The fusion output rate can however be lowered further to
 * reduce the number or calculations performed.
 */
enum SensorFusionOdr
{
	/*! \brief Disable sensor fusion processing. */
	SensorFusion_disable = 0,
	/*! \brief Limit sensor fusion output to 8Hz. */
	SensorFusion_8Hz = 8,
	/*! \brief Limit sensor fusion output to 16Hz. */
	SensorFusion_16Hz = 16,
	/*! \brief Limit sensor fusion output to 32Hz. */
	SensorFusion_32Hz = 32,
	/*! \brief Limit sensor fusion output to 64Hz. */
	SensorFusion_64Hz = 64,
	/*! \brief Limit sensor fusion output to 128Hz. */
	SensorFusion_128Hz = 128,
	/*! \brief Limit sensor fusion output to 256Hz. */
	SensorFusion_256Hz = 256,
	/*! \brief Limit sensor fusion output to 512Hz. */
	SensorFusion_512Hz = 512,
	/*! \brief Limit sensor fusion output to 1024Hz. */
	SensorFusion_1024Hz = 1024
};

/*!
 * \brief Sensor fusion configuration parameters.
 *
 * Recommended default values can be configured using
 * SensorFusionConfig_default().
 */
struct SensorFusionConfig
{
	struct FisImuConfig fisConfig;

	/*!
	 * \brief Sensor fusion maximum output data rate.
	 *
	 * Set the maximum rate in Hz at which new sensor fusion output values are
	 * computed. The output rate of the fusion system is determined as the
	 * maximum of the gyroscope or AttitudeEngine ODR and this parameter.
	 */
	enum SensorFusionOdr fusionOdr;

	/*!
	 * \brief The sensor fusion scenario.
	 */
	enum SensorFusionScenario filterScenario;

	/*!
	 * \brief Use external magnetometer instead of FIS slave magnetometer.
	 * \warning If this is set to \c true the externalMagnetometerStdDev
	 * field must be set to match the noise standard deviation of the
	 * external magnetometer hardware.
	 */
	bool useExternalMagnetometer;

	/*!
	 * \brief The standard deviation of the external magnetometer noise in micro-Tesla.
	 *
	 * The magnetometer noise should be measured by recording data from the
	 * magnetometer in the final design in an undisturbed magnetic field and
	 * calculating the sample standard deviation.
	 *
	 * This value could be generated for each device as part of a production
	 * calibration procedure, or an average value used.
	 */
	float externalMagnetometerStdDev;

	/*!
	 * \brief Determines if the velocity output is valid.
	 */
	bool calculateVelocity;

};

#ifdef __cplusplus
extern "C" {
#endif

bool SensorFusionConfig_validate(struct SensorFusionConfig* config);
void SensorFusionConfig_default(struct SensorFusionConfig* config);

#ifdef __cplusplus
}
#endif // extern "C"

/*! \} */

#endif // __SENSORFUSIONCONFIG_H
