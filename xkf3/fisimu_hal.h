/*!
 * \file
 * \version 1.1.1 rev. 55984
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

#ifndef __FISIMU_HAL_H
#define __FISIMU_HAL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \brief FIS product version.
 */
enum Fis_ImuVersion
{
	/*! \brief FIS1100. */
	FisImu_Fis1100		= 0x01,
	/*! \brief FIS2108. */
	FisImu_Fis2108		= 0x04,
	/*! \brief FIS2100. */
	FisImu_Fis2100		= 0x08
};

/*!
 * \addtogroup FISDRIVER
 * \{
 */

/*!
 * \brief FIS interrupt lines.
 */
enum FisImu_Interrupt
{
	/*! \brief FIS INT1 line. */
	Fis_Int1 = (0 << 6),
	/*! \brief FIS INT2 line. */
	Fis_Int2 = (1 << 6)
};

/*!
 * \brief FIS interrupt events.
 */
enum FisImu_InterruptEvent
{
	/*! \brief Positive edge (line went from low to high). */
	FisInt_positiveEdge,
	/*! \brief Line is high. */
	FisInt_high,
	/*! \brief Negative edge (line went from high to low). */
	FisInt_negativeEdge,
	/*! \brief Line is low. */
	FisInt_low
};

/*!
 * \brief FIS Hardware Abstraction Layer functions.
 *
 * The functions defined in the HAL must be implemented for the target platform.
 */
struct FisImuHal
{
	/*!
	 * \brief Write data to the FIS.
	 * \param data Pointer to the data to write.
	 * \param dataLength The number of bytes of data in \a data buffer.
	 *
	 * Implementation should perform a single blocking write transaction to
	 * write the contents of the passed data buffer to the FIS. For example,
	 * when using SPI the chip select should be asserted prior to the
	 * transaction and then deasserted at the end.
	 *
	 * \note The passed buffer includes the register address as the first byte
	 * of data.
	 *
	 * \remark Driver assumes that write transactions block until the write is
	 * completed. If using DMA then this function must monitor for completion of
	 * the transaction before returning.
	 */
	void (*writeData)(const uint8_t* data, uint8_t dataLength);

	/*!
	 * \brief Read data from the FIS.
	 * \param address The register address to read data from.
	 * \param data Pointer to buffer to read data into.
	 * \param dataLength The number of bytes to read.
	 *
	 * Implementation should perform a single blocking read transaction.
	 *
	 * For SPI the implementation should send the \a address to the device
	 * followed by \a dataLength zeros. The first byte of the returned data (the
	 * value received when sending the address byte) should be discarded and the
	 * remainder placed in the \a data buffer.
	 *
	 * For I2C the implementation should first write the \a address to the
	 * device and then read \a dataLength bytes into the \a data buffer. A
	 * repeated start operation should be used between the write and read parts
	 * of the transaction.
	 *
	 * \remark Driver assumes that read transactions block until the read is
	 * completed. If using DMA then this function must monitor for completion of
	 * the transaction before returning.
	 */
	void (*readData)(uint8_t address, uint8_t* data, uint8_t dataLength);

	/*!
	 * \brief Set the state of the hardware reset line.
	 * \param assert \c true if reset should be asserted (high), else \c false.
	 */
	void (*assertReset)(bool assert);

	/*!
	 * \brief Indicate if FIS INT1 pin is asserted (high).
	 * \return \c true if asserted, else \c false.
	 */
	bool (*int1Asserted)(void);

	/*!
	 * \brief Indicate if FIS INT2 pin is asserted (high).
	 * \return \c true if asserted, else \c false.
	 */
	bool (*int2Asserted)(void);

	/*!
	 * \brief Wait for a FIS interrupt event to occur.
	 * \param interrupt The interrupt line to monitor.
	 * \param event The interrupt event to wait for.
	 *
	 * This function should block execution until the specified \a event has
	 * occurred.
	 *
	 * The FisImu_busyWaitForEvent() function can be used as an implementation
	 * if it is acceptable to busy wait.
	 *
	 */
	void (*waitForEvent)(enum FisImu_Interrupt interrupt, enum FisImu_InterruptEvent event);

	/*!
	 * \brief Delay for \a delay microseconds.
	 * \param delay The minimum required delay in microseconds.
	 */
	void (*delayMicroseconds)(uint32_t delay);

	/*!
	 * \brief True if HAL uses SPI for communication, else false..
	 */
	bool spiMode;

	/*! \brief The FIS product version in use. */
	enum Fis_ImuVersion deviceVersion;
};

/*! \} */

#ifdef __cplusplus
}
#endif // extern "C"

#endif // __FISIMU_HAL_H
