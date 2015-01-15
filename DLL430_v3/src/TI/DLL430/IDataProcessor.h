/*
 * IDataProcessor.h
 *
 * Generic Interface class for providing a module that can take serial input data and produce output data.
 *
 * Copyright (C) 2007 - 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

namespace TI
{
	namespace DLL430
	{
		class IDataProcessor
		{
		public:
			/**
			 * \brief Constructor creates the two buffers
			 * \param size The desired size of the buffers
			 */
			IDataProcessor()
			{
			}

			virtual ~IDataProcessor()
			{
			}

			/**
			 * \brief Resets the internal state of the processor
			 */
			virtual void Reset() = 0;

			/**
			 * \brief Add data to be processed
			 * \param data The data element to be written
			 * \return Indicates whether new output data is available for reading
			 */
			virtual bool AddData(void *data, size_t size) = 0;

			/**
			 * \brief Get the pointer for reading data from the buffer
			 * \return A pointer to the current read buffer
			 */
			virtual void* GetReadBufferPtr() = 0;

			/**
			 * \brief Get the buffer size
			 * \return the size
			 */
			virtual size_t GetReadBufferSize() = 0;

			virtual void setTimerStep(uint32_t step) {}

			virtual void setResistorValues(double *resistorValues) {}

			virtual void setCalibrationValues(double *calibrationValues, uint16_t vcc) = 0;

			virtual void setPrevSample(double){};
		};
	}
}
