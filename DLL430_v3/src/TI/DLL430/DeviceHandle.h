/*
 * DeviceHandle.h 
 *
 * Interface for providing pointers to classes necessary to communicate with one device.
 *
 * Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include "MemoryManager.h"
#include "DebugManager.h"
#include "EM/EmulationManager/IEmulationManager.h"
#include "FileFunc.h"


namespace TI
{
	namespace DLL430
	{
		class DeviceHandle
		{
		public:
			virtual ~DeviceHandle () {};

			/** \brief the memory manager
			 *
			 * \return pointer to a memory manager instance
			 */
			virtual MemoryManager* getMemoryManager () = 0;

			/** \brief the emulation manager
			 *
			 * \return shared_pointer to a emulation manager instance
			 */
			virtual EmulationManagerPtr getEmulationManager () = 0;

			/** \brief the debug manager
			 *
			 * \return pointer to a debug manager instance
			 */
			virtual DebugManager* getDebugManager () = 0;

			/** \brief file handle
			 *
			 * \return pointer to the file referece instance
			 */
			virtual FileFunc* getFileRef() = 0;

			/** \brief write segment data to device
			 *
			 * the data of a file is written to the device using the memory manager
			 *
			 * \return true on success
			 */
			virtual bool writeSegments() = 0;
			/** \brief identify device by stopping and sending into LPM4 or BSL memory
			 *
			 * depending on the family the device is detected
			 *
			 * \return true on success
			 */
			virtual long magicPatternSend(uint16_t ifMode) =0;

			/** \brief identify device
			 *
			 * depending on the family the device is detected
			 *
			 * \return true on success
			 */
			virtual long identifyDevice (uint32_t activationKey, bool afterMagicPattern) = 0;

			/** \brief return description string of device
			 *
			 * return the description string of the device as defined in database
			 *
			 * \return string
			 */
			virtual const std::string & getDescription() = 0;

			/** \brief write segment data to device
			 *
			 * verify the data of a file with written data using the memory manager
			 *
			 * \return true on success
			 */
			virtual bool verifySegments() = 0;

			/** \brief reset the device (power-on-reset)
			 * 
			 * \return true if every step was successful, else false
			 */
			virtual bool reset () = 0;

			/** \brief stores the device ID which was read out from the target
			 * 
			 * \param id which was given back from the target
			 */
			virtual void setDeviceId (long id) = 0;

			/** \brief returns the jtag id of the identified device
			 * 
			 * \return stored jtag id, defaults to 0x00 if no detection happened
			 */
			virtual uint8_t getJtagId() = 0;

			/** \brief checks if the device is locked (fuse blown)
			 * 
			 * \return true if device is locked
			 */
			virtual bool isJtagFuseBlown() =0;

			/** \brief retrieve sub id from device
			 * 
			 * \param info_len info data size of device
			 * \param deviceIdPtr address of info data
			 * \param pc program counter to restore after reading
			 *
			 * \return sub id
			 */
			virtual int16_t getSubID(uint32_t info_len, uint32_t deviceIdPtr , uint32_t pc) = 0;


			/** \brief returns the address of the TLV area of the device
			 * 
			 * \return stored address, defaults to 0x00 if not avaiable
			 */
			virtual uint32_t getDeviceIdPtr() = 0;

			/** \brief returns the EEM Id
			 * 
			 * \return value of EEM address 0x86 (EEMVER), defaults to 0
			 */
			virtual uint32_t getEemVersion() = 0;

			/** \brief JTAG mode */
			enum jtagMode {
				JTAG_MODE_4WIRE = 0,		/**< 4-wire JTAG directly */
				JTAG_MODE_SPYBIWIRE = 1,	/**< SpyBiWire (2-wire JTAG) */
				JTAG_MODE_4AFTER2 = 2,		/**< use 4-wire JTAG after entry sequence with SpyBiWire */
			};

			/** \brief secure the device by blowing the fuse
			 *
			 * \return true if securing the device successfully, else false
			 */
			virtual bool secure () = 0;

			/** \brief return whether the device has FRAM
			 *
			 * \return true if the device has FRAM
			 */
			virtual bool hasFram() const = 0;

			/** \brief return whether the device supports LPMx.5
			 *
			 * \return true if the device supports LPMx.5
			 */
			virtual bool hasLPMx5() const = 0;

			/** \brief disable device halt when waking from LPMx.5
			 */
			virtual void disableHaltOnWakeup() = 0;

			/** \brief is EEM module accessible while in LPM
			 */
			virtual bool eemAccessibleInLpm() const = 0;

			/** \brief is EnergyTrace (ET7) supported by selected device and revision
			 */
			virtual bool deviceSupportsEnergyTrace() const = 0;

			/** \brief check id of corresponding HAL function
			 */
			virtual uint32_t checkHalId(uint32_t base_id) const= 0;

		protected:

		private:

		};

	}
}
