/*
 * DeviceHandleV3.h
 *
 * Communication with target device.
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

#include "DeviceHandle.h"
#include "MemoryManagerV3.h"
#include "DebugManagerV3.h"
#include "DeviceInfo.h"
#include "FileFuncImpl.h"
#include "WatchdogControl.h"
#include "FuncletCode.h"
#include "DeviceChainInfo.h"

namespace TI
{
	namespace DLL430
	{
		class FetHandleV3;
		class DeviceInfo;
		class DeviceChainInfo;
		class FetControl;
		class HalCommand;
		class ClockCalibration;

		class DeviceHandleV3  : public DeviceHandle, boost::noncopyable
		{
		public:
			DeviceHandleV3 (FetHandleV3*, DeviceChainInfo*, uint32_t deviceCode);
			~DeviceHandleV3 ();

			EmulationManagerPtr getEmulationManager();
			MemoryManagerV3* getMemoryManager ();
			DebugManagerV3* getDebugManager ();
			FetHandleV3* getFetHandle () { return parent; }
			ClockCalibration* getClockCalibration() { return clockCalibration; }

			FileFunc* getFileRef();
			bool writeSegments();
		
			/// \brief Run the boot code writing the the specified command in the mailbox first
			/// \param command[in] Command to be put in the mailbox
			/// \return True if bootcode execution succeeded
			long magicPatternSend(uint16_t ifMode);
			long identifyDevice (uint32_t activationKey, bool afterMagicPattern);
			const std::string & getDescription();
			bool secure ();
			bool reset ();

			bool verifySegments();

			FetControl* getControl ();

			bool send (HalExecCommand &command);

			DeviceChainInfo* getDevChainInfo ();

			void setWatchdogControl (boost::shared_ptr<WatchdogControl>);
			boost::shared_ptr<WatchdogControl> getWatchdogControl() const;
			uint8_t getDeviceJtagId();
			uint8_t getJtagId();
			uint32_t getDeviceIdPtr();
			uint32_t getEemVersion();
			bool isJtagFuseBlown();
			int16_t getSubID(uint32_t info_len, uint32_t deviceIdPtr , uint32_t pc);
			uint32_t getDeviceCode() const;

			void setDeviceId (long id);

			uint32_t checkHalId(uint32_t base_id) const;

			const FuncletCode& getFunclet(FuncletCode::Type funclet);

			bool supportsQuickMemRead() const;
			uint16_t getMinFlashVcc() const;
			bool hasFram() const;
			bool hasLPMx5() const;

			void disableHaltOnWakeup();

			bool eemAccessibleInLpm() const;

			bool deviceSupportsEnergyTrace() const;

		protected:

		private:
			FetHandleV3* parent;
			DeviceChainInfo* deviceChainInfo;
			EmulationManagerPtr emulationManager;
			MemoryManagerV3* memoryManager;
			DebugManagerV3* debugManager;
			FileFuncImpl* fileManager;
			ClockCalibration* clockCalibration;

			uint16_t minFlashVcc;
			bool hasTestVpp;
			bool quickMemRead;
			bool deviceHasFram;
			bool deviceHasLPMx5;
			ClockSystem clockSystem;

			uint8_t jtagId;
			uint32_t deviceIdPtr;
			uint32_t eemVersion;
			enum DeviceHandle::jtagMode mode;
			uint32_t deviceCode;

			uint32_t powerTestRegDefault;
			uint16_t powerTestReg3VDefault;

			boost::shared_ptr<WatchdogControl> wdt;
			DeviceInfo::function_map_type map;
			DeviceInfo::funclet_map_type funcletTable;

			std::string description;

			typedef boost::array<uint8_t, DeviceInfo::nrUsedClockModules> EtwCodes;
			EtwCodes etwCodes;

			void configure (const DeviceInfo* info);
			long getDeviceIdentity(uint32_t activationKey, uint32_t* pc, uint32_t* sr, bool afterMagicPattern);

			bool sendDeviceConfiguration(uint32_t parameter, uint32_t value);
		};
	}
}
