/*
 * ConfigManagerV3.h
 *
 * Functionality for configuring target device.
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



#include "ConfigManager.h"
#include "HalExecCommand.h"
#include "FileFuncImpl.h"

namespace TI
{
	namespace DLL430
	{
		class FetHandleV3;
		class FileFuncImpl;
		class UpdateManager;
		class FetHandleManager;
		class EnergyTraceManager;

		class ConfigManagerV3 : public ConfigManager, boost::noncopyable
		{
		public:
			ConfigManagerV3(FetHandleV3*, FetHandleManager*);
			~ConfigManagerV3();

			void init();
			
			VersionInfo getHalVersion() const;

			bool isUpdateRequired() const;

			bool setDeviceVcc(uint16_t vcc);
			
			uint16_t getDeviceVcc() const;
				
			void setJtagMode(ConfigManager::jtagMode mode);
			ConfigManager::jtagMode getInterfaceMode() const;
			uint16_t getExternalVcc() const;
			int16_t start();
			int16_t start(const std::string& pwd, uint32_t deviceCode);
			bool stop();

			bool reset(bool vcc, bool nmi, uint16_t JtagId, uint32_t rstHalId);
			
			bool firmWareUpdate(const char * fname, UpdateNotifyCallback callback, bool* coreUpdate);

			void setPassword(const string& pwd);
			bool setDeviceCode(uint32_t deviceCode);
			bool setJtagSpeed(JTAG_4WIRE_SPEED speedJtag, JTAG_2WIRE_SPEED speedSbw);
			bool jtagErase(uint16_t eraseKey);
			void setEnergyTraceManager(EnergyTraceManager*);
			bool isEnergyTraceSupported();
			void setCurrentDrive(uint32_t value);
			bool configureJtagSpeed(uint32_t speed);
			
			bool freqCalibrationEnabled() const;
			bool ulpDebugEnabled() const;
			void setUlpDebug(bool ulp); 
			long MSP430I_MagicPattern(uint16_t ifMode);

			bool configureOverCurrent(bool state);

		private:
			FetHandleV3* parent;
			UpdateManager* updateManagerFet;
			uint16_t vcc;
			enum ConfigManager::jtagMode mode;
			EnergyTraceManager* mEnergyTraceManager;

			HalExecCommand updateCmd;

			bool setVccEzFet(uint16_t vcc);
			bool setVccMspFetUif(uint16_t vcc);
			bool setVccMspFET(uint16_t vcc);

			std::string password;
			uint32_t deviceCode;
			uint32_t mhighres;
			bool freqCalibration;
			bool ulpDebug;
		};
	}
}
