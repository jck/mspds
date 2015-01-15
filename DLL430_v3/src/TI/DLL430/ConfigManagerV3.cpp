/*
 * ConfigManagerV3.cpp
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

#include <pch.h>
#include "VersionInfo.h"
#include "FetHandleV3.h"
#include "ConfigManagerV3.h"
#include "DeviceDbManagerExt.h"
#include "DeviceInfo.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "Record.h"
#include "PinSequence.h"
#include "DeviceHandleV3.h"
#include "UpdateManagerFet.h"
#include "UpdateManagerMSP_FET430.h"
#include "UpdateManagerDummy.h"
#include "FetHandleManager.h"
#include "ConfigureParameters.h"
#include "EnergyTraceManager.h"
#include "JtagId.h"

using namespace TI::DLL430;
using namespace std;

ConfigManagerV3::ConfigManagerV3 (FetHandleV3* parent, FetHandleManager* fhManager)
	: parent(parent)
	, vcc(0)
	, mode(ConfigManager::JTAG_MODE_4WIRE)
	, deviceCode(0)
	, freqCalibration(true)
	, mEnergyTraceManager(0)
	, mhighres(0)
	, ulpDebug(false)
{
	updateCmd.setTimeout(20000);
	updateManagerFet = 0;

	FetControl* control = this->parent->getControl();
	if(control->getFetToolId()== eZ_FET_WITH_DCDC || control->getFetToolId() == eZ_FET_NO_DCDC
		|| control->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		this->updateManagerFet = new UpdateManagerFet(parent, this, fhManager);
	}
	else if(control->getFetToolId()== MSP_FET430)
	{
		this->updateManagerFet = new UpdateManagerMSP_FET430(parent, this);
	}
	else
	{
		this->updateManagerFet = new UpdateManagerDummy();
	}
}

ConfigManagerV3::~ConfigManagerV3()
{
	FetControl* control=this->parent->getControl();
	
	// send reset command to FET
	if((control != NULL)&&(control->hasCommunication()))
	{
		this->stop();
	}
	delete updateManagerFet;
}

bool ConfigManagerV3::isUpdateRequired() const 
{
	return updateManagerFet->isUpdateRequired();
}

bool ConfigManagerV3::isEnergyTraceSupported()  
{
	FetControl* fet = this->parent->getControl();
	const uint16_t toolId = fet ? fet->getFetToolId() : 0;
	return toolId == eZ_FET_WITH_DCDC || toolId == MSP_FET_WITH_DCDC;
}

void ConfigManagerV3::init ()
{
	string tag;
	string value;

	// If any kind of update is required do not configure any JTAG or SBW Speed-> perhaps you will use a null pointer
	if(updateManagerFet->isUpdateRequired())
	{
		return;
	}
	// read configuration for JTAG speed EDT trace.... 
	JTAG_4WIRE_SPEED jtagSpeed = JTAG_4WIRE_SPEED_4_MHZ;
	JTAG_2WIRE_SPEED sbwSpeed  = JTAG_2WIRE_SPEED_400_KHZ;

	string iniFile = "MSP430DLL.INI";

	if (const char* iniPathEnv = getenv("MSP430_DLL_INI_PATH"))
	{
		iniFile = string(iniPathEnv) + "/" + iniFile;
	}
	
	ifstream DllV3Ini(iniFile.c_str());

	while (DllV3Ini && !DllV3Ini.eof())
	{
		DllV3Ini >> tag >> value; 
		if (tag == "SBW_SPEED")
		{
			if (value == "JTAG_2WIRE_SPEED_600_KHZ")
			{
				sbwSpeed = JTAG_2WIRE_SPEED_600_KHZ;
			}
			if (value == "JTAG_2WIRE_SPEED_400_KHZ")
			{
				sbwSpeed = JTAG_2WIRE_SPEED_400_KHZ;
			}
			if (value == "JTAG_2WIRE_SPEED_200_KHZ")
			{
				sbwSpeed = JTAG_2WIRE_SPEED_200_KHZ;
			}
			if (value == "JTAG_2WIRE_SPEED_100_KHZ")
			{
				sbwSpeed = JTAG_2WIRE_SPEED_100_KHZ;
			}
		}
		if (tag == "JTAG_SPEED")
		{
			if (value == "JTAG_4WIRE_SPEED_10_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_10_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_8_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_8_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_4_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_4_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_2_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_2_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_1_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_1_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_750_KHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_750_KHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_500_KHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_500_KHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_250_KHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_250_KHZ;
			}
		}
		if (tag == "ULP_DEBUG")
		{
			ulpDebug = (value == "ON");
		}

		if (tag == "DCO_CALIBRATION")
		{
			freqCalibration = (value != "OFF");
		}
	}
	this->setJtagSpeed(jtagSpeed,sbwSpeed);
}

VersionInfo ConfigManagerV3::getHalVersion() const
{
	return this->updateManagerFet->getHalVersion();
}


void ConfigManagerV3::setJtagMode (enum ConfigManager::jtagMode mode)
{
	this->mode = mode;
}

int16_t ConfigManagerV3::start()
{
	return this->start(password, deviceCode);
}

bool ConfigManagerV3::jtagErase(uint16_t eraseKey)
{
	if(this->start() != 0x1)
	{
		return false;
	}
	HalExecCommand cmd;
	cmd.setTimeout(10000);
	HalExecElement* el = new HalExecElement(ID_SendJtagMailboxXv2);
	el->appendInputData16(LONG_MAILBOX_MODE);
	el->appendInputData16(STOP_DEVICE);
	el->appendInputData16(eraseKey);
	cmd.elements.push_back(el);

	if (!this->parent->send(cmd))
	{
		return false;
	}

	// assert hard RST/NMI and feed in magic pattern to stop device execution
	// thr RST/NMI will remove the register protection
	if(!this->reset(false, true, 0x99, ID_ResetXv2))
	{
		return false;
	}
	// restart jtag connection and if needed feed in JTAG passowrd
	if(this->start() != 0x1)
	{
		return false;
	}
	return true;
}

uint16_t AsciiToHex(const char* password)
{
	return strtoul(std::string(password, 4).c_str(), 0, 16) & 0xFFFF;
}

ConfigManager::jtagMode ConfigManagerV3::getInterfaceMode () const
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_GetInterfaceMode);
	cmd.elements.push_back(el);

	if (this->parent->send(cmd))
	{
		const uint16_t jtagID = el->getOutputAt16(0);
		const uint16_t ifMode = el->getOutputAt16(2);

		if (jtagID != 0xFFFF)
		{
			switch (ifMode)
			{
				case 0: // JTAG Mode
					return JTAG_MODE_4WIRE;
				case 1: // SBW2 Mode
					return JTAG_MODE_SPYBIWIRE;
				case 2: // SBW4 Mode
					return JTAG_MODE_4AFTER2;
				default: break;
			}
		}
	}
	return JTAG_MODE_UNDEF;
}

int16_t ConfigManagerV3::start(const string& pwd, uint32_t deviceCode)
{
	//#words in hex, ie. #characters / 4
	const uint16_t pwLength = (uint16_t)pwd.length() / 4;
	
	// if we have an L092 Rom device
	if (deviceCode == 0xDEADBABE)
	{
		if (pwLength > 4)
		{
			return -2;
		}

		HalExecElement* el = new HalExecElement(ID_UnlockC092);
		el->appendInputData16(pwLength);

		const char* pwdinternal = pwd.c_str();
		for (uint16_t i = 0; i < pwLength; i++)
		{
			uint16_t hexWord = AsciiToHex(pwdinternal);
			el->appendInputData16(hexWord);
			pwdinternal += 4;      
		} 

		HalExecCommand cmd;
		cmd.elements.push_back(el);

		if (!this->parent->send(cmd))
		{
			return -2;
		}
		return 1;
	}
	// if we have an L092 device
	if(deviceCode == 0xA55AA55A|| deviceCode == 0x5AA55AA5) 
	{
		HalExecElement* el = new HalExecElement(ID_StartJtagActivationCode);		

		el->appendInputData8(0);
		el->appendInputData8(0);
		el->appendInputData32(deviceCode);
		
		HalExecCommand cmd;
		cmd.elements.push_back(el);
		cmd.setTimeout(10000);

		if (!this->parent->send(cmd))
		{
			return -2;
		}
		return 1;
	}
	// if we have a device locked with a custom password
	if ( !pwd.empty() )
	{
		if (pwLength > 60)
		{
			return 0;
		}

		HalExecElement* el = new HalExecElement(ID_UnlockDeviceXv2);
		switch (this->mode) 
		{
			case ConfigManager::JTAG_MODE_4WIRE:
				el->appendInputData16(0);
				break;
			case ConfigManager::JTAG_MODE_SPYBIWIRE:
				el->appendInputData16(1);
				break;
			case ConfigManager::JTAG_MODE_4AFTER2:	
				el->appendInputData16(2);
				break;
			default:
				delete el;
				return 0;
		}

		el->appendInputData16(pwLength);	

		const char* pwdinternal = pwd.c_str();
		for (uint16_t i = 0; i < pwLength; i++)
		{
			const uint16_t hexWord = AsciiToHex(pwdinternal);
			el->appendInputData16(hexWord);
			pwdinternal += 4;      
		}
		
		HalExecCommand cmd;
		cmd.elements.push_back(el);

		if (!this->parent->send(cmd))
		{
			return -2;
		}
		#ifndef NDEBUG
			printf("Unlock device\n");
		#endif

		return 1;
	}
	// if we have a "normal" msp430 device without special handling
 	if((deviceCode != 0xA55AA55A && deviceCode != 0x5AA55AA5) || deviceCode == 0x80058005)
	{
		HalExecCommand startJtag;
		HalExecElement* e2 = new HalExecElement(ID_StartJtag);
		switch (this->mode) {
		case ConfigManager::JTAG_MODE_4WIRE:
			e2->appendInputData8(0);
			break;
		case ConfigManager::JTAG_MODE_SPYBIWIRE:
			e2->appendInputData8(1);
			break;
		case ConfigManager::JTAG_MODE_4AFTER2:	
			e2->appendInputData8(2);
			break;
		case ConfigManager::JTAG_MODE_SPYBIWIRE_SUBMCU:
			e2->appendInputData8(5);
			break;
		case ConfigManager::JTAG_MODE_SPYBIWIRE_MSP_FET:
			e2->appendInputData8(6);
			break;
		default:
			delete e2;
			return 0;
		}

		startJtag.elements.push_back(e2);
		if (!this->parent->send(startJtag))
		{
			return -1;
		}
		const uint8_t numOfDevices = startJtag.elements.at(0).getOutputAt8(0);
	#ifndef NDEBUG
		printf("num of devices %i\n",numOfDevices);
	#endif
		return numOfDevices;
	}
	return 0;
}

bool ConfigManagerV3::configureJtagSpeed(uint32_t speed)
{
	enum INTERFACE_SPEED iSpeed = (enum INTERFACE_SPEED)speed;	
	bool retValue = false;
	switch (iSpeed) 
	{
		case FAST:
			retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_8_MHZ,JTAG_2WIRE_SPEED_600_KHZ);
			break;
		case MEDIUM:
			retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_4_MHZ,JTAG_2WIRE_SPEED_400_KHZ);
			break;
		case SLOW:
			retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_1_MHZ,JTAG_2WIRE_SPEED_200_KHZ);
			break;
		default:
			retValue = this->setJtagSpeed(JTAG_4WIRE_SPEED_4_MHZ,JTAG_2WIRE_SPEED_600_KHZ);
			break;
	}
	return retValue;
}

bool ConfigManagerV3::setJtagSpeed(JTAG_4WIRE_SPEED speedJtag, JTAG_2WIRE_SPEED speedSbw)
{
	FetControl * control=this->parent->getControl();
	if(control->getFetToolId() != MSP_FET430)
	{
		HalExecElement* el = new HalExecElement(ID_Configure);
		el->appendInputData32(CONFIG_PARAM_JTAG_SPEED);
		el->appendInputData32(speedJtag);
		el->appendInputData32(speedSbw);

		HalExecCommand configCmd;
		configCmd.elements.push_back(el);
		return this->parent->send(configCmd);
	}
	return true;
}

bool ConfigManagerV3::stop ()
{
	HalExecCommand stopJtag;
	stopJtag.elements.push_back(new HalExecElement(ID_StopJtag));
	return this->parent->send(stopJtag);
}

long ConfigManagerV3::MSP430I_MagicPattern(uint16_t ifMode)
{
	uint16_t mode[2] = {0};

	if(ifMode == ConfigManager::JTAG_MODE_AUTOMATIC)
	{
		mode[0]	= ConfigManager::JTAG_MODE_SPYBIWIRE;
		mode[1]	= ConfigManager::JTAG_MODE_4AFTER2;
	}
	else
	{
		mode[0]	= ifMode;
		mode[1]	= ifMode;
	}

	for(uint16_t i = 0; i < 2; i++)
	{
		setJtagMode((ConfigManager::jtagMode)mode[i]);
		this->start();

		HalExecElement* el = new HalExecElement(ID_Reset430I);
		HalExecCommand cmd;
		cmd.elements.push_back(el);

		bool success = this->parent->send(cmd);

		if (success)
		{
			//GET Chain Length
			uint8_t chainLen= cmd.elements.at(0).getOutputAt8(0);
			//Get JTAG ID
			uint8_t iJtagID= cmd.elements.at(0).getOutputAt8(1);

			if ((chainLen > 0) && (iJtagID == 0x89))
			{
				// Return the protocol
				return 0;
			}
		}
	}
	return -1;
}


bool ConfigManagerV3::reset(bool vcc, bool nmi, uint16_t JtagId, uint32_t rstHalId)
{
	if (jtagIdIsXv2(JtagId) || deviceCode == 0x20404020)
	{
		if (vcc)
		{
			uint16_t voltage = this->getDeviceVcc();
			if (!this->setDeviceVcc(0))
			{
				return false;
			}
			// keep voltage 0 for minmum 5 seconds
			boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(5));

			if (!this->setDeviceVcc(voltage))
			{
				return false;
			}
			this->start();
		}

		if (vcc || nmi)
		{
			hal_id resetMacro;
			if(deviceCode == 0x20404020)
			{
				resetMacro = ID_Reset430I;
			}
			else if(deviceCode == 0xA55AA55A || deviceCode == 0x5AA55AA5)
			{
				resetMacro = ID_ResetL092;
			}
			else
			{
				resetMacro = (hal_id)rstHalId;
			}

			HalExecElement* el = new HalExecElement(resetMacro);
			if(deviceCode == 0xA55AA55A || deviceCode == 0x5AA55AA5)
			{
				el->appendInputData32(deviceCode);
			}

			HalExecCommand cmd;
			cmd.setTimeout(10000);
			cmd.elements.push_back(el);
			if (!this->parent->send(cmd))
			{
				return false;
			}
		}
	}
	else
	{
		/* for all other CPU architectures we just toggle the RST pint to create a BOR */
		if (nmi)
		{
			list<PinState> pinStates;
			pinStates.push_back( PinState(JTAG_PIN_SELTST, false)(JTAG_PIN_RST, false).setDelay(10) );
			pinStates.push_back( PinState(JTAG_PIN_SELTST, false)(JTAG_PIN_RST, true) );
			if (!sendPinSequence(pinStates, parent))
			{
				return false;
			}
		}
		if (vcc) 
		{
			uint16_t voltage = this->getDeviceVcc();
			if (!this->setDeviceVcc(0))
			{
				return false;
			}
			// keep voltate 0 for minmum 5 seconds 
			boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(5));
			if (!this->setDeviceVcc(voltage))
			{
				return false;
			}
		}
	}
	return true;
}

#define ENERGYTRACE_FINE_MODE 0x8000;
void ConfigManagerV3::setCurrentDrive(uint32_t value) 
{
	if(value)
	{
		mhighres = ENERGYTRACE_FINE_MODE;
	}
	else
	{
		mhighres = 0;
	}
}

void ConfigManagerV3::setEnergyTraceManager(EnergyTraceManager * etm)
{
	mEnergyTraceManager	= etm;		
}

bool ConfigManagerV3::setVccEzFet(uint16_t vcc)
{
	#ifndef NDEBUG
		printf("VCC  in[mV]: %i\n",vcc);
	#endif

	if (vcc)
	{
		uint16_t actualVcc = 0;
		//calculate VCC to configure sub mcu.
		for(uint16_t i = 0 ; i < 4; i++)
		{
			actualVcc += this->getDeviceVcc();
		}
		actualVcc = actualVcc / 4;

		actualVcc = ((actualVcc + 50) / 100) * 100;

		if(actualVcc > 3600)
		{
			actualVcc = 3600;
		}

		if(actualVcc < 1800)
		{
			actualVcc = 1800;
		}

		//  send VCC set comand to SubMco
		HalExecElement* el = new HalExecElement(ID_Zero, dcdcSetVcc);
		el->appendInputData16(actualVcc + mhighres); // mhighres Added for high resolution mode --> default 0;

		HalExecCommand dcdcCmd;
		dcdcCmd.elements.push_back(el);
		if (!this->parent->send(dcdcCmd))
		{
			return false;
		}
		// now run calibration to support Energy Trace
		if(mEnergyTraceManager)
		{
			// Cut the power switch to the target first before doing calibration
			HalExecElement* el = new HalExecElement(ID_SetVcc);
			el->appendInputData16(0);

			HalExecCommand dcdcCmd;
			dcdcCmd.elements.push_back(el);
			if (!this->parent->getControl()->send(dcdcCmd))
			{
				return false;
			}

			this->mEnergyTraceManager->doCalibration(actualVcc);
		}
		// then send power on command to hil module to switch MOSFET
		el = new HalExecElement(ID_SetVcc);
		el->appendInputData16(actualVcc);
		HalExecCommand hilCmd;
		hilCmd.elements.push_back(el);
		if (!this->parent->send(hilCmd))
		{
			return false;
		}
		this->vcc = actualVcc;

		if(vcc)
		{
			boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(500));
		}
	}
	else // just shutdown voltage
	{
		// then send power on command to hil module to switch MOSFET
		HalExecElement* el = new HalExecElement(ID_SetVcc);
		el->appendInputData16(0);

		HalExecCommand hilCmd;
		hilCmd.elements.push_back(el);
		if (!this->parent->send(hilCmd))
		{
			return false;
		}

		// Send power down comand to Sub mcu Firmware
		el = new HalExecElement(ID_Zero, dcdcPowerDown);

		HalExecCommand dcdcCmd;
		dcdcCmd.elements.push_back(el);
		if (!this->parent->send(dcdcCmd))
		{
			return false;
		}
	}
	return true;
}

bool ConfigManagerV3::setVccMspFET(uint16_t vcc)
{
	HalExecCommand cmd;
	HalExecElement* el;

	el = new HalExecElement(ID_SwitchMosfet);
	el->appendInputData16(0);
	cmd.elements.push_back(el);
	if (!this->parent->send(cmd))
	{
		return false;
	}

	if(vcc)
	{
		// send VCC set command to SubMcu - configure PWM pulses
		el = new HalExecElement(ID_Zero, dcdcSetVcc);
		el->appendInputData16(vcc);
		cmd.setTimeout(10000);
		cmd.elements.clear();
		cmd.elements.push_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}

		el = new HalExecElement(ID_SetVcc);
		el->appendInputData16(vcc);
		cmd.elements.clear();
		cmd.setTimeout(10000);
		cmd.elements.push_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}

		// now run calibration to support Energy Trace
		if(mEnergyTraceManager)
		{
			this->mEnergyTraceManager->doCalibration(vcc);
		}
		// then send power on command to hil module to switch MOSFET
		el = new HalExecElement(ID_SwitchMosfet);
		el->appendInputData16(1);
		cmd.elements.clear();
		cmd.setTimeout(10000);
		cmd.elements.push_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}
	}
	else // just shotdown voltate
	{
		el = new HalExecElement(ID_SetVcc);
		el->appendInputData16(vcc);
		cmd.elements.clear();
		cmd.setTimeout(10000);
		cmd.elements.push_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}
		// wait for regulation to regulate DAC
		boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(100));

		// Send power down command to Sub mcu Firmware
		el = new HalExecElement(ID_Zero, dcdcPowerDown);
		cmd.elements.clear();
		cmd.elements.push_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}
	}
	return true;
}

bool ConfigManagerV3::setVccMspFetUif(uint16_t vcc)
{
	HalExecCommand cmd;
	#ifndef NDEBUG
		printf("VCC  in[mV]: %i\n",vcc);
	#endif
		HalExecElement* el = new HalExecElement(ID_SetVcc);
		el->appendInputData16(vcc);
		cmd.elements.push_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}
		this->vcc = vcc;
	#ifndef NDEBUG
		printf("VCC out[mV]: %i\n",this->getDeviceVcc ());
	#endif
		if(vcc)
		{
			boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(1000));
		}
	#ifndef NDEBUG
		printf("VCC out[mV]: %i\n",this->getDeviceVcc ());
	#endif
		return true;
}

bool ConfigManagerV3::setDeviceVcc (uint16_t vcc)
{
	FetControl * control=this->parent->getControl();
	if(control->getFetToolId() == eZ_FET_WITH_DCDC)
	{
		return setVccEzFet(vcc);
	}
	else if(control->getFetToolId() == eZ_FET_NO_DCDC)
	{
		// fixed LDO voltage, No power switch
		return true;
	}
	else if(control->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		return setVccMspFET(vcc);
	}
	else if(control->getFetToolId() == MSP_FET430)
	{
		return setVccMspFetUif(vcc);
	}
	return false;
}

uint16_t ConfigManagerV3::getDeviceVcc () const
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_GetVcc);

	cmd.elements.push_back(el);
	if (!this->parent->send(cmd))
	{
		return 0;
	}
	return el->getOutputAt16(0);
}

uint16_t ConfigManagerV3::getExternalVcc () const
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_GetVcc);

	cmd.elements.push_back(el);
	if (!this->parent->send(cmd))
		return 0;

	return el->getOutputAt16(2);
}

bool ConfigManagerV3::configureOverCurrent(bool state)
{
	FetControl * control=this->parent->getControl();
	if(control->getFetToolId() != MSP_FET430)
	{
		HalExecCommand cmd;
		HalExecElement* el = new HalExecElement(ID_Zero, CmdOverCurrent);
		el->appendInputData8(state);
		cmd.elements.push_back(el);
		if (!this->parent->send(cmd))
		{
			return false;
		}
	}
	return true;
}


bool ConfigManagerV3::firmWareUpdate(const char* fname, UpdateNotifyCallback callback, bool* coreUpdate)
{	
	return this->updateManagerFet->firmWareUpdate(fname, callback, coreUpdate);
}

void ConfigManagerV3::setPassword(const string& pwd)
{
	this->password = pwd;
}

bool ConfigManagerV3::setDeviceCode(uint32_t deviceCode)
{
	 this->deviceCode = deviceCode;
	 return true;
}

bool ConfigManagerV3::freqCalibrationEnabled() const
{
	return freqCalibration;
}

bool ConfigManagerV3::ulpDebugEnabled() const
{
	return ulpDebug;
}

void ConfigManagerV3::setUlpDebug(bool ulp) 
{
	ulpDebug = ulp;
}

