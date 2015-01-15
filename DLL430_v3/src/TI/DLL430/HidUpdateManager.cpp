/*
 * HidUpdateManager.h
 *
 * Recovery for broken eZ-FETs and MSP-FET Debuggers
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
#include "HidUpdateManager.h"

#include <hidapi/hidapi.h>

#include <BSL430_DLL/Connections/MSPBSL_Connection5xxUSB.h>
#include <BSL430_DLL/MSPBSL_Factory.h>
#include <BSL430_DLL/Utility_Classes/MSPBSL_CRCEngine.h>

#include "FileFuncImpl.h"
#include "TemplateDeviceDb/Utility.h"

#include "../../../../Bios/include/eZ_FetCore.h"
#include "../../../../Bios/include/MSP_FetCore.h"

using namespace TI::DLL430;
using namespace std;


uint32_t HidUpdateManager::countHidDevices(uint16_t productId)
{
	hid_device_info *hidDevices = hid_enumerate(MSPBSL_STANDARD_USB_VID, productId);     	
	hid_device_info *hidDevicesIt = hidDevices;
	
	uint32_t count = 0; 
	while(hidDevicesIt != NULL)
	{
		count++;
		hidDevicesIt =  hidDevicesIt->next;	
	}
	hid_free_enumeration(hidDevices);

	return count;
}



HidUpdateManager::HidUpdateManager () : BslFet(0)
{
}

HidUpdateManager::~HidUpdateManager (){}


bool HidUpdateManager::hid_firmWareUpdate(const char * fname, UpdateNotifyCallback callback)
{
	bool returnValue = false;
	try
	{
		const bool eZRecoveryNeeded = (HidUpdateManager::countHidDevices(MSPBSL_EZ_FET_USB_PID) > 0);
		const bool mspFetRecoveryNeeded = (HidUpdateManager::countHidDevices(MSPBSL_MSP_FET_USB_PID) > 0);

		uint16_t currentPid = 0;
		if(eZRecoveryNeeded)
		{
			currentPid = MSPBSL_EZ_FET_USB_PID;
		}
		else if(mspFetRecoveryNeeded)
		{
			currentPid = MSPBSL_MSP_FET_USB_PID;
		}
		
		//------------------------------------------------------------------------------------------
		std::string version =  hid_enumerateBSL(currentPid);
		auto_ptr<MSPBSL_Connection5xxUSB> bslFetCleanup(BslFet);

		if(version == "BUG")
		{
			return false;
		}

		uint32_t requiredUpdates = 3;
		const uint32_t percent = 100/requiredUpdates;		

		if(callback)
		{
			callback(BL_INIT, 0, 0);
			callback(BL_PROGRAM_FIRMWARE, 0, 0);
			callback(BL_DATA_BLOCK_PROGRAMMED, 0, 0);
		}

		FileFuncImpl firmware;

		if (eZRecoveryNeeded)
		{
			uint16_t toolIdBSL = hid_getBSLToolId();
			if(toolIdBSL == eZ_FET_WITH_DCDC || toolIdBSL == eZ_FET_WITH_DCDC_0X3FF || toolIdBSL ==  eZ_FET_NO_DCDC)
			{
				//load eZ-FET CORE imag ewith TOOL ID 0xAAAA -> DCDC Sub mcu is present
				firmware.readFirmware(eZ_FetCoreImage, eZ_FetCoreImage_address, eZ_FetCoreImage_length_of_sections, eZ_FetCoreImage_sections);
			}
		}
		else if (mspFetRecoveryNeeded)
		{
			uint16_t toolIdBSL = hid_getBSLToolId();
			if(toolIdBSL == MSP_FET_WITH_DCDC)
			{
				firmware.readFirmware(MSP_FetCoreImage, MSP_FetCoreImage_address, MSP_FetCoreImage_length_of_sections, MSP_FetCoreImage_sections);
			}
		}

		if(callback)
		{
			callback(BL_DATA_BLOCK_PROGRAMMED, (100-(--requiredUpdates)*percent), 0);
		}

		returnValue = hid_updateCore(firmware);

		if(callback)
		{
			callback(BL_DATA_BLOCK_PROGRAMMED, (100-(--requiredUpdates)*percent), 0);
		}

		/*Reset FET*/
		if (BslFet)
		{
			BslFet->closeBslconnection();
		}
	}
	catch(...)
	{
		returnValue = false;
	}

	if(callback)
	{
		callback(BL_DATA_BLOCK_PROGRAMMED,100,0);
		callback(BL_UPDATE_DONE,0,0);
		callback(BL_EXIT,0,0);
	}
	return returnValue;
}

uint16_t HidUpdateManager::hid_getBSLToolId()
{
	uint8_t data[2];
	// read tool-ID stored in BSL memory
	BslFet->TX_DataBlock(data,0x100e,2);
	uint16_t toolId = data[1];
	toolId = toolId<<8;
	toolId |=data[0];   
	return toolId;
}

std::string HidUpdateManager::hid_enumerateBSL(uint16_t currentPid)
{
	string verString = "BUG";
	string currentDevice = "";

	if(currentPid == MSPBSL_MSP_FET_USB_PID)
	{
		currentDevice = "MSP430F6638";
	}
	if(currentPid == MSPBSL_EZ_FET_USB_PID)
	{
		currentDevice = "MSP430F5528";
	}

	BslFet = dynamic_cast<MSPBSL_Connection5xxUSB*>(MSPBSL_Factory::getMSPBSL_Connection("DEVICE:" + currentDevice + " VID:0x2047 PID:0x"+ convertPid(currentPid) +"")); // works for 6638 and 5528 because of generic usb bsl
	
	if (BslFet)
	{   
		if(BslFet->loadRAM_BSL(currentPid) != 0)
		{
			//Reset FET
			BslFet->closeBslconnection();
			return verString;
		}
		BslFet->TX_BSL_Version(verString);
	}
	return verString;
}

bool HidUpdateManager::hid_updateCore(const FileFuncImpl &firmware)const
{
	// erase reset vector of core	
	BslFet->massErase();	
	BslFet->eraseSegment(0x197F);
	BslFet->eraseSegment(0x18FF);
	BslFet->eraseSegment(0x187F);

	// Info A hanlding -> unlock it
	BslFet->toggleInfo();
	// erase Info A
	BslFet->eraseSegment(0x19FF);

	if((firmware.getNumberOfSegments())==0)
	{
		return false;
	}
	
	for (size_t i = 0; i < firmware.getNumberOfSegments(); i++)
	{
		const DownloadSegment *seg = firmware.getFirmwareSeg(i);
		if (seg == NULL)
		{
			return false;
		}

		vector<uint8_t> Buffer(seg->size);	
		
		MSPBSL_CRCEngine crcEngine("5xx_CRC");
		crcEngine.initEngine(0xFFFF);

		for(uint32_t n=0; n < seg->size; n++)
		{
			Buffer[n]= (seg->data[n] & 0xff);
			crcEngine.addByte(seg->data[n] & 0xff);
		}

		BslFet->RX_DataBlockFast(&Buffer[0], (uint32_t)seg->startAddress&0xfffffffe,(uint16_t)seg->size);

		uint16_t currentCoreCRC[1]; 
		BslFet->CRC_Check(currentCoreCRC, (uint32_t)seg->startAddress&0xfffffffe, seg->size);

		uint32_t expectedCoreCRC = crcEngine.getHighByte()<<8;
		expectedCoreCRC |= crcEngine.getLowByte();

		if(expectedCoreCRC != currentCoreCRC[0])
		{
			if(i != 0)// just debug exeption
			{				
				BslFet->closeBslconnection();		
				return false;
			}
		}

	}
	return true;
}
