/*
 * UpdateManagerFet.cpp
 *
 * Functionality for updating eZ-FET & MSP-FET debugger
 *
 * Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *	Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 *
 *	Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in the
 *	documentation and/or other materials provided with the
 *	distribution.
 *
 *	Neither the name of Texas Instruments Incorporated nor the names of
 *	its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
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

#include <BSL430_DLL/Connections/MSPBSL_Connection5xxUSB.h>
#include <BSL430_DLL/MSPBSL_Factory.h>
#include <BSL430_DLL/Utility_Classes/MSPBSL_CRCEngine.h>

#include "VersionInfo.h"
#include "FetHandleV3.h"
#include "FetHandleManager.h"
#include "ConfigManagerV3.h"
#include "UpdateManagerFet.h"
#include "DeviceDbManagerExt.h"
#include "DeviceInfo.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "DeviceHandleV3.h"
#include "HidUpdateManager.h"

#include "../../../../Bios/include/eZ_FetDcdc.h"
#include "../../../../Bios/include/eZ_FetHal.h"
#include "../../../../Bios/include/eZ_FetHil.h"
#include "../../../../Bios/include/eZ_FetCore.h"
#include "../../../../Bios/include/eZ_FetComChannel.h"
#include "../../../../Bios/include/eZ_FetDcdcController.h"


#include "../../../../Bios/include/MSP_FetHal.h"
#include "../../../../Bios/include/MSP_FetHil.h"
#include "../../../../Bios/include/MSP_FetCore.h"
#include "../../../../Bios/include/MSP_FetDcdc.h"
#include "../../../../Bios/include/MSP_FetDcdcController.h"

//Removed due to license limitations
//#define FPGA_UPDATE

#ifdef FPGA_UPDATE
#include "../../../../Bios/include/MSP_FetFpgaHal.h"
#endif

#include "../../../../Bios/include/MSP_FetComChannel.h"

#include "../../../../Bios/include/ConfigureParameters.h"

using namespace TI::DLL430;
using namespace std;


static string UpdateLog;

UpdateManagerFet::UpdateManagerFet(FetHandleV3* fetHandle, ConfigManagerV3* configManagerV3, FetHandleManager* fhManager)
	: fetHandle(fetHandle)
	, configManagerV3(configManagerV3)
	, fetHandleManager(fhManager)
	, intCallback(NULL)
{
}


bool UpdateManagerFet::updateCore(FileFuncImpl &firmware)
{
	double requiredCoreUpdates = 4 + firmware.getNumberOfSegments();

	try
	{
		int currentPid = 0;
		string currentDevice;
		if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC )
		{
			currentPid = MSPBSL_MSP_FET_USB_PID;
			currentDevice = "MSP430F6638";
		}
		else
		{
			currentPid = MSPBSL_EZ_FET_USB_PID;
			currentDevice = "MSP430F5528";
		}

		UpdateLog.append("----TRACE---------------eZ_FET start BSL update------------------------------;\n");

		// erase reset vector of core
		upCoreErase();

		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)( 100-(--requiredCoreUpdates)*requiredCoreUpdates), 0);
		}

		boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(4));

		fetHandle->shutdown();

		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)(100-(--requiredCoreUpdates)*requiredCoreUpdates), 0);
		}

		uint32_t countedHidDevices = 0, timeout = 50;
		do
		{
			boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(1));
			countedHidDevices = HidUpdateManager::countHidDevices(currentPid);

			if(intCallback)
			{
				intCallback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)(100-(requiredCoreUpdates)*requiredCoreUpdates), 0);
			}
		}
		while(!countedHidDevices && timeout--);

		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)(100-(--requiredCoreUpdates)*requiredCoreUpdates), 0);
		}

		UpdateLog.append("----TRACE----fetHandle->shutdown()\n");
		auto_ptr<MSPBSL_Connection5xxUSB> eZ_FET(dynamic_cast<MSPBSL_Connection5xxUSB*>(MSPBSL_Factory::getMSPBSL_Connection("DEVICE:" + currentDevice + " VID:0x2047 PID:0x"+ convertPid(currentPid) +"")));

		if (eZ_FET.get() == NULL)
		{
			UpdateLog.append("----TRACE----MSPBSL_Factory::getMSPBSL_Connection()\n");
			return false;
		}

		UpdateLog.append("----TRACE----auto_ptr<MSPBSL_Connection5xxUSB> eZ_FET\n");

		if(eZ_FET->loadRAM_BSL(currentPid) != 0)
		{
			/*Reset FET*/
			eZ_FET->closeBslconnection();
			UpdateLog.append("----TRACE----eZ_FET->loadRAM_BSL() != 0 \n");
			return false;
		}

		string verString = "";
		eZ_FET->TX_BSL_Version(verString);
		UpdateLog.append("----TRACE----eZ_FET->TX_BSL_Version(verString);\n");

		eZ_FET->massErase();
		UpdateLog.append("----TRACE----eZ_FET->massErase();\n");

		if((firmware.getNumberOfSegments())==0)
		{
			return false;
		}

		for (size_t i = 0; i < firmware.getNumberOfSegments(); i++)
		{
			const DownloadSegment *seg = firmware.getFirmwareSeg(i);

			if (seg == NULL)
			{
				UpdateLog.append("----TRACE----eZ_FET end BSL update faild\n");
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

			eZ_FET->RX_DataBlockFast(&Buffer[0], (uint32_t)seg->startAddress&0xfffffffe,(uint16_t)seg->size);

			uint16_t currentCoreCRC[1] = {0};
			eZ_FET->CRC_Check(currentCoreCRC, (uint32_t)seg->startAddress&0xfffffffe, seg->size);

			uint32_t expectedCoreCRC = crcEngine.getHighByte()<<8;
			expectedCoreCRC |= crcEngine.getLowByte();

			if(expectedCoreCRC != currentCoreCRC[0])
			{
				if(i != 0)// just debug exeption
				{
					eZ_FET->closeBslconnection();
					UpdateLog.append("----TRACE----eZ_FET end BSL update faild\n");
					return false;
				}
			}
			if(intCallback)
			{
				intCallback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)( 100-(--requiredCoreUpdates)*requiredCoreUpdates), 0);
			}
		}
		UpdateLog.append("----TRACE---------------eZ_FET end BSL update------------------------------;\n");
		eZ_FET->closeBslconnection();
	}
	catch(...)
	{
		UpdateLog.append("----TRACE----eZ_FET end BSL update failed\n");
		return false;
	}

	if(intCallback)
	{
		intCallback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)( 100-(--requiredCoreUpdates)*requiredCoreUpdates), 0);
	}

	return true;
}


bool UpdateManagerFet::isUpdateRequired() const
{
	bool isUpdateRequired = false;
	if (checkHalVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkCoreVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkDcdcLayerVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkDcdcSubMcuVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkHilVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkFpgaVersion() != 0)
	{
		isUpdateRequired = true;
	}
	if (checkUartVersion() != 0)
	{
		isUpdateRequired = true;
	}
	return isUpdateRequired;
}


uint16_t UpdateManagerFet::checkHilVersion() const
{
	//get current hil version from FET
	const uint16_t currentHilVersion = fetHandle->getControl()->getHilVersion();
	//get hil CRC from FET
	const uint16_t currentHilCrc = fetHandle->getControl()->getFetHilCrc();

	uint16_t expectedHilVersion = 0;
	uint16_t expectedHilCrc = 0;

	Record *image = 0;
	uint16_t retVal = 0;

	if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		image = new Record(MSP_FetHilImage, MSP_FetHilImage_address, MSP_FetHilImage_length_of_sections, MSP_FetHilImage_sections);
	}
	else
	{
		image = new Record(eZ_FetHilImage, eZ_FetHilImage_address, eZ_FetHilImage_length_of_sections, eZ_FetHilImage_sections);
	}

	//if hil versions or CRC's do not match, update hil
	if(image && image->getWordAtAdr(0x18F6, &expectedHilVersion) && image->getWordAtAdr(0x18FA, &expectedHilCrc))
	{
		if((expectedHilVersion != currentHilVersion) || (expectedHilCrc != currentHilCrc))
		{
			retVal = 1;
		}
	}

	delete image;

	return retVal;
}

uint16_t UpdateManagerFet::checkUartVersion() const
{
	uint16_t retVal = 0;

	const uint16_t currentUartVersion = fetHandle->getControl()->getFetComChannelVersion();
	const uint16_t currentFetComChannelCrc= fetHandle->getControl()->getFetComChannelCrc();

	uint16_t expectedUartVersion = 0;
	uint16_t expectedFetComChannelCRC = 0;

	Record *image = 0;

	if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		image = new Record(MSP_FetComChannelImage, MSP_FetComChannelImage_address, MSP_FetComChannelImage_length_of_sections, MSP_FetComChannelImage_sections);
	}
	else
	{
		image = new Record(eZ_FetComChannelImage, eZ_FetComChannelImage_address, eZ_FetComChannelImage_length_of_sections, eZ_FetComChannelImage_sections);
	}

	if(image && image->getWordAtAdr(0x1984, &expectedUartVersion) && image->getWordAtAdr(0x19FA, &expectedFetComChannelCRC))
	{
		if((expectedUartVersion != currentUartVersion) || (expectedFetComChannelCRC != currentFetComChannelCrc))
		{
			retVal = 1;
		}
	}

	delete image;

	return retVal;
}

uint16_t UpdateManagerFet::checkFpgaVersion() const
{
	uint16_t retVal = 0;

#ifdef FPGA_UPDATE
	if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		const uint16_t currentFpgaVersion = fetHandle->getControl()->getFetFpgaVersion();
		uint16_t expectedFpgaVersion;

		Record fpgaVersion(MSP_FetFpgaHalImage, MSP_FetFpgaHalImage_address, MSP_FetFpgaHalImage_length_of_sections, MSP_FetFpgaHalImage_sections);

		if(fpgaVersion.getWordAtAdr(0x1978, &expectedFpgaVersion))
		{
			if(expectedFpgaVersion > currentFpgaVersion)
			{
				retVal = 1;
			}
		}
	}
#endif

	return retVal;
}

uint16_t UpdateManagerFet::checkDcdcLayerVersion() const
{
	//get current dcdc layer version from FET
	const uint32_t currentDcdcLayerVersion = fetHandle->getControl()->getDcdcLayerVersion();
	//get dcdc layer CRC from FET
	const uint16_t currentDcdcCrc= fetHandle->getControl()->getFetDcdcCrc();

	uint16_t expectedDcdcCrc = 0;
	uint16_t expectedDcdcLayerVersion = 0;

	Record *image = 0;
	uint16_t retVal = 0;

	if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		image = new Record(MSP_FetDcdcImage, MSP_FetDcdcImage_address, MSP_FetDcdcImage_length_of_sections, MSP_FetDcdcImage_sections);
	}
	else
	{
		image = new Record(eZ_FetDcdcImage, eZ_FetDcdcImage_address, eZ_FetDcdcImage_length_of_sections, eZ_FetDcdcImage_sections);
	}

	//if hil versions or CRC's do not match, update hil
	if(image && image->getWordAtAdr(0x1804, &expectedDcdcLayerVersion) && image->getWordAtAdr(0x187A,&expectedDcdcCrc))
	{
		if((expectedDcdcLayerVersion != currentDcdcLayerVersion) || (expectedDcdcCrc != currentDcdcCrc))
		{
			retVal = 1;
		}
	}

	delete image;

	return retVal;
}

uint16_t UpdateManagerFet::checkDcdcSubMcuVersion() const
{
	const uint32_t currentDcdcSubMcuVersion = fetHandle->getControl()->getDcdcSubMcuVersion();
	uint16_t expectedDcdcSubMcuVersion = 0;

	Record *image = 0;
	uint16_t retVal = 0;

	if(fetHandle->getControl()->getFetToolId() == eZ_FET_WITH_DCDC)
	{
		image = new Record(eZ_FetDcdcControllerImage, eZ_FetDcdcControllerImage_address, eZ_FetDcdcControllerImage_length_of_sections, eZ_FetDcdcControllerImage_sections);
	}
	else if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		image = new Record(MSP_FetDcdcControllerImage, MSP_FetDcdcControllerImage_address, MSP_FetDcdcControllerImage_length_of_sections, MSP_FetDcdcControllerImage_sections);
	}

	if(image && image->getWordAtAdr(0x1000, &expectedDcdcSubMcuVersion))
	{
		//if dcdc sub-mcu versions do not match, update sub-mcu
		if(currentDcdcSubMcuVersion != expectedDcdcSubMcuVersion)
		{
			retVal = 1;
		}
	}

	delete image;

	return retVal;
}

uint16_t UpdateManagerFet::checkHalVersion() const
{
	//get hal CRC from FET
	const uint16_t currentHalCrc = fetHandle->getControl()->getFetHalCrc();
	uint16_t expectedHalCrc  = 0;

	Record *image;
	uint16_t retVal = 0;

	if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		image = new Record(MSP_FetHalImage, MSP_FetHalImage_address, MSP_FetHalImage_length_of_sections, MSP_FetHalImage_sections);
	}
	else
	{
		image = new Record(eZ_FetHalImage, eZ_FetHalImage_address, eZ_FetHalImage_length_of_sections, eZ_FetHalImage_sections);
	}

	if(image->getWordAtAdr(0x197A, &expectedHalCrc))
	{
		if(expectedHalCrc != currentHalCrc)
		{
			retVal = 1;
		}
	}

	delete image;

	return retVal;
}


uint16_t UpdateManagerFet::checkCoreVersion() const
{
	//get current core version from FET
	const uint16_t actualFetCoreVersion = fetHandle->getControl()->getFetCoreVersion();
	//get core CRC from FET
	const uint16_t currentCoreCrc= fetHandle->getControl()->getFetCoreCrc();

	uint16_t expectedFetCoreVersion = 0;
	uint16_t expectedCoreCrc = 0;

	uint16_t versionLocation = 0;
	uint16_t crcLocation = 0;

	Record *image;
	uint16_t retVal = 0;

	if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC )
	{
		image = new Record(MSP_FetCoreImage, MSP_FetCoreImage_address, MSP_FetCoreImage_length_of_sections, MSP_FetCoreImage_sections);
		versionLocation = 0x8004;
		crcLocation = 0x8002;
	}
	else
	{
		image = new Record(eZ_FetCoreImage, eZ_FetCoreImage_address, eZ_FetCoreImage_length_of_sections, eZ_FetCoreImage_sections);
		versionLocation = 0x4404;
		crcLocation = 0x4402;
	}

	if(image->getWordAtAdr(versionLocation, &expectedFetCoreVersion) && image->getWordAtAdr(crcLocation, &expectedCoreCrc))
	{
		//if core versions or CRC's do not match, update core
		if((expectedFetCoreVersion != actualFetCoreVersion) || (currentCoreCrc != expectedCoreCrc))
		{
			retVal = 1;
		}
	}

	delete image;

	return retVal;
}

VersionInfo UpdateManagerFet::getHalVersion() const
{
	std::vector<uint8_t> * sw_info = this->fetHandle->getSwVersion();
	const uint16_t currentHalCrc = fetHandle->getControl()->getFetHalCrc();
	uint16_t expectedHalCrc = 0;

	Record *image;

	if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		image = new Record(MSP_FetHalImage, MSP_FetHalImage_address, MSP_FetHalImage_length_of_sections, MSP_FetHalImage_sections);
	}
	else
	{
		image = new Record(eZ_FetHalImage, eZ_FetHalImage_address, eZ_FetHalImage_length_of_sections, eZ_FetHalImage_sections);
	}

	if(image->getWordAtAdr(0x197A, &expectedHalCrc))
	{
		if(expectedHalCrc != currentHalCrc)
		{
			delete image;
			return VersionInfo(1, 0, 0, 0);
		}
	}

	delete image;

	if(sw_info==NULL)
	{
		return VersionInfo(0, 0, 0, 0);
	}
	if(sw_info->size()<4)
	{
		return VersionInfo(0, 0, 0, 0);
	}

	unsigned char major = sw_info->at(1);

	return VersionInfo((((major&0xC0)>>6)+1),(major&0x3f),sw_info->at(0),
					   (sw_info->at(3)<<8)+sw_info->at(2));
}

bool UpdateManagerFet::updateHal()
{
	bool returnValue = false;
	FileFuncImpl firmware;

	if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC )
	{
		firmware.readFirmware(MSP_FetHalImage, MSP_FetHalImage_address, MSP_FetHalImage_length_of_sections, MSP_FetHalImage_sections);
	}
	else
	{
		firmware.readFirmware(eZ_FetHalImage, eZ_FetHalImage_address, eZ_FetHalImage_length_of_sections, eZ_FetHalImage_sections);
	}
	returnValue = updateFirmware(firmware);
	if(!returnValue)
	{
		UpdateLog.append("----TRACE----HalLayer update failed\n");
	}

	return returnValue;
}

bool UpdateManagerFet::updateHil()
{
	bool returnValue = false;
	FileFuncImpl firmware;

	if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC )
	{
		firmware.readFirmware(MSP_FetHilImage, MSP_FetHilImage_address, MSP_FetHilImage_length_of_sections, MSP_FetHilImage_sections);
	}
	else
	{
		firmware.readFirmware(eZ_FetHilImage, eZ_FetHilImage_address, eZ_FetHilImage_length_of_sections, eZ_FetHilImage_sections);
	}

	returnValue = updateFirmware(firmware);
	if(!returnValue)
	{
		UpdateLog.append("----TRACE----HilLayer update failed\n");
	}

	return returnValue;
}

bool UpdateManagerFet::updateFpga()
{
	bool returnValue = true;

#ifdef FPGA_UPDATE
	FileFuncImpl firmware;

	firmware.readFirmware(MSP_FetFpgaHalImage, MSP_FetFpgaHalImage_address, MSP_FetFpgaHalImage_length_of_sections, MSP_FetFpgaHalImage_sections);

	if(!firmware.getNumberOfSegments())
	{
		returnValue = false;
	}

	this->upInit(1);

	if(returnValue && !this->upErase(firmware))
	{
		returnValue = false;
	}
	if(returnValue && !this->upWrite(firmware))
	{
		returnValue = false;
	}

	// Start the FPGA update
	this->upInit(3);

	boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(1));

	// Restore the normal HAL module
	if(returnValue && !updateHal())
	{
		returnValue = false;
	}

	if(!returnValue)
	{
		UpdateLog.append("----TRACE----FPGA update failed\n");
	}
#endif
	return returnValue;
}

bool UpdateManagerFet::updateDcdcLayer()
{
	bool returnValue = false;
	FileFuncImpl firmware;

	if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC )
	{
		firmware.readFirmware(MSP_FetDcdcImage, MSP_FetDcdcImage_address, MSP_FetDcdcImage_length_of_sections, MSP_FetDcdcImage_sections);
	}
	else
	{
		firmware.readFirmware(eZ_FetDcdcImage, eZ_FetDcdcImage_address, eZ_FetDcdcImage_length_of_sections, eZ_FetDcdcImage_sections);
	}
	returnValue = updateFirmware(firmware);
	if(!returnValue)
	{
		UpdateLog.append("----TRACE----DcdcLayer update failed \n");
	}

	return returnValue;
}

bool UpdateManagerFet::updateSubMcu()
{
	bool returnValue = true;
	FileFuncImpl firmware;

	DeviceHandleManager* dhm = this->fetHandle->getDeviceHandleManager();
	DeviceChainInfoList* dcil = dhm->getDeviceChainInfo();

	 // Now programm the Sub MCU
	this->upInit(2);

	if (dcil->empty())
	{
		UpdateLog.append("----TRACE---- DeviceChainInfoList empty\n");
		return false;
	}

	// the first device is the only one
	DeviceChainInfoList::iterator itsdcil = dcil->begin();

	// if we are alreaddy connected to a device.
	if(itsdcil->isInUse())
	{
		itsdcil->setInUse(false);
	}
	//-----------------------------------------
	configManagerV3->setJtagMode(ConfigManager::JTAG_MODE_SPYBIWIRE_SUBMCU);
	if(!configManagerV3->start())
	{
		UpdateLog.append("----TRACE---- configManagerV3->start() \n");
	}

	// save the device handle to work with
	DeviceHandle* singleDevice = dhm->createDeviceHandle(itsdcil,0);

	if (singleDevice == NULL)
	{
		configManagerV3->stop();
		UpdateLog.append("----TRACE---- singleDevice==NULL \n");
		return false;
	}

	if (singleDevice->getJtagId() != 0x89)
	{
		returnValue = false;
		UpdateLog.append("----TRACE---- singleDevice->getJtagId() != 0x89 \n");
	}

	if (returnValue)
	{
		const long setId = singleDevice->identifyDevice(0, false);
		if(setId == -5555)
		{
			returnValue = false;
			UpdateLog.append("----TRACE---- Fuse Blown\n");
		}
		else if (setId < 0)
		{
			returnValue = false;
			UpdateLog.append("----TRACE----No device detected\n");
		}
	}

	if (returnValue)
	{
		returnValue = returnValue && this->programmSubMcu(singleDevice);
		if(!returnValue)
		{
			UpdateLog.append("----TRACE----programm the Sub MCU update failed \n");
		}

		this->upInit(0);

		if(!configManagerV3->stop())
		{
			UpdateLog.append("----TRACE----Stop JTAG done failed \n");
		}

		// destroy device handle
		dhm->destroyDeviceHandle(singleDevice);
		itsdcil->setInUse(false);
	}

	return returnValue;
}

bool UpdateManagerFet::updateComChannel()
{
	FileFuncImpl firmware;

	if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC )
	{
		firmware.readFirmware(MSP_FetComChannelImage, MSP_FetComChannelImage_address, MSP_FetComChannelImage_length_of_sections, MSP_FetComChannelImage_sections);
	}
	else
	{
		firmware.readFirmware(eZ_FetComChannelImage, eZ_FetComChannelImage_address, eZ_FetComChannelImage_length_of_sections, eZ_FetComChannelImage_sections);
	}

	bool returnValue = updateFirmware(firmware);
	if(!returnValue)
	{
		UpdateLog.append("----TRACE----Uart Layer update failed \n");
	}

	return returnValue;
}

bool UpdateManagerFet::firmWareUpdate(const char* fname, UpdateNotifyCallback callback, bool* coreUpdate)
{
	FetControl* control = this->fetHandle->getControl();
	intCallback = NULL;
	if(control == NULL)
	{
		return false;
	}
	if(callback)
	{
		intCallback = callback;
	}
	bool returnValue = true;
	uint32_t halVersion = getHalVersion().get();

	UpdateLog.clear();
	UpdateLog.append("\n\n\n ------------------------Start Firmware update--------------------------- \n");

	if(intCallback)
	{
		intCallback(BL_INIT, 0, 0);
		intCallback(BL_PROGRAM_FIRMWARE, 0, 0);
	}

	if(checkCoreVersion() != 0)
	{
		*coreUpdate = true;
		FileFuncImpl firmware;
		if(control->getFetToolId() == MSP_FET_WITH_DCDC )
		{
			firmware.readFirmware(MSP_FetCoreImage, MSP_FetCoreImage_address, MSP_FetCoreImage_length_of_sections, MSP_FetCoreImage_sections);
		}
		else
		{
			firmware.readFirmware(eZ_FetCoreImage, eZ_FetCoreImage_address, eZ_FetCoreImage_length_of_sections, eZ_FetCoreImage_sections);
		}
		UpdateLog.append("----TRACE----call updateCore(firmware)\n");
		bool retval = updateCore(firmware);

		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED,100,0);
		}
		return retval;
	}

	// just for core update test do not call during normal debug
	if(fname && string(fname).find("CORE_RST_VECTOR_ERASE") != string::npos)
	{
		upCoreErase();
		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED,100,0);
			intCallback(BL_UPDATE_DONE,0,0);
			intCallback(BL_EXIT,0,0);
		}
		UpdateLog.append("----TRACE----CORE_RST_VECTOR_ERASE done\n");
		return true;
	}
	else if (fname)
	{
		FileFuncImpl firmware;
		if (!firmware.readFirmware(fname))
		{
			UpdateLog.append("----TRACE--- firmware.readFirmware(fname)faild \n");
			return false;
		}
		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 0, 0);
		}
		returnValue = updateFirmware(firmware);
		if(!returnValue)
		{
			UpdateLog.append("----TRACE--- returnValue = updateFirmware(firmware) faild  \n");
		}

		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100, 0);
			intCallback(BL_UPDATE_DONE,0,0);
			intCallback(BL_EXIT,0,0);
		}
	}
	else
	{
		requiredUpdates = 6;
		percent = 100 / requiredUpdates;
		this->upInit(4);// stop vcc supervision
		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(requiredUpdates)*percent, 0);
		}
		//--------------------------------------HalUpdate-----------------------------------------------------
		if(halVersion == 10000000)
		{
			returnValue = updateHal();			control->resetCommunication();
		}
		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(--requiredUpdates)*percent, 0);
		}
		//--------------------------------------HilUpdate-----------------------------------------------------
		if(returnValue && checkHilVersion() != 0)
		{
			returnValue = updateHil();

			control->resetCommunication();
		}

		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(--requiredUpdates)*percent, 0);
		}

		//--------------------------------------FpgaUpdate-----------------------------------------------------
		if(returnValue && checkFpgaVersion() != 0)
		{
			returnValue = updateFpga();

			control->resetCommunication();
		}

		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED,(100-(--requiredUpdates)*percent),0);
		}

		//--------------------------------------DcdcLayerUpdate-----------------------------------------------------
		if(returnValue && checkDcdcLayerVersion() != 0)
		{
			returnValue = updateDcdcLayer();

			control->resetCommunication();
		}

		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(--requiredUpdates)*percent, 0);
		}

		//--------------------------------------SubMcuUpdate-----------------------------------------------------
		if(returnValue && checkDcdcSubMcuVersion() != 0)
		{
			returnValue = updateSubMcu();

			control->resetCommunication();
		}

		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(--requiredUpdates)*percent, 0);
		}

		//--------------------------------------ComChannelUpdate-----------------------------------------------------
		if(returnValue && checkUartVersion() != 0)
		{
			returnValue = updateComChannel();

			control->resetCommunication();
		}
		this->upInit(5);// start vcc supervision
	}

	if(intCallback)
	{
		intCallback(BL_DATA_BLOCK_PROGRAMMED,100,0);
		intCallback(BL_UPDATE_DONE,0,0);
		intCallback(BL_EXIT,0,0);
	}

	if(!returnValue)
	{
#if defined(_WIN32) || defined(_WIN64)
		char binaryPath[256] = {0};
		uint32_t pathLength = 0;

		pathLength = GetModuleFileName(0, binaryPath, sizeof(binaryPath));
		while (pathLength > 0 && binaryPath[pathLength-1] != '\\')
		{
			--pathLength;
		}
		string logfile = string(binaryPath, pathLength) + "Update.log";

		UpdateLog.append("\n---------------------Firmware upate end--------------------------\n");

		ofstream(logfile.c_str(), ios::app | ios::out) << UpdateLog;
#endif
	}
	return returnValue;
}

bool UpdateManagerFet::programmSubMcu(DeviceHandle * singleDevice)
{
	FileFuncImpl firmware;

	if(fetHandle->getControl()->getFetToolId() == MSP_FET_WITH_DCDC )
	{
		firmware.readFirmware(MSP_FetDcdcControllerImage, MSP_FetDcdcControllerImage_address, MSP_FetDcdcControllerImage_length_of_sections, MSP_FetDcdcControllerImage_sections);
	}
	else
	{
		firmware.readFirmware(eZ_FetDcdcControllerImage, eZ_FetDcdcControllerImage_address, eZ_FetDcdcControllerImage_length_of_sections, eZ_FetDcdcControllerImage_sections);
	}

	if((firmware.getNumberOfSegments())==0)
	{
		return false;
	}
	if(!singleDevice)
	{
		UpdateLog.append("----TRACE---- SUB mcu !singleDevice\n");
		return false;
	}
	MemoryManager* mm = singleDevice->getMemoryManager();

	if(!mm)
	{
		UpdateLog.append("----TRACE---- SUB mcu !mm\n");
		return false;
	}
	MemoryArea* main = mm->getMemoryArea(MemoryArea::MAIN);

	singleDevice->reset();

	if(intCallback)
	{
		intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(requiredUpdates)*percent, 0);
	}

	//erase sub MCU.
	bool eraseSubMcuMain = main->erase();
	if(!eraseSubMcuMain)
	{
		UpdateLog.append("----TRACE---- SUB mcu !eraseSubMcuMain\n");
		return false;
	}

	MemoryArea* info = mm->getMemoryArea(MemoryArea::INFO);
	singleDevice->reset();

	//erase sub MCU.
	bool eraseSubMcuinfo = info->erase();
	if(!eraseSubMcuinfo)
	{
		UpdateLog.append("----TRACE---- SUB mcu !eraseSubMcuinfo\n");
		return false;
	}

	if(intCallback)
	{
		intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(requiredUpdates)*percent, 0);
	}

	singleDevice->reset();

	bool writeSubMcu = firmware.writeSegs(singleDevice);
	if(!writeSubMcu)
	{
		UpdateLog.append("----TRACE---- SUB mcu !writeSubMcu = firmware.writeSegs(singleDevice\n");
	}
	firmware.close();
	return writeSubMcu;
}

bool UpdateManagerFet::updateFirmware(const FileFuncImpl &firmware)
{
	if((firmware.getNumberOfSegments())==0)
	{
		return false;
	}
	// start HAL update routine

	this->upInit(1);

	if(!this->upErase(firmware))
	{
		return false;
	}
	if(!this->upWrite(firmware))
	{
		return false;
	}
	this->upInit(0);

	// give the firmware time to execute initialisation
	boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(1));
	return true;
}

void UpdateManagerFet::upInit(unsigned char level)
{
	HalExecCommand updateCmd;
	updateCmd.setTimeout(10000);
	HalExecElement* el = new HalExecElement(ID_Zero, UpInit);
	el->setAddrFlag(false);
	el->appendInputData8(level);

	updateCmd.elements.push_back(el);
	this->fetHandle->send(updateCmd);
}

bool UpdateManagerFet::upErase(const FileFuncImpl& firmware)
{
	for (size_t i = 0; i < firmware.getNumberOfSegments(); ++i)
	{
		const DownloadSegment *seg = firmware.getFirmwareSeg(i);
		if (seg == NULL)
		{
			return false;
		}

		HalExecElement* el = new HalExecElement(ID_Zero, UpErase);
		el->setAddrFlag(false);

		el->appendInputData32(seg->startAddress&0xfffffffe);
		el->appendInputData32(seg->size);

		HalExecCommand updateCmd;
		updateCmd.setTimeout(40000);
		updateCmd.elements.push_back(el);
		if(this->fetHandle->send(updateCmd)==false)
		{
			return false;
		}
		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(requiredUpdates)*percent, 0);
		}
	}
	return true;
}

bool UpdateManagerFet::upWrite(const FileFuncImpl& firmware)
{
	for (size_t i = firmware.getNumberOfSegments(); i > 0; i--)
	{
		const DownloadSegment *seg = firmware.getFirmwareSeg(i-1);

		if (seg == NULL)
		{
			return false;
		}

		// create Core telegram -> update write
		HalExecElement* el = new HalExecElement(ID_Zero, UpWrite);
		// no HAL id needed for update
		el->setAddrFlag(false);

		const uint32_t padding = seg->size % 2;
		const uint32_t data2send = seg->size + padding;

		// add address data
		el->appendInputData32(seg->startAddress&0xfffffffe);
		el->appendInputData32(data2send);

		// add update data
		for(uint32_t n=0; n < seg->size; n++)
		{
			el->appendInputData8(seg->data[n] & 0xff);
		}
		for(uint32_t p=0 ; p < padding; p++)
		{
			el->appendInputData8(0xff);
		}

		HalExecCommand updateCmd;
		updateCmd.setTimeout(45000);
		updateCmd.elements.push_back(el);
		if(this->fetHandle->send(updateCmd)==false)
		{
			return false;
		}
		if(intCallback)
		{
			intCallback(BL_DATA_BLOCK_PROGRAMMED, 100-(requiredUpdates)*percent, 0);
		}
	}
	return true;
}

bool UpdateManagerFet::upCoreErase()
{
	FetControl * control=this->fetHandle->getControl();
	// create 0x55 command erase signature
	// command forces safecore to search for new core on reset
	std::vector<uint8_t> data_55;
	data_55.push_back(0x03);
	data_55.push_back(0x55);
	uint8_t id=control->createResponseId();
	data_55.push_back(id);
	data_55.push_back(0x00);

	control->sendData(data_55);
	control->clearResponse();
	return true;
}

bool UpdateManagerFet::upCoreWrite()
{
	return true;
}

//reads back flashed core data and compares it to data in core image
//returns: true, if data on FET and core image are equal
bool UpdateManagerFet::upCoreRead()
{
	return true;
}
/*EOF*/
