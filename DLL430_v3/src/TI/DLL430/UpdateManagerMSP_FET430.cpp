/*
 * UpdateManagerMSP_FET430.cpp
 *
 * Functionality for updating MSP-FET430UIF debugger
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
#include <VersionInfo.h>

#include "FetHandleV3.h"
#include "FetHandleManager.h"
#include "ConfigManagerV3.h"
#include "UpdateManagerMSP_FET430.h"
#include "DeviceDbManagerExt.h"
#include "DeviceInfo.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "Record.h"
#include "DeviceHandleV3.h"

#include "../../Bios/include/UifHal.h"
#include "../../Bios/include/UifBiosCore.h"
#include "../../Bios/include/ConfigureParameters.h"

using namespace TI::DLL430;
using namespace std;

UpdateManagerMSP_FET430::UpdateManagerMSP_FET430(FetHandleV3* fetHandle, ConfigManagerV3* configManagerV3)
 : fetHandle(fetHandle), configManagerV3(configManagerV3)
{
	updateCmd.setTimeout(20000);
}

UpdateManagerMSP_FET430::~UpdateManagerMSP_FET430(){}


bool UpdateManagerMSP_FET430::isUpdateRequired() const
{	 
	uint16_t expectedVersionmMajor = 0 , expectedVersionmMinor =0;
	Record fetHalImage(UifHalImage, UifHalImage_address, UifHalImage_length_of_sections, UifHalImage_sections);

	fetHalImage.getWordAtAdr(0x253C, &expectedVersionmMajor);
	fetHalImage.getWordAtAdr(0x253E, &expectedVersionmMinor);
	
	std::vector<uint8_t> sw_info;

	sw_info.push_back(expectedVersionmMajor&0xFF);
	sw_info.push_back(expectedVersionmMajor>>8);
	sw_info.push_back(expectedVersionmMinor&0xFF);
	sw_info.push_back(expectedVersionmMinor>>8);

	unsigned char major=sw_info.at(1);
	 VersionInfo exptectedHallVersion((((major&0xC0)>>6)+1),(major&0x3f),sw_info.at(0), 
		(sw_info.at(3)<<8)+sw_info.at(2));

 	bool isUpdateRequired = false;
	if (exptectedHallVersion.get() != (getHalVersion().get()))
	{
		isUpdateRequired = true;
	}
	if (checkCoreVersion() != 0)
	{
		isUpdateRequired = true;
	}
	return isUpdateRequired;
}


uint16_t UpdateManagerMSP_FET430::checkCoreVersion() const
{
	FetControl * control=this->fetHandle->getControl();
	
	//get current core version from FET
	const uint16_t actualFetCoreVersion = control->getFetCoreVersion();
	uint16_t expectedFetCoreVersion = 0;

	Record fetCoreImage(UifBiosCoreImage, UifBiosCoreImage_address, UifBiosCoreImage_length_of_sections, UifBiosCoreImage_sections);
	//get core version from image (core version is stored in address 0x4400)
	if(fetCoreImage.getWordAtAdr(0xFDD8, &expectedFetCoreVersion))
	{
		//if core versions do not match, update core
		if((expectedFetCoreVersion != actualFetCoreVersion))
		{
			return 1;
		}
	}
	return 0;
}

VersionInfo UpdateManagerMSP_FET430::getHalVersion() const
{
	std::vector<uint8_t> * sw_info=this->fetHandle->getSwVersion();	
	if(sw_info==NULL)
		return VersionInfo(0, 0, 0, 0);

	if(sw_info->size()<4)
		return VersionInfo(0, 0, 0, 0);

	unsigned char major=sw_info->at(1);
	return VersionInfo((((major&0xC0)>>6)+1),(major&0x3f),sw_info->at(0), 
		(sw_info->at(3)<<8)+sw_info->at(2));
}


bool UpdateManagerMSP_FET430::firmWareUpdate(const char* fname, UpdateNotifyCallback callback, bool*)
{
	FetControl* control=this->fetHandle->getControl();
	
	if(control == NULL)
		return false;

	const uint32_t biosVersion = getHalVersion().get();

	// core/HAL communication has changed with version 3.2.0.8
	// and after HAL update the core/HAL communication would not work
	// do not update as long as there is no direct core update 
	if((biosVersion > 30200000) && (biosVersion < 30200008) && (fname == NULL))
	{
		return false;
	}
	
	// core will be updated through internal image
	if (checkCoreVersion() != 0)
	{
		if (callback)
		{
			callback(BL_INIT,0,0);
		}
		if (!this->upInit(1))
		{
			return false;
		}
		if (callback)
		{
			callback(BL_ERASE_FIRMWARE,0,0);
		}
		if (!this->upCoreErase())
		{
			return false;
		}
		if (callback)
		{
			callback(BL_PROGRAM_FIRMWARE,0,0);
		}
		if (!this->upCoreWrite())
		{
			return false;
		}
		if (!this->upCoreRead())
		{
			return false;
		}
		if (callback)
		{
			callback(BL_EXIT,0,0);
		}
		
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

		boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(8));
	}

	FileFuncImpl firmware;

	if (fname)
	{
		//Do not allow using a file if there is no valid HAL on the UIF
		if ((biosVersion < 20000000) || !firmware.readFirmware(fname))
		{
			return false;
		}
	}
	else
	{
		firmware.readFirmware(UifHalImage, UifHalImage_address, UifHalImage_length_of_sections, UifHalImage_sections);
	}

	if ((firmware.getNumberOfSegments())==0)
		return false;

	// start HAL update routine
	if (callback)
	{
		callback(BL_INIT,0,0);
	}
	if (!this->upInit(1))
	{
		return false;
	}
	if (callback)
	{
		callback(BL_ERASE_FIRMWARE,0,0);
	}
	if (!this->upErase(firmware))
	{
		return false;
	}
	if (callback)
	{
		callback(BL_PROGRAM_FIRMWARE,0,0);
	}
	if (!this->upWrite(firmware, callback))
	{
		return false;
	}
	if (callback)
	{
		callback(BL_EXIT,0,0);
	}
	if (!this->upInit(0))
	{
		return false;
	}

	fetHandle->getControl()->resetCommunication();

	// activate the new HAL (in case of downgrade: change to VCP)
	HalExecCommand initCmd;
	HalExecElement* el = new HalExecElement(ID_Init);
	initCmd.elements.push_back(el);
	
	const bool initFailed = !this->fetHandle->send(initCmd);

	// give the firmware time to execute initialisation
	boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(1));

	//Only an error if no user specified file is used (ie. init will fail for a downgrade)
	if (initFailed && !fname)
	{
		return false;
	}

	if (!initFailed)
	{
		//Perform a reset to make sure everything is correctly initialized
		HalExecElement* el = new HalExecElement(ID_Zero);
		el->appendInputData8(STREAM_CORE_ZERO_PUC_RESET);

		HalExecCommand cmd;
		cmd.elements.push_back(el);

		fetHandle->send(cmd);
		boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(2));

		if(callback)
			callback(BL_UPDATE_DONE,0,0);
	}

	return true;
}

bool UpdateManagerMSP_FET430::upInit(unsigned char level)
{
	HalExecElement* el = new HalExecElement(ID_Zero, UpInit);
	el->setAddrFlag(false);
	el->appendInputData8(level);

	HalExecCommand cmd;
	cmd.elements.push_back(el);

	return this->fetHandle->send(cmd);
}

bool UpdateManagerMSP_FET430::upErase(const FileFuncImpl& firmware)
{
	for (size_t i = 0; i < firmware.getNumberOfSegments(); ++i)
	{
		const DownloadSegment *seg = firmware.getFirmwareSeg(i);
		if (seg == NULL)
			return false;

		HalExecElement* el = new HalExecElement(ID_Zero, UpErase);
		el->setAddrFlag(false);
		el->appendInputData32(seg->startAddress&0xfffffffe);
		el->appendInputData32(seg->size);

		updateCmd.elements.clear();
		updateCmd.elements.push_back(el);
		if (!this->fetHandle->send(updateCmd))
		{
			return false;
		}
	}
	return true;
}

bool UpdateManagerMSP_FET430::upWrite(const FileFuncImpl& firmware, UpdateNotifyCallback callback)
{
	uint32_t percent = 100/(uint32_t)firmware.getNumberOfSegments();

	for (size_t i = 0; i < firmware.getNumberOfSegments(); ++i)
	{
		if (callback)
			callback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)i*percent, 0);

		const DownloadSegment *seg = firmware.getFirmwareSeg(i);
	
		if (seg == NULL)
			return false;

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
		for (uint32_t n=0; n < seg->size; n++)
			el->appendInputData8(seg->data[n] & 0xff);

		for (uint32_t p=0 ; p < padding; p++)
			el->appendInputData8(0xff);

		updateCmd.elements.clear();		
		updateCmd.elements.push_back(el);
		if (!this->fetHandle->send(updateCmd))
		{
			return false;
		}
	}
	if(callback)
		callback(BL_DATA_BLOCK_PROGRAMMED,100,0);

	return true;
}


bool UpdateManagerMSP_FET430::upCoreErase()
{
	//create erase command
	updateCmd.elements.clear();
	HalExecElement* el = new HalExecElement(ID_Zero, UpErase);
	el->setAddrFlag(false);
	//append start address and length in bytes
	el->appendInputData32(0x2500);
	el->appendInputData32(0xbb00);

	updateCmd.elements.push_back(el);
	return this->fetHandle->send(updateCmd);
}

bool UpdateManagerMSP_FET430::upCoreWrite()
{
	Record fetCoreImage(UifBiosCoreImage, UifBiosCoreImage_address, UifBiosCoreImage_length_of_sections, UifBiosCoreImage_sections);

	updateCmd.elements.clear();
	HalExecElement* el = new HalExecElement(ID_Zero, UpWrite);
	el->setAddrFlag(false);
	//append flash start-address where following data will be flashed to
	el->appendInputData32(0x2500);
	//append number of data bytes, including number of sections ,section start addresses and core signature
	el->appendInputData32((fetCoreImage.getNumOfAllDataWords() + fetCoreImage.getNumOfManageWords(true))*2);
	//append core signature
	el->appendInputData32(Record::coreSignature);
	//append number of sections
	el->appendInputData16( (uint16_t) fetCoreImage.getSectCount());

	while(fetCoreImage.hasNextSect())
	{
		//append start address and length information of each section
		el->appendInputData16( (uint16_t) fetCoreImage.getSectStartAdr());
		el->appendInputData16( (uint16_t) fetCoreImage.getSectLength());

		while(fetCoreImage.sectHasNextWord())
		{
			el->appendInputData16(fetCoreImage.getNextWord());
		}
		fetCoreImage.nextSection();
	}
	updateCmd.elements.push_back(el);
	return this->fetHandle->send(updateCmd);
}

//reads back flashed core data and compares it to data in core image
//returns: true, if data on FET and core image are equal 
bool UpdateManagerMSP_FET430::upCoreRead()
{
	Record fetCoreImage(UifBiosCoreImage, UifBiosCoreImage_address, UifBiosCoreImage_length_of_sections, UifBiosCoreImage_sections);

	updateCmd.elements.clear();
	HalExecElement* el = new HalExecElement(ID_Zero, UpRead);
	el->setAddrFlag(false);
	//append start address for read access
	el->appendInputData32(0x2500);
	//append number of data bytes, including number of sections ,section start addresses and core signature
	el->appendInputData32((fetCoreImage.getNumOfAllDataWords() + fetCoreImage.getNumOfManageWords(true))*2);
	updateCmd.elements.push_back(el);
	
	if (!this->fetHandle->send(updateCmd))
	{
		return false;
	}
	//compare core signature
	if (el->getOutputAt32(0) != Record::coreSignature)
	{
		return false;
	}
	//compare number of sections
	if(el->getOutputAt16(4) != fetCoreImage.getSectCount())
	{
		return false;
	}
	uint32_t overhead = 6;

	//compare data of current section, including start adress and length
	while(fetCoreImage.hasNextSect())
	{
		if(el->getOutputAt16(fetCoreImage.getCurrentPosByte()-2 + overhead) != fetCoreImage.getSectStartAdr())
		{
			return false;
		}
		overhead += 2;
		if(el->getOutputAt16(fetCoreImage.getCurrentPosByte()-2 + overhead) != fetCoreImage.getSectLength())
		{
			return false;
		}
		overhead += 2;

		while(fetCoreImage.sectHasNextWord())
		{
			if(el->getOutputAt16(fetCoreImage.getCurrentPosByte()-2 + overhead) != fetCoreImage.getNextWord())
			{
				return false;
			}
		}
		fetCoreImage.nextSection();
	}
	return true;
}
/*EOF*/
