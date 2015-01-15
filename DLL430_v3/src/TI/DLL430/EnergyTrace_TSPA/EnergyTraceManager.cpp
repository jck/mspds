/*
 * EnergyTraceManager.cpp
 *
 * Functionality for EnergyTrace.
 *
 * Copyright (c) 2007 - 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 * All rights reserved not granted herein.
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free,
 * non-exclusive license under copyrights and patents it now or hereafter
 * owns or controls to make, have made, use, import, offer to sell and sell ("Utilize")
 * this software subject to the terms herein.  With respect to the foregoing patent
 * license, such license is granted  solely to the extent that any such patent is necessary
 * to Utilize the software alone.  The patent license shall not apply to any combinations which
 * include this software, other than combinations with devices manufactured by or for TI (“TI Devices”).
 * No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license (including the
 * above copyright notice and the disclaimer and (if applicable) source code license limitations below)
 * in the documentation and/or other materials provided with the distribution
 *
 * Redistribution and use in binary form, without modification, are permitted provided that the following
 * conditions are met:
 *
 *	* No reverse engineering, decompilation, or disassembly of this software is permitted with respect to any
 *     software provided in binary form.
 *	* any redistribution and use are licensed by TI for use only with TI Devices.
 *	* Nothing shall obligate TI to provide you with source code for the software licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the source code are permitted
 * provided that the following conditions are met:
 *
 *   * any redistribution and use of the source code, including any resulting derivative works, are licensed by
 *     TI for use only with TI Devices.
 *   * any redistribution and use of any object code compiled from the source code and any resulting derivative
 *     works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TI AND TI’S LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <pch.h>
#include <float.h>

#include "EnergyTraceManager.h"
#include "EnergyTraceProcessorId7.h"
#include "EnergyTraceProcessorId8.h"
#include "FetHandleV3.h"
#include "HalExecCommand.h"
#include "HalResponse.h"
#include "HalResponseHandler.h"
#include "FetControl.h"
#include "PollingManager.h"


using namespace TI::DLL430;

static const uint32_t ENERGYTRACE_BUFFER_SIZE = 12;

//-------------------------------------------------------------
EnergyTraceManager::EnergyTraceManager(FetHandleV3* parent , PollingManager* pollingManager)
 : mParent(parent)
 , mCbx(0)
 , mPollingManager(pollingManager)
 , vcc(0)
{
	mPollingManager->setEnergyTraceCallback(boost::bind(&EnergyTraceManager::runEvent, this, _1));

	if (parent->getControl()->getFetToolId() == eZ_FET_WITH_DCDC)
	{
		calibrationValues.resize(2);
		const double resistors[2] = {DBL_MAX, 2200.0};
		resistorValues = vector<double>(resistors, resistors+2);

		uint16_t calResistors[] = {0, 1};
		calibrationResistors = vector<uint16_t>(calResistors, calResistors + (sizeof(calResistors) / sizeof(uint16_t))); // no load, Calibrate Resistor 4
		timerStep = 640;
	}

	if (parent->getControl()->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		calibrationValues.resize(5);
		const double resistors[5] = {DBL_MAX, 1000.0+0.75, 180+0.75, 82.0+0.75, 51.98};
		resistorValues = vector<double>(resistors, resistors+5);

		uint16_t calResistors[] = {0, 4, 1, 7};
		calibrationResistors = vector<uint16_t>(calResistors, calResistors + (sizeof(calResistors) / sizeof(uint16_t))); // no load, Calibrate Resistor 2 - 1000Ohm, Calibrate Resistor 0 - 82Ohm, Calibrate Resistor 0||1||2 - 52Ohm
		timerStep = 800;
	}
}

//-------------------------------------------------------------
EnergyTraceManager::~EnergyTraceManager ()
{
	mPollingManager->setEnergyTraceCallback(NULL);
}

bool EnergyTraceManager::ResetEnergyTrace()
{
	if(mDataProcessor)
	{
		mDataProcessor->Reset();
		mDataProcessor->setCalibrationValues(&calibrationValues[0], vcc);
		return true;
	}
	return false;
}

//-------------------------------------------------------------
bool EnergyTraceManager::startEnergyTrace(DebugEventTarget * cb, ETMode_t mode, ETCallback_mode callbackMode, DeviceHandle* devHandle)
{
	// Reset the processor at the start of a new EnergyTrace session
	if(cb!=0)
	{
		mCbx = cb;
	}

	boost::mutex::scoped_lock lock(callbackMutex);

	// Event type 7 ------------------------------------------------------------------------------------
	if(mode == ET_PROFILING_ANALOG_DSTATE)
	{
		mDataProcessor.reset(new EnergyTraceProcessorId7(calibrationValues.size(), ENERGYTRACE_BUFFER_SIZE));
		// Pass on the calibration values to the processor
		mDataProcessor->setTimerStep(timerStep);
		mDataProcessor->setResistorValues(&resistorValues[0]);
		mDataProcessor->setCalibrationValues(&calibrationValues[0], vcc);

		if((callbackMode == ET_CALLBACKS_ONLY_DURING_RUN) && (devHandle))
		{
			if(!mPollingManager->startEnergyTracePolling(ET_POLLING_ANALOG_DSTATE, ET_POLLING_GATED_ON))
			{
				return false;
			}
		}
		else
		{
			return false;
		}
	}

	// Event type 8 ---------------------------------------------------------------------------------------
	if(mode == ET_PROFILING_ANALOG)
	{
		mDataProcessor.reset(new EnergyTraceProcessorId8(calibrationValues.size(), ENERGYTRACE_BUFFER_SIZE));
		// Pass on the calibration values to the processor
		mDataProcessor->setTimerStep(timerStep);
		mDataProcessor->setResistorValues(&resistorValues[0]);
		mDataProcessor->setCalibrationValues(&calibrationValues[0], vcc);

		if(callbackMode == ET_CALLBACKS_ONLY_DURING_RUN)
		{
			if(!mPollingManager->startEnergyTracePolling(ET_POLLING_ANALOG, ET_POLLING_GATED_ON))
			{
				return false;
			}
		}
		else
		{
			if(!mPollingManager->startEnergyTracePolling(ET_POLLING_ANALOG, ET_POLLING_GATED_OFF))
			{
				return false;
			}
		}

	}
	return true;
}

//-------------------------------------------------------------
void EnergyTraceManager::runEvent(MessageDataPtr messageData)
{
	uint16_t eventMask = 0;
	(*messageData) >> eventMask;

	if(mCbx)
	{
		uint8_t numRecords = 0;
		uint8_t sizeOfRecords = 0;

		(*messageData) >> numRecords;
		(*messageData) >> sizeOfRecords;

		boost::mutex::scoped_lock lock(callbackMutex);

		if(this->mDataProcessor->AddData((void*)messageData->data(), numRecords * sizeOfRecords))
		{
			mCbx->event(DebugEventTarget::EnergyTraceData);
		}
	}
}

//-------------------------------------------------------------
void* EnergyTraceManager::getEnergyTraceBuffer()
{
	if(mDataProcessor)
	{
		return mDataProcessor->GetReadBufferPtr();
	}
	return NULL;
}

//-------------------------------------------------------------
size_t EnergyTraceManager::getEnergyTraceBufferSize()
{
	if(mDataProcessor)
	{
		return mDataProcessor->GetReadBufferSize();
	}
	return 0;
}

//-------------------------------------------------------------
void EnergyTraceManager::doCalibration(uint16_t vcc)
{
	if (mParent->getControl()->getFetToolId() == eZ_FET_WITH_DCDC)
	{
		calibrateResistor(vcc);
	}

	if (mParent->getControl()->getFetToolId() == MSP_FET_WITH_DCDC)
	{
		calibrateResistor(vcc);

		calibrationValues[4] = calibrationValues[3];
		calibrationValues[3] = calibrationValues[2];

		double temp1 = calibrationValues[0] +(calibrationValues[1] - calibrationValues[0]) * resistorValues[1] / resistorValues[2];
		double temp2 = calibrationValues[0] +(calibrationValues[3] - calibrationValues[0]) * resistorValues[3] / resistorValues[2];

		calibrationValues[2] = ((resistorValues[1] - resistorValues[2]) / (resistorValues[1] - resistorValues[3])) * temp1 +
			((1 - ((resistorValues[1] - resistorValues[2]) / (resistorValues[1] - resistorValues[3]))) * temp2);
	}

	this->vcc = vcc;
}

//-------------------------------------------------------------
void EnergyTraceManager::calibrateResistor(uint16_t vcc)
{
	// send Calibration comand to dcdc Firmware
	HalExecElement* el = new HalExecElement(ID_Zero, dcdcCalibrate);
	el->setAddrFlag(false);

	el->appendInputData16(calibrationResistors.size()); //count resistors

	// now add Resistors
	for(size_t i = 0; i < calibrationResistors.size(); i++)
	{
		el->appendInputData16(calibrationResistors[i]);
	}
	//cal. VCC
	el->appendInputData16(vcc);

	HalExecCommand dcdcCmd;
	dcdcCmd.setTimeout(20000);
	dcdcCmd.elements.push_back(el);
	mParent->getControl()->send(dcdcCmd);

	int pos = 0;
	for(size_t i = 0; i < calibrationResistors.size(); i++, pos+=8)
	{
		double ticks = el->getOutputAt32(pos);
		double time = el->getOutputAt32(pos + 4) * timerStep; // time in ns
		calibrationValues[i] = ticks * 1000.0 * 1000.0 / time;
	}
}

//-------------------------------------------------------------
void EnergyTraceManager::pausePolling()
{
	mPollingManager->pausePolling();
}

//-------------------------------------------------------------
void EnergyTraceManager::resumePolling()
{
	mPollingManager->resumePolling();
}

//-------------------------------------------------------------
void EnergyTraceManager::stopPolling()
{
	mPollingManager->stopEnergyTracePolling();
}
