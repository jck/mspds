/*
 * MSP430G2xx5.cpp
 *
 * Definition MSP430G2xx5 devices.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

typedef ClockInfo<GCC_STANDARD_I, 0x60D7, EmptyEemTimer, TAClkEemClockNames> MSP430G2xx5_ClockInfo;

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00> MSP430G2xx5IdMask;

typedef Match< IdCode<0x5529, 0,0,0,0,0,0>, MSP430G2xx5IdMask> MSP430G2xx5_Match;

typedef Features<BC_2xx, false, true, true, false, false> MSP430G2xx5_Features;


template<
	const char* description,
	class MatchType,
	const unsigned int FlashSize,
	const unsigned int FlashOffset,
	const unsigned int RamSize,
	const unsigned int Ram2Size,
	class Features = DefaultFeatures
>
struct MSP430G2xx5 : Device<
		description, 
		DefaultBits16Type, 
		regular, 
		MatchType,
		LowEemMode,
		MSP430F2xxx_DefaultVoltageTestVpp,
		MSP430G2xx5_ClockInfo,
		FunctionMappingNone,
		FuncletMapping1_2xx,
		MemoryList<boost::tuple<
			MSP430F2xxx_MainFlashMemory< Size<FlashSize>, Offset<FlashOffset> >, 
			MSP430F2xxx_InfoFlashMemoryInfo,
			MSP430F2xxx_BootFlashMemoryInfo,
			MSP430F2xxx_SystemRamInfo< Size<RamSize> >,
			MSP430F2xxx_SystemRam2Info< Size<Ram2Size> >,
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_330f1f0fffff>,
			MSP430F2xxx_CPUMemoryInfo,
			MSP430F2xxx_EEMMemoryInfo
		> >, //until C++0x, the space between the brackets is important
		Features
	> {};


extern const char MSP430G2955[] = "MSP430G2x55";


static const DeviceRegistrator< 
	MSP430G2xx5<MSP430G2955, MSP430G2xx5_Match, 0xDF00, 0x2100, 0x800, 0x1000, MSP430G2xx5_Features> 
> regMSP430G2955;
