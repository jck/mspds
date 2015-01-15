/*
 * MSP430F662x.cpp
 *
 * Definition MSP430F662x devices.
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
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
#include "UsbTypes.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;


static const ClockPair FG_TA3_1 = {"Timer1_A3", 0x8E};
static const ClockPair FG_TA3_2 = {"Timer2_A3", 0x8F};

struct MSP430FG662x_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FG662x_EemTimer()
		  : EemTimerImpl(
			Eem::Empty, Eem::LCD_B,	Eem::DAC12_0, Eem::COMP_B,
			Eem::Empty, Eem::RTC,   Eem::USCI3, Eem::USCI2,
			Eem::USCI1, Eem::USCI0, Eem::Empty, Eem::TB7_0,
			FG_TA3_2,   FG_TA3_1,   Eem::TA5_0, Eem::WDT_A,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
			Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty)
	{}
};


typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430FG662x_EemTimer, EmptyEemClockNames> MSP430FG662x_ClockInfo;


template<const unsigned int nrBanks>
struct MSP430FG662x_MainFlashMemory : MemoryInfo<
									MemoryArea::MAIN, FlashType, Mapped, NotProtectable, Bits16Type,
									Size<nrBanks*0x8000> , Offset<0x4400>, SegmentSize<0x200>,
									BankSize<0x8000>, Banks<nrBanks>, NoMask> {};


template<const unsigned int nrBanks>
struct MSP430FG662x_Memory : MemoryList<boost::tuple<
									MSP430FG662x_MainFlashMemory<nrBanks>,
									MSP430F5xxx_InfoFlashMemoryInfo,
									MSP430F5xxx_BootFlashMemoryInfo,
									MSP430F5xxx_BootCodeMemoryInfo,
									UsbTypeRamInfo,
									UsbTypeSystemRamInfo< Size<0x2000>, Banks<4> >,
									MSP430F5xxx_peripherl16lbitMemoryInfo,
									MSP430F5xxx_CPUMemoryInfo,
									MSP430F5xxx_EEMMemoryInfo> > {};

template<const unsigned int nrBanks>
struct MSP430FG642x_Memory : MemoryList<boost::tuple<
									MSP430FG662x_MainFlashMemory<nrBanks>,
									MSP430F5xxx_InfoFlashMemoryInfo,
									MSP430F5xxx_BootFlashMemoryInfo,
									MSP430F5xxx_BootCodeMemoryInfo,
									MSP430F5xxx_SystemRamInfo< Size<0x2800> >,
									MSP430F5xxx_peripherl16lbitMemoryInfo,
									MSP430F5xxx_CPUMemoryInfo,
									MSP430F5xxx_EEMMemoryInfo> > {};

template<
	const char* description,
	const unsigned int versionId,
	class MemoryInfo
>
struct MSP430FG662x : Device<
							description,
							DefaultBits20Type,
							enhanced,
							MSP430F5xxx_Match<versionId>,
							LargeEemMode,
							MSP430F5xxx_DefaultVoltageTestVpp,
							MSP430FG662x_ClockInfo,
							FunctionMappingXv2,
							FuncletMappingXv2,
							MemoryInfo,
							MSP430F5xxx_Features,
							MSP430F5xxx_ExtFeatures,
							PowerSettings<
								0x0, // Test reg mask
								0x0, // Test reg default
								0x0, // Test reg value to enable LPMx.5
								0x0, // Test reg value to disable LPMx.5
								0x4020,	  // 3V Test reg mask
								0x0001,   // 3V Test reg default
								0x4020,	  // 3V Test reg value to enable LPMx.5
								0x4020	  // 3V Test reg value to disable LPMx.5
							>
						> {};



extern const char MSP430FG6626[] = "MSP430FG6626";
extern const char MSP430FG6625[] = "MSP430FG6625";

extern const char MSP430FG6426[] = "MSP430FG6426";
extern const char MSP430FG6425[] = "MSP430FG6425";


static const DeviceRegistrator< MSP430FG662x<MSP430FG6626, 0x8234, MSP430FG662x_Memory<4> > > regMSP430FG6626;
static const DeviceRegistrator< MSP430FG662x<MSP430FG6625, 0x8235, MSP430FG662x_Memory<2> > > regMSP430FG6625;

static const DeviceRegistrator< MSP430FG662x<MSP430FG6426, 0x8236, MSP430FG642x_Memory<4> > > regMSP430FG6426;
static const DeviceRegistrator< MSP430FG662x<MSP430FG6425, 0x8237, MSP430FG642x_Memory<2> > > regMSP430FG6425;
