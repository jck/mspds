/*
 * MSP430I20xx.cpp
 *
 * Definition MSP430F20xx devices.
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


struct i20xxClockNames : EemClocksImpl
{
	typedef EemClocksImpl::Clocks Clock;
	i20xxClockNames() : EemClocksImpl(
		Clock::Empty, Clock::Empty, Clock::Empty, Clock::Empty,
		Clock::Empty, Clock::Empty, Clock::Empty, Clock::Empty,
		Clock::Empty, Clock::Empty, Clock::TACLK, Clock::Empty,
		Clock::Empty, Clock::Empty, Clock::ACLK, Clock::Empty
		)
	{}
};

typedef ClockInfo<GCC_STANDARD_I, 0x60D7, EmptyEemTimer, i20xxClockNames> MSP430I20xx_ClockInfo;

template<class FlashSizeType, class OffsetType>
struct MSP430I20xx_MainFlashMemory : MemoryInfo<
				MemoryArea::MAIN, FlashType, Mapped, NotProtectable, Bits16Type,
				FlashSizeType , OffsetType, SegmentSize<0x400>, BankSize<0x10000>, Banks<1>,
				NoMask> {};

typedef MemoryInfo<
	MemoryArea::INFO, FlashType, Mapped, NotProtectable, Bits16Type,
	Size<0x400> , Offset<0x1000>, SegmentSize<0x400>, BankSize<0x400>, Banks<1>,
	NoMask, MemoryCreator<InformationFlashAccess>
> MSP430I20xx_InfoFlashMemoryInfo;

typedef IdCode<0xFFFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00> MSP430I40xxIdMask;

struct MSP430I20xx_Match : Match< IdCode<0x2040, 0x0, 0, 0, 0, 0, 0>, MSP430I40xxIdMask> {};

extern const uint8_t sfrMaskData_430I[SFR_MASK_SIZE] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
typedef MemoryMask<sfrMaskData_430I, sizeof(sfrMaskData_430I)> sfrMask_430I;

template<class FlashOffset, class FlashSize>
struct MemoryModel : MemoryList<boost::tuple<
			MSP430I20xx_MainFlashMemory<FlashSize, FlashOffset>,
			MSP430I20xx_InfoFlashMemoryInfo,
			MSP430F2xxx_BootFlashMemoryInfo,
			MSP430F2xxx_SystemRamInfo< Size<0x800> >,
			MSP430F1_2_4xxx_peripherl16lbitMemoryInfo,
			MSP430F1_2_4xxx_peripherl8lbitMemoryInfo<sfrMask_430I>,
			MSP430F2xxx_CPUMemoryInfo,
			MSP430F2xxx_EEMMemoryInfo
		> > {};

template<
	const char* description,
	class MemoryModelType
>
struct MSP430I20xx_type : Device<
		description,
		DefaultBits16Type,
		regular,
		MSP430I20xx_Match,
		LowEemMode,
		MSP430F2xxx_DefaultVoltageTestVpp,
		MSP430I20xx_ClockInfo,
		FunctionMapping430I,
		FuncletMapping430I,
		MemoryModelType
	> {};


typedef MemoryModel< Offset<0x8000>, Size<0x8000> > MemoryI20xx;

extern const char MSP430I2040[] = "MSP430I204x_I203x_I202x";

static const DeviceRegistrator< MSP430I20xx_type<MSP430I2040, MemoryI20xx> > regMSP430I2040;
