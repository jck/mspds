/*
* RF430FRL152H.cpp
*
* Definition of RF430FRL152H devices.
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
#include "LockableRamMemoryAccess.h"
#include "FramMemoryAccessBase.h"
#include "MpuWriteProtection.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;

typedef IdCode<0xFFFF, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFFFFFFFF> RF430FRL15xHIdMask;

template<const unsigned int versionId, const unsigned int activationKey>
struct RF430FRL15xH_Match : Match<
	IdCode<versionId, 0x0, 0x00, 0x00, 0x00, 0x00, 0x0, activationKey>,
	RF430FRL15xHIdMask> {};


struct RF430FRL15xH_TimerBase : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	RF430FRL15xH_TimerBase(ClockPair eUSCI, ClockPair SD) : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::RF13M, Eem::APOOL, eUSCI,		SD,
		Eem::Empty, Eem::TA3_0, Eem::Empty, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
		)
	{}
};

struct RF430FRL152H_Timer : RF430FRL15xH_TimerBase
{
	RF430FRL152H_Timer() : RF430FRL15xH_TimerBase(Eem::eUSCIB0, Eem::RFSD14) {}
};
struct RF430FRL153H_Timer : RF430FRL15xH_TimerBase
{
	RF430FRL153H_Timer() : RF430FRL15xH_TimerBase(Eem::Empty, Eem::RFSD14) {}
};
struct RF430FRL154H_Timer : RF430FRL15xH_TimerBase
{
	RF430FRL154H_Timer() : RF430FRL15xH_TimerBase(Eem::eUSCIB0, Eem::Empty) {}
};


typedef ClockInfo<GCC_EXTENDED, 0x0045, RF430FRL152H_Timer, EmptyEemClockNames> RF430FRL152H_Clock;
typedef ClockInfo<GCC_EXTENDED, 0x0045, RF430FRL153H_Timer, EmptyEemClockNames> RF430FRL153H_Clock;
typedef ClockInfo<GCC_EXTENDED, 0x0045, RF430FRL154H_Timer, EmptyEemClockNames> RF430FRL154H_Clock;


typedef MpuWriteProtection<0x190, 0x700> MpuFRL15x;

typedef MemoryInfo<
	MemoryArea::MAIN, RamType, Mapped, NotProtectable, Bits16Type,
	Size<0x7C0>, Offset<0xF840>, SegmentSize<0x1>,
	BankSize<0x0>, Banks<1>, NoMask, MemoryCreator<FramMemoryAccessBase<MpuFRL15x> >
> RF430FRL15xH_MainFramMemory;

template<class SizeType>
struct RF430FRL15xH_SystemRamMemory : MemoryInfo<
	MemoryArea::RAM, RamType, Mapped, NotProtectable, Bits16Type,
	SizeType, Offset<0x1C00>, SegmentSize<0x1>,
	BankSize<0x0>, Banks<1>, NoMask
> {};

typedef MemoryInfo<
	MemoryArea::BOOT, RomType, Mapped, NotProtectable, Bits16Type,
	Size<0x40>, Offset<0x1A00>, SegmentSize<0x1>, BankSize<0>, Banks<1>,
	NoMask, MemoryCreator<BootcodeRomAccess>
> RF430FRL15xH_BootCodeMemoryInfo;

template<class SizeType>
struct RF430FRL15xH_ApplicationRomMemoryInfo : MemoryInfo<
	MemoryArea::BOOT, RomType, Mapped, NotProtectable, Bits16Type,
	SizeType, Offset<0x4400>, SegmentSize<0x1>, BankSize<0>, Banks<1>,
	NoMask, MemoryCreator<BootcodeRomAccess>
> {};

typedef MemoryInfo <
	MemoryArea::RAM, RamType, Mapped, NotProtectable, Bits16Type,
	Size<0xE00>, Offset<0x1E00>, SegmentSize<0x1>, BankSize<0>, Banks<1>,
	NoMask
> RF430FRL15xH_DevelopmentMemoryInfo;


typedef MemoryList<boost::tuple <
	RF430FRL15xH_MainFramMemory,
	RF430FRL15xH_SystemRamMemory<Size<0x1000> >,
	RF430FRL15xH_BootCodeMemoryInfo,
	RF430FRL15xH_ApplicationRomMemoryInfo<Size<0x1C00> >,
	MSP430F5xxx_peripherl16lbitMemoryInfo,
	MSP430F5xxx_CPUMemoryInfo,
	MSP430F5xxx_EEMMemoryInfo
> > RF430FRL152xH_Memory;


typedef MemoryList < boost::tuple <
	RF430FRL15xH_MainFramMemory,
	RF430FRL15xH_SystemRamMemory<Size<0x200> >,
	RF430FRL15xH_BootCodeMemoryInfo,
	RF430FRL15xH_ApplicationRomMemoryInfo<Size<0xE00> >,
	RF430FRL15xH_DevelopmentMemoryInfo,
	MSP430F5xxx_peripherl16lbitMemoryInfo,
	MSP430F5xxx_CPUMemoryInfo,
	MSP430F5xxx_EEMMemoryInfo
> > RF430FRL152xH_Rom_Memory;


typedef VoltageInfo<1450, 1650, 0, 0, 0, 0, false> RF430FRL15xHVoltageInfo;


typedef Features<FLLPLUS, false, false, true, false, true> RF430FRL15xH_Features;


template<
	const char* description,
	const unsigned int versionId,
	const unsigned int activationCode,
	class ClockType,
	class MemoryType
>
struct RF430FRL15xH : Device<
	description,
	DefaultBits20Type,
	regular,
	RF430FRL15xH_Match<versionId, activationCode>,
	ExtraSmallEemMode,
	RF430FRL15xHVoltageInfo,
	ClockType,
	FunctionMappingXv2FRAM,
	FuncletMappingXv2FRAM,
	MemoryType,
	RF430FRL15xH_Features
> {};


extern const char RF430FRL152H[] = "RF430FRL152H";
extern const char RF430FRL153H[] = "RF430FRL153H";
extern const char RF430FRL154H[] = "RF430FRL154H";

static const DeviceRegistrator< RF430FRL15xH<RF430FRL152H, 0x81E7, 0xA55AA55A, RF430FRL152H_Clock, RF430FRL152xH_Memory> > regRF430FRL152H_type;
static const DeviceRegistrator< RF430FRL15xH<RF430FRL153H, 0x81FB, 0xA55AA55A, RF430FRL153H_Clock, RF430FRL152xH_Memory> > regRF430FRL153H_type;
static const DeviceRegistrator< RF430FRL15xH<RF430FRL154H, 0x81FC, 0xA55AA55A, RF430FRL154H_Clock, RF430FRL152xH_Memory> > regRF430FRL154H_type;

static const DeviceRegistrator< RF430FRL15xH<RF430FRL152H, 0x81E7, 0x5AA55AA5, RF430FRL152H_Clock, RF430FRL152xH_Rom_Memory> > regRF430FRL152H_Rom_type;
static const DeviceRegistrator< RF430FRL15xH<RF430FRL153H, 0x81FB, 0x5AA55AA5, RF430FRL153H_Clock, RF430FRL152xH_Rom_Memory> > regRF430FRL153H_Rom_type;
static const DeviceRegistrator< RF430FRL15xH<RF430FRL154H, 0x81FC, 0x5AA55AA5, RF430FRL154H_Clock, RF430FRL152xH_Rom_Memory> > regRF430FRL154H_Rom_type;
