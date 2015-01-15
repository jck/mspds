/*
 * MSP430F52xx.cpp
 *
 * Definition MSP430F52xx devices.
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

template< const unsigned int offset, const unsigned int nrBanks >
struct MSP430F52xx_MainFlashMemory : MemoryInfo<
										MemoryArea::MAIN, FlashType, Mapped, NotProtectable, Bits16Type,
										Size< nrBanks * 0x8000> , Offset<offset>, SegmentSize<0x200>, 
										BankSize<0x8000>, Banks<nrBanks>, NoMask> {};

template< const unsigned int size >
struct MSP430F52xx_SystemRamInfo : MemoryInfo<
										MemoryArea::RAM, RamType, Mapped, NotProtectable, Bits16Type,
										Size<size> , Offset<0x2400>, SegmentSize<0x1>, 
										BankSize<0x0>, Banks<4>, NoMask> {};


const ClockPair MSP430F52xxTimerTA3_1 = {"Timer1_A3", 0x8E};
const ClockPair MSP430F52xxTimerTA3_2 = {"Timer2_A3", 0x8F};


struct MSP430F52xx_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430F52xx_EemTimer(const ClockPair& adc): EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_B, 
		adc,		Eem::RTC,	Eem::USCI3,	Eem::USCI2,
		Eem::USCI1, Eem::USCI0, Eem::Empty, Eem::TB7_0,
		MSP430F52xxTimerTA3_2, MSP430F52xxTimerTA3_1, Eem::TA5_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty)
	{}
};


struct MSP430F522x_EemTimer : MSP430F52xx_EemTimer {
	MSP430F522x_EemTimer() :  MSP430F52xx_EemTimer(EemTimerImpl::Timer::ADC12_A) {}		
};

struct MSP430F521x_EemTimer : MSP430F52xx_EemTimer {
	MSP430F521x_EemTimer() :  MSP430F52xx_EemTimer(EemTimerImpl::Timer::Empty) {}
};


typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430F522x_EemTimer, EmptyEemClockNames> F522x_ClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x041F, MSP430F521x_EemTimer, EmptyEemClockNames> F521x_ClockInfo;


template<
	const char* description,
	const unsigned int versionId,
	class ClockInfo,
	const unsigned int flashOffset,
	const unsigned int nrFlashBanks,
	const unsigned int ramSize
>
struct MSP430F52xx : Device<
						description, 
						DefaultBits20Type, 
						enhanced, 
						MSP430F5xxx_Match<versionId>, 
						SmallEemMode, 
						MSP430F5xxx_DefaultVoltageTestVpp,
						ClockInfo,
						FunctionMappingXv2,
						FuncletMappingXv2,
						MemoryList<boost::tuple<
							MSP430F52xx_MainFlashMemory<flashOffset, nrFlashBanks>,
							MSP430F5xxx_InfoFlashMemoryInfo, 
							MSP430F5xxx_BootFlashMemoryInfo, 
							MSP430F5xxx_BootCodeMemoryInfo,
							MSP430F52xx_SystemRamInfo<ramSize>, 
							MSP430F5xxx_peripherl16lbitMemoryInfo, 
							MSP430F5xxx_CPUMemoryInfo, 
							MSP430F5xxx_EEMMemoryInfo
						> >, //until C++0x, the space between the brackets is important
						MSP430F5xxx_Features,
						MSP430F5xxx_ExtFeatures
					> {};


extern const char MSP430F5229[] = "MSP430F5229";
extern const char MSP430F5228[] = "MSP430F5228";
extern const char MSP430F5227[] = "MSP430F5227";
extern const char MSP430F5224[] = "MSP430F5224";
extern const char MSP430F5223[] = "MSP430F5223";
extern const char MSP430F5222[] = "MSP430F5222";
extern const char MSP430F5219[] = "MSP430F5219";
extern const char MSP430F5218[] = "MSP430F5218";
extern const char MSP430F5217[] = "MSP430F5217";
extern const char MSP430F5214[] = "MSP430F5214";
extern const char MSP430F5213[] = "MSP430F5213";
extern const char MSP430F5212[] = "MSP430F5212";

extern const char MSP430F5249[] = "MSP430F5249";
extern const char MSP430F5247[] = "MSP430F5247";
extern const char MSP430F5244[] = "MSP430F5244";
extern const char MSP430F5242[] = "MSP430F5242";
extern const char MSP430F5239[] = "MSP430F5239";
extern const char MSP430F5237[] = "MSP430F5237";
extern const char MSP430F5234[] = "MSP430F5234";
extern const char MSP430F5232[] = "MSP430F5232";

extern const char MSP430F5259[] = "MSP430F5259";
extern const char MSP430F5258[] = "MSP430F5258";
extern const char MSP430F5257[] = "MSP430F5257";
extern const char MSP430F5256[] = "MSP430F5256";
extern const char MSP430F5255[] = "MSP430F5255";
extern const char MSP430F5254[] = "MSP430F5254";
extern const char MSP430F5253[] = "MSP430F5253";
extern const char MSP430F5252[] = "MSP430F5252";

static const DeviceRegistrator< MSP430F52xx<MSP430F5229, 0x8151, F522x_ClockInfo, 0x4400, 4, 0x2000> > regMSP430F5229;
static const DeviceRegistrator< MSP430F52xx<MSP430F5228, 0x8150, F522x_ClockInfo, 0x4400, 3, 0x2000> > regMSP430F5228;
static const DeviceRegistrator< MSP430F52xx<MSP430F5227, 0x814F, F522x_ClockInfo, 0x4400, 2, 0x2000> > regMSP430F5227;
static const DeviceRegistrator< MSP430F52xx<MSP430F5224, 0x814C, F522x_ClockInfo, 0x4400, 4, 0x2000> > regMSP430F5224;
static const DeviceRegistrator< MSP430F52xx<MSP430F5223, 0x814B, F522x_ClockInfo, 0x4400, 3, 0x2000> > regMSP430F5223;
static const DeviceRegistrator< MSP430F52xx<MSP430F5222, 0x814A, F522x_ClockInfo, 0x4400, 2, 0x2000> > regMSP430F5222;

static const DeviceRegistrator< MSP430F52xx<MSP430F5219, 0x8147, F521x_ClockInfo, 0x4400, 4, 0x2000> > regMSP430F5219;
static const DeviceRegistrator< MSP430F52xx<MSP430F5218, 0x8146, F521x_ClockInfo, 0x4400, 3, 0x2000> > regMSP430F5218;
static const DeviceRegistrator< MSP430F52xx<MSP430F5217, 0x8145, F521x_ClockInfo, 0x4400, 2, 0x2000> > regMSP430F5217;
static const DeviceRegistrator< MSP430F52xx<MSP430F5214, 0x8142, F521x_ClockInfo, 0x4400, 4, 0x2000> > regMSP430F5214;
static const DeviceRegistrator< MSP430F52xx<MSP430F5213, 0x8141, F521x_ClockInfo, 0x4400, 3, 0x2000> > regMSP430F5213;
static const DeviceRegistrator< MSP430F52xx<MSP430F5212, 0x8140, F521x_ClockInfo, 0x4400, 2, 0x2000> > regMSP430F5212;


static const DeviceRegistrator< MSP430F52xx<MSP430F5259, 0x81FF, F522x_ClockInfo, 0xA400, 4, 0x8000> > regMSP430F5259;
static const DeviceRegistrator< MSP430F52xx<MSP430F5258, 0x8200, F521x_ClockInfo, 0xA400, 4, 0x8000> > regMSP430F5258;
static const DeviceRegistrator< MSP430F52xx<MSP430F5257, 0x8201, F522x_ClockInfo, 0xA400, 4, 0x4000> > regMSP430F5257;
static const DeviceRegistrator< MSP430F52xx<MSP430F5256, 0x8202, F521x_ClockInfo, 0xA400, 4, 0x4000> > regMSP430F5256;
static const DeviceRegistrator< MSP430F52xx<MSP430F5255, 0x8203, F522x_ClockInfo, 0xA400, 4, 0x8000> > regMSP430F5255;
static const DeviceRegistrator< MSP430F52xx<MSP430F5254, 0x8204, F521x_ClockInfo, 0xA400, 4, 0x8000> > regMSP430F5254;
static const DeviceRegistrator< MSP430F52xx<MSP430F5253, 0x8205, F522x_ClockInfo, 0xA400, 4, 0x4000> > regMSP430F5253;
static const DeviceRegistrator< MSP430F52xx<MSP430F5252, 0x8206, F521x_ClockInfo, 0xA400, 4, 0x4000> > regMSP430F5252;

static const DeviceRegistrator< MSP430F52xx<MSP430F5249, 0x81F3, F522x_ClockInfo, 0x4400, 4, 0x2000> > regMSP430F5249;
static const DeviceRegistrator< MSP430F52xx<MSP430F5247, 0x81F4, F522x_ClockInfo, 0x4400, 2, 0x2000> > regMSP430F5247;
static const DeviceRegistrator< MSP430F52xx<MSP430F5244, 0x81F5, F522x_ClockInfo, 0x4400, 4, 0x2000> > regMSP430F5244;
static const DeviceRegistrator< MSP430F52xx<MSP430F5242, 0x81F6, F522x_ClockInfo, 0x4400, 2, 0x2000> > regMSP430F5242;

static const DeviceRegistrator< MSP430F52xx<MSP430F5239, 0x81F7, F521x_ClockInfo, 0x4400, 4, 0x2000> > regMSP430F5239;
static const DeviceRegistrator< MSP430F52xx<MSP430F5237, 0x81F8, F521x_ClockInfo, 0x4400, 2, 0x2000> > regMSP430F5237;
static const DeviceRegistrator< MSP430F52xx<MSP430F5234, 0x81F9, F521x_ClockInfo, 0x4400, 4, 0x2000> > regMSP430F5234;
static const DeviceRegistrator< MSP430F52xx<MSP430F5232, 0x81FA, F521x_ClockInfo, 0x4400, 2, 0x2000> > regMSP430F5232;
