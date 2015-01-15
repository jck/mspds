/*
 * MSP430FR69xx.cpp
 *
 * Definition MSP430FR69xx.
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
#include "MpuFRx.h"
#include "FramMemoryAccessFRx9.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;


typedef IdCode<0xFFFF, 0xFFFF, 0xFF, 0x00, 0x00, 0x00, 0x0, 0x0, 0xFF> MSP430FR69xxPGIdMask;

template<const unsigned int versionId, const unsigned char revision, const unsigned char revisionMax>
struct MSP430FR69xx_Match : Match<
						IdCode<versionId, 0x00, revision, 0x00, 0x00, 0x00, 0x0,0x00, revisionMax>, MSP430FR69xxPGIdMask> {};

static const ClockPair FR_TA2_2 = {"Timer2_A2", 0x8B};
static const ClockPair FR_TA5_3 = {"Timer3_A5", 0x91};

struct MSP430FR69xx_EemTimerLarge : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FR69xx_EemTimerLarge(const ClockPair& lcd) : EemTimerImpl(
		Eem::PORT, lcd, Eem::Empty, Eem::COMP_E,
		Eem::ADC12_B, Eem::RTC, Eem::eUSCIB1, Eem::eUSCIB0,
		Eem::eUSCIA1, Eem::eUSCIA0, Eem::TB7_0, Eem::TA3_0,
		Eem::TA3_1, FR_TA2_2,   FR_TA5_3,   Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct FR69xx_Timer: MSP430FR69xx_EemTimerLarge {
	FR69xx_Timer() : MSP430FR69xx_EemTimerLarge(Eem::Empty) {}
};

struct FR69xx_Timer_Lcd : MSP430FR69xx_EemTimerLarge {
	FR69xx_Timer_Lcd() : MSP430FR69xx_EemTimerLarge(Eem::LCD_C) {}
};

typedef ClockInfo<GCC_EXTENDED, 0x243F, FR69xx_Timer, EmptyEemClockNames> FR69xx_ClockInfo;
typedef ClockInfo<GCC_EXTENDED, 0x243F, FR69xx_Timer_Lcd, EmptyEemClockNames> FR69xx_LcdClockInfo;

typedef Trigger<EMEX_SMALL_5XX, 0x03, 0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00> FR69xxTrigger;

typedef EemInfo<0x00, 0x01, 0x01, FR69xxTrigger, SmallSequencer> FR69xxEemMode;


template<class SizeType, class OffsetType>
struct MSP430FR69xx_MainFramMemory : MemoryInfo<
	MemoryArea::MAIN, RamType, Mapped, NotProtectable, Bits16Type,
	SizeType, OffsetType, SegmentSize<0x01>,
	BankSize<0x0>, Banks<1>, NoMask, MemoryCreator<FramMemoryAccessFRx9<MpuFRx> >
> {};


typedef MemoryInfo<
	MemoryArea::INFO, RamType, Mapped, NotProtectable, Bits16Type,
	Size<0x200> , Offset<0x1800>, SegmentSize<0x01>, BankSize<0x0>, Banks<1>,
	NoMask, MemoryCreator<FramMemoryAccessFRx9<MpuFRx> >
> MSP430FR69xx_InfoFramMemoryInfo;


typedef MemoryInfo<
	MemoryArea::BSL, RomType, Mapped, Protectable, Bits16Type,
	Size<0x800> , Offset<0x1000>, SegmentSize<0x01>, BankSize<0>, Banks<1>,
	NoMask, MemoryCreator<BslRomAccess>
> MSP430FR69xx_BootFramMemoryInfo;

typedef Features<MOD_OSC, false, true, true, false, true> MSP430FR69xx_Features;

struct FunctionMappingMSP430FR69xx : public FunctionMappingBase
{
	FunctionMappingMSP430FR69xx(){
		FunctionMappingImpl::fcntMap_ = boost::assign::map_list_of
			(ID_BlowFuse, ID_BlowFuseFram)
			(ID_SyncJtag_AssertPor_SaveContext, ID_SyncJtag_AssertPor_SaveContextXv2)
			(ID_SyncJtag_Conditional_SaveContext,ID_SyncJtag_Conditional_SaveContextXv2)
			(ID_RestoreContext_ReleaseJtag,ID_RestoreContext_ReleaseJtagXv2)
			(ID_ReadMemBytes,ID_ReadMemWordsXv2)
			(ID_ReadMemWords,ID_ReadMemWordsXv2)
			(ID_ReadMemQuick,ID_ReadMemQuickXv2)
			(ID_WriteMemBytes,ID_WriteMemWordsXv2)
			(ID_WriteMemWords,ID_WriteMemWordsXv2)
			(ID_EemDataExchange,ID_EemDataExchangeXv2)
			(ID_ReadAllCpuRegs,ID_ReadAllCpuRegsXv2)
			(ID_WriteAllCpuRegs,ID_WriteAllCpuRegsXv2)
			(ID_Psa,ID_PsaXv2)
			(ID_ExecuteFunclet,ID_ExecuteFuncletXv2)
			(ID_WaitForStorage,ID_WaitForStorageX)
			(ID_WaitForEem,ID_PollJStateReg)
			(ID_SingleStep,ID_SingleStepJStateXv2);
		}
};

struct FunctionMappingMSP430FR69xxPG20 : public FunctionMappingMSP430FR69xx
{
	FunctionMappingMSP430FR69xxPG20(){
		ReplacePair(ID_WriteFramQuickXv2,ID_WriteMemWordsXv2);
	}
};

typedef PowerSettings<0x00010018, // Test reg mask
					  0x00010000, // Test reg default
					  0x00010018, // Test reg value to enable LPMx.5
					  0x00010000, // Test reg value to disable LPMx.5
					  0xC0A0,	  // 3V Test reg mask
					  0x0000,     // 3V Test reg default
					  0xC020,	  // 3V Test reg value to enable LPMx.5
					  0x40A0>	  // 3V Test reg value to disable LPMx.5
					  MSP430FR69xx_PowerSettings;


template<
	const char* description,
	const unsigned int versionId,
	class ClockInfo,
	class MainFramSize
>
struct MSP430FR698x_type : Device<
	description,
	DefaultBits20Type,
	regular,
	MSP430FR69xx_Match<versionId, 0x21, 0xFF>,
	FR69xxEemMode,
	MSP430F5xxx_DefaultVoltageTestVpp,
	ClockInfo,
	FunctionMappingMSP430FR69xx,
	FuncletMappingXv2FRAM,
	MemoryList<boost::tuple<
		MSP430FR69xx_MainFramMemory<MainFramSize, Offset<0x4400> >,
		MSP430FR69xx_InfoFramMemoryInfo,
		MSP430FR69xx_BootFramMemoryInfo,
		MSP430F5xxx_BootCodeMemoryInfo,
		MSP430F5xxx_SystemRamInfo< Size<0x800> >,
		MSP430F5xxx_peripherl16lbitMemoryInfo,
		MSP430F5xxx_CPUMemoryInfo,
		MSP430F5xxx_EEMMemoryInfo
	> >, //until C++0x, the space between the brackets is important
	MSP430FR69xx_Features,
	NoExtendedFeatures,
	MSP430FR69xx_PowerSettings
> {};


extern const char MSP430FR6989[] = "MSP430FR6989";
extern const char MSP430FR6988[] = "MSP430FR6988";
extern const char MSP430FR6987[] = "MSP430FR6987";

extern const char MSP430FR5989[] = "MSP430FR5989";
extern const char MSP430FR5988[] = "MSP430FR5988";
extern const char MSP430FR5987[] = "MSP430FR5987";
extern const char MSP430FR5986[] = "MSP430FR5986";

extern const char MSP430FR6889[] = "MSP430FR6889";
extern const char MSP430FR6888[] = "MSP430FR6888";
extern const char MSP430FR6887[] = "MSP430FR6887";

extern const char MSP430FR5889[] = "MSP430FR5889";
extern const char MSP430FR5888[] = "MSP430FR5888";
extern const char MSP430FR5887[] = "MSP430FR5887";

extern const char MSP430FR6979[] = "MSP430FR6979";
extern const char MSP430FR6977[] = "MSP430FR6977";

extern const char MSP430FR6879[] = "MSP430FR6879";
extern const char MSP430FR6877[] = "MSP430FR6877";

extern const char MSP430FR6928[] = "MSP430FR6928";
extern const char MSP430FR6927[] = "MSP430FR6927";


static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR6989, 0x81A8, FR69xx_LcdClockInfo, Size<0x1FC00> > > regMSP430FR6989;
static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR6988, 0x81A7, FR69xx_LcdClockInfo, Size<0x17C00> > > regMSP430FR6988;
static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR6987, 0x81A6, FR69xx_LcdClockInfo, Size<0x0FC00> > > regMSP430FR6987;

static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR5989, 0x81AB, FR69xx_ClockInfo, Size<0x1FC00> > > regMSP430FR5989;
static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR5988, 0x81AA, FR69xx_ClockInfo, Size<0x17C00> > > regMSP430FR5988;
static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR5987, 0x81A9, FR69xx_ClockInfo, Size<0x0FC00> > > regMSP430FR5987;
static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR5986, 0x81DF, FR69xx_ClockInfo, Size<0xBC00> > > regMSP430FR5986;

static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR6889, 0x81C0, FR69xx_LcdClockInfo, Size<0x1FC00> > > regMSP430FR6889;
static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR6888, 0x81BF, FR69xx_LcdClockInfo, Size<0x17C00> > > regMSP430FR6888;
static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR6887, 0x81BE, FR69xx_LcdClockInfo, Size<0x0FC00> > > regMSP430FR6887;

static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR5889, 0x81C3, FR69xx_ClockInfo, Size<0x1FC00> > > regMSP430FR5889;
static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR5888, 0x81C2, FR69xx_ClockInfo, Size<0x17C00> > > regMSP430FR5888;
static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR5887, 0x81C1, FR69xx_ClockInfo, Size<0x0FC00> > > regMSP430FR5887;

static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR6979, 0x81AE, FR69xx_LcdClockInfo, Size<0x1FC00> > > regMSP430FR6979;
static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR6977, 0x81AC, FR69xx_LcdClockInfo, Size<0x0FC00> > > regMSP430FR6977;

static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR6879, 0x81C6, FR69xx_LcdClockInfo, Size<0x1FC00> > > regMSP430FR6879;
static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR6877, 0x81C4, FR69xx_LcdClockInfo, Size<0x0FC00> > > regMSP430FR6877;

static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR6928, 0x81B3, FR69xx_LcdClockInfo, Size<0x17C00> > > regMSP430FR6928;
static const DeviceRegistrator<	MSP430FR698x_type<MSP430FR6927, 0x81B2, FR69xx_LcdClockInfo, Size<0x0FC00> > > regMSP430FR6927;



template<
	const char* description,
	const unsigned int versionId,
	class ClockInfo,
	class MainFramSize,
	class MainFramOffset
>
struct MSP430FR697x_type : Device<
	description,
	DefaultBits20Type,
	regular,
	MSP430F5xxx_Match<versionId>,
	FR69xxEemMode,
	MSP430F5xxx_DefaultVoltageTestVpp,
	ClockInfo,
	FunctionMappingMSP430FR69xx,
	FuncletMappingXv2FRAM,
	MemoryList<boost::tuple<
		MSP430FR69xx_MainFramMemory<MainFramSize, MainFramOffset>,
		MSP430FR69xx_InfoFramMemoryInfo,
		MSP430FR69xx_BootFramMemoryInfo,
		MSP430F5xxx_BootCodeMemoryInfo,
		MSP430F5xxx_SystemRamInfo< Size<0x800> >,
		MSP430F5xxx_peripherl16lbitMemoryInfo,
		MSP430F5xxx_CPUMemoryInfo,
		MSP430F5xxx_EEMMemoryInfo
	> >, //until C++0x, the space between the brackets is important
	MSP430FR69xx_Features,
	NoExtendedFeatures,
	MSP430FR69xx_PowerSettings
> {};


extern const char MSP430FR6970[] = "MSP430FR6970";
extern const char MSP430FR6972[] = "MSP430FR6972";

extern const char MSP430FR6870[] = "MSP430FR6870";
extern const char MSP430FR6872[] = "MSP430FR6872";

extern const char MSP430FR6920[] = "MSP430FR6920";
extern const char MSP430FR6922[] = "MSP430FR6922";

extern const char MSP430FR6820[] = "MSP430FR6820";
extern const char MSP430FR6822[] = "MSP430FR6822";

extern const char MSP430FR5970[] = "MSP430FR5970";
extern const char MSP430FR5972[] = "MSP430FR5972";

extern const char MSP430FR5870[] = "MSP430FR5870";
extern const char MSP430FR5872[] = "MSP430FR5872";


static const DeviceRegistrator< MSP430FR697x_type<MSP430FR6970, 0x8249, FR69xx_LcdClockInfo, Size<0x8000>, Offset<0x8000> > > regMSP430FR6970;
static const DeviceRegistrator< MSP430FR697x_type<MSP430FR6972, 0x824B, FR69xx_LcdClockInfo, Size<0xFC00>, Offset<0x4400> > > regMSP430FR6972;

static const DeviceRegistrator< MSP430FR697x_type<MSP430FR6870, 0x824C, FR69xx_LcdClockInfo, Size<0x8000>, Offset<0x8000> > > regMSP430FR6870;
static const DeviceRegistrator< MSP430FR697x_type<MSP430FR6872, 0x824E, FR69xx_LcdClockInfo, Size<0xFC00>, Offset<0x4400> > > regMSP430FR6872;

static const DeviceRegistrator< MSP430FR697x_type<MSP430FR6920, 0x824F, FR69xx_LcdClockInfo, Size<0x8000>, Offset<0x8000> > > regMSP430FR6920;
static const DeviceRegistrator< MSP430FR697x_type<MSP430FR6920, 0x8250, FR69xx_LcdClockInfo, Size<0x8000>, Offset<0x8000> > > regMSP430FR6920_;
static const DeviceRegistrator< MSP430FR697x_type<MSP430FR6922, 0x8253, FR69xx_LcdClockInfo, Size<0xFC00>, Offset<0x4400> > > regMSP430FR6922;
static const DeviceRegistrator< MSP430FR697x_type<MSP430FR6922, 0x8254, FR69xx_LcdClockInfo, Size<0xFC00>, Offset<0x4400> > > regMSP430FR6922_;

static const DeviceRegistrator< MSP430FR697x_type<MSP430FR6820, 0x8255, FR69xx_LcdClockInfo, Size<0x8000>, Offset<0x8000> > > regMSP430FR6820;
static const DeviceRegistrator< MSP430FR697x_type<MSP430FR6820, 0x8256, FR69xx_LcdClockInfo, Size<0x8000>, Offset<0x8000> > > regMSP430FR6820_;
static const DeviceRegistrator< MSP430FR697x_type<MSP430FR6822, 0x8259, FR69xx_LcdClockInfo, Size<0xFC00>, Offset<0x4400> > > regMSP430FR6822;
static const DeviceRegistrator< MSP430FR697x_type<MSP430FR6822, 0x825A, FR69xx_LcdClockInfo, Size<0xFC00>, Offset<0x4400> > > regMSP430FR6822_;

static const DeviceRegistrator< MSP430FR697x_type<MSP430FR5970, 0x825B, FR69xx_ClockInfo, Size<0x8000>, Offset<0x8000> > > regMSP430FR5970;
static const DeviceRegistrator< MSP430FR697x_type<MSP430FR5972, 0x825D, FR69xx_ClockInfo, Size<0xFC00>, Offset<0x4400> > > regMSP430FR5972;

static const DeviceRegistrator< MSP430FR697x_type<MSP430FR5870, 0x825E, FR69xx_ClockInfo, Size<0x8000>, Offset<0x8000> > > regMSP430FR5870;
static const DeviceRegistrator< MSP430FR697x_type<MSP430FR5872, 0x8260, FR69xx_ClockInfo, Size<0xFC00>, Offset<0x4400> > > regMSP430FR5872;
