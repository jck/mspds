/*
 * MSP430FR59xx.cpp
 *
 * Definition MSP430FR59xx.
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

typedef IdCode<0xFFFF, 0xFFFF, 0xFF, 0x00, 0x00, 0x00, 0x0, 0x00, 0xFF> MSP430FR59xxPGIdMask;

template<const unsigned int versionId, const unsigned char revision, const unsigned char revisionMax>
struct MSP430FR59xx_Match : Match<
						IdCode<versionId, 0x00, revision, 0x00, 0x00, 0x00, 0x0,0x00, revisionMax>, MSP430FR59xxPGIdMask> {};

static const ClockPair FR_TA2_2 = {"Timer2_A2", 0x8B};
static const ClockPair FR_TA2_3 = {"Timer3_A2", 0x8C};

struct MSP430FR59xx_EemTimer : EemTimerImpl
{
	typedef EemTimerImpl::Timer Eem;
	MSP430FR59xx_EemTimer() : EemTimerImpl(
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::COMP_E,
		Eem::ADC12_B, Eem::RTC, Eem::Empty, Eem::eUSCIB0,
		Eem::eUSCIA1, Eem::eUSCIA0, Eem::TB7_0, FR_TA2_3,
		FR_TA2_2, Eem::TA3_1, Eem::TA3_0, Eem::WDT_A,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty, 
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty,
		Eem::Empty, Eem::Empty, Eem::Empty, Eem::Empty
	)
	{}
};

struct FR59xx_Timer : MSP430FR59xx_EemTimer { FR59xx_Timer() : MSP430FR59xx_EemTimer() {} };

typedef ClockInfo<GCC_EXTENDED, 0x243F, FR59xx_Timer, EmptyEemClockNames> FR59xx_ClockInfo;

typedef Trigger<EMEX_SMALL_5XX, 0x03, 0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x02, 0x00, 0x00> FR59xxTrigger;

typedef EemInfo<0x00, 0x01, 0x01, FR59xxTrigger, SmallSequencer> FR59xxEemMode;


template<const unsigned int size, const unsigned int offset>
struct MSP430FR59xx_MainFramMemory : MemoryInfo<
										MemoryArea::MAIN, RamType, Mapped, NotProtectable, Bits16Type,
										Size<size> , Offset<offset>, SegmentSize<0x01>,
										BankSize<0x0>, Banks<1>, NoMask, MemoryCreator<FramMemoryAccessFRx9<MpuFRx> >
									> {};

typedef MSP430FR59xx_MainFramMemory<0xFC00, 0x4400> MainFramMemory_63k;
typedef MSP430FR59xx_MainFramMemory<0xBC00, 0x4400> MainFramMemory_47k;
typedef MSP430FR59xx_MainFramMemory<0x8000, 0x8000> MainFramMemory_32k;


template<const unsigned size>
struct MSP430FR59xx_SystemRamInfo : MemoryInfo<
										MemoryArea::RAM, RamType, Mapped, NotProtectable, Bits16Type,
										Size<size>, Offset<0x1C00>, SegmentSize<0x1>,
										BankSize<0x0>, Banks<1>, NoMask> {};

typedef MSP430FR59xx_SystemRamInfo<0x800> SystemRam_2k;
typedef MSP430FR59xx_SystemRamInfo<0x400> SystemRam_1k;

typedef MemoryInfo<
	MemoryArea::INFO, RamType, Mapped, NotProtectable, Bits16Type,
	Size<0x200> , Offset<0x1800>, SegmentSize<0x01>, BankSize<0x0>, Banks<1>,
	NoMask, MemoryCreator<FramMemoryAccessFRx9<MpuFRx> >
> MSP430FR59xx_InfoFramMemoryInfo;


typedef MemoryInfo<
	MemoryArea::BSL, RomType, Mapped, Protectable, Bits16Type,
	Size<0x800> , Offset<0x1000>, SegmentSize<0x01>, BankSize<0>, Banks<1>, 
	NoMask, MemoryCreator<BslRomAccess>
> MSP430FR59xx_BootFramMemoryInfo;


typedef Features<MOD_OSC, false, true, true, false, true> MSP430FR59xx_Features;


struct FunctionMappingMSP430FR59xx : public FunctionMappingBase
{
	FunctionMappingMSP430FR59xx(){
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
			(ID_SingleStep,ID_SingleStepJStateXv2) 
			(ID_ReadAllCpuRegs,ID_ReadAllCpuRegsXv2)
			(ID_WriteAllCpuRegs,ID_WriteAllCpuRegsXv2)
			(ID_Psa,ID_PsaXv2)
			(ID_WaitForEem,ID_PollJStateReg)
			(ID_ExecuteFunclet,ID_ExecuteFuncletXv2)
			(ID_WaitForStorage,ID_WaitForStorageX);
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
					  MSP430FR59xx_PowerSettings;


template<
	const char* description,
	const unsigned int versionId,
	class MainFramType,
	class RamType
>
struct MSP430FR59xx_type : Device<
							description, 
							DefaultBits20Type, 
							regular, 
							MSP430FR59xx_Match<versionId, 0x21, 0xFF>, //MSP430F5xxx_Match<versionId>,
							FR59xxEemMode, 
							MSP430F5xxx_DefaultVoltageTestVpp,
							FR59xx_ClockInfo,
							FunctionMappingMSP430FR59xx,
							FuncletMappingXv2FRAM,
							MemoryList<boost::tuple<
								MainFramType,
								MSP430FR59xx_InfoFramMemoryInfo, 
								MSP430FR59xx_BootFramMemoryInfo,
								MSP430F5xxx_BootCodeMemoryInfo,
								RamType,
								MSP430F5xxx_peripherl16lbitMemoryInfo, 
								MSP430F5xxx_CPUMemoryInfo, 
								MSP430F5xxx_EEMMemoryInfo 
							> >, //until C++0x, the space between the brackets is important
							MSP430FR59xx_Features,
							NoExtendedFeatures,
							MSP430FR59xx_PowerSettings
						> {};


extern const char MSP430FR5969[] = "MSP430FR5969";
extern const char MSP430FR5968[] = "MSP430FR5968";
extern const char MSP430FR5967[] = "MSP430FR5967";

extern const char MSP430FR5959[] = "MSP430FR5959";
extern const char MSP430FR5958[] = "MSP430FR5958";
extern const char MSP430FR5957[] = "MSP430FR5957";

extern const char MSP430FR5949[] = "MSP430FR5949";
extern const char MSP430FR5948[] = "MSP430FR5948";
extern const char MSP430FR5947[] = "MSP430FR5947";

extern const char MSP430FR5869[] = "MSP430FR5869";
extern const char MSP430FR5868[] = "MSP430FR5868";
extern const char MSP430FR5867[] = "MSP430FR5867";

extern const char MSP430FR5859[] = "MSP430FR5859";
extern const char MSP430FR5858[] = "MSP430FR5858";
extern const char MSP430FR5857[] = "MSP430FR5857";

extern const char MSP430FR5849[] = "MSP430FR5849";
extern const char MSP430FR5848[] = "MSP430FR5848";
extern const char MSP430FR5847[] = "MSP430FR5847";


static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5969, 0x8169, MainFramMemory_63k, SystemRam_2k> > regMSP430FR5969;
static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5968, 0x8168, MainFramMemory_47k, SystemRam_2k> > regMSP430FR5968;
static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5967, 0x8167, MainFramMemory_32k, SystemRam_1k> > regMSP430FR5967;

static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5959, 0x8165, MainFramMemory_63k, SystemRam_2k> > regMSP430FR5959;
static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5958, 0x8164, MainFramMemory_47k, SystemRam_2k> > regMSP430FR5958;
static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5957, 0x8163, MainFramMemory_32k, SystemRam_1k> > regMSP430FR5957;

static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5949, 0x8161, MainFramMemory_63k, SystemRam_2k> > regMSP430FR5949;
static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5948, 0x8160, MainFramMemory_47k, SystemRam_2k> > regMSP430FR5948;
static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5947, 0x815F, MainFramMemory_32k, SystemRam_1k> > regMSP430FR5947;


static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5869, 0x815D, MainFramMemory_63k, SystemRam_2k> > regMSP430FR5869;
static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5868, 0x815C, MainFramMemory_47k, SystemRam_2k> > regMSP430FR5868;
static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5867, 0x815B, MainFramMemory_32k, SystemRam_1k> > regMSP430FR5867;

static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5859, 0x8159, MainFramMemory_63k, SystemRam_2k> > regMSP430FR5859;
static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5858, 0x8158, MainFramMemory_47k, SystemRam_2k> > regMSP430FR5858;
static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5857, 0x8157, MainFramMemory_32k, SystemRam_1k> > regMSP430FR5857;

static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5849, 0x8155, MainFramMemory_63k, SystemRam_2k> > regMSP430FR5849;
static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5848, 0x8154, MainFramMemory_47k, SystemRam_2k> > regMSP430FR5848;
static const DeviceRegistrator< MSP430FR59xx_type<MSP430FR5847, 0x8153, MainFramMemory_32k, SystemRam_1k> > regMSP430FR5847;
