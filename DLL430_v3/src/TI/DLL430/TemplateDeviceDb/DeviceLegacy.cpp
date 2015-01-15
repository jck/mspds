/*
 * LegacyDevice.cpp
 *
 * Definition for devices that are no longer supported
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
#include "MSP430Defaults.h"
#include "Registration.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;
using namespace TemplateDeviceDb::Memory;


typedef IdCode<0xFFFF, 0xFFFF, 0xFF, 0x00, 0x00, 0x00, 0x0, 0x00, 0xFF> LegacyDeviceIdMask;

template<const unsigned int versionId, const unsigned char revision, const unsigned char revisionMax>
struct LegacyDevice_Match : Match<
	IdCode<versionId, 0x00, revision, 0x00, 0x00, 0x00, 0x0, 0x00, revisionMax>, LegacyDeviceIdMask> {};


typedef ClockInfo<GCC_NONE, 0x0000, EmptyEemTimer, EmptyEemClockNames> NoClockInfo;

typedef MemoryInfo<
	MemoryArea::NONE, FlashType, Mapped, NotProtectable, Bits16Type,
	Size<0>, Offset<0>, SegmentSize<0>, BankSize<0>, Banks<1>
> NoMemoryType;

extern const char LegacyDevice[] = "Legacy";


template<
	const unsigned int versionId,
	const unsigned char revision,
	const unsigned char revisionMax
>
struct LegacyDevice_type : Device <
	LegacyDevice,
	DefaultBits20Type,
	regular,
	LegacyDevice_Match<versionId, revision, revisionMax>,
	NoneEemMode,
	VoltageInfo<1800, 3600, 2700, 2500, 6000, 7000, false>,
	NoClockInfo,
	FunctionMappingNone,
	FuncletMappingNone,
	MemoryList < boost::tuple<NoMemoryType> >
> {};


static const DeviceRegistrator< LegacyDevice_type<0x8169, 0x0, 0x20> > regLegacyMSP430FR5969;
static const DeviceRegistrator< LegacyDevice_type<0x8168, 0x0, 0x20> > regLegacyMSP430FR5968;
static const DeviceRegistrator< LegacyDevice_type<0x8167, 0x0, 0x20> > regLegacyMSP430FR5967;
static const DeviceRegistrator< LegacyDevice_type<0x8165, 0x0, 0x20> > regLegacyMSP430FR5959;
static const DeviceRegistrator< LegacyDevice_type<0x8164, 0x0, 0x20> > regLegacyMSP430FR5958;
static const DeviceRegistrator< LegacyDevice_type<0x8163, 0x0, 0x20> > regLegacyMSP430FR5957;
static const DeviceRegistrator< LegacyDevice_type<0x8161, 0x0, 0x20> > regLegacyMSP430FR5949;
static const DeviceRegistrator< LegacyDevice_type<0x8160, 0x0, 0x20> > regLegacyMSP430FR5948;
static const DeviceRegistrator< LegacyDevice_type<0x815F, 0x0, 0x20> > regLegacyMSP430FR5947;
static const DeviceRegistrator< LegacyDevice_type<0x8187, 0x0, 0x20> > regLegacyMSP430FR5929;
static const DeviceRegistrator< LegacyDevice_type<0x815D, 0x0, 0x20> > regLegacyMSP430FR5869;
static const DeviceRegistrator< LegacyDevice_type<0x815C, 0x0, 0x20> > regLegacyMSP430FR5868;
static const DeviceRegistrator< LegacyDevice_type<0x815B, 0x0, 0x20> > regLegacyMSP430FR5867;
static const DeviceRegistrator< LegacyDevice_type<0x8159, 0x0, 0x20> > regLegacyMSP430FR5859;
static const DeviceRegistrator< LegacyDevice_type<0x8158, 0x0, 0x20> > regLegacyMSP430FR5858;
static const DeviceRegistrator< LegacyDevice_type<0x8157, 0x0, 0x20> > regLegacyMSP430FR5857;
static const DeviceRegistrator< LegacyDevice_type<0x8155, 0x0, 0x20> > regLegacyMSP430FR5849;
static const DeviceRegistrator< LegacyDevice_type<0x8154, 0x0, 0x20> > regLegacyMSP430FR5848;
static const DeviceRegistrator< LegacyDevice_type<0x8153, 0x0, 0x20> > regLegacyMSP430FR5847;

static const DeviceRegistrator<	LegacyDevice_type<0x81A8, 0x0, 0x20> > regLegacyMSP430FR6989;
static const DeviceRegistrator<	LegacyDevice_type<0x81A7, 0x0, 0x20> > regLegacyMSP430FR6988;
static const DeviceRegistrator<	LegacyDevice_type<0x81A6, 0x0, 0x20> > regLegacyMSP430FR6987;
static const DeviceRegistrator<	LegacyDevice_type<0x81AB, 0x0, 0x20> > regLegacyMSP430FR5989;
static const DeviceRegistrator<	LegacyDevice_type<0x81AA, 0x0, 0x20> > regLegacyMSP430FR5988;
static const DeviceRegistrator<	LegacyDevice_type<0x81A9, 0x0, 0x20> > regLegacyMSP430FR5987;
static const DeviceRegistrator<	LegacyDevice_type<0x81DF, 0x0, 0x20> > regLegacyMSP430FR5986;
static const DeviceRegistrator<	LegacyDevice_type<0x81C0, 0x0, 0x20> > regLegacyMSP430FR6889;
static const DeviceRegistrator<	LegacyDevice_type<0x81BF, 0x0, 0x20> > regLegacyMSP430FR6888;
static const DeviceRegistrator<	LegacyDevice_type<0x81BE, 0x0, 0x20> > regLegacyMSP430FR6887;
static const DeviceRegistrator<	LegacyDevice_type<0x81C3, 0x0, 0x20> > regLegacyMSP430FR5889;
static const DeviceRegistrator<	LegacyDevice_type<0x81C2, 0x0, 0x20> > regLegacyMSP430FR5888;
static const DeviceRegistrator<	LegacyDevice_type<0x81C1, 0x0, 0x20> > regLegacyMSP430FR5887;
static const DeviceRegistrator<	LegacyDevice_type<0x81AE, 0x0, 0x20> > regLegacyMSP430FR6979;
static const DeviceRegistrator<	LegacyDevice_type<0x81AC, 0x0, 0x20> > regLegacyMSP430FR6977;
static const DeviceRegistrator<	LegacyDevice_type<0x81C6, 0x0, 0x20> > regLegacyMSP430FR6879;
static const DeviceRegistrator<	LegacyDevice_type<0x81C4, 0x0, 0x20> > regLegacyMSP430FR6877;
static const DeviceRegistrator<	LegacyDevice_type<0x81B3, 0x0, 0x20> > regLegacyMSP430FR6928;
static const DeviceRegistrator<	LegacyDevice_type<0x81B2, 0x0, 0x20> > regLegacyMSP430FR6927;
