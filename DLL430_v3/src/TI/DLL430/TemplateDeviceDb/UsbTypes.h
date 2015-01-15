/*
 * UsbAccessType.h
 *
 * Handles access to USB RAM memory.
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

#pragma once

#include <RandomMemoryAccess.h>
#include <MemoryManager.h>
#include "MSP430F5xxx.h"

namespace TI {
	namespace DLL430 {

class DeviceHandleV3;

namespace Tdd = TemplateDeviceDb;
namespace Mem = TemplateDeviceDb::Memory;

class UsbRamAccess : public TI::DLL430::RandomMemoryAccess
{
public:
	UsbRamAccess (
				MemoryArea::Name name,
				TI::DLL430::DeviceHandleV3* devHandle,
				uint32_t start,
				uint32_t end,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				const bool protectable,
				MemoryManager* mm,
				uint8_t psa)
	: RandomMemoryAccess(
		name,devHandle,
		start,end,seg,banks,
		mapped,protectable,mm,psa
	) {}

	virtual bool doRead (uint32_t address, uint32_t* buffer, size_t count){
		return MemoryAreaBase::doRead (address, buffer, count);
	}
};

typedef TemplateDeviceDb::MemoryInfo<
	MemoryArea::USB_RAM, Mem::RamType, Mem::Mapped, Mem::NotProtectable, Mem::Bits16Type,
	Mem::Size<0x800> , Mem::Offset<0x1c00>, Mem::SegmentSize<0x1>, Mem::BankSize<0x0>, Mem::Banks<1>, 
	Mem::NoMask, Mem::MemoryCreator<UsbRamAccess>
> UsbTypeRamInfo;


template<class SizeType, class BanksType>
struct UsbTypeSystemRamInfo : TemplateDeviceDb::MemoryInfo<
								MemoryArea::RAM, Mem::RamType, Mem::Mapped,
								Mem::NotProtectable, Mem::Bits16Type, 
								SizeType , Mem::Offset<0x2400>, Mem::SegmentSize<0x1>, 
								Mem::BankSize<0x0>, BanksType, Mem::NoMask> {};
}//namespace DLL430
}//namespace TI
