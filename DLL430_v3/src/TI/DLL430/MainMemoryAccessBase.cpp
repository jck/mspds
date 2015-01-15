/*
 * MainMemoryAccessBase.cpp
 *
 * Base class for Flash and FRAM memory access
 *
 * Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/
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
#include "MainMemoryAccessBase.h"
#include "HalExecCommand.h"
#include "DeviceHandleV3.h"

using namespace TI::DLL430;
using boost::bind;
using boost::shared_ptr;

MainMemoryAccessBase::MainMemoryAccessBase (
				MemoryArea::Name name,
				DeviceHandleV3* devHandle,
				uint32_t start,
				uint32_t end,
				uint32_t seg,
				uint32_t banks,
				bool mapped,
				const bool isProtected,
				MemoryManager* mm,
				uint8_t psa
)
 : MemoryAreaBase(name, devHandle, start, end, seg, banks, mapped, isProtected, psa)
 , mm(mm)
{
}

MainMemoryAccessBase::~MainMemoryAccessBase ()
{
}

bool MainMemoryAccessBase::doRead(uint32_t address, uint32_t* buffer, size_t count)
{
	MemoryArea* cpu = mm->getMemoryArea(MemoryArea::CPU);
	if (!cpu)
		return false;

	uint32_t pc = 0;
	cpu->read(0, &pc, 1);

	bool omitFirst = (address & 0x1);
	if (omitFirst) {
		--address;
		++count;
	}
	bool omitLast = (count & 1);
	if (omitLast) {
		++count;
	}

	const hal_id readMacro = devHandle->supportsQuickMemRead() ?
								ID_ReadMemQuick : ID_ReadMemWords;

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(readMacro));
	el->appendInputData32(this->getStart() + address);
	el->appendInputData32(static_cast<uint32_t>(count/2));
	el->appendInputData32(pc);

	el->setOutputSize(count);

	ReadElement r(buffer, count, omitFirst, omitLast, 0);
	this->readMap[this->elements.size()] = r;
	this->elements.push_back(el);
	return true;
}


bool MainMemoryAccessBase::doWrite(uint32_t address, uint32_t value)
{
	return this->doWrite(address, &value, 1);
}


bool MainMemoryAccessBase::erase(uint32_t start, uint32_t end)
{
	return erase(start, end, this->getSegmentSize(), ERASE_SEGMENT);
}


bool MainMemoryAccessBase::erase()
{
	const uint32_t bank_size = this->getSize() / this->getBanks();

	return erase(this->getStart(), this->getEnd(), bank_size, ERASE_MAIN);
}


bool MainMemoryAccessBase::uploadFunclet(FuncletCode::Type type)
{
	bool success = false;

	const FuncletCode& funclet = devHandle->getFunclet(type);

	if ( !funclet.code() )
	{
		success = true;
		ramBackup.clear();
	}
	else if ( MemoryArea* ram = mm ? mm->getMemoryArea(MemoryArea::RAM, 0) : 0 )
	{
		if ( funclet.codeSize() <= ram->getSize() )
		{
			if ( mm && mm->getRamPreserveMode() )
			{
				const size_t backupSize = min((size_t)ram->getSize(),
										   funclet.codeSize() + funclet.maxPayloadSize());

				ramBackup.resize( backupSize );
				if (!ram->read(0, &ramBackup[0], ramBackup.size()) || !ram->sync())
					return false;
			}
			else
			{
				ramBackup.clear();
			}

			const uint8_t* code = (uint8_t*)funclet.code();
			const size_t count = funclet.codeSize();

			vector<uint32_t> tmp(count);
			for (size_t i = 0; i < count; ++i)
			{
				tmp[i] = static_cast<uint32_t>(code[i]);
			}
			success = ram->write(0, &tmp[0], count) && ram->sync();
		}
	}
	return success;
}


void MainMemoryAccessBase::restoreRam()
{
	if (!ramBackup.empty())
	{
		if ( MemoryArea* ram = mm->getMemoryArea(MemoryArea::RAM, 0) )
		{
			size_t count = ramBackup.size();
			vector<uint32_t> tmp(count);
			for (size_t i = 0; i < count; ++i)
				tmp[i] = ramBackup[i];

			ram->write(0, &tmp[0], count) && ram->sync();
		}
		ramBackup.clear();
	}
}


bool MainMemoryAccessBase::postSync(const HalExecCommand& cmd)
{
	restoreRam();

	for (size_t n = 1; n < cmd.elements.size(); ++n)
	{
		ReadElement_map::iterator it = this->readMap.find(n-1);
		if (it != this->readMap.end())
		{
			ReadElement r = it->second;
			size_t size = r.size;
			if (r.omitLast)
				--size;

			const HalExecElement& el = cmd.elements.at(n);
			for (size_t i = 0, k = (r.omitFirst? 1: 0); k < size; ++k, ++i)
			{
				r.v_buffer[i] = el.getOutputAt8(k);
			}
			this->readMap.erase(it);
		}
	}

	return true;
}
