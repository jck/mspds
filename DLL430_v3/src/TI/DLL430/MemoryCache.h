/*
 * MemoryCache.h
 *
 * Interface for handling caching memory.
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

#pragma once

#include "MemoryManager.h"

namespace TI
{
	namespace DLL430
	{

		class MemoryCache : public MemoryCacheCtrl
		{
		public:
			virtual ~MemoryCache () {};

			virtual bool read (uint32_t address, uint32_t* buffer, size_t count) = 0;
			virtual bool write (uint32_t address, uint32_t* buffer, size_t count) = 0;
			virtual bool write (uint32_t address, uint32_t value) = 0;
			virtual bool sync () { return true; };
			virtual bool erase () { return false; };
			virtual bool erase (uint32_t start, uint32_t end)  { return false; };
			virtual bool verify(uint32_t address, uint32_t* buffer, size_t count) { return false; };
		
			virtual void enable (bool) = 0;
			virtual bool fill (uint32_t address, size_t count) = 0;
			virtual bool flush (uint32_t address, size_t count) = 0;
			virtual void clear (uint32_t address, size_t count) = 0;

			virtual bool isReadOnly()=0;
		};

	}
}
