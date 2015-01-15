/*
 * MpuWriteProtection.h
 *
 * Functionality for configuring MPU.
 *
 * Copyright (C) 2007 - 2014 Texas Instruments Incorporated - http://www.ti.com/
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


namespace TI
{
	namespace DLL430
	{
		class MemoryManager;
		class DeviceHandleV3;

		template<uint32_t lockRegister, uint16_t lockBits, uint16_t lockMask = 0xFFFF, uint16_t lockPwd = 0x0>
		class MpuWriteProtection
		{
		public:
			MpuWriteProtection(DeviceHandleV3* devHandle, MemoryManager* mm)
				: mm(mm), registerValue(0), registerBackup(0) {}

			bool disableIfEnabled(bool = false)
			{
				//Must not read again without restoring first
				if (registerBackup != registerValue)
				{
					return false;
				}
				if (!readSettings())
				{
					return false;
				}
				return (registerValue & lockBits) ? disable() : true;
			}

			void restore()
			{
				if (registerBackup != registerValue)
				{
					if (MemoryArea* peripheral = mm->getMemoryArea(MemoryArea::PERIPHERY_16BIT, 0))
					{
						if (peripheral->write(lockRegister, registerBackup) && peripheral->sync())
						{
							registerValue = registerBackup;
						}
					}
				}
			}

		private:
			bool readSettings()
			{
				bool success = false;

				if (MemoryArea* peripheral = mm->getMemoryArea(MemoryArea::PERIPHERY_16BIT, 0))
				{
					uint32_t buffer[2] = { 0 };
					success = peripheral->read(lockRegister, buffer, 2) && peripheral->sync();
					if (success)
					{
						registerValue = (buffer[0] + (buffer[1] << 8)) & 0xFFFF;

						//Replace pwd read value with write value
						if (lockPwd != 0)
						{
							registerValue = (registerValue & lockMask) | lockPwd;
						}
						registerBackup = registerValue;
					}
				}
				return success;
			}

			bool disable()
			{
				bool success = false;

				if (MemoryArea* peripheral = mm->getMemoryArea(MemoryArea::PERIPHERY_16BIT, 0))
				{
					const uint16_t newRegisterValue = (registerValue & ~lockBits);
					success = peripheral->write(lockRegister, newRegisterValue) && peripheral->sync();
					if (success)
					{
						registerValue = newRegisterValue;
					}
				}
				return success;
			}

			MemoryManager* mm;

			uint16_t registerValue;
			uint16_t registerBackup;
		};
	};
};
