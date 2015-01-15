/*
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

/* http://srecord.sourceforge.net/ */
const uint16_t MSP_FetDcdcControllerImage[] =
{
0x000D, 0x0390, 0x0302, 0x0302, 0x0302, 0x0402, 0x0402, 0x0502, 0x0502,
0x0502, 0x0502, 0x0502, 0x0502, 0x0602, 0x0602, 0x0603, 0x0603, 0x0703,
0x0703, 0x0703, 0x0803, 0x0803, 0x0803, 0x0803, 0x0903, 0x0903, 0xFF03,
0x4031, 0x0300, 0x403C, 0x0200, 0x403D, 0x0003, 0x12B0, 0xE3F6, 0x12B0,
0xE1AE, 0x12B0, 0xE40C, 0x120F, 0x120E, 0xB3E2, 0x0079, 0x2804, 0x43A2,
0x0204, 0x4382, 0x0206, 0x421F, 0x0204, 0x830F, 0x249C, 0x832F, 0x240F,
0x832F, 0x2419, 0x832F, 0x2434, 0x832F, 0x2437, 0x832F, 0x243D, 0x823F,
0x246B, 0x832F, 0x2474, 0x832F, 0x247A, 0x3C8B, 0x425E, 0x007B, 0xF07E,
0x00E0, 0x527E, 0x4EC2, 0x007B, 0xC3E2, 0x0079, 0x42A2, 0x0204, 0x3C7F,
0xD3E2, 0x0078, 0x425E, 0x007C, 0xF07E, 0x00FE, 0x925E, 0xE000, 0x200C,
0xB3D2, 0x007C, 0x2804, 0x40B2, 0x0012, 0x0204, 0x3C02, 0x42B2, 0x0204,
0x43C2, 0x007C, 0x3C05, 0x43F2, 0x007C, 0x40B2, 0x0006, 0x0204, 0xD3D2,
0x007B, 0x3C62, 0xC3E2, 0x0078, 0x4382, 0x0204, 0x3C5D, 0xC3E2, 0x0078,
0xD2F2, 0x007B, 0x40B2, 0x000A, 0x0204, 0x3C55, 0xD3E2, 0x0078, 0x90B2,
0x0003, 0x0206, 0x2C07, 0x421F, 0x0206, 0x42DF, 0x007C, 0x020D, 0x5392,
0x0206, 0x43C2, 0x007C, 0x90B2, 0x0003, 0x0206, 0x2818, 0x425F, 0x020D,
0x4F4F, 0x4F82, 0x0208, 0x425F, 0x020E, 0x4F4F, 0xF03F, 0x00FF, 0x108F,
0x4F82, 0x020A, 0x425F, 0x020F, 0x4F4F, 0x5F82, 0x020A, 0x40B2, 0x0006,
0x0204, 0x4382, 0x0206, 0x3C02, 0x42B2, 0x0204, 0xD3D2, 0x007B, 0x3C25,
0xD3E2, 0x0078, 0x42D2, 0x1001, 0x007C, 0xD2F2, 0x007B, 0x40B2, 0x0014,
0x0204, 0x3C1A, 0xC3E2, 0x0078, 0xD3D2, 0x007B, 0x40B2, 0x0016, 0x0204,
0x3C12, 0xB3D2, 0x007C, 0x2805, 0xC3E2, 0x0078, 0x4382, 0x0204, 0x3C0A,
0xD3E2, 0x0078, 0x42D2, 0x1000, 0x007C, 0xD2F2, 0x007B, 0x40B2, 0x0014,
0x0204, 0xC3D2, 0x0079, 0xC0B1, 0x00D0, 0x0004, 0x413E, 0x413F, 0x1300,
0x40B2, 0x5A80, 0x0120, 0xD2E2, 0x0026, 0xF0F2, 0x00F5, 0x0021, 0x40F2,
0x000E, 0x0022, 0xF0F2, 0x003F, 0x002E, 0xF0F2, 0x003F, 0x0042, 0x43C2,
0x0029, 0x43F2, 0x002A, 0x40F2, 0x0021, 0x005B, 0x40F2, 0x002C, 0x005A,
0x42F2, 0x0059, 0x40F2, 0x00A0, 0x0056, 0x40F2, 0x008F, 0x0057, 0x40B2,
0x0060, 0x0164, 0x42B2, 0x0172, 0x43A2, 0x0174, 0x4034, 0x0224, 0x4305,
0xD0F2, 0x0020, 0x0053, 0x40B2, 0x5A3F, 0x0120, 0xD3D2, 0x0000, 0x43C2,
0x020C, 0x43B2, 0x0208, 0x4382, 0x0204, 0x4382, 0x0206, 0x40F2, 0x00C1,
0x0078, 0x40F2, 0x0070, 0x0079, 0x40F2, 0x00EE, 0x007A, 0xD0F2, 0x0020,
0x007B, 0xC3D2, 0x0078, 0xC3D2, 0x0079, 0xD232, 0x403C, 0x0BB8, 0x12B0,
0xE276, 0xB3D2, 0x005A, 0x2C04, 0x4482, 0x0160, 0x4303, 0x3C03, 0x42A2,
0x0160, 0x5315, 0x93B2, 0x0208, 0x27F3, 0x421C, 0x020A, 0x12B0, 0xE318,
0x3FEE, 0x120A, 0x4C0A, 0x430C, 0x4A0F, 0xF03F, 0x7FFF, 0x4F82, 0x0200,
0x4A0F, 0xE33F, 0x5F0F, 0x7F0F, 0xF35F, 0x4FC2, 0x0202, 0x90B2, 0x04B0,
0x0200, 0x3819, 0x90B2, 0x0E11, 0x0200, 0x3415, 0x421F, 0x0200, 0x503F,
0xFB50, 0x4F0C, 0x403E, 0x0064, 0x12B0, 0xE39A, 0x4C0E, 0x5E0E, 0x425F,
0x0202, 0x4F4F, 0x5F0E, 0x4E5F, 0xE001, 0x4F4F, 0x531F, 0x4F82, 0x0172,
0x413A, 0x4130, 0x120F, 0x120E, 0xB0F2, 0x0010, 0x0020, 0x2C19, 0x9305,
0x2013, 0x425E, 0x020C, 0x535E, 0x4EC2, 0x020C, 0x926E, 0x2810, 0xD2F2,
0x0021, 0x403F, 0x12CE, 0x403E, 0x0013, 0x533F, 0x633E, 0x2FFD, 0x43C2,
0x020C, 0x3C04, 0xC2F2, 0x0021, 0x43C2, 0x020C, 0x4305, 0x413E, 0x413F,
0x1300, 0x120A, 0x4C0A, 0x421F, 0x0208, 0x831F, 0x2407, 0x831F, 0x2409,
0x831F, 0x240B, 0x831F, 0x240D, 0x3C0F, 0x4A0C, 0x12B0, 0xE276, 0x3C0B,
0x4A0C, 0x12B0, 0xE3B2, 0x3C07, 0x4A0C, 0x12B0, 0xE358, 0x3C03, 0x4A0C,
0x12B0, 0xE3E0, 0x43B2, 0x0208, 0x413A, 0x4130, 0xD3E2, 0x0022, 0xD0F2,
0x00C0, 0x002A, 0xB31C, 0x2804, 0xD0F2, 0x0040, 0x0029, 0x3C03, 0xC0F2,
0x0040, 0x0029, 0xB32C, 0x2803, 0xD3E2, 0x0021, 0x3C02, 0xC3E2, 0x0021,
0xB22C, 0x2804, 0xD0F2, 0x0080, 0x0029, 0x4130, 0xC0F2, 0x0080, 0x0029,
0x4130, 0xF37C, 0xF37E, 0x4E0F, 0x4C0D, 0x430E, 0x431C, 0x5D0D, 0x6E0E,
0x9F0E, 0x2801, 0x8F0E, 0x6C0C, 0x2BF9, 0x4130, 0x930C, 0x2002, 0x4224,
0x4130, 0x903C, 0x000A, 0x3404, 0x4C82, 0x0172, 0x4034, 0x0224, 0x4130,
0x120A, 0x8321, 0x4C0A, 0x4A81, 0x0000, 0x410D, 0x530D, 0x435C, 0x12B0,
0xE410, 0x3FF8, 0x903C, 0x5A5A, 0x2007, 0xC3D2, 0x0000, 0xD032, 0x00D8,
0x4303, 0xD3D2, 0x0000, 0x4130, 0x4C0F, 0x5D0F, 0x3C03, 0x43CC, 0x0000,
0x531C, 0x9F0C, 0x23FB, 0x4130, 0x4030, 0xE3CA, 0x4030, 0xE408, 0x4130,
0xE04C, 0xE2D2, 0xE034,
};

const uint32_t MSP_FetDcdcControllerImage_address[] =
{
0x00001000, 0x0000E000, 0x0000FFE8, 0x0000FFF4, 0x0000FFFE,
};
const uint32_t MSP_FetDcdcControllerImage_length_of_sections[] =
{
0x00000001, 0x00000209, 0x00000001, 0x00000001, 0x00000001,
};
const uint32_t MSP_FetDcdcControllerImage_sections    = 0x00000005;
const uint32_t MSP_FetDcdcControllerImage_termination = 0x00000000;
const uint32_t MSP_FetDcdcControllerImage_start       = 0x00001000;
const uint32_t MSP_FetDcdcControllerImage_finish      = 0x00010000;
const uint32_t MSP_FetDcdcControllerImage_length      = 0x0000F000;

#define MSP_FETDCDCCONTROLLERIMAGE_TERMINATION 0x00000000
#define MSP_FETDCDCCONTROLLERIMAGE_START       0x00001000
#define MSP_FETDCDCCONTROLLERIMAGE_FINISH      0x00010000
#define MSP_FETDCDCCONTROLLERIMAGE_LENGTH      0x0000F000
#define MSP_FETDCDCCONTROLLERIMAGE_SECTIONS    0x00000005
