/**
* \ingroup MODULHAL
*
* \file edt.h
*
* \brief <FILEBRIEF>
*
*/
/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

/**
 @page edt Enhanced Device Test layer (EDT)
 The EDT layer is sort of abstraction of the target devices' JTAG interface.
 Beside some initialization and configuration functions it provides both
 high level functions to access the JTAG Instruction/Data registers plus
 a set of low level symbols to manipulate individual signals of the JTAG
 interface. The EDT layer is described in detail via @ref edt.h.

 Proceed to the @ref hal.
*/

/**
 @file edt.h
 @brief Enhanced Device Test layer (EDT)

 This file defines and exports all the objects and symbols required to use
 the Enhanced Device Test layer (EDT).

 All access to EDT high level functions and low level symbols is realized
 through function/variable pointers. This enables straight forward update
 strategies as none of the resources are accessed via absolute addresses.
 Refer to @ref update for more details on this topic.
*/
#ifndef _EDT_H_
#define _EDT_H_
#include "JTAG_defs.h"
#include "hil_Structs.h"

typedef void (*HilInitFunc)(); // * hal_size, void * stream_adr)

typedef void (*HilInitGetEdtCommenFunc)(edt_common_methods_t* edt_commen); // * hal_size, void * stream_adr)
typedef void (*HilInitGetEdtDistinctFunc)(edt_distinct_methods_t* edt_distinct); // * hal_size, void * stream_adr)
/**
 @brief Constant structure holding all EDT function pointers.

 One constant structure of type \a edt_common_methods. It holds all EDT
 function pointers that are independant from the low level JTAG communication
 protocol (4-wire JTAG or Spy-Bi-Wire).
 Another variable structure of type \a edt_distinct_methods. It holds all EDT
 function pointers that are remapped depending on the low level JTAG prtocol
 selection (4-wire JTAG or Spy-Bi-Wire).

 @warning It is not recommended to use symbols of this structure in other
          modules direclty! Use the hash define symbols instead.
*/

extern edt_common_methods_t  _edt_Common_Methods;
extern edt_distinct_methods_t _edt_Distinct_Methods;

extern unsigned short altRomAddressForCpuRead;
extern unsigned short wdtctlAddress5xx;

#define EDT_Init           (*_edt_Common_Methods.Init)
#define EDT_SetVcc         (*_edt_Common_Methods.SetVcc)
#define EDT_SwitchVccFET   (*_edt_Common_Methods.SwitchVccFET)
#define EDT_GetVcc         (*_edt_Common_Methods.GetVcc)
#define EDT_SetProtocol    (*_edt_Common_Methods.SetProtocol)
#define EDT_SetPsaSetup    (*_edt_Common_Methods.SetPsaSetup)
#define EDT_SetPsaTCLK     (*_edt_Common_Methods.SetPsaTCLK)
#define EDT_Open           (*_edt_Common_Methods.Open)
#define EDT_Close          (*_edt_Common_Methods.Close)
#define EDT_Delay_1us      (*_edt_Common_Methods.Delay_1us)
#define EDT_Delay_1ms      (*_edt_Common_Methods.Delay_1ms)
#define EDT_Loop           (*_edt_Common_Methods.Loop)
#define EDT_EntrySequences (*_edt_Common_Methods.EntrySequences)

#define EDT_BSL_EntrySequence (*_edt_Common_Methods.BSL_EntrySequence)

#define EDT_SetReset       (*_edt_Common_Methods.SetReset)
#define EDT_SetTest        (*_edt_Common_Methods.SetTest)
#define EDT_SetTMS         (*_edt_Common_Methods.SetTMS)
#define EDT_SetTCK         (*_edt_Common_Methods.SetTCK)
#define EDT_SetTDI         (*_edt_Common_Methods.SetTDI)
#define EDT_InitDelayTimer (*_edt_Common_Methods.initDelayTimer)

#ifdef eZ_FET
#define EDT_GetDCDCStatus() DCDC_STATUS
#endif

#define EDT_TapReset            (*_edt_Distinct_Methods.TapReset)
#define EDT_CheckJtagFuse       (*_edt_Distinct_Methods.CheckJtagFuse)
#define EDT_EnumChain           (*_edt_Distinct_Methods.EnumChain)
#define EDT_Instr               (*_edt_Distinct_Methods.Instr)
#define EDT_SetReg_XBits        (*_edt_Distinct_Methods.SetReg_XBits)
#define EDT_SetReg_XBits08      (*_edt_Distinct_Methods.SetReg_XBits08)
#define EDT_SetReg_XBits16      (*_edt_Distinct_Methods.SetReg_XBits16)
#define EDT_SetReg_XBits20      (*_edt_Distinct_Methods.SetReg_XBits20)
#define EDT_SetReg_XBits32      (*_edt_Distinct_Methods.SetReg_XBits32)
#define EDT_SetReg_XBits64      (*_edt_Distinct_Methods.SetReg_XBits64)
#define EDT_SetReg_XBits8_64    (*_edt_Distinct_Methods.SetReg_XBits8_64)
#define EDT_Tclk                (*_edt_Distinct_Methods.Tclk)
#define EDT_SetupPsa            (*_edt_Distinct_Methods.SetupPsa)
#define EDT_StepPsa             (*_edt_Distinct_Methods.StepPsa)
#define EDT_EndPsa              (*_edt_Distinct_Methods.EndPsa)
#define EDT_BlowFuse            (*_edt_Distinct_Methods.BlowFuse)
#define EDT_SetJtagSpeed        (*_edt_Common_Methods.SetJtagSpeed)
#define EDT_ConfigureSetPc      (*_edt_Common_Methods.ConfigureSetPc)

#define EDT_GetPrevInstruction (*_edt_Distinct_Methods.GetPrevInstruction)


#define EDT_TCLK {EDT_Tclk(0); EDT_Tclk(1);}

// further definitions and macros

#define decl_io             short lIn; short lOut;
#define decl_in             short lIn;
#define decl_out            short lOut  = 0;
#define decl_out_long       unsigned long lOut_long  = 0;
#define decl_out_long_long  unsigned long long lOut_long_long = 0;
#define decl_state          short lPass = 0; short lFail = 0;
#define decl_pass           short lPass = 0;
#define decl_fail           short lFail = 0;
#define decl_isInstrLoad    short lIsInstrLoad = -1;
#define decl_instrLoad      short lInstrLoad = -1;
#define decl_jtagMailboxIn  short jtagMailboxIn = -1;
#define decl_jtagMailboxOut short jtagMailboxOu = -1;

// JTAG instruction register access
#define bypass              { lOut = EDT_Instr(0xFF); }
#define cntrl_sig_high_byte { EDT_Instr(IR_CNTRL_SIG_HIGH_BYTE); }
#define cntrl_sig_low_byte  { EDT_Instr(IR_CNTRL_SIG_LOW_BYTE); }
#define cntrl_sig_capture   { lOut = EDT_Instr(IR_CNTRL_SIG_CAPTURE); }
#define cntrl_sig_16bit     { EDT_Instr(IR_CNTRL_SIG_16BIT); }
#define cntrl_sig_release   { EDT_Instr(IR_CNTRL_SIG_RELEASE); }
#define addr_16bit          { EDT_Instr(IR_ADDR_16BIT); }
#define addr_capture        { EDT_Instr(IR_ADDR_CAPTURE); }
#define data_16bit          { EDT_Instr(IR_DATA_16BIT); }
#define data_capture        { EDT_Instr(IR_DATA_CAPTURE); }
#define data_to_addr        { EDT_Instr(IR_DATA_TO_ADDR); }
#define data_quick          { EDT_Instr(IR_DATA_QUICK); }
#define config_fuses        { EDT_Instr(IR_CONFIG_FUSES); }
#define eem_data_exchange   { EDT_Instr(IR_EMEX_DATA_EXCHANGE); }
#define eem_data_exchange32 { EDT_Instr(IR_EMEX_DATA_EXCHANGE32); }
#define eem_read_control    { EDT_Instr(IR_EMEX_READ_CONTROL); }
#define eem_write_control   { EDT_Instr(IR_EMEX_WRITE_CONTROL); }
#define eem_data_exchange32 { EDT_Instr(IR_EMEX_DATA_EXCHANGE32); }
#define eem_read_trigger    { EDT_Instr(IR_EMEX_READ_TRIGGER); }
#define data_psa            { EDT_Instr(IR_DATA_PSA); }
#define shift_out_psa       { EDT_Instr(IR_SHIFT_OUT_PSA); }
#define flash_16bit_update  { EDT_Instr(IR_FLASH_16BIT_UPDATE); }
#define tclk_0_inst         { EDT_Instr(0x3A); }//3A
#define tclk_1_inst         { EDT_Instr(0x3B); }//3B
#define tclk_toggle_inst    { EDT_Instr(0x39); }//39
#define jmb_exchange        { EDT_Instr(IR_JMB_EXCHANGE); }
#define device_ip_pointer   { EDT_Instr(IR_DEVICE_ID);}
#define core_ip_pointer     { EDT_Instr(IR_COREIP_ID);}
#define jstate_read         { EDT_Instr(IR_JSTATE_ID);}
#define test_reg            { EDT_Instr(IR_TEST_REG);}
#define test_reg_3V         { EDT_Instr(IR_TEST_3V_REG);}


// JTAG data register access
#define SetReg_8Bits(x)   { lOut      = (short)EDT_SetReg_XBits08(x); }
#define SetReg_8Bits_(x)  {             EDT_SetReg_XBits08(x); }
#define SetReg_16Bits(x)  { lOut      = (short)EDT_SetReg_XBits16(x); }
#define SetReg_16Bits_(x) {             EDT_SetReg_XBits16(x); }
#define SetReg_20Bits(x)  { lOut_long = EDT_SetReg_XBits20(x); }
#define SetReg_20Bits_(x) {             EDT_SetReg_XBits20(x); }
#define SetReg_32Bits(x)  { lOut_long = EDT_SetReg_XBits32(x); }
#define SetReg_32Bits_(x) {             EDT_SetReg_XBits32(x); }
#define SetReg_64Bits(x)  { lOut_long_long = EDT_SetReg_XBits64(x); }
#define SetReg_64Bits_(x) {             EDT_SetReg_XBits64(x); }

#define SetReg8_64Bits(x,y,z)  { lOut_long_long = EDT_SetReg_XBits8_64(x,y,z); }
#define SetReg8_64Bits_(x,y,z) {             EDT_SetReg_XBits8_64(x,y,z); }


// JTAG macros (used for code inlining)
#define isInstrLoad      { cntrl_sig_capture                                        \
                           lIsInstrLoad = 0;                                        \
                           SetReg_16Bits(0);                                        \
                           if((lOut & (CNTRL_SIG_INSTRLOAD | CNTRL_SIG_READ)) != \
                              (CNTRL_SIG_INSTRLOAD | CNTRL_SIG_READ))               \
                           {                                                        \
                               lIsInstrLoad = -1;                                   \
                           } }

#define instrLoad        { unsigned short i;             \
                           decl_isInstrLoad              \
                           lInstrLoad = 0;               \
                           cntrl_sig_low_byte            \
                           SetReg_8Bits_(CNTRL_SIG_READ) \
                           EDT_Tclk(1);                      \
                           for(i = 0; i < 10; i++)       \
                           {                             \
                               isInstrLoad               \
                               if(lIsInstrLoad == 0)     \
                               {                         \
                                   break;                \
                               }                         \
                               EDT_TCLK                      \
                           }                             \
                           if(i == 10)                   \
                           {                             \
                               --lInstrLoad;             \
                           } }

#define halt_cpu          { data_16bit             \
                            SetReg_16Bits_(0x3FFF) \
                            EDT_Tclk(0);            \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x2409) \
                            EDT_Tclk(1);  }

#define release_cpu       { EDT_Tclk(0);            \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x2401) \
                            addr_capture           \
                            EDT_Tclk(1);  }
#define ReadMemWord(a,d)  { halt_cpu               \
                            EDT_Tclk(0);            \
                            addr_16bit             \
                            SetReg_16Bits(a)       \
                            data_to_addr           \
                            EDT_Tclk(1);            \
                            EDT_Tclk(0);            \
                            SetReg_16Bits(0)       \
                            d = lOut;              \
                            release_cpu }

#define ReadMemWordX(a,d) { halt_cpu               \
                            EDT_Tclk(0);            \
                            addr_16bit             \
                            SetReg_20Bits(a)       \
                            data_to_addr           \
                            EDT_Tclk(1);            \
                            EDT_Tclk(0);            \
                            SetReg_16Bits(0)       \
                            d = lOut;              \
                            release_cpu }

#define ReadMemWordXv2(a,d){decl_out               \
                            EDT_Tclk(0);            \
                            addr_16bit             \
                            SetReg_20Bits(a)       \
                            EDT_Tclk(1);            \
                            EDT_Tclk(0);            \
                            data_capture           \
                            SetReg_16Bits(0);      \
                            d = lOut;              \
                            EDT_Tclk(1);            \
                            EDT_Tclk(0);            \
                            EDT_Tclk(1); }

#define WriteMemWord(a,v) { halt_cpu               \
                            EDT_Tclk(0);            \
                            cntrl_sig_low_byte     \
                            SetReg_8Bits_(0x08)    \
                            addr_16bit             \
                            SetReg_16Bits_(a)      \
                            data_to_addr           \
                            SetReg_16Bits_(v)      \
                            EDT_Tclk(1);            \
                            release_cpu }

#define WriteMemWordX(a,v) {halt_cpu               \
                            EDT_Tclk(0);            \
                            cntrl_sig_low_byte     \
                            SetReg_8Bits_(0x08)    \
                            addr_16bit             \
                            SetReg_20Bits_(a)      \
                            data_to_addr           \
                            SetReg_16Bits_(v)      \
                            EDT_Tclk(1);               \
                            release_cpu }

#define WriteMemWordXv2(a,v){EDT_Tclk(0);              \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x0500) \
                            addr_16bit             \
                            SetReg_20Bits_(a)      \
                            EDT_Tclk(1);               \
                            data_to_addr           \
                            SetReg_16Bits_(v)      \
                            EDT_Tclk(0);               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x0501) \
                            EDT_Tclk(1);               \
                            EDT_Tclk(0);               \
                            EDT_Tclk(1);    }

#define WriteMemByte(a,v){halt_cpu                              \
                            EDT_Tclk(0);                        \
                            addr_16bit                          \
                            SetReg_16Bits_(a)                   \
                            data_to_addr                        \
                            SetReg_8Bits_(v)                    \
                            EDT_Tclk(1);                        \
                            EDT_Tclk(0);                        \
                            release_cpu }


#define ReadCpuReg_uShort(n,d)   { short op;       \
                            cntrl_sig_16bit        \
                            SetReg_16Bits(0x3401)  \
                            data_16bit             \
                            op = ((n << 8) & 0x0F00) | 0x4082; \
                            SetReg_16Bits(op)      \
                            EDT_Tclk(0);               \
                            data_capture           \
                            EDT_Tclk(1);               \
                            data_16bit             \
                            SetReg_16Bits(0x00fe)  \
                            EDT_Tclk(0);               \
                            data_capture           \
                            EDT_Tclk(1);               \
                            EDT_Tclk(0);               \
                            EDT_Tclk(1);               \
                            SetReg_16Bits(0)       \
                            d = (unsigned short)lOut;              \
                            EDT_Tclk(0);               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits(0x2401)  \
                            EDT_Tclk(1); }

#define ReadCpuReg(n,d)   { short op;              \
                            cntrl_sig_16bit        \
                            SetReg_16Bits(0x3401)  \
                            data_16bit             \
                            op = ((n << 8) & 0x0F00) | 0x4082; \
                            SetReg_16Bits(op)      \
                            EDT_Tclk(0);               \
                            data_capture           \
                            EDT_Tclk(1);               \
                            data_16bit             \
                            SetReg_16Bits(0x00fe)  \
                            EDT_Tclk(0);               \
                            data_capture           \
                            EDT_Tclk(1);               \
                            EDT_Tclk(0);               \
                            EDT_Tclk(1);               \
                            SetReg_16Bits(0)       \
                            d = lOut;              \
                            EDT_Tclk(0);               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits(0x2401)  \
                            EDT_Tclk(1); }

#define ReadCpuRegX(n,d)  { short op;              \
                            unsigned short Rx_l;   \
                            unsigned short Rx_h;   \
                            cntrl_sig_high_byte    \
                            SetReg_16Bits(0x34)    \
                            op = ((n << 8) & 0x0F00) | 0x60; \
                            data_16bit             \
                            EDT_Tclk(1);               \
                            SetReg_16Bits(op)      \
                            EDT_Tclk(0);               \
                            data_capture           \
                            EDT_Tclk(1);               \
                            data_16bit             \
                            EDT_Tclk(1);               \
                            SetReg_16Bits(0x00fc)  \
                            EDT_Tclk(0);               \
                            data_capture           \
                            EDT_Tclk(1);               \
                            SetReg_16Bits(0)       \
                            Rx_l = lOut;           \
                            EDT_Tclk(0);               \
                            EDT_Tclk(1);               \
                            SetReg_16Bits(0)       \
                            Rx_h = lOut;           \
                            EDT_Tclk(0);               \
                            cntrl_sig_high_byte    \
                            SetReg_8Bits(0x24)     \
                            EDT_Tclk(1);               \
                            d = ((unsigned long)Rx_h<<16) + Rx_l; }

#define ReadCpuRegXv2(n,d){ unsigned short Rx_l;   \
                            unsigned short Rx_h;   \
                            unsigned short jtagId = EDT_Instr(IR_CNTRL_SIG_CAPTURE);\
                            const unsigned short jmbAddr = (jtagId == 0x98) ? 0x14c : 0x18c;\
                            EDT_Tclk(0);           \
                            data_16bit             \
                            EDT_Tclk(1);           \
                            SetReg_16Bits(n)       \
                            cntrl_sig_16bit        \
                            SetReg_16Bits(0x1401)  \
                            data_16bit             \
                            EDT_TCLK               \
                            if (altRomAddressForCpuRead)\
                            {                      \
                                SetReg_16Bits(0x0ff6) \
                            }                      \
                            else                   \
                            {                      \
                                SetReg_16Bits(jmbAddr)\
                            }                      \
                            EDT_TCLK               \
                            SetReg_16Bits(0x3ffd)  \
                            EDT_Tclk(0);           \
                            if (altRomAddressForCpuRead)\
                            {                      \
                                cntrl_sig_16bit    \
                                SetReg_16Bits(0x0501)  \
                            }                      \
                            data_capture           \
                            EDT_Tclk(1);           \
                            SetReg_16Bits(0)       \
                            Rx_l = lOut;           \
                            EDT_TCLK               \
                            SetReg_16Bits(0)       \
                            Rx_h = lOut;           \
                            EDT_TCLK               \
                            EDT_TCLK               \
                            EDT_TCLK               \
                            if (!altRomAddressForCpuRead)\
                            {                      \
                                cntrl_sig_16bit    \
                                SetReg_16Bits(0x0501)  \
                            }                      \
                            EDT_Tclk(0);           \
                            data_capture           \
                            EDT_Tclk(1);           \
                            d = ((unsigned long)Rx_h<<16) + Rx_l; }

#define WriteCpuReg(n,s)   { short op;             \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x3401) \
                            data_16bit             \
                            op = (0x4030 | n);     \
                            SetReg_16Bits_(op)     \
                            EDT_Tclk(0);               \
                            data_capture           \
                            EDT_Tclk(1);               \
                            data_16bit             \
                            SetReg_16Bits_(s)      \
                            EDT_Tclk(0);               \
                            data_capture           \
                            EDT_Tclk(1);               \
                            data_16bit             \
                            SetReg_16Bits_(0x3ffd) \
                            EDT_Tclk(0);               \
                            data_capture           \
                            EDT_Tclk(1);               \
                            EDT_Tclk(0);               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x2401) \
                            EDT_Tclk(1); }

#define WriteCpuRegX(n,s)  { short op = 0x0080 | n | ((s>>8) & 0x0F00); \
                            cntrl_sig_high_byte    \
                            SetReg_8Bits_(0x34)    \
                            data_16bit             \
                            EDT_Tclk(1);               \
                            SetReg_16Bits_(op)     \
                            EDT_Tclk(0);               \
                            data_capture           \
                            EDT_Tclk(1);               \
                            data_16bit             \
                            EDT_Tclk(1);               \
                            SetReg_16Bits_(s&0xFFFF)\
                            EDT_Tclk(0);               \
                            data_capture           \
                            EDT_Tclk(1);               \
                            data_16bit             \
                            EDT_Tclk(1);               \
                            SetReg_16Bits_(0x3ffd) \
                            EDT_Tclk(0);               \
                            data_capture           \
                            EDT_Tclk(1);               \
                            EDT_Tclk(0);               \
                            cntrl_sig_high_byte    \
                            SetReg_8Bits_(0x24)    \
                            EDT_Tclk(1); }


#define WriteCpuRegXv2(n,s){EDT_Tclk(0);                \
                            data_16bit              \
                            EDT_Tclk(1);                \
                            SetReg_16Bits_(n)       \
                            cntrl_sig_16bit         \
                            SetReg_16Bits_(0x1401)  \
                            data_16bit              \
                            EDT_TCLK                    \
                            SetReg_16Bits_(s)       \
                            EDT_TCLK                    \
                            SetReg_16Bits_(0x3ffd)  \
                            EDT_TCLK                    \
                            EDT_Tclk(0);                \
                            addr_capture            \
                            SetReg_20Bits(0x00000)  \
                            EDT_Tclk(1);                \
                            cntrl_sig_16bit         \
                            SetReg_16Bits_(0x0501)  \
                            EDT_TCLK                    \
                            EDT_Tclk(0);                \
                            data_capture            \
                            EDT_Tclk(1); }


#define isInstrFetch      { cntrl_sig_capture      \
                            SetReg_16Bits(0)       \
                            if(lOut & 0x0080)      \
                            {                      \
                              lPass++;             \
                            }                      \
                            else                   \
                            {                      \
                              lFail++;             \
                            }                      \
                          }


#define SetPc(x)          { cntrl_sig_16bit        \
                            SetReg_16Bits_(0x3401) \
                            data_16bit             \
                            SetReg_16Bits_(0x4030) \
                            EDT_TCLK               \
                            SetReg_16Bits_(x)      \
                            EDT_TCLK               \
                            addr_capture           \
                            EDT_Tclk(0);           \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x2401) \
                            EDT_Tclk(1);           \
                          }

#define SetPcJtagBug(x)   {                         \
                            data_16bit              \
                            SetReg_16Bits_(0x4030)  \
                            EDT_Tclk(1);            \
                            EDT_Tclk(0);            \
                            SetReg_16Bits_(x)       \
                            EDT_Tclk(1);            \
                            EDT_Tclk(0);            \
                            EDT_Tclk(1);            \
                            EDT_Tclk(0);            \
                        }

#define SetPcX(x)       {   unsigned short pc_high = (unsigned short)(0x80 | (((x)>>8) & 0x0F00));\
                            unsigned short pc_low  = (unsigned short)((x) & 0xFFFF); \
                            cntrl_sig_high_byte    \
                            SetReg_8Bits_(0x34)    \
                            data_16bit             \
                            SetReg_16Bits_(pc_high)\
                            EDT_TCLK               \
                            SetReg_16Bits_(pc_low) \
                            EDT_TCLK               \
                            addr_capture           \
                            EDT_Tclk(0);           \
                            cntrl_sig_high_byte    \
                            SetReg_8Bits_(0x24)    \
                            EDT_Tclk(1);           \
                        }

#define SetPcXv2(Mova,X) {  cntrl_sig_capture      \
                            SetReg_16Bits_(0x0000) \
                            EDT_Tclk(0);              \
                            data_16bit             \
                            EDT_Tclk(1);               \
                            SetReg_16Bits(Mova)    \
                            EDT_Tclk(0);               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits(0x1400)  \
                            data_16bit             \
                            EDT_Tclk(0);               \
                            EDT_Tclk(1);               \
                            SetReg_16Bits(X)       \
                            EDT_Tclk(0);               \
                            EDT_Tclk(1);               \
                            SetReg_16Bits(0x4303)  \
                            EDT_Tclk(0);               \
                            addr_capture           \
                            SetReg_20Bits_(0x00000)}

#define SyncJtag()        { cntrl_sig_16bit                    \
                            SetReg_16Bits(0x2401)              \
                            cntrl_sig_capture                  \
                            i = 50;                            \
                            do                                 \
                            {                                  \
                              SetReg_16Bits(0x0000)            \
                              i--;                             \
                            }                                  \
                            while(((lOut == 0xFFFF) || !(lOut & 0x0200)) && i);  \
                          }

#define SyncJtagX()        { cntrl_sig_16bit                    \
                            SetReg_16Bits(0x2401)              \
                            cntrl_sig_capture                  \
                            SetReg_16Bits(0x0000)              \
                            EDT_Tclk(1);                           \
                            if (!(lOut & 0x0200))              \
                            {                                  \
                                cntrl_sig_high_byte            \
                                SetReg_8Bits(0x34)             \
                                eem_data_exchange32            \
                                SetReg_32Bits_(0x89)           \
                                SetReg_32Bits(0)               \
                                eem_data_exchange32            \
                                SetReg_32Bits_(0x88)           \
                                SetReg_32Bits(lOut|0x40)       \
                                eem_data_exchange32            \
                                SetReg_32Bits_(0x88)           \
                                SetReg_32Bits(lOut)            \
                            }                                  \
                            cntrl_sig_capture                  \
                            i = 50;                            \
                            do                                 \
                            {                                  \
                              SetReg_16Bits(0x0000)            \
                              i--;                             \
                            }                                  \
                            while(((lOut == 0xFFFF) || !(lOut & 0x0200)) && i);\
                          }

#define SyncJtagXv2()     { cntrl_sig_16bit                    \
                            SetReg_16Bits(0x1501)              \
                            cntrl_sig_capture                  \
                            i = 50;                            \
                            do                                 \
                            {                                  \
                              SetReg_16Bits(0x0000)            \
                              i--;                             \
                            }                                  \
                            while(((lOut == 0xFFFF) || !(lOut & 0x0200)) && i);  \
                          }

#define wait_for_synch     {int i = 0;                                    \
                            lPASS = lFAIL = 0;                            \
                            cntrl_sig_capture                             \
                            SetReg_16Bits(0)                              \
                            while(!(lOut & 0x0200) && i < 50)             \
                            {                                             \
                              SetReg_16Bits(0)                            \
                              i++;                                        \
                            };                                            \
                            if(i >= 50)                                   \
                            {                                             \
                              lFAIL++;                                    \
                            }                                             \
                            else                                          \
                            {                                             \
                              lPASS++;                                    \
                            }                                             \
                          }

#define Shift_Bits(NumOfBits,TdiVal){\
                            unsigned short tempVar;           \
                            if(TdiVal)                        \
                              TDIset1;                        \
                            else                              \
                              TDIset0;                        \
                            for(tempVar = NumOfBits;tempVar > 0; tempVar--)\
                            {\
                               TCKset0;\
                               TCKset1;\
                            }\
                          }

#define RestoreTestRegs(){ if(devicePowerSettings.powerTestReg3VMask) \
                           {                                          \
                             test_reg_3V                              \
                             SetReg_16Bits(devicePowerSettings.powerTestReg3VDefault) \
                             EDT_Delay_1ms(20);                       \
                           }                                          \
                           if(devicePowerSettings.powerTestRegMask)   \
                           {                                          \
                             test_reg                                 \
                             SetReg_32Bits(devicePowerSettings.powerTestRegDefault) \
                             EDT_Delay_1ms(20);                       \
                           }                                          \
                         }

#define EnableLpmx5()   { if(devicePowerSettings.powerTestReg3VMask)  \
                          {                                           \
                            test_reg_3V                               \
                            SetReg_16Bits(devicePowerSettings.powerTestReg3VDefault) \
                                                                      \
                            test_reg_3V                               \
                            SetReg_16Bits_(lOut &                     \
                              ~devicePowerSettings.powerTestReg3VMask|\
                              devicePowerSettings.enableLpmx5TestReg3V)\
                                                                      \
                            EDT_Delay_1ms(20);                        \
                          }                                           \
                                                                      \
                          if(devicePowerSettings.powerTestRegMask)    \
                          {                                           \
                            test_reg                                  \
                            SetReg_32Bits(devicePowerSettings.powerTestRegDefault) \
                                                                      \
                            test_reg                                  \
                            SetReg_32Bits_(lOut_long &                \
                              ~devicePowerSettings.powerTestRegMask|  \
                              devicePowerSettings.enableLpmx5TestReg) \
                                                                      \
                            EDT_Delay_1ms(20);                        \
                          }                                           \
                        }

#define DisableLpmx5()   { if(devicePowerSettings.powerTestReg3VMask) \
                          {                                           \
                            test_reg_3V                               \
                            SetReg_16Bits(devicePowerSettings.powerTestReg3VDefault)  \
                                                                      \
                            test_reg_3V                               \
                            SetReg_16Bits_(lOut &                     \
                              ~devicePowerSettings.powerTestReg3VMask|\
                              devicePowerSettings.disableLpmx5TestReg3V)\
                                                                      \
                            EDT_Delay_1ms(20);                        \
                          }                                           \
                                                                      \
                          if(devicePowerSettings.powerTestRegMask)    \
                          {                                           \
                            test_reg                                  \
                            SetReg_32Bits(devicePowerSettings.powerTestRegDefault) \
                                                                      \
                            test_reg                                  \
                            SetReg_32Bits_(lOut_long &                \
                              ~devicePowerSettings.powerTestRegMask|  \
                              devicePowerSettings.disableLpmx5TestReg)\
                                                                      \
                            EDT_Delay_1ms(20);                        \
                          }                                           \
                        }


#define OUT1RDY 0x0008
#define OUT0RDY 0x0004
#define IN1RDY  0x0002
#define IN0RDY  0x0001

#define JMB32B  0x0010
#define OUTREQ  0x0004
#define INREQ   0x0001

#define i_ReadJmbOut(X) { unsigned short sJMBINCTL;           \
                          unsigned short sJMBOUT0, sJMBOUT1;  \
                          sJMBINCTL = 0;                      \
                          jmb_exchange                        \
                          SetReg_16Bits(sJMBINCTL)            \
                          if(lOut & OUT1RDY)                  \
                          {                                   \
                              sJMBINCTL |= JMB32B + OUTREQ;   \
                              SetReg_16Bits(sJMBINCTL)        \
                              SetReg_16Bits(0)                \
                              sJMBOUT0 = lOut;                \
                              SetReg_16Bits(0)                \
                              sJMBOUT1 = lOut;                \
                              X = ((unsigned long)sJMBOUT1<<16) + sJMBOUT0; \
                          }                                   \
                         }

#define i_ReadJmbOut16(X) { unsigned short sJMBINCTL;         \
                          unsigned short sJMBOUT0;            \
                          sJMBINCTL = 0;                      \
                          jmb_exchange                        \
                          SetReg_16Bits(sJMBINCTL)            \
                          if(lOut & OUT0RDY)                  \
                          {                                   \
                              sJMBINCTL |=  OUTREQ;           \
                              SetReg_16Bits(sJMBINCTL)        \
                              SetReg_16Bits(0)                \
                              sJMBOUT0 = lOut;                \
                              X = sJMBOUT0;                   \
                          }                                   \
                         }

#define i_WriteJmbIn(X) { unsigned short sJMBINCTL;                   \
                          unsigned short sJMBIN0;                     \
                          unsigned long Timeout = 0;                  \
                          jtagMailboxIn = 0;                          \
                          sJMBIN0 = (unsigned short)(X & 0x0000FFFF); \
                          sJMBINCTL = INREQ;                          \
                          jmb_exchange                                \
                          do                                          \
                          {                                           \
                            SetReg_16Bits(0x0000)                     \
                            Timeout++;                                \
                            if(Timeout >= 3000)                       \
                            {                                         \
                                jtagMailboxIn = 1;                    \
                                break;                                \
                            }                                         \
                          }                                           \
                          while(!(lOut & IN0RDY) && Timeout < 3000);  \
                          if(Timeout < 3000)                          \
                          {                                           \
                            SetReg_16Bits(sJMBINCTL)                  \
                            SetReg_16Bits(sJMBIN0)                    \
                          }                                           \
                        }

#define i_WriteJmbIn32(X,Y) { unsigned short sJMBINCTL;                \
                          unsigned short sJMBIN0,sJMBIN1;              \
                          unsigned long Timeout = 0;                   \
                          jtagMailboxIn = 0;                           \
                          sJMBIN0 = (unsigned short)(X & 0x0000FFFF);  \
                          sJMBIN1 = (unsigned short)(Y & 0x0000FFFF);  \
                          sJMBINCTL =  JMB32B | INREQ;                 \
                          jmb_exchange                                 \
                          do                                           \
                          {                                            \
                            SetReg_16Bits(0x0)                         \
                            Timeout++;                                 \
                            if(Timeout >= 3000)                        \
                            {                                          \
                                jtagMailboxIn = 1;                     \
                                break;                                 \
                            }                                          \
                          }                                            \
                          while(!(lOut & IN1RDY) && Timeout < 3000);   \
                          if(Timeout < 3000)                           \
                          {                                            \
                            sJMBINCTL = 0x11;                          \
                            SetReg_16Bits(sJMBINCTL)                   \
                            SetReg_16Bits(sJMBIN0)                     \
                            SetReg_16Bits(sJMBIN1)                     \
                          }                                            \
                        }


#define i_InitStreamJmbIn32() { unsigned short sJMBINCTL;              \
                          unsigned long Timeout = 0;                   \
                          jtagMailboxIn = 0;                           \
                          jmb_exchange                                 \
                          do                                           \
                          {                                            \
                            SetReg_16Bits(0x0)                         \
                            Timeout++;                                 \
                            if(Timeout >= 3000)                        \
                            {                                          \
                                jtagMailboxIn = 1;                     \
                                break;                                 \
                            }                                          \
                          }                                            \
                          while(!(lOut & IN1RDY) && Timeout < 3000);   \
                          if(Timeout < 3000)                           \
                          {                                            \
                            sJMBINCTL = 0x11;                          \
                            SetReg_16Bits(sJMBINCTL)                   \
                          }                                            \
                        }

#define i_StreamDataJmbIn32(X,Y) { unsigned short sJMBINCTL;           \
                          unsigned short sJMBIN0,sJMBIN1;              \
                          unsigned long Timeout = 0;                   \
                          jtagMailboxIn = 0;                           \
                          sJMBIN0 = (X );                              \
                          sJMBIN1 = (Y );                              \
                          Timeout =0;                                  \
                          sJMBINCTL = 0x11;                            \
                          do                                           \
                          {                                            \
                            SetReg_16Bits(sJMBINCTL)                   \
                            Timeout++;                                 \
                          }                                            \
                          while(!(lOut & IN1RDY)&& Timeout <1500);     \
                          Timeout =0;                                  \
                          do                                           \
                          {                                            \
                            SetReg_16Bits(sJMBIN0)                     \
                            Timeout++;                                 \
                          }                                            \
                          while(lOut != 0xFFFF && Timeout <1500);      \
                            if(Timeout >= 1500){                       \
                                 jtagMailboxIn = 1;                    \
                                  break;   }                           \
                          Timeout =0;                                  \
                          do                                           \
                          {                                            \
                            SetReg_16Bits(sJMBIN1)                     \
                             Timeout++;                                \
                          }                                            \
                          while(lOut != 0xFFFF && Timeout <1500);      \
                            if(Timeout >= 1500){                       \
                                 jtagMailboxIn = 1;                    \
                                 break;   }                            \
                       }
#endif
// this last rising edge in the SetPc macro is required for debugging,
// otherwise the EEM will trigger immediately on the instruction fetch signal
