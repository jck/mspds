/*
 * DatabaseImplementation.cpp
 *
 * Definition of default Eem Timer Name-Value mapping.
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
#include "DatabaseImplementation.h"

using namespace TI::DLL430::TemplateDeviceDb;

//default initializations - when c++0x is used someday, define inline, because intention is more clear when visible in h-file at declaration
const ClockPair EemTimerImpl::Timer::Empty = {"", 0};
const ClockPair EemTimerImpl::Timer::CRC16 = {"CRC16", 0};
const ClockPair EemTimerImpl::Timer::WDT_A = {"Watchdog Timer", 0x0A};
const ClockPair EemTimerImpl::Timer::CCS = {"CCS", 0x1E};
const ClockPair EemTimerImpl::Timer::USB = {"USB", 0x40};
const ClockPair EemTimerImpl::Timer::AES = {"AES128", 0x60};

const ClockPair EemTimerImpl::Timer::TA = {"TimerA", 0};
const ClockPair EemTimerImpl::Timer::TB = {"TimerB", 0};

const ClockPair EemTimerImpl::Timer::TA3 = {"TimerA3", 0};
const ClockPair EemTimerImpl::Timer::TB3 = {"TimerB3", 0};

const ClockPair EemTimerImpl::Timer::BT = {"BasicTimer", 0};
const ClockPair EemTimerImpl::Timer::BT_RTC = {"BasicTimer/RTC", 0};

const ClockPair EemTimerImpl::Timer::TA2_0 = {"Timer0_A2", 0x8B};
const ClockPair EemTimerImpl::Timer::TA2_1 = {"Timer1_A2", 0x8C};
const ClockPair EemTimerImpl::Timer::TA2_2 = {"Timer2_A2", 0x8D};
const ClockPair EemTimerImpl::Timer::TA3_0 = {"Timer0_A3", 0x8E};
const ClockPair EemTimerImpl::Timer::TA3_1 = {"Timer1_A3", 0x8F};
const ClockPair EemTimerImpl::Timer::TA3_2 = {"Timer2_A3", 0x90};
const ClockPair EemTimerImpl::Timer::TA3_3 = {"Timer3_A3", 0x88};
const ClockPair EemTimerImpl::Timer::TA5_0 = {"Timer0_A5", 0x91};
const ClockPair EemTimerImpl::Timer::TA5_1 = {"Timer1_A5", 0x92};
const ClockPair EemTimerImpl::Timer::TA5_2 = {"Timer2_A5", 0x93};
const ClockPair EemTimerImpl::Timer::TA7_0 = {"Timer0_A7", 0x94};
const ClockPair EemTimerImpl::Timer::TA7_1 = {"Timer1_A7", 0x95};
const ClockPair EemTimerImpl::Timer::TA7_2 = {"Timer2_A7", 0x96};

const ClockPair EemTimerImpl::Timer::TD3_0 = {"Timer0_D3", 0x74};
const ClockPair EemTimerImpl::Timer::TD3_1 = {"Timer1_D3", 0x75};
const ClockPair EemTimerImpl::Timer::TD3_2 = {"Timer2_D3", 0x76};
const ClockPair EemTimerImpl::Timer::TD3_3 = {"Timer3_D3", 0x77};
const ClockPair EemTimerImpl::Timer::TB3_0 = {"Timer0_B3", 0x97};
const ClockPair EemTimerImpl::Timer::TB3_1 = {"Timer1_B3", 0x98};
const ClockPair EemTimerImpl::Timer::TB3_2 = {"Timer2_B3", 0x99};
const ClockPair EemTimerImpl::Timer::TB5_0 = {"Timer0_B5", 0x9A};
const ClockPair EemTimerImpl::Timer::TB5_1 = {"Timer1_B5", 0x9B};
const ClockPair EemTimerImpl::Timer::TB5_2 = {"Timer2_B5", 0x9C};
const ClockPair EemTimerImpl::Timer::TB7_0 = {"Timer0_B7", 0x9D};
const ClockPair EemTimerImpl::Timer::TB7_1 = {"Timer1_B7", 0x9E};
const ClockPair EemTimerImpl::Timer::TB7_2 = {"Timer2_B7", 0x9F};

const ClockPair EemTimerImpl::Timer::FLASH_CTRL = {"Flash Control", 0};
const ClockPair EemTimerImpl::Timer::FLASH_CTRLER = {"Flash Controller", 0};

const ClockPair EemTimerImpl::Timer::USART0 = {"USART0", 0};
const ClockPair EemTimerImpl::Timer::USART1 = {"USART1", 0};

const ClockPair EemTimerImpl::Timer::USCI0 = {"USCI0", 0x28};
const ClockPair EemTimerImpl::Timer::USCI1 = {"USCI1", 0x29};
const ClockPair EemTimerImpl::Timer::USCI2 = {"USCI2", 0x2A};
const ClockPair EemTimerImpl::Timer::USCI3 = {"USCI3", 0x2B};

const ClockPair EemTimerImpl::Timer::eUSCIA0 = {"eUSCIA0", 0x2C};
const ClockPair EemTimerImpl::Timer::eUSCIA1 = {"eUSCIA1", 0x2D};
const ClockPair EemTimerImpl::Timer::eUSCIA2 = {"eUSCIA2", 0x2E};
const ClockPair EemTimerImpl::Timer::eUSCIA3 = {"eUSCIA3", 0x2F};
const ClockPair EemTimerImpl::Timer::eUSCIB0 = {"eUSCIB0", 0x30};
const ClockPair EemTimerImpl::Timer::eUSCIB1 = {"eUSCIB1", 0x31};

const ClockPair EemTimerImpl::Timer::TB_MCLK = {"TimerB/MCLK (Pin)", 0};
const ClockPair EemTimerImpl::Timer::TA_SMCLK = {"TimerA/SMCLK (Pin)", 0};
const ClockPair EemTimerImpl::Timer::WDT_ACLK = {"Watchdog Timer/ACLK (Pin)", 0};

const ClockPair EemTimerImpl::Timer::MCLKpin = {"MCLK (Pin)", 0};
const ClockPair EemTimerImpl::Timer::SMCLKpin = {"SMCLK (Pin)", 0};
const ClockPair EemTimerImpl::Timer::ACLKpin = {"ACLK (Pin)", 0};

const ClockPair EemTimerImpl::Timer::RTC = {"RTC", 0x8A};
const ClockPair EemTimerImpl::Timer::BTRTC = {"BTRTC", 0x8A};

const ClockPair EemTimerImpl::Timer::COMP_B = {"Comparator B", 0xA8};
const ClockPair EemTimerImpl::Timer::COMP_D = {"Comparator D", 0xA8};
const ClockPair EemTimerImpl::Timer::COMP_E = {"Comparator E", 0xA8};
const ClockPair EemTimerImpl::Timer::LCD_B = {"LCDB", 0xB0};
const ClockPair EemTimerImpl::Timer::LCD_C = {"LCDC", 0xB0};
const ClockPair EemTimerImpl::Timer::LCD_E = {"LCDE", 0xB0};

const ClockPair EemTimerImpl::Timer::LCD_FREQ = {"LCD Frequency", 0};

const ClockPair EemTimerImpl::Timer::APOOL = {"APOOL", 0xB5};

const ClockPair EemTimerImpl::Timer::RF1A = {"RF1A", 0xBC};
const ClockPair EemTimerImpl::Timer::RF1B = {"RF1B", 0xBD};
const ClockPair EemTimerImpl::Timer::RF2A = {"RF2A", 0xBE};
const ClockPair EemTimerImpl::Timer::RF2B = {"RF2B", 0xBF};

const ClockPair EemTimerImpl::Timer::RF13M = {"RF13M", 0xBB};
const ClockPair EemTimerImpl::Timer::RFSD14 = {"RFSD14", 0xD9};

const ClockPair EemTimerImpl::Timer::DAC12_0 = {"DAC12", 0xC0};
const ClockPair EemTimerImpl::Timer::DAC12_1 = {"DAC12", 0xC1};
const ClockPair EemTimerImpl::Timer::SD16 = {"SD16", 0};
const ClockPair EemTimerImpl::Timer::SD16A_4 = {"SD16A", 0xD4};
const ClockPair EemTimerImpl::Timer::SD24B = {"SD24B", 0xD5};
const ClockPair EemTimerImpl::Timer::ADC10_A = {"ADC10", 0xD6};
const ClockPair EemTimerImpl::Timer::ADC10_B = {"ADC10B", 0xD6};
const ClockPair EemTimerImpl::Timer::ADC12 = {"ADC12", 0};
const ClockPair EemTimerImpl::Timer::ADC12_A = {"ADC12A", 0xD8};
const ClockPair EemTimerImpl::Timer::ADC12_B = {"ADC12B", 0xD8};

const ClockPair EemTimerImpl::Timer::PORT = {"PORT", 0x50};
const ClockPair EemTimerImpl::Timer::CAPTIVATE = { "Captivate", 0xB7 };

const ClockName EemClocksImpl::Clocks::Empty = "";
const ClockName EemClocksImpl::Clocks::TACLK = "TACLK";
const ClockName EemClocksImpl::Clocks::SMCLK = "SMCLK";
const ClockName EemClocksImpl::Clocks::ACLK = "ACLK";
