/*
 * ESP32 ULP Program
 *
 * Copyright (C) 2018 KIMATA Tetsuya <kimata@green-rabbit.net>
 *
 * This program is free software ; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "util_macro.S"

.set LOW_HYST_BIT,3
.set SENSE_ENTRY_SIZE,(2+2+4+4+4)
.set SENSE_BUF_SIZE,SENSE_ENTRY_SIZE*100

    .bss
    .balign 4
    .global sense_count
sense_count: .long 0
    
    .global sense_data
sense_data: .long 0
    .skip SENSE_BUF_SIZE
    
    .global prev_sense_data
prev_sense_data: .long 0
    .skip SENSE_ENTRY_SIZE
    
stack:
    .skip 32
    .global stackEnd
stackEnd:
    .long 0

    .text
    .global entry
entry:
    move r3,stackEnd

loop:
    halt
    
wake_cpu:
    wake
    // stop wake up timer
    WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
exit:   
    halt
    
// Local Variables:
// mode: asm
// comment-start: "// "
// End:     
