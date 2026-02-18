/*
 *	dfu_load.c
 *	(enter file description here)
 *
 *	Created by zuidec on 02/17/26
 */
#include "main.h"
#include "dfu_load.h"


#define DFU_MAGIC 0xB00710ADU   // Arbitrary number

void reboot_dfu(void)   {
    *((uint32_t*)0x20000000) = DFU_MAGIC;
    NVIC_SystemReset();
}
