/*
 *	dfu_load.h
 *	(enter file description here)
 *
 *	Created by zuidec on 02/17/26
 */

#ifndef DFU_LOAD_H
#define DFU_LOAD_H	// BEGIN DFU_LOAD_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 *	Includes
 */



/*
 *	Defines
 */



/*
 *	Structs, unions, etc...
 */



/*
 *	Function prototypes
 */

void reboot_dfu(void);

/*****************************************************************************/
 // Add the following to the startup.s file:
 //
 //         Reset_Handler:  
 //         ldr r0, =0x20000000
 //         ldr r1, =0xB00710AD
 //         ldr r2, [r0, #0]
 //         str r0, [r0, #0]
 //         cmp r2, r1
 //         beq Reboot_Loader
 //         ldr   sp, =_estack     /* set stack pointer */
 //         
 //       /* Call the clock system initialization function.*/
 //         bl  SystemInit  

 //       /* Copy the data segment initializers from flash to SRAM */  
 //         ldr r0, =_sdata
 //         ldr r1, =_edata
 //         ldr r2, =_sidata
 //         movs r3, #0
 //         b LoopCopyDataInit

 //       /* begin jump to DFU */
 //       Reboot_Loader:
 //         ldr r0, =0x1FFF0000
 //         ldr sp, [r0, #0]
 //         ldr r0, [r0, #4]
 //         bx r0
 //       /* end jump to DFU */

 //       CopyDataInit:
 //         ldr r4, [r2, r3]
 //         str r4, [r0, r3]
 //         adds r3, r3, #4

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif	// END DFU_LOAD_H
