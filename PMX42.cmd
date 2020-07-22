/******************************************************************************
 *
 * Default Linker Command file for the Texas Instruments TM4C1294NCPDT
 *
 * This is derived from revision 11167 of the TivaWare Library.
 *
 *****************************************************************************/

/*
 * Settings for standard non-boot loader compilation. Reset vectors at zero.
 */

//#define APP_BASE 0x00000000
//#define APP_LENG 0x00100000

/*
 * Settings for use with bootloader compilation (app base is offset, app length is smaller)
 *
 * NOTE - for TI-RTOS you must also edit the XDC CFG file and add the following
 *
 *     Program.sectMap[".resetVecs"].loadAddress = 4096;
 *
 * The APP_BASE and the resetVecs parameters must match for the bootloader to enter
 * our application snf interrupt vectors at the proper address. We've allowed
 * 4k space for our bootloader and the application starts at this offset.
 */

#define	APP_BASE 0x00004000
#define	APP_LENG 0x000FC000

#define	RAM_BASE 0x20000000
#define RAM_LENG 0x00040000

MEMORY
{
    /* BOOT (RX)  : origin = 0x00000000, length = 0x00001000 */
    FLASH (RX) : origin = APP_BASE, length = APP_LENG
    SRAM (RWX) : origin = RAM_BASE, length = RAM_LENG
}

/* Section allocation in memory */

SECTIONS
{
    .text   :   > FLASH
#ifdef __TI_COMPILER_VERSION
#if __TI_COMPILER_VERSION >= 15009000
    .TI.ramfunc : {} load=FLASH, run=SRAM, table(BINIT)
#endif
#endif
    .const  :   > FLASH
    .cinit  :   > FLASH
    .pinit  :   > FLASH
    .init_array : > FLASH

    .data   :   > SRAM
    .bss    :   > SRAM
    .sysmem :   > SRAM
    .stack  :   > SRAM
}
