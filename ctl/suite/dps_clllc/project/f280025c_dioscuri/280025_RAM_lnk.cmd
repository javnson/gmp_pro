MEMORY
{
   BEGIN           	: origin = 0x000000, length = 0x000002
   BOOT_RSVD        : origin = 0x00000002, length = 0x00000126
   RAMM0            : origin = 0x00000128, length = 0x000002D8
   RAMM1            : origin = 0x00000400, length = 0x000003F8     /* on-chip RAM block M1 */
// RAMM1_RSVD       : origin = 0x000007F8, length = 0x00000008 /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */
/* RAMLS4           : origin = 0x0000A000, length = 0x00000800
   RAMLS5           : origin = 0x0000A800, length = 0x00000800
   RAMLS6           : origin = 0x0000B000, length = 0x00000800
   RAMLS7           : origin = 0x0000B800, length = 0x00000800 */

   /* Combining all the LS RAMs */
   RAMLS4567        : origin = 0x0000A000, length = 0x00002000
   RAMGS0           : origin = 0x0000C000, length = 0x000007F8
// RAMGS0_RSVD      : origin = 0x0000C7F8, length = 0x00000008 /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   BOOTROM          : origin = 0x003F0000, length = 0x00008000
   BOOTROM_EXT      : origin = 0x003F8000, length = 0x00007FC0
   RESET            : origin = 0x003FFFC0, length = 0x00000002
}


SECTIONS
{
   .text            : > RAMLS4567
   .TI.ramfunc      : > RAMM0
   .cinit           : > RAMM0
   .switch          : > RAMM0
   .reset           : > RESET,                  TYPE = DSECT /* not used, */
   .cio             : > RAMGS0
   codestart        : > BEGIN

   .stack           : > RAMM1
#if defined(__TI_EABI__)
   .bss             : > RAMLS4567
   .bss:output      : > RAMLS4567
   .init_array      : > RAMM0
   .const           : > RAMGS0
   .data            : > RAMLS4567
   .sysmem          : > RAMLS4567
#else
   .pinit           : > RAMM0
   .ebss            : > RAMLS4567
   .econst          : > RAMGS0
   .esysmem         : > RAMLS4567
#endif

    ramgs0 : > RAMGS0

    /*  Allocate IQ math areas: */
   IQmath           : > RAMLS4567
   IQmathTables     : > RAMLS4567
}
/*
//===========================================================================
// End of file.
//===========================================================================
*/