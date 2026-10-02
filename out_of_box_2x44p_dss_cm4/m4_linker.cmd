
/* make sure below retain is there in your linker command file, it keeps the vector table in the final binary */
--retain="*(.vectors)"
/* This is the stack that is used by code running within main()
 * In case of NORTOS,
 * - This means all the code outside of ISR uses this stack
 * In case of FreeRTOS
 * - This means all the code until vTaskStartScheduler() is called in main()
 *   uses this stack.
 * - After vTaskStartScheduler() each task created in FreeRTOS has its own stack
 */
--stack_size=2048
/* This is the heap size for malloc() API in NORTOS and FreeRTOS
 * This is also the heap used by pvPortMalloc in FreeRTOS
 */
--heap_size=1536


SECTIONS
{
    /* This has the M4 entry point and vector table, this MUST be at 0x0 */
    .vectors:{} palign(8) > M4_VECS
    .text:   {} palign(8) > M4_RAM     /* This is where code resides */

    .bss:    {} palign(8) > M4_RAM     /* This is where uninitialized globals go */
    RUN_START(__BSS_START)
    RUN_END(__BSS_END)

    .data:   {} palign(8) > M4_RAM   /* This is where initialized globals and static go */
    .rodata: {} palign(8) > M4_RAM   /* This is where const's go */
    .sysmem: {} palign(8) > M4_RAM   /* This is where the malloc heap goes */
    .stack:  {} palign(8) > M4_RAM   /* This is where the main() stack goes */
    .dpcGlobals: {} palign(8) > M4_RAM | DSS_L3_BSS /* This is where DPC global object goes */

    /* Sections needed for C++ projects */
    .ARM.exidx:     {} palign(8) > M4_RAM  /* Needed for C++ exception handling */
    .init_array:    {} palign(8) > M4_RAM  /* Contains function pointers called before main */
    .fini_array:    {} palign(8) > M4_RAM  /* Contains function pointers called after main */

    /* this is used only when IPC RPMessage is enabled, else this is not used */
    .bss.ipc_vring_mem   (NOLOAD) : {} > RTOS_NORTOS_IPC_SHM_MEM
}

MEMORY
{
    M4_VECS : ORIGIN = 0x00000000 , LENGTH = 0x00000200
    M4_RAM  : ORIGIN = 0x00000200 , LENGTH = 0x0000FE00
    DSS_L3  : ORIGIN = 0x88000000 , LENGTH = 0x002C0000
    /* Section for storing code, which can overwritten by allocations in DSS_L3 */
    DSS_L3_REUSABLE: ORIGIN = 0x882C0000 , LENGTH = 0x20000
    /* Section for allocating persistent buffers during config, unused part can be overwritten by allocations in DSS_L3 */
    DSS_L3_CONFIG_ALLOCATION: ORIGIN = 0x882E0000, LENGTH = 0x1E000
    DSS_L3_BSS : ORIGIN = 0x882FE000, LENGTH = 0x00002000

    /* 1st 512 B of DSS mailbox memory and MSS mailbox memory is used for IPC with R4 and should not be used by application */
    /* MSS mailbox memory is used as shared memory, we dont use bottom 32*12 bytes, since its used as SW queue by ipc_notify */
    RTOS_NORTOS_IPC_SHM_MEM : ORIGIN = 0xC5000200, LENGTH = 0x1C80

}
