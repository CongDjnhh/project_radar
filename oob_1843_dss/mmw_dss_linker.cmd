/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS
{
    systemHeap : {} >> L2SRAM_UMAP0 | L2SRAM_UMAP1
    .hwaBufs: load = HWA_RAM, type = NOINIT
    .l3ram: {} >> L3SRAM
    .dpc_l2Heap: { } >> L2SRAM_UMAP0 | L2SRAM_UMAP1
    .dpc_l1Heap: { } >> L1DSRAM
    /* Current-frame navigation classification scratch. NOINIT avoids startup
       copy/clear cost; nav_point_gen initializes every array before use. */
    .navScratch: load = L1DSRAM, type = NOINIT
    /* Small FFT scratch buffers for dense_elevation.c (P1.3).
       gDenseFftInput (512 B) and gDenseFftTwiddle (512 B) are accessed on every range
       bin during Doppler FFT. Placing them in L1D SRAM (0 wait-state) removes the
       L3 cache-miss penalty from the hottest inner loops. Total: 1,024 bytes --
       well within the free L1DSRAM after .navScratch. NOINIT is correct: dense_elevation.c
       initialises them before use. */
    .l1dscratch: load = L1DSRAM, type = NOINIT
    /* P3.1: Map .l1psram to the 16KB of free L1P SRAM (since cache is only 16KB).
       Used to store the hottest inner loop (processDoppler) for 0-wait-state
       instruction fetches. */
    .l1psram: load = L1PSRAM
    .demoSharedMem: { } >> HSRAM
}
/*----------------------------------------------------------------------------*/

