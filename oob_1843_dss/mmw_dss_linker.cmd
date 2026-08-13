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
    .demoSharedMem: { } >> HSRAM
}
/*----------------------------------------------------------------------------*/

