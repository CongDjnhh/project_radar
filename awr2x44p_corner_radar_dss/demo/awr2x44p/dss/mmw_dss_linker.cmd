/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS
{
    systemHeap : {} >> DSS_L2
    .hwaBufs: load = HWA_RAM, type = NOINIT
    .l3ram: {} >> DSS_L3
    .dpc_l2Heap: { } >> DSS_L2
    .demoSharedMem: { } >> DSS_L3
    .calibSharedMem: { } >> DSS_L3
    .preProcBuf: {} >> DSS_L2
    .customCode: { } >> DSS_L2
}
/*----------------------------------------------------------------------------*/
/*.demoSharedMem: { } >> HSRAM*/
