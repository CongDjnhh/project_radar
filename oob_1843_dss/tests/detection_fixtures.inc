/* Target-only fixtures, MMWDEMO_SURFACE_SELFTEST. Requires the baseline
 * profile to have been configured; the startup hook runs before RF starts.
 * Replaces frame scratch/cube contents; acquisition must remain stopped. */
volatile uint32_t gSurfaceDetectionSelfTestFailures;
#pragma RETAIN(MmwDemo_SurfaceDetection_selfTest)
#pragma CODE_SECTION(MmwDemo_SurfaceDetection_selfTest, ".surfaceSelfTest")
#pragma CODE_SECTION(DetectionFixture_caso, ".surfaceSelfTest")
volatile uint32_t gSurfaceCfarReferenceCount[2];
volatile uint32_t gSurfaceCfarHardwareCount[2];
#pragma DATA_SECTION(gSurfaceDenseExpected, ".l3ram")
static MmwDemo_DenseElevation_OutputSample gSurfaceDenseExpected[576];

/* Independent scalar CASO, matching the installed SDK test convention:
 * one-sided range edges, wrapped Doppler, log-domain additive threshold.
 * Physical winLen is in cells; the HWA noise-sample register uses pairs. */
static uint8_t DetectionFixture_caso(const uint16_t *data, uint16_t n,
    uint16_t stride, uint16_t index, const DPU_CFARCAProc_CfarCfg *cfg)
{
    uint32_t left=0U,right=0U,noise;
    uint16_t k;
    uint16_t extent=cfg->guardLen+cfg->winLen;
    for (k=1U;k<=cfg->winLen;k++) {
        int32_t l=(int32_t)index-cfg->guardLen-k;
        int32_t r=(int32_t)index+cfg->guardLen+k;
        if (cfg->cyclicMode) {
            l=(l+n)%n; r%=n;
        }
        if (l>=0) left+=data[l*stride];
        if (r<n) right+=data[r*stride];
    }
    if (!cfg->cyclicMode && index<extent) noise=right;
    else if (!cfg->cyclicMode && index>=n-extent) noise=left;
    else noise=(left<right) ? left : right;
    noise>>=cfg->noiseDivShift;
    return data[index*stride]>noise+cfg->thresholdScale;
}

uint32_t MmwDemo_SurfaceDetection_selfTest(void)
{
    ObjDetObj *obj=gObjDetObj;
    SubFrameObj *sf;
    MmwDemo_DenseElevation_StaticCfg dc;
    MmwDemo_DenseElevation_Cfg cfg;
    cmplx16ImRe_t comp[SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL];
    DPIF_CFARDetList list[3],tmp;
    cmplx16ImRe_t *cube;
    uint16_t *matrix;
    uint32_t fail=0U,bytes=0U,i,j,total=0U,seed=1U,pattern;
    MmwDemo_DenseElevation_OutputHeader *header=(void *)gNavCandidateScratch;
    MmwDemo_DenseElevation_OutputSample *samples=(void *)(gNavCandidateScratch+sizeof(*header));
    if (!obj) return 0x8000U;
    sf=&obj->subFrameObj[obj->subFrameIndx];
    if (sf->staticCfg.numRangeBins!=256U || sf->staticCfg.numDopplerBins!=16U ||
        sf->staticCfg.numDopplerChirps!=16U ||
        sf->dynCfg.cfarCfgRange.averageMode!=2U ||
        sf->dynCfg.cfarCfgDoppler.averageMode!=2U ||
        sf->dynCfg.cfarCfgRange.cyclicMode ||
        !sf->dynCfg.cfarCfgDoppler.cyclicMode ||
        sf->dynCfg.cfarCfgRange.peakGroupingEn ||
        sf->dynCfg.cfarCfgDoppler.peakGroupingEn) return 0x8000U;
    memset(&dc,0,sizeof(dc)); memset(&cfg,0,sizeof(cfg)); memset((void *)list,0,sizeof(list));
    cfg.minAzimuthDeg=-90.0f; cfg.maxAzimuthDeg=90.0f;
    cfg.minElevationDeg=-90.0f; cfg.maxElevationDeg=90.0f;
    cfg.minRangeBin=0U; cfg.maxRangeBin=255U;
    dc.radarCube=sf->dpuCfg.rangeCfg.hwRes.radarCube;
    dc.detMatrix=sf->dpuCfg.dopplerCfg.hwRes.detMatrix;
    dc.antDef=sf->staticCfg.antDef;
    dc.numRangeBins=256U; dc.numDopplerBins=16U; dc.numDopplerChirps=16U;
    dc.numTxAntennas=sf->staticCfg.numTxAntennas;
    dc.numRxAntennas=sf->staticCfg.ADCBufData.dataProperty.numRxAntennas;
    dc.isTdmMimoValid=1U;
    dc.subFrameNum=obj->subFrameIndx;
    DPC_ObjDet_GenDopplerWindow(&sf->dpuCfg.dopplerCfg);
    dc.dopplerWindow=sf->dpuCfg.dopplerCfg.hwRes.hwaCfg.window;
#ifdef DPC_USE_SYMMETRIC_WINDOW_DOPPLER_DPU
    dc.dopplerWindowIsSymmetric=1U;
#endif
    for (i=0U;i<sizeof(comp)/sizeof(comp[0]);i++) {
        comp[i].real=32767; comp[i].imag=0;
    }
    dc.rxChPhaseComp=comp;
    cube=dc.radarCube.data; matrix=dc.detMatrix.data;
    for (i=0U;i<dc.numTxAntennas;i++) for (j=0U;j<dc.numRxAntennas;j++) {
        uint32_t chirp;
        for (chirp=0U;chirp<16U;chirp++) {
            uint32_t base=256U*(j+dc.numRxAntennas*(chirp+16U*i));
            float wave=cosf(6.28318530718f*(float)chirp/16.0f);
            cube[base+10U].real=(int16_t)(2000.0f+700.0f*wave);
            cube[base+10U].imag=0;
            cube[base+20U].real=(int16_t)(700.0f+300.0f*wave);
            cube[base+20U].imag=0;
        }
    }
    for (i=0U;i<4096U;i++) matrix[i]=1000U;
    list[0].rangeIdx=10U; list[1].rangeIdx=10U; list[2].rangeIdx=20U;
    list[0].dopplerIdx=0U; list[1].dopplerIdx=1U; list[2].dopplerIdx=0U;
    for (i=0U;i<3U;i++) {
        list[i].snr=300;
        dc.cfarRngDopSnrList=&list[i]; dc.numCfarPeaks=1U;
        if (MmwDemo_DenseElevation_process(&cfg,&dc,gNavCandidateScratch,
            sizeof(gNavCandidateScratch),3072U,&bytes) || !header->numSamples ||
            header->numSamples>2U) { fail|=1UL; break; }
        memcpy(&gSurfaceDenseExpected[total],samples,header->numSamples*sizeof(*samples));
        total+=header->numSamples;
    }
    tmp=list[1]; list[1]=list[2]; list[2]=tmp; /* A0, B0, A1 */
    dc.cfarRngDopSnrList=list; dc.numCfarPeaks=3U;
    if (MmwDemo_DenseElevation_process(&cfg,&dc,gNavCandidateScratch,
        sizeof(gNavCandidateScratch),3072U,&bytes) || header->numSamples!=total ||
        (uint16_t)header->reservedPad!=3U || (header->reservedPad>>16U)!=2U ||
        memcmp(samples,gSurfaceDenseExpected,total*sizeof(*samples))) fail|=1UL;
    tmp=list[0]; list[0]=list[2]; list[2]=tmp;
    if (MmwDemo_DenseElevation_process(&cfg,&dc,gNavCandidateScratch,
        sizeof(gNavCandidateScratch),3072U,&bytes) || header->numSamples!=total ||
        memcmp(samples,gSurfaceDenseExpected,total*sizeof(*samples))) fail|=1UL;

    /* 96 occupied ranges, three Doppler cells each. Compare one-range-at-a-
     * time reference against a reversed/interleaved 288-candidate list.
     * Require >256 outputs so neither the old FFT nor cluster cutoff hides. */
    total=0U;
    for (i=0U;i<dc.numTxAntennas;i++) for (j=0U;j<dc.numRxAntennas;j++) {
        uint32_t chirp,r;
        for (chirp=0U;chirp<16U;chirp++) {
            uint32_t base=256U*(j+dc.numRxAntennas*(chirp+16U*i));
            float wave=cosf(6.28318530718f*(float)chirp/16.0f);
            for (r=3U;r<99U;r++) {
                cube[base+r].real=(int16_t)((1500.0f+3.0f*(float)r)+700.0f*wave);
                cube[base+r].imag=0;
            }
        }
    }
    for (i=3U;i<99U;i++) {
        for (j=0U;j<3U;j++) {
            list[j].rangeIdx=(uint16_t)i;
            list[j].dopplerIdx=(j<2U)?(uint16_t)j:15U;
            list[j].snr=300;
        }
        dc.cfarRngDopSnrList=list; dc.numCfarPeaks=3U;
        if (MmwDemo_DenseElevation_process(&cfg,&dc,gNavCandidateScratch,
            sizeof(gNavCandidateScratch),3072U,&bytes) || !header->numSamples ||
            header->numSamples>6U) { fail|=8UL; break; }
        memcpy(&gSurfaceDenseExpected[total],samples,header->numSamples*sizeof(*samples));
        total+=header->numSamples;
    }
    for (j=0U;j<3U;j++) for (i=0U;i<96U;i++) {
        DPIF_CFARDetList *entry=&sf->dpuCfg.cfarCfg.res.cfarRngDopSnrList[j*96U+i];
        memset((void *)entry,0,sizeof(*entry));
        entry->rangeIdx=(uint16_t)(98U-i);
        entry->dopplerIdx=(j<2U)?(uint16_t)j:15U;
        entry->snr=300;
    }
    dc.cfarRngDopSnrList=sf->dpuCfg.cfarCfg.res.cfarRngDopSnrList;
    dc.numCfarPeaks=288U;
    if (MmwDemo_DenseElevation_process(&cfg,&dc,gNavCandidateScratch,
        sizeof(gNavCandidateScratch),3072U,&bytes) || total<=256U ||
        header->numSamples!=total || (uint16_t)header->reservedPad!=288U ||
        (header->reservedPad>>16U)!=96U ||
        memcmp(samples,gSurfaceDenseExpected,total*sizeof(*samples))) fail|=8UL;
    for (i=0U;i<144U;i++) {
        DPIF_CFARDetList *entries=sf->dpuCfg.cfarCfg.res.cfarRngDopSnrList;
        tmp=entries[i]; entries[i]=entries[287U-i]; entries[287U-i]=tmp;
    }
    if (MmwDemo_DenseElevation_process(&cfg,&dc,gNavCandidateScratch,
        sizeof(gNavCandidateScratch),3072U,&bytes) || header->numSamples!=total ||
        memcmp(samples,gSurfaceDenseExpected,total*sizeof(*samples))) fail|=8UL;

    /* Compare HWA's complete intersection with scalar CASO for both sparse
     * zero/positive/negative-Doppler targets and intermediate-list pressure.
     * A mismatch under pressure is a qualification failure, not a DSP fix. */
    for (pattern=0U;pattern<2U;pattern++) {
        DPU_CFARCAProcHWA_OutParams result;
        uint32_t expected=0U,missing=0U;
        int32_t minR=(int32_t)(gSurfaceRawFov[obj->subFrameIndx].min/sf->staticCfg.rangeStep+0.5f);
        int32_t maxR=(int32_t)(gSurfaceRawFov[obj->subFrameIndx].max/sf->staticCfg.rangeStep+0.5f);
        DPU_CFARCAProc_FovCfg *dv=&sf->dynCfg.fovDoppler;
        int32_t minD=(int32_t)(dv->min/sf->staticCfg.dopplerStep+((dv->min<0.0f)?-0.5f:0.5f));
        int32_t maxD=(int32_t)(dv->max/sf->staticCfg.dopplerStep+((dv->max<0.0f)?-0.5f:0.5f));
        for (i=0U;i<4096U;i++) {
            seed=1664525UL*seed+1013904223UL;
            matrix[i]=pattern ? (((seed>>28U)<12U)?10000U:0U) : 1000U;
        }
        if (!pattern) {
            matrix[20U*16U]=10000U; matrix[40U*16U+2U]=10000U;
            matrix[60U*16U+15U]=10000U;
        }
        if (DPU_CFARCAProcHWA_config(sf->dpuCFARCAObj,&sf->dpuCfg.cfarCfg) ||
            DPU_CFARCAProcHWA_process(sf->dpuCFARCAObj,&result)) {
            fail|=2UL<<pattern; continue;
        }
        for (i=0U;i<256U;i++) for (j=0U;j<16U;j++) {
            int32_t sd=(j<8U) ? (int32_t)j : (int32_t)j-16;
            uint32_t k;
            if ((int32_t)i<minR || (int32_t)i>maxR || sd<minD || sd>maxD) continue;
            if (!DetectionFixture_caso(&matrix[j],256U,16U,(uint16_t)i,&sf->dynCfg.cfarCfgRange) ||
                !DetectionFixture_caso(&matrix[i*16U],16U,1U,(uint16_t)j,&sf->dynCfg.cfarCfgDoppler)) continue;
            expected++;
            for (k=0U;k<result.numCfarDetectedPoints;k++)
                if (sf->dpuCfg.cfarCfg.res.cfarRngDopSnrList[k].rangeIdx==i &&
                    sf->dpuCfg.cfarCfg.res.cfarRngDopSnrList[k].dopplerIdx==j) break;
            if (k==result.numCfarDetectedPoints) missing++;
        }
        gSurfaceCfarReferenceCount[pattern]=expected;
        gSurfaceCfarHardwareCount[pattern]=result.numCfarDetectedPoints;
        if (missing || expected!=result.numCfarDetectedPoints) fail|=2UL<<pattern;
    }
    MmwDemo_DenseElevation_reset();
    MmwDemo_SurfaceGen_reset();
    gSurfaceDetectionSelfTestFailures=fail;
    return fail;
}
