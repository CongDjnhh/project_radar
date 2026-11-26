/**
 *   @file  detsel_region_grow.h
 *
 *   @brief
 *      Pure C99 hardware-independent Azimuth-Guided Hysteresis Region Growing
 *      Stage-2 selector over DDMA Stage-1 candidates.
 */
#ifndef DETSEL_REGION_GROW_H
#define DETSEL_REGION_GROW_H

#include <stdint.h>
#include "detsel_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Return status codes */
#define DETSEL_OK                    (0)
#define DETSEL_ERR_NULL_PTR          (-1)
#define DETSEL_ERR_INVALID_PHYSICS   (-2)
#define DETSEL_ERR_INVALID_CFG       (-3)
#define DETSEL_ERR_TOO_MANY_CAND     (-4)
#define DETSEL_ERR_NON_MONOTONIC     (-5)
#define DETSEL_ERR_RANGE_OOB         (-6)

/**
 * @brief Compact Stage-1 candidate descriptor (12 bytes).
 */
typedef struct DetSel_Cand_t
{
    uint16_t rangeIdx;       /* [0 .. numRangeBins-1] */
    uint16_t dopIdxActual;   /* [0 .. numDopplerBins-1] (demodulated DDMA Doppler index) */
    uint8_t  azimIdx;        /* [0 .. numAzimFFTBins-1] (32-bin Azimuth FFT peak index) */
    uint8_t  initTag;        /* Initial DETSEL_TAG_* bitmask (RANGECFAR_HIT | BASELINE) */
    int16_t  marginQ11;      /* azimPeakSamples[1] - dopCfarNoise - lutOffset (Q11 log2) */
    uint16_t peakQ11;        /* azimPeakSamples[1] (Q11 log2) */
    uint16_t origObjIdx;     /* Original index in detObjList[0..dopNumObjOut-1] */
} DetSel_Cand;

_Static_assert(sizeof(DetSel_Cand) == 12U, "DetSel_Cand size mismatch");

/**
 * @brief Active waveform & antenna array physics used to derive discrete bin windows.
 */
typedef struct DetSel_Physics_t
{
    float    rangeStep;          /* [m/bin] */
    float    dopplerStep;        /* [m/s/bin] */
    uint16_t numRangeBins;       /* e.g. 192 */
    uint16_t numDopplerBins;     /* e.g. 96 */
    uint16_t numAzimFFTBins;     /* e.g. 32 */
    uint16_t numVirtualAntAzim;  /* e.g. 12 */
} DetSel_Physics;

/**
 * @brief Dynamically derived discrete bin windows & Q11 thresholds (no magic numbers).
 */
typedef struct DetSel_DerivedBins_t
{
    uint16_t rWin;               /* Range gate neighborhood radius [bins] */
    uint16_t dWin;               /* Circular Actual-Doppler radius [bins] */
    uint16_t aWin;               /* Circular Azimuth-FFT radius [bins] */
    int16_t  baseThrQ11;         /* TI Baseline Doppler CFAR threshold [Q11 log2] */
    int16_t  growThrQ11;         /* Hysteresis growth threshold [Q11 log2] */
    uint16_t sidelobeQ11;        /* Max peak drop from root seed at |dr| >= 2 [Q11 log2] */
    int16_t  zeroDopMidThrQ11;   /* Clutter guard threshold on dopIdxActual == 0 [Q11 log2] */
} DetSel_DerivedBins;

/**
 * @brief Pre-allocated DSS_L3 scratch memory (~40.5 KB; zero M4_RAM footprint).
 */
typedef struct DetSel_ScratchBuf_t
{
    DetSel_Cand cand[DETSEL_MAX_CANDIDATES];
    uint8_t     tag[DETSEL_MAX_CANDIDATES];
    uint16_t    bfsQueue[DETSEL_MAX_CANDIDATES];
    uint16_t    hop[DETSEL_MAX_CANDIDATES];
    uint16_t    rootPeak[DETSEL_MAX_CANDIDATES];
    uint16_t    rootRange[DETSEL_MAX_CANDIDATES];
    uint16_t    gateStart[DETSEL_MAX_RANGE_BINS + 1U];
    uint16_t    selIdx[DETSEL_FINAL_MAX_OBJS];
} DetSel_ScratchBuf;

/**
 * @brief Summary counts returned by DetSel execution.
 */
typedef struct DetSel_OutStats_t
{
    uint16_t numSeeds;
    uint16_t numGrown;
    uint16_t numSelected;
    uint8_t  truncated;
    uint8_t  reserved;
} DetSel_OutStats;

/**
 * @brief Convert dB threshold to HWA 2.1 Q11 log2 threshold matching
 *        MmwDemo_CLICfarCfg + MmwDemo_convertDopplerCfarToThresh bit-for-bit.
 */
int16_t DetSel_dbToQ11(float dbVal);

/**
 * @brief Validate DetSel_Cfg parameters for any mode (0, 1, 2).
 */
int32_t DetSel_validateCfg(const DetSel_Cfg *cfg);

/**
 * @brief Derive discrete bin windows and Q11 thresholds from physics and DetSel_Cfg.
 */
int32_t DetSel_deriveBins(
    const DetSel_Physics *phys,
    const DetSel_Cfg     *cfg,
    DetSel_DerivedBins   *outBins);

/**
 * @brief Execute Azimuth-Guided Hysteresis Region Growing on Stage-1 candidates
 *        sorted in non-decreasing rangeIdx order (scratch->cand[0..numCand-1]),
 *        populating scratch->tag[0..numCand-1].
 */
int32_t DetSel_regionGrow(
    const DetSel_Physics *phys,
    const DetSel_Cfg     *cfg,
    uint16_t              numCand,
    DetSel_ScratchBuf    *scratch,
    DetSel_OutStats      *outStats);

/**
 * @brief Deterministic priority-preserving index selector into selIdx[0..maxOut-1]
 *        (keeps 100% of primaryTag first, then secondaryTag ordered by marginQ11 DESC,
 *        and outputs ascending candidate indices for monotonically ordered EDMA copies).
 */
uint16_t DetSel_selectIndices(
    const DetSel_Cand *cand,
    const uint8_t     *tag,
    uint16_t           numCand,
    uint8_t            primaryMask,
    uint8_t            secondaryMask,
    uint16_t           maxOut,
    uint16_t          *bfsScratchIdx,
    uint16_t          *selIdx,
    uint8_t           *outTruncated);

#ifdef __cplusplus
}
#endif

#endif /* DETSEL_REGION_GROW_H */
