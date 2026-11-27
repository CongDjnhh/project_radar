/**
 *   @file  detsel_common.h
 *
 *   @brief
 *      Shared cross-core ABI definitions for Stage-2 Detection Selection
 *      (TI RangeCFAR Baseline, Candidate Dump, and Azimuth-Guided Hysteresis
 *      Region Growing) between MSS R5F and DSS Cortex-M4F on AWR2944P DDMA.
 */
#ifndef DETSEL_COMMON_H
#define DETSEL_COMMON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L) && !defined(__TINYC__)
#define DETSEL_STATIC_ASSERT(cond, name) _Static_assert((cond), #name)
#else
#define DETSEL_STATIC_ASSERT(cond, name) \
    typedef char detsel_static_assert_##name[(cond) ? 1 : -1]
#endif

/** @brief Stage-2 selection modes */
#define DETSEL_MODE_TI_BASELINE      (0U) /* Exact TI RangeCFAR intersection */
#define DETSEL_MODE_CAND_DUMP        (1U) /* Export tagged Stage-1 candidates for G1 gate & offline RG */
#define DETSEL_MODE_REGION_GROW      (2U) /* Azimuth-Guided Hysteresis Region Growing (SEED U GROWN) */

/** @brief Capacity bounds matching DDMA DPC limits */
#define DETSEL_MAX_CANDIDATES        (1800U)
#define DETSEL_MAX_RANGE_BINS        (1024U)
#define DETSEL_FINAL_MAX_OBJS        (793U)

/** @brief Per-object provenance bitmask packed into DetObjParams.dopIdx[31:16] */
#define DETSEL_TAG_RANGECFAR_HIT     (1U << 0) /* Present in HWA Stage-2 rangeCfarList */
#define DETSEL_TAG_BASELINE          (1U << 1) /* (peak - noise >= T_base_Q11) && RANGECFAR_HIT */
#define DETSEL_TAG_SEED              (1U << 2) /* Seed initiating Region Growing */
#define DETSEL_TAG_GROWN             (1U << 3) /* Candidate recovered by Region Growing */
#define DETSEL_TAG_STAGE1_ONLY       (1U << 4) /* Unselected Stage-1 candidate (exported in Mode 1) */
#define DETSEL_TAG_SHIFT             (16U)
#define DETSEL_DOPIDX_MASK           (0x0000FFFFUL)

/** @brief Custom DPM IOCTL command ID (SDK uses DPM_CMD_DPC_START_INDEX + 0..16) */
#ifdef DPM_CMD_DPC_START_INDEX
#define DPC_OBJDET_IOCTL__CUSTOM_DETSEL_CFG   ((uint32_t)(DPM_CMD_DPC_START_INDEX + 20U))
#else
#define DPC_OBJDET_IOCTL__CUSTOM_DETSEL_CFG   ((uint32_t)(100U + 20U))
#endif

/**
 * @brief Physical Stage-2 selection configuration sent from MSS CLI to CM4 (32 bytes).
 *        All grid windows are derived dynamically from active waveform & antenna physics.
 */
typedef struct DetSel_Cfg_t
{
    uint8_t  subFrameIdx;
    uint8_t  mode;              /* DETSEL_MODE_* */
    uint8_t  reserved0[2];
    float    baselineThrDb;     /* [dB] Reference Doppler CFAR threshold of TI Baseline (e.g. 24.0) */
    float    growThrDb;         /* [dB] Hysteresis expansion threshold T_grow (e.g. 14.0) */
    float    sidelobeGuardDb;   /* [dB] Max amplitude drop below root seed peak for |dr| >= 2 bins */
    float    growRangeM;        /* [m] Physical range neighborhood radius -> ceil(growRangeM / rangeStep) */
    float    growDopplerMps;    /* [m/s] Physical velocity radius -> ceil(growDopplerMps / dopplerStep) */
    float    growAzimBeams;     /* [Rayleigh beams] Azimuth radius -> ceil(k * N_azFFT / N_vAzim) */
    uint16_t maxGrowHops;       /* Max BFS propagation hops from any seed */
    uint16_t reserved1;
} DetSel_Cfg;

DETSEL_STATIC_ASSERT(sizeof(DetSel_Cfg) == 32U, DetSel_Cfg_ABI_size_mismatch);

/**
 * @brief Pack CM4 per-frame Stage-2 telemetry into stats.interChirpProcessingMargin (32-bit):
 *        [10:0]  stage1Cnt  (0..2047)
 *        [18:11] capGates   (0..255)
 *        [30:19] selUs      (0..4095 us)
 *        [31]    truncFlag  (1 if selected objects > finalMaxNumDetObjs)
 */
static inline uint32_t DetSel_packCm4Stats(
    uint32_t stage1Cnt,
    uint32_t capGates,
    uint32_t selUs,
    uint32_t truncFlag)
{
    uint32_t s = (stage1Cnt > 0x7FFUL)  ? 0x7FFUL  : stage1Cnt;
    uint32_t c = (capGates  > 0xFFUL)   ? 0xFFUL   : capGates;
    uint32_t u = (selUs     > 0xFFFUL)  ? 0xFFFUL  : selUs;
    uint32_t t = (truncFlag != 0UL)     ? 1UL      : 0UL;
    return s | (c << 11U) | (u << 19U) | (t << 31U);
}

#define DETSEL_PACK_CM4_STATS(stage1Cnt, capGates, selUs) \
    DetSel_packCm4Stats((uint32_t)(stage1Cnt), (uint32_t)(capGates), (uint32_t)(selUs), 0UL)

#define DETSEL_UNPACK_STAGE1_CNT(v)  ((uint16_t)((uint32_t)(v) & 0x7FFUL))
#define DETSEL_UNPACK_CAP_GATES(v)   ((uint16_t)(((uint32_t)(v) >> 11U) & 0xFFUL))
#define DETSEL_UNPACK_SEL_US(v)      ((uint16_t)(((uint32_t)(v) >> 19U) & 0xFFFUL))
#define DETSEL_UNPACK_TRUNC_FLAG(v)  ((uint8_t)(((uint32_t)(v) >> 31U) & 0x1UL))

#ifdef __cplusplus
}
#endif

#endif /* DETSEL_COMMON_H */
