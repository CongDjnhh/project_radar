/**
 * @file region_grow_2d.h
 * @brief Hardware-agnostic 2D Anisotropic Range-Doppler Region Growing Core
 *        with Sinc Sidelobe Rejection, Zero-Doppler Clutter Guard,
 *        Topographic Valley Splitting, and Per-Range Doppler Ridge Condensation.
 */
#ifndef ALG_REGION_GROW_2D_H
#define ALG_REGION_GROW_2D_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ALG_RG2D_NUM_PARAMS                 (14U)
#define ALG_RG2D_MAX_BITMAP_DOPPLER_BINS    (32U)

/* Parameter indices in params[ALG_RG2D_NUM_PARAMS] */
#define ALG_RG2D_PARAM_ENTER_Q8             (0U)  /* Entry delta above local anchor (Q8 log2) */
#define ALG_RG2D_PARAM_EXIT_Q8              (1U)  /* Exit delta above frozen anchor (Q8 log2) */
#define ALG_RG2D_PARAM_PROM_Q8              (2U)  /* Topographic prominence drop to enter valley (Q8 log2) */
#define ALG_RG2D_PARAM_RISE_Q8              (3U)  /* Topographic rise from valley floor to split (Q8 log2) */
#define ALG_RG2D_PARAM_AUDIT_CASO_Q8        (4U)  /* Extended target recovery delta above range CFAR noise (Q8 log2) */
#define ALG_RG2D_PARAM_AUDIT_ANCHOR_Q8      (5U)  /* Extended target recovery delta above local anchor (Q8 log2) */
#define ALG_RG2D_PARAM_MAX_RANGE_SPAN       (6U)  /* Max forward range span in bins (1..64) */
#define ALG_RG2D_PARAM_MAX_BACKTRACK        (7U)  /* Max leftward range backtrack in bins (0..16) */
#define ALG_RG2D_PARAM_FAR_OUTER            (8U)  /* Far noise window outer offset in range bins (1..16) */
#define ALG_RG2D_PARAM_FAR_INNER            (9U)  /* Far noise window inner offset in range bins (1..16) */
#define ALG_RG2D_PARAM_NEAR_OUTER           (10U) /* Near noise window outer offset in range bins (1..16) */
#define ALG_RG2D_PARAM_NEAR_INNER           (11U) /* Near noise window inner offset in range bins (1..16) */
#define ALG_RG2D_PARAM_MAX_DOPPLER_SPAN     (12U) /* Max circular Doppler half-span from seed (0..8) */
#define ALG_RG2D_PARAM_SIDELOBE_GUARD_Q8    (13U) /* Max drop below cluster peak for |r - rPeak| >= 2 (Q8 log2) */

typedef struct Alg_RegionGrow2D_QueueNode_t
{
    uint16_t rangeIdx;
    uint16_t dopplerIdx;
} Alg_RegionGrow2D_QueueNode;

typedef char Alg_RegionGrow2D_QueueNodeSizeCheck[
    (sizeof(Alg_RegionGrow2D_QueueNode) == 4U) ? 1 : -1];

typedef struct Alg_RegionGrow2D_Workspace_t
{
    uint32_t *seedMask;
    uint32_t *grownMask;
    uint16_t *cellSnr;
    uint16_t maxDopplerStride;
    Alg_RegionGrow2D_QueueNode *bfsQueue;
    uint16_t queueCapacity;
} Alg_RegionGrow2D_Workspace;

uint16_t Alg_RegionGrow2D_noiseAnchor(
    const uint16_t *detMat,
    uint16_t numDopplerBins,
    uint16_t minRange,
    uint16_t maxRange,
    uint16_t rSeed,
    uint16_t dopIdx,
    const uint16_t params[ALG_RG2D_NUM_PARAMS]);

uint16_t Alg_RegionGrow2D_growFromSeed2D(
    const uint16_t *detMat,
    uint16_t numDopplerBins,
    uint16_t minRange,
    uint16_t maxRange,
    uint16_t rSeed,
    uint16_t dopIdx,
    uint8_t staticClutterRemovalEnabled,
    const uint16_t params[ALG_RG2D_NUM_PARAMS],
    Alg_RegionGrow2D_Workspace *ws);

void Alg_RegionGrow2D_condenseDopplerRidges(
    const uint16_t *detMat,
    uint16_t numDopplerBins,
    uint16_t minRange,
    uint16_t maxRange,
    const uint16_t params[ALG_RG2D_NUM_PARAMS],
    Alg_RegionGrow2D_Workspace *ws);

#ifdef __cplusplus
}
#endif

#endif /* ALG_REGION_GROW_2D_H */
