/**
 * @file dense_elevation.c
 *
 * @brief Frame-synchronous navigation source extraction with continuous
 *        elevation filtering, seeded by the TI CFAR range-Doppler detection
 *        list supplied in MmwDemo_DenseElevation_StaticCfg.
 */

#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/mathlib/src/atan2sp/c674/atan2sp.h>

#include "dense_elevation.h"

#define DENSE_PI                         (3.14159265358979323846f)
#define DENSE_Q15_SCALE                  (32768.0f)
/* Must equal DPC_OBJDET_QFORMAT_DOPPLER_FFT (17); name kept for history. */
#define DENSE_Q19_SHIFT                  (17U)
#define DENSE_MIN_POWER                  (1.0f)
#define DENSE_MAX_AZIMUTH_PEAKS          (2U)
#define DENSE_AZIMUTH_GUARD_BINS         (2U)
#define DENSE_AZIMUTH_SECTORS            (8U)
/* Reject elevation phase when the normalized secondary-row power is more
 * than 16 dB below the reference row. This is a phase-quality check, not an
 * elevation-angle or occupancy threshold. */
#define DENSE_MIN_ELEVATION_ROW_RATIO    (0.025118864f)
/* Maximum number of occupied range bins permitted to regenerate Doppler FFT per frame.
 * Hard-limits C674x DSP cycle consumption to guarantee zero frame-overruns under heavy clutter. */
#define MMWDEMO_DENSE_ELEV_MAX_DOPPLER_FFT_RANGES (16U)

/* DSP_fft16x16 is provided by dsplib.ae64P, already linked by the demo. */
extern void DSP_fft16x16(const short * restrict ptrW,
                         int npoints,
                         short * restrict ptrX,
                         short * restrict ptrY);

typedef struct MmwDemo_DenseComplexFloat_t
{
    float real;
    float imag;
} MmwDemo_DenseComplexFloat;

typedef struct MmwDemo_DenseDopplerCandidate_t
{
    uint16_t index;
    uint16_t snrDb10;
} MmwDemo_DenseDopplerCandidate;

/* Geometry-dependent azimuth steering. Elevation is estimated from the
 * inter-row phase (TI AoAProc convention), not from zero-padding two rows. */
#pragma DATA_SECTION(gDenseAzSteering, ".l3ram");
#pragma DATA_ALIGN(gDenseAzSteering, 8);
static far MmwDemo_DenseComplexFloat
    gDenseAzSteering[MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS]
                       [MMWDEMO_DENSE_ELEV_MAX_VIRTUAL_ANTENNAS];

#pragma DATA_SECTION(gDenseDopplerFft, ".l3ram");
#pragma DATA_ALIGN(gDenseDopplerFft, 8);
static far int16_t
    gDenseDopplerFft[MMWDEMO_DENSE_ELEV_MAX_VIRTUAL_ANTENNAS]
                    [2U * MMWDEMO_DENSE_ELEV_MAX_DOPPLER_BINS];

#pragma DATA_SECTION(gDenseFftInput, ".l1dscratch");
#pragma DATA_ALIGN(gDenseFftInput, 8);
/* P1.3: L1D SRAM (0 wait-state). Written/cleared for every range bin Ă— TX Ă— RX
 * iteration.  At 128 Doppler bins Ă— 12 antennas Ă— ~100 occupied range bins,
 * this 512-byte buffer is touched 1,200+ times per frame. */
static int16_t gDenseFftInput[2U * MMWDEMO_DENSE_ELEV_MAX_DOPPLER_BINS];

#pragma DATA_SECTION(gDenseFftTwiddle, ".l1dscratch");
#pragma DATA_ALIGN(gDenseFftTwiddle, 8);
/* P1.3: L1D SRAM (0 wait-state). Read-only after geometry reconfiguration;
 * passed as the W-table argument to DSP_fft16x16() for every antenna Ă—
 * range bin call.  Constant per Doppler bin count so cache-warming would
 * help, but L1D placement guarantees zero-wait-state access. */
static int16_t gDenseFftTwiddle[2U * MMWDEMO_DENSE_ELEV_MAX_DOPPLER_BINS];


#pragma DATA_SECTION(gDenseAzPower, ".l3ram");
#pragma DATA_ALIGN(gDenseAzPower, 8);
static far float gDenseAzPower[MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS];

#pragma DATA_SECTION(gDenseAzSourceVQ15, ".l3ram");
#pragma DATA_ALIGN(gDenseAzSourceVQ15, 8);
static far int16_t gDenseAzSourceVQ15[MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS];

#pragma DATA_SECTION(gDenseAzValid, ".l3ram");
#pragma DATA_ALIGN(gDenseAzValid, 8);
static far uint8_t gDenseAzValid[MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS];

#pragma DATA_SECTION(gDenseAzElevValid, ".l3ram");
#pragma DATA_ALIGN(gDenseAzElevValid, 8);
static far uint8_t gDenseAzElevValid[MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS];

#pragma DATA_SECTION(gDenseOutputPriority, ".l3ram");
#pragma DATA_ALIGN(gDenseOutputPriority, 8);
static far uint32_t
    gDenseOutputPriority[MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES];

#pragma DATA_SECTION(gDenseRangeFftDone, ".l3ram");
static far uint8_t gDenseRangeFftDone[1024];

#pragma DATA_SECTION(gDenseTdmComp, ".l3ram");
#pragma DATA_ALIGN(gDenseTdmComp, 8);
static far MmwDemo_DenseComplexFloat
    gDenseTdmComp[MMWDEMO_DENSE_ELEV_MAX_DOPPLER_BINS]
                 [SYS_COMMON_NUM_TX_ANTENNAS];

static int16_t gDenseAntRow[MMWDEMO_DENSE_ELEV_MAX_VIRTUAL_ANTENNAS];
static int16_t gDenseRowZ[MMWDEMO_DENSE_ELEV_MAX_ROWS];
static uint8_t gDenseRowAntCount[MMWDEMO_DENSE_ELEV_MAX_ROWS];
static uint8_t gDenseNumRows;
static uint8_t gDenseReferenceRow;
static uint8_t gDenseElevationRow;
static int16_t gDenseVerticalBaseline;
static uint32_t gDenseGeometrySignature;
static uint8_t gDenseGeometryValid;
static uint16_t gDenseTwiddleSize;

static int16_t MmwDemo_DenseElevation_sat16(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16_t)value;
}

static int16_t MmwDemo_DenseElevation_floatToQ15(float value)
{
    int32_t rounded;

    value *= 32767.5f;
    rounded = (value >= 0.0f) ? (int32_t)(value + 0.5f)
                              : (int32_t)(value - 0.5f);
    if (rounded < -32767)
    {
        rounded = -32767;
    }
    return MmwDemo_DenseElevation_sat16(rounded);
}

static int16_t MmwDemo_DenseElevation_sourceVToQ15(float value)
{
    int32_t rounded;

    if (value >= 1.0f)
    {
        return 32767;
    }
    if (value <= -1.0f)
    {
        return -32767;
    }
    rounded = (value >= 0.0f)
                ? (int32_t)(value * 32767.0f + 0.5f)
                : (int32_t)(value * 32767.0f - 0.5f);
    return MmwDemo_DenseElevation_sat16(rounded);
}

/* Generate the special DSPLIB FFT16x16 twiddle ordering. */
static void MmwDemo_DenseElevation_genTwiddle(uint16_t n)
{
    uint16_t i;
    uint16_t j;
    uint16_t k = 0U;

    memset(gDenseFftTwiddle, 0, sizeof(gDenseFftTwiddle));
    for (j = 1U; j < (n >> 2U); j <<= 2U)
    {
        for (i = 0U; i < (n >> 2U); i += (j << 1U))
        {
            float phase0 = 2.0f * DENSE_PI * (float)i / (float)n;
            float phase1 = 2.0f * DENSE_PI * (float)(i + j) / (float)n;

            gDenseFftTwiddle[k + 11U] = MmwDemo_DenseElevation_floatToQ15(cosf(3.0f * phase1));
            gDenseFftTwiddle[k + 10U] = MmwDemo_DenseElevation_floatToQ15(sinf(3.0f * phase1));
            gDenseFftTwiddle[k + 9U]  = MmwDemo_DenseElevation_floatToQ15(cosf(3.0f * phase0));
            gDenseFftTwiddle[k + 8U]  = MmwDemo_DenseElevation_floatToQ15(sinf(3.0f * phase0));
            gDenseFftTwiddle[k + 7U]  = (int16_t)-MmwDemo_DenseElevation_floatToQ15(cosf(2.0f * phase1));
            gDenseFftTwiddle[k + 6U]  = (int16_t)-MmwDemo_DenseElevation_floatToQ15(sinf(2.0f * phase1));
            gDenseFftTwiddle[k + 5U]  = (int16_t)-MmwDemo_DenseElevation_floatToQ15(cosf(2.0f * phase0));
            gDenseFftTwiddle[k + 4U]  = (int16_t)-MmwDemo_DenseElevation_floatToQ15(sinf(2.0f * phase0));
            gDenseFftTwiddle[k + 3U]  = MmwDemo_DenseElevation_floatToQ15(cosf(phase1));
            gDenseFftTwiddle[k + 2U]  = MmwDemo_DenseElevation_floatToQ15(sinf(phase1));
            gDenseFftTwiddle[k + 1U]  = MmwDemo_DenseElevation_floatToQ15(cosf(phase0));
            gDenseFftTwiddle[k + 0U]  = MmwDemo_DenseElevation_floatToQ15(sinf(phase0));
            k += 12U;
        }
    }
    gDenseTwiddleSize = n;
}

static uint32_t MmwDemo_DenseElevation_hashStaticCfg(
    const MmwDemo_DenseElevation_StaticCfg *cfg)
{
    uint32_t hash = 2166136261U;
    uint32_t tx;
    uint32_t rx;

#define DENSE_HASH_VALUE(v) do { hash ^= (uint32_t)(v); hash *= 16777619U; } while (0)
    DENSE_HASH_VALUE(cfg->numTxAntennas);
    DENSE_HASH_VALUE(cfg->numRxAntennas);
    DENSE_HASH_VALUE(cfg->numRangeBins);
    DENSE_HASH_VALUE(cfg->numDopplerChirps);
    DENSE_HASH_VALUE(cfg->numDopplerBins);
    for (tx = 0U; tx < cfg->numTxAntennas; tx++)
    {
        DENSE_HASH_VALUE((uint16_t)cfg->antDef.txAnt[tx].azimuthOffset);
        DENSE_HASH_VALUE((uint16_t)cfg->antDef.txAnt[tx].elevationOffset);
    }
    for (rx = 0U; rx < cfg->numRxAntennas; rx++)
    {
        DENSE_HASH_VALUE((uint16_t)cfg->antDef.rxAnt[rx].azimuthOffset);
        DENSE_HASH_VALUE((uint16_t)cfg->antDef.rxAnt[rx].elevationOffset);
    }
#undef DENSE_HASH_VALUE
    return hash;
}

static int32_t MmwDemo_DenseElevation_configure(
    const MmwDemo_DenseElevation_StaticCfg *cfg)
{
    uint32_t signature;
    uint32_t numVirtualAntennas;
    uint32_t tx;
    uint32_t rx;
    uint32_t ant;
    uint32_t row;
    uint32_t a;

    if (cfg == NULL)
    {
        return MMWDEMO_DENSE_EINVAL;
    }
    if (cfg->radarCube.data == NULL)
    {
        return MMWDEMO_DENSE_ERADARCUBE_NULL;
    }
    if (cfg->radarCube.datafmt != DPIF_RADARCUBE_FORMAT_1)
    {
        return MMWDEMO_DENSE_ERADARCUBE_FORMAT;
    }
    if (cfg->detMatrix.data == NULL)
    {
        return MMWDEMO_DENSE_EDETMATRIX_NULL;
    }
    if (cfg->detMatrix.datafmt != DPIF_DETMATRIX_FORMAT_1)
    {
        return MMWDEMO_DENSE_EDETMATRIX_FORMAT;
    }
    if (cfg->isTdmMimoValid == 0U)
    {
        return MMWDEMO_DENSE_EUNSUPPORTED_MIMO;
    }
    if ((cfg->numTxAntennas == 0U) ||
        (cfg->numTxAntennas > SYS_COMMON_NUM_TX_ANTENNAS) ||
        (cfg->numRxAntennas == 0U) ||
        (cfg->numRxAntennas > SYS_COMMON_NUM_RX_CHANNEL))
    {
        return MMWDEMO_DENSE_EANTENNA_CFG;
    }
    if (cfg->numRangeBins == 0U)
    {
        return MMWDEMO_DENSE_ERANGE_CFG;
    }
    if ((cfg->numDopplerChirps == 0U) ||
        (cfg->numDopplerBins < 8U) ||
        (cfg->numDopplerChirps > cfg->numDopplerBins) ||
        (cfg->numDopplerBins > MMWDEMO_DENSE_ELEV_MAX_DOPPLER_BINS) ||
        ((cfg->numDopplerBins & (cfg->numDopplerBins - 1U)) != 0U))
    {
        return MMWDEMO_DENSE_EDOPPLER_CFG;
    }

    numVirtualAntennas = cfg->numTxAntennas * cfg->numRxAntennas;
    if (numVirtualAntennas > MMWDEMO_DENSE_ELEV_MAX_VIRTUAL_ANTENNAS)
    {
        return MMWDEMO_DENSE_EANTENNA_CFG;
    }
    if (((uint32_t)cfg->numRangeBins * cfg->numDopplerChirps *
         numVirtualAntennas * sizeof(cmplx16ImRe_t)) > cfg->radarCube.dataSize)
    {
        return MMWDEMO_DENSE_ERADARCUBE_SIZE;
    }
    if (((uint32_t)cfg->numRangeBins * cfg->numDopplerBins *
         sizeof(uint16_t)) > cfg->detMatrix.dataSize)
    {
        return MMWDEMO_DENSE_EDETMATRIX_SIZE;
    }

    signature = MmwDemo_DenseElevation_hashStaticCfg(cfg);
    if ((gDenseGeometryValid != 0U) &&
        (signature == gDenseGeometrySignature))
    {
        return 0;
    }

    /* A failed rebuild must never leave a cacheable, partially updated
     * geometry. The next call will rebuild all derived state. */
    gDenseGeometryValid = 0U;
    gDenseNumRows = 0U;
    memset(gDenseRowAntCount, 0, sizeof(gDenseRowAntCount));
    ant = 0U;
    for (tx = 0U; tx < cfg->numTxAntennas; tx++)
    {
        for (rx = 0U; rx < cfg->numRxAntennas; rx++, ant++)
        {
            int16_t z = cfg->antDef.txAnt[tx].elevationOffset +
                        cfg->antDef.rxAnt[rx].elevationOffset;
            int16_t x = cfg->antDef.txAnt[tx].azimuthOffset +
                        cfg->antDef.rxAnt[rx].azimuthOffset;

            for (row = 0U; row < gDenseNumRows; row++)
            {
                if (gDenseRowZ[row] == z)
                {
                    break;
                }
            }
            if (row == gDenseNumRows)
            {
                if (gDenseNumRows >= MMWDEMO_DENSE_ELEV_MAX_ROWS)
                {
                    return MMWDEMO_DENSE_EGEOMETRY_ROWS;
                }
                gDenseRowZ[gDenseNumRows++] = z;
            }
            gDenseAntRow[ant] = (int16_t)row;
            gDenseRowAntCount[row]++;

            for (a = 0U; a < MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS; a++)
            {
                int32_t signedA =
                    (a < (MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS / 2U))
                    ? (int32_t)a
                    : (int32_t)a - (int32_t)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
                float u = 2.0f * (float)signedA /
                          (float)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
                float phase = DENSE_PI * (float)x * u;
                gDenseAzSteering[a][ant].real = cosf(phase);
                gDenseAzSteering[a][ant].imag = sinf(phase);
            }
        }
    }

    if (gDenseNumRows < 2U)
    {
        return MMWDEMO_DENSE_EGEOMETRY_ROWS;
    }

    {
        uint32_t rowA;
        uint32_t rowB;
        uint32_t bestPairScore = 0U;

        /* Use the strongest lambda/2-separated row pair. This resolves the
         * IWR1843BOOST 8+4 geometry without hard-coded TX or chirp order. */
        for (rowA = 0U; rowA < gDenseNumRows; rowA++)
        {
            for (rowB = rowA + 1U; rowB < gDenseNumRows; rowB++)
            {
                int32_t dz = (int32_t)gDenseRowZ[rowA] -
                             (int32_t)gDenseRowZ[rowB];
                uint32_t pairScore;

                if ((dz != 1) && (dz != -1))
                {
                    continue;
                }
                pairScore = (uint32_t)gDenseRowAntCount[rowA] *
                            (uint32_t)gDenseRowAntCount[rowB];
                if (pairScore > bestPairScore)
                {
                    bestPairScore = pairScore;
                    if (gDenseRowAntCount[rowA] >= gDenseRowAntCount[rowB])
                    {
                        gDenseReferenceRow = (uint8_t)rowA;
                        gDenseElevationRow = (uint8_t)rowB;
                    }
                    else
                    {
                        gDenseReferenceRow = (uint8_t)rowB;
                        gDenseElevationRow = (uint8_t)rowA;
                    }
                }
            }
        }
        if (bestPairScore == 0U)
        {
            return MMWDEMO_DENSE_EGEOMETRY_ROWS;
        }
        gDenseVerticalBaseline =
            gDenseRowZ[gDenseReferenceRow] - gDenseRowZ[gDenseElevationRow];
    }

    for (a = 0U; a < cfg->numDopplerBins; a++)
    {
        int32_t signedD = (a < (cfg->numDopplerBins / 2U))
                            ? (int32_t)a
                            : (int32_t)a - (int32_t)cfg->numDopplerBins;
        for (tx = 0U; tx < cfg->numTxAntennas; tx++)
        {
            float phase = -2.0f * DENSE_PI * (float)(signedD * (int32_t)tx) /
                          (float)(cfg->numDopplerBins * cfg->numTxAntennas);
            gDenseTdmComp[a][tx].real = cosf(phase);
            gDenseTdmComp[a][tx].imag = sinf(phase);
        }
    }

    if ((cfg->numDopplerBins >= 16U) &&
        (cfg->numDopplerBins != gDenseTwiddleSize))
    {
        MmwDemo_DenseElevation_genTwiddle(cfg->numDopplerBins);
    }

    gDenseGeometrySignature = signature;
    gDenseGeometryValid = 1U;
    return 0;
}

static int32_t MmwDemo_DenseElevation_windowSample(
    int16_t sample,
    uint16_t chirp,
    const MmwDemo_DenseElevation_StaticCfg *cfg)
{
    uint16_t windowIndex = chirp;
    int32_t windowValue;
    int64_t product;

    if (cfg->dopplerWindow == NULL)
    {
        return sample;
    }
    if ((cfg->dopplerWindowIsSymmetric != 0U) &&
        (windowIndex >= ((cfg->numDopplerChirps + 1U) / 2U)))
    {
        windowIndex = cfg->numDopplerChirps - 1U - windowIndex;
    }
    windowValue = cfg->dopplerWindow[windowIndex];
    product = (int64_t)sample * (int64_t)windowValue;
    if (product >= 0)
    {
        return (int32_t)((product +
            (1LL << (DENSE_Q19_SHIFT - 1U))) >> DENSE_Q19_SHIFT);
    }
    return (int32_t)(-(((-product) +
        (1LL << (DENSE_Q19_SHIFT - 1U))) >> DENSE_Q19_SHIFT));
}

static void MmwDemo_DenseElevation_dopplerFftForRange(
    uint16_t rangeIdx,
    const MmwDemo_DenseElevation_StaticCfg *cfg)
{
    const cmplx16ImRe_t *cube = (const cmplx16ImRe_t *)cfg->radarCube.data;
    uint32_t tx;
    uint32_t rx;
    uint32_t chirp;
    uint32_t doppler;
    uint32_t virtualAnt = 0U;

    for (tx = 0U; tx < cfg->numTxAntennas; tx++)
    {
        for (rx = 0U; rx < cfg->numRxAntennas; rx++, virtualAnt++)
        {
            memset(gDenseFftInput, 0,
                   2U * cfg->numDopplerBins * sizeof(gDenseFftInput[0]));
            for (chirp = 0U; chirp < cfg->numDopplerChirps; chirp++)
            {
                uint32_t offset = rangeIdx + cfg->numRangeBins *
                    (rx + cfg->numRxAntennas *
                    (chirp + cfg->numDopplerChirps * tx));
                int32_t realValue = MmwDemo_DenseElevation_windowSample(
                    cube[offset].real, (uint16_t)chirp, cfg);
                int32_t imagValue = MmwDemo_DenseElevation_windowSample(
                    cube[offset].imag, (uint16_t)chirp, cfg);
                gDenseFftInput[2U * chirp] =
                    MmwDemo_DenseElevation_sat16(realValue);
                gDenseFftInput[2U * chirp + 1U] =
                    MmwDemo_DenseElevation_sat16(imagValue);
            }

            if (cfg->numDopplerBins >= 16U)
            {
                DSP_fft16x16(gDenseFftTwiddle,
                             (int)cfg->numDopplerBins,
                             gDenseFftInput,
                             gDenseDopplerFft[virtualAnt]);
            }
            else
            {
                for (doppler = 0U; doppler < cfg->numDopplerBins; doppler++)
                {
                    float sumReal = 0.0f;
                    float sumImag = 0.0f;
                    for (chirp = 0U; chirp < cfg->numDopplerChirps; chirp++)
                    {
                        float phase = -2.0f * DENSE_PI *
                                      (float)(doppler * chirp) /
                                      (float)cfg->numDopplerBins;
                        float wr = cosf(phase);
                        float wi = sinf(phase);
                        float xr = (float)gDenseFftInput[2U * chirp];
                        float xi = (float)gDenseFftInput[2U * chirp + 1U];
                        sumReal += xr * wr - xi * wi;
                        sumImag += xr * wi + xi * wr;
                    }
                    sumReal /= (float)cfg->numDopplerChirps;
                    sumImag /= (float)cfg->numDopplerChirps;
                    gDenseDopplerFft[virtualAnt][2U * doppler] =
                        MmwDemo_DenseElevation_sat16((int32_t)sumReal);
                    gDenseDopplerFft[virtualAnt][2U * doppler + 1U] =
                        MmwDemo_DenseElevation_sat16((int32_t)sumImag);
                }
            }
        }
    }
}

static void MmwDemo_DenseElevation_prepareSignal(
    uint16_t doppler,
    const MmwDemo_DenseElevation_StaticCfg *cfg,
    MmwDemo_DenseComplexFloat *signal)
{
    uint32_t numVirtualAntennas = cfg->numTxAntennas * cfg->numRxAntennas;
    uint32_t ant;

    for (ant = 0U; ant < numVirtualAntennas; ant++)
    {
        uint32_t tx = ant / cfg->numRxAntennas;
        float xr = (float)gDenseDopplerFft[ant][2U * doppler];
        float xi = (float)gDenseDopplerFft[ant][2U * doppler + 1U];
        float cr = 1.0f;
        float ci = 0.0f;
        float correctedReal;
        float correctedImag;
        float tr = gDenseTdmComp[doppler][tx].real;
        float ti = gDenseTdmComp[doppler][tx].imag;

        if (cfg->rxChPhaseComp != NULL)
        {
            cr = (float)cfg->rxChPhaseComp[ant].real / DENSE_Q15_SCALE;
            ci = (float)cfg->rxChPhaseComp[ant].imag / DENSE_Q15_SCALE;
        }
        correctedReal = xr * cr - xi * ci;
        correctedImag = xr * ci + xi * cr;
        signal[ant].real = correctedReal * tr - correctedImag * ti;
        signal[ant].imag = correctedReal * ti + correctedImag * tr;
    }
}


static uint16_t MmwDemo_DenseElevation_circularDistance(
    uint16_t a,
    uint16_t b,
    uint16_t size)
{
    uint16_t distance = (a > b) ? (a - b) : (b - a);
    uint16_t wrapped = size - distance;
    return (distance < wrapped) ? distance : wrapped;
}

static uint8_t MmwDemo_DenseElevation_sector(int8_t signedAzimuth)
{
    int32_t shifted = (int32_t)signedAzimuth +
                      (int32_t)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS / 2;
    int32_t sector = shifted * (int32_t)DENSE_AZIMUTH_SECTORS /
                     (int32_t)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;

    if (sector < 0)
    {
        sector = 0;
    }
    if (sector >= (int32_t)DENSE_AZIMUTH_SECTORS)
    {
        sector = (int32_t)DENSE_AZIMUTH_SECTORS - 1;
    }
    return (uint8_t)sector;
}

/* Bounded output preserves azimuth-sector coverage. Once full, a sample can
 * only replace a weaker sample; all ranges are still evaluated in this frame. */
static void MmwDemo_DenseElevation_insertOutput(
    const MmwDemo_DenseElevation_OutputSample *sample,
    MmwDemo_DenseElevation_OutputSample *output,
    uint16_t outputCapacity,
    uint16_t *outputCount,
    uint16_t *sectorCount,
    uint8_t *outputLimited)
{
    uint32_t priority = ((uint32_t)sample->snrDb10 << 16U) |
                        (uint32_t)(0xFFFFU - sample->rangeIdx);
    uint8_t newSector = MmwDemo_DenseElevation_sector(sample->azimuthIdx);

    if (*outputCount < outputCapacity)
    {
        output[*outputCount] = *sample;
        gDenseOutputPriority[*outputCount] = priority;
        sectorCount[newSector]++;
        (*outputCount)++;
        return;
    }

    {
        uint16_t quota = outputCapacity / DENSE_AZIMUTH_SECTORS;
        uint16_t weakest = 0U;
        uint32_t weakestPriority = 0xFFFFFFFFU;
        uint8_t found = 0U;
        uint16_t i;

        if (quota == 0U)
        {
            quota = 1U;
        }

        /* First protect an azimuth sector that is below its fair share. */
        if (sectorCount[newSector] < quota)
        {
            for (i = 0U; i < outputCapacity; i++)
            {
                uint8_t oldSector = MmwDemo_DenseElevation_sector(
                    output[i].azimuthIdx);
                if ((sectorCount[oldSector] > quota) &&
                    (gDenseOutputPriority[i] < weakestPriority))
                {
                    weakest = i;
                    weakestPriority = gDenseOutputPriority[i];
                    found = 1U;
                }
            }
        }

        /* Otherwise compare within the same sector. This prevents one strong
         * wall sector from deleting every weaker navigation direction. */
        if (found == 0U)
        {
            for (i = 0U; i < outputCapacity; i++)
            {
                if ((MmwDemo_DenseElevation_sector(output[i].azimuthIdx) ==
                     newSector) &&
                    (gDenseOutputPriority[i] < weakestPriority))
                {
                    weakest = i;
                    weakestPriority = gDenseOutputPriority[i];
                    found = 1U;
                }
            }
        }

        /* With capacity below the number of sectors, a new sector may not
         * have a same-sector entry. Fall back to the global weakest entry. */
        if (found == 0U)
        {
            for (i = 0U; i < outputCapacity; i++)
            {
                if (gDenseOutputPriority[i] < weakestPriority)
                {
                    weakest = i;
                    weakestPriority = gDenseOutputPriority[i];
                    found = 1U;
                }
            }
        }

        *outputLimited = 1U;
        if ((found != 0U) && (priority > weakestPriority))
        {
            uint8_t oldSector = MmwDemo_DenseElevation_sector(
                output[weakest].azimuthIdx);
            sectorCount[oldSector]--;
            sectorCount[newSector]++;
            output[weakest] = *sample;
            gDenseOutputPriority[weakest] = priority;
        }
    }
}

static void MmwDemo_DenseElevation_processDoppler(
    uint16_t rangeIdx,
    const MmwDemo_DenseDopplerCandidate *candidate,
    const MmwDemo_DenseElevation_Cfg *cfg,
    const MmwDemo_DenseElevation_StaticCfg *staticCfg,
    MmwDemo_DenseElevation_OutputSample *output,
    uint16_t outputCapacity,
    uint16_t *outputCount,
    uint16_t *sectorCount,
    uint8_t *outputLimited)
{
    MmwDemo_DenseComplexFloat signal[MMWDEMO_DENSE_ELEV_MAX_VIRTUAL_ANTENNAS];
    MmwDemo_DenseComplexFloat rowSum[MMWDEMO_DENSE_ELEV_MAX_ROWS];
    uint32_t numVirtualAntennas =
        staticCfg->numTxAntennas * staticCfg->numRxAntennas;
    float minAzSin = sinf(cfg->minAzimuthDeg * DENSE_PI / 180.0f);
    float maxAzSin = sinf(cfg->maxAzimuthDeg * DENSE_PI / 180.0f);
    float minElevSin = sinf(cfg->minElevationDeg * DENSE_PI / 180.0f);
    float maxElevSin = sinf(cfg->maxElevationDeg * DENSE_PI / 180.0f);
    float referenceAntNorm =
        (gDenseRowAntCount[gDenseReferenceRow] > 0U) ?
        (1.0f / ((float)gDenseRowAntCount[gDenseReferenceRow] *
                 (float)gDenseRowAntCount[gDenseReferenceRow])) : 1.0f;
    float elevationAntNorm =
        (gDenseRowAntCount[gDenseElevationRow] > 0U) ?
        (1.0f / ((float)gDenseRowAntCount[gDenseElevationRow] *
                 (float)gDenseRowAntCount[gDenseElevationRow])) : 1.0f;
    uint32_t ant;
    uint32_t a;
    uint32_t row;
    uint16_t peakIndex[DENSE_MAX_AZIMUTH_PEAKS];
    float peakPower[DENSE_MAX_AZIMUTH_PEAKS];
    uint16_t peakCount = 0U;

    memset(gDenseAzPower, 0, sizeof(gDenseAzPower));
    memset(gDenseAzValid, 0, sizeof(gDenseAzValid));
    memset(gDenseAzElevValid, 0, sizeof(gDenseAzElevValid));
    MmwDemo_DenseElevation_prepareSignal(candidate->index, staticCfg, signal);

    for (a = 0U; a < MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS; a++)
    {
        int32_t signedA =
            (a < (MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS / 2U))
                ? (int32_t)a
                : (int32_t)a -
                  (int32_t)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
        float u = 2.0f * (float)signedA /
                  (float)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
        float referencePower;
        float elevationPower;
        float crossReal;
        float crossImag;
        float phase;
        float v;

        memset(rowSum, 0, sizeof(rowSum));
        for (ant = 0U; ant < numVirtualAntennas; ant++)
        {
            float sr = signal[ant].real;
            float si = signal[ant].imag;
            float wr = gDenseAzSteering[a][ant].real;
            float wi = gDenseAzSteering[a][ant].imag;
            row = (uint32_t)gDenseAntRow[ant];
            rowSum[row].real += sr * wr + si * wi;
            rowSum[row].imag += si * wr - sr * wi;
        }

        referencePower =
            rowSum[gDenseReferenceRow].real * rowSum[gDenseReferenceRow].real +
            rowSum[gDenseReferenceRow].imag * rowSum[gDenseReferenceRow].imag;
        elevationPower =
            rowSum[gDenseElevationRow].real * rowSum[gDenseElevationRow].real +
            rowSum[gDenseElevationRow].imag * rowSum[gDenseElevationRow].imag;

        if (referencePower < DENSE_MIN_POWER)
        {
            continue;
        }

        /* Check configured azimuth FOV */
        if ((u < minAzSin) || (u > maxAzSin))
        {
            continue;
        }

        /* Normalize coherent row power by the number of elements before
         * comparing rows with different antenna counts. */
        if ((elevationPower >= DENSE_MIN_POWER) &&
            (referencePower > 0.0f) &&
            ((elevationPower * elevationAntNorm) >=
             (referencePower * referenceAntNorm *
              DENSE_MIN_ELEVATION_ROW_RATIO)))
        {
            crossReal =
                rowSum[gDenseReferenceRow].real * rowSum[gDenseElevationRow].real +
                rowSum[gDenseReferenceRow].imag * rowSum[gDenseElevationRow].imag;
            crossImag =
                rowSum[gDenseReferenceRow].imag * rowSum[gDenseElevationRow].real -
                rowSum[gDenseReferenceRow].real * rowSum[gDenseElevationRow].imag;
            phase = atan2sp(crossImag, crossReal);
            v = phase / (DENSE_PI * (float)gDenseVerticalBaseline);

            if ((v < minElevSin) || (v > maxElevSin))
            {
                continue;
            }

            /* Ensure u^2 + v^2 <= 1.0f by clamping v if necessary */
            float maxV2 = 1.0f - (u * u);
            if (maxV2 > 0.0f)
            {
                float maxV = sqrtf(maxV2);
                if (v > maxV) { v = maxV; }
                if (v < -maxV) { v = -maxV; }
            }
            else
            {
                v = 0.0f;
            }
            if ((v < minElevSin) || (v > maxElevSin))
            {
                continue;
            }
            gDenseAzElevValid[a] = 1U;
        }
        else
        {
            /* Keep a boresight estimate only when zero elevation is in the
             * configured FOV; do not let it affect measured height bounds. */
            if ((0.0f < minElevSin) || (0.0f > maxElevSin))
            {
                continue;
            }
            v = 0.0f;
        }

        gDenseAzPower[a] = referencePower;
        gDenseAzSourceVQ15[a] = MmwDemo_DenseElevation_sourceVToQ15(v);
        gDenseAzValid[a] = 1U;
    }

    /* Circular azimuth non-maximum suppression followed by a small top-K.
     * This removes beam sidelobe density without invoking TI CFAR. */
    for (a = 0U; a < MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS; a++)
    {
        uint32_t previous = (a == 0U)
                            ? (MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS - 1U)
                            : (a - 1U);
        uint32_t next = (a + 1U == MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS)
                        ? 0U : (a + 1U);
        uint16_t insertAt;
        uint16_t k;

        if ((gDenseAzValid[a] == 0U) ||
            ((gDenseAzValid[previous] != 0U) &&
             (gDenseAzPower[a] < gDenseAzPower[previous])) ||
            ((gDenseAzValid[next] != 0U) &&
             (gDenseAzPower[a] <= gDenseAzPower[next])))
        {
            continue;
        }

        for (k = 0U; k < peakCount; k++)
        {
            if (MmwDemo_DenseElevation_circularDistance(
                    (uint16_t)a, peakIndex[k],
                    MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS) <=
                DENSE_AZIMUTH_GUARD_BINS)
            {
                break;
            }
        }
        if (k < peakCount)
        {
            if (gDenseAzPower[a] > peakPower[k])
            {
                peakIndex[k] = (uint16_t)a;
                peakPower[k] = gDenseAzPower[a];
            }
            continue;
        }

        insertAt = peakCount;
        for (k = 0U; k < peakCount; k++)
        {
            if (gDenseAzPower[a] > peakPower[k])
            {
                insertAt = k;
                break;
            }
        }
        if (insertAt < DENSE_MAX_AZIMUTH_PEAKS)
        {
            uint16_t move = (peakCount < DENSE_MAX_AZIMUTH_PEAKS)
                            ? peakCount
                            : (DENSE_MAX_AZIMUTH_PEAKS - 1U);
            while (move > insertAt)
            {
                peakIndex[move] = peakIndex[move - 1U];
                peakPower[move] = peakPower[move - 1U];
                move--;
            }
            peakIndex[insertAt] = (uint16_t)a;
            peakPower[insertAt] = gDenseAzPower[a];
            if (peakCount < DENSE_MAX_AZIMUTH_PEAKS)
            {
                peakCount++;
            }
        }
    }

    for (a = 0U; a < peakCount; a++)
    {
        uint16_t azimuth = peakIndex[a];
        int32_t signedA =
            (azimuth < (MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS / 2U))
                ? (int32_t)azimuth
                : (int32_t)azimuth -
                  (int32_t)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
        int32_t signedD =
            (candidate->index < (staticCfg->numDopplerBins / 2U))
                ? (int32_t)candidate->index
                : (int32_t)candidate->index -
                  (int32_t)staticCfg->numDopplerBins;
                  
        int32_t left = azimuth;
        int32_t right = azimuth;
        float powerDropThreshold = gDenseAzPower[azimuth] / 4.0f;  /* -6 dB */
        float sumPower = 0.0f;
        float sumAz = 0.0f;
        /* Elevation aggregation across the azimuth region:
         *   v_cm    = Î£(v_i * P_i) / Î£(P_i)   [weighted centroid, Q15]
         *   v_min/v_max track the span of valid elevation estimates in the
         *              region, giving a physical sense of surface height. */
        float sumVPower = 0.0f;   /* numerator:   Î£(v_i * P_i) */
        float sumVValid = 0.0f;   /* denominator: Î£(P_i) for valid v bins */
        float v_min = 1.0f;
        float v_max = -1.0f;
        float u_cm, u_width;
        float v_cm, v_width;
        uint16_t distance;
        int32_t i;
        MmwDemo_DenseElevation_OutputSample sample;

        while (1)
        {
            int32_t nextLeft = left - 1;
            if (nextLeft < 0)
            {
                nextLeft += MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
            }
            if ((gDenseAzValid[nextLeft] == 0U) ||
                (gDenseAzPower[nextLeft] < powerDropThreshold) ||
                (MmwDemo_DenseElevation_circularDistance((uint16_t)azimuth, (uint16_t)nextLeft, MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS) > 12U))
            {
                break;
            }
            left = nextLeft;
        }

        while (1)
        {
            int32_t nextRight = right + 1;
            if (nextRight >= MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS)
            {
                nextRight -= MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
            }
            if ((gDenseAzValid[nextRight] == 0U) ||
                (gDenseAzPower[nextRight] < powerDropThreshold) ||
                (MmwDemo_DenseElevation_circularDistance((uint16_t)azimuth, (uint16_t)nextRight, MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS) > 12U))
            {
                break;
            }
            right = nextRight;
        }

        i = left;
        while (1)
        {
            int32_t signedI = (i < (MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS / 2U))
                                  ? i
                                  : i - MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
            float pi = gDenseAzPower[i];
            sumPower += pi;
            sumAz += (float)signedI * pi;

            /* Elevation-invalid bins may still contribute to azimuth CoM,
             * but must not stretch the vertical extent toward boresight. */
            if (gDenseAzElevValid[i] != 0U)
            {
                float vi = (float)gDenseAzSourceVQ15[i] / 32767.0f;
                sumVPower += vi * pi;
                sumVValid += pi;
                if (vi < v_min)
                {
                    v_min = vi;
                }
                if (vi > v_max)
                {
                    v_max = vi;
                }
            }
            if (i == right)
            {
                break;
            }
            i++;
            if (i >= MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS)
            {
                i -= MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
            }
        }

        /* Azimuth center-of-mass and angular width. */
        if (sumPower > 0.0f)
        {
            u_cm = 2.0f * (sumAz / sumPower) / (float)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
        }
        else
        {
            u_cm = 2.0f * (float)signedA / (float)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
        }
        distance = MmwDemo_DenseElevation_circularDistance((uint16_t)left, (uint16_t)right,
                       MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS);
        if (distance == 0U)
        {
            distance = 1U;
        }
        u_width = 2.0f * (float)distance / (float)MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;

        /* Elevation weighted centroid and span across the region.
         * v_cm  : sub-bin accurate elevation using all valid bins in region.
         * v_width: physical elevation extent (max - min elevation sine).
         *
         * If no bin in the region passed the elevation quality filter, fall
         * back to the peak-bin estimate (safe, consistent with v6 behaviour). */
        if (sumVValid > 0.0f)
        {
            v_cm    = sumVPower / sumVValid;
            v_width = (v_max >= v_min) ? (v_max - v_min) : 0.0f;
        }
        else
        {
            /* Fallback: single-bin estimate from the peak azimuth bin. */
            v_cm    = (float)gDenseAzSourceVQ15[azimuth] / 32767.0f;
            v_width = 0.0f;
        }

        if (u_cm < -1.0f) { u_cm = -1.0f; }
        if (u_cm >  1.0f) { u_cm =  1.0f; }
        if (u_width > 2.0f) { u_width = 2.0f; }
        /* Clamp elevation: v is sin(elevation) âˆˆ [-1, 1]; width âˆˆ [0, 2]. */
        if (v_cm < -1.0f) { v_cm = -1.0f; }
        if (v_cm >  1.0f) { v_cm =  1.0f; }
        if (v_width < 0.0f)  { v_width = 0.0f; }
        if (v_width > 2.0f)  { v_width = 2.0f; }

        sample.rangeIdx           = rangeIdx;
        sample.azimuthIdx         = (int8_t)signedA;
        sample.reserved0          = 0U;
        sample.dopplerIdx         = (int16_t)signedD;
        sample.azimuthUQ15        = (int16_t)(u_cm * 32767.0f);
        sample.azimuthWidthUQ15   = (uint16_t)(u_width * 32767.0f);
        sample.snrDb10            = candidate->snrDb10;
        /* sourceVQ15: weighted elevation centroid over the full 6-dB azimuth
         * region, not just the peak bin. Improves accuracy on extended
         * surfaces where elevation varies across the illuminated footprint. */
        sample.sourceVQ15         = MmwDemo_DenseElevation_sourceVToQ15(v_cm);
        /* elevationWidthUQ15: span of sin(elevation) across the region.
         * Encodes the physical height extent of the reflecting surface. */
        sample.elevationWidthUQ15 = (uint16_t)(v_width * 32767.0f);
        sample.reserved1          = 0U;
        {
            float power = gDenseAzPower[azimuth];
            if (!(power > 0.0f) || !isfinite(power))
            {
                sample.powerDb10 = (int16_t)-32768;
            }
            else
            {
                float powerDb10Scaled = 100.0f * log10f(power);
                if (powerDb10Scaled >= 32767.0f)
                {
                    sample.powerDb10 = 32767;
                }
                else if (powerDb10Scaled <= -32768.0f)
                {
                    sample.powerDb10 = (int16_t)-32768;
                }
                else
                {
                    sample.powerDb10 = (int16_t)((powerDb10Scaled >= 0.0f)
                        ? (powerDb10Scaled + 0.5f)
                        : (powerDb10Scaled - 0.5f));
                }
            }
        }
        MmwDemo_DenseElevation_insertOutput(
            &sample, output, outputCapacity, outputCount,
            sectorCount, outputLimited);
    }
}

static int16_t MmwDemo_DenseElevation_deg10(float degree)
{
    float scaled = degree * 10.0f;
    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    return (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f)
                                         : (scaled - 0.5f));
}

int32_t MmwDemo_DenseElevation_process(
    const MmwDemo_DenseElevation_Cfg *cfg,
    const MmwDemo_DenseElevation_StaticCfg *staticCfg,
    uint8_t *outBuffer,
    uint32_t outBufferSize,
    uint16_t maxOutputSamples,
    uint32_t *outSize)
{
    MmwDemo_DenseElevation_OutputHeader *header;
    MmwDemo_DenseElevation_OutputSample *output;
    uint16_t minRange;
    uint16_t maxRange;
    uint16_t outputCapacity;
    uint16_t outputCount = 0U;
    uint16_t sectorCount[DENSE_AZIMUTH_SECTORS];
    uint16_t flags = MMWDEMO_DENSE_FLAG_CFAR_BYPASSED |
                     MMWDEMO_DENSE_FLAG_DOPPLER_PREFILTER |
                     MMWDEMO_DENSE_FLAG_RX_PHASE_COMPENSATED |
                     MMWDEMO_DENSE_FLAG_TDM_COMPENSATED |
                     MMWDEMO_DENSE_FLAG_FOV_FILTERED |
                     MMWDEMO_DENSE_FLAG_FRAME_COMPLETE |
                     MMWDEMO_DENSE_FLAG_AZIMUTH_NMS;
    uint8_t outputLimited = 0U;
    uint32_t processingStartCycles;
    uint32_t processingCycles;
    int32_t configureError;

    if (outSize != NULL)
    {
        *outSize = 0U;
    }
    if ((cfg == NULL) || (staticCfg == NULL) || (outBuffer == NULL) ||
        (outSize == NULL) ||
        (outBufferSize < sizeof(MmwDemo_DenseElevation_OutputHeader)) ||
        (staticCfg->subFrameNum >= MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES) ||
        (cfg->minElevationDeg != cfg->minElevationDeg) ||
        (cfg->maxElevationDeg != cfg->maxElevationDeg) ||
        (cfg->minAzimuthDeg != cfg->minAzimuthDeg) ||
        (cfg->maxAzimuthDeg != cfg->maxAzimuthDeg) ||
        (cfg->minElevationDeg < -90.0f) || (cfg->maxElevationDeg > 90.0f) ||
        (cfg->minAzimuthDeg < -90.0f) || (cfg->maxAzimuthDeg > 90.0f) ||
        (cfg->minElevationDeg > cfg->maxElevationDeg) ||
        (cfg->minAzimuthDeg > cfg->maxAzimuthDeg))
    {
        return MMWDEMO_DENSE_EINVAL;
    }

    processingStartCycles = Cycleprofiler_getTimeStamp();
    configureError = MmwDemo_DenseElevation_configure(staticCfg);
    if (configureError != 0)
    {
        return configureError;
    }

    outputCapacity = (uint16_t)((outBufferSize - sizeof(*header)) /
        sizeof(MmwDemo_DenseElevation_OutputSample));
    if (outputCapacity > MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES)
    {
        outputCapacity = MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES;
    }
    if ((maxOutputSamples != 0U) && (outputCapacity > maxOutputSamples))
    {
        outputCapacity = maxOutputSamples;
    }
    if (outputCapacity == 0U)
    {
        return MMWDEMO_DENSE_EOUTPUT_CAPACITY;
    }

    minRange = cfg->minRangeBin;
    maxRange = cfg->maxRangeBin;
    if (minRange >= staticCfg->numRangeBins)
    {
        minRange = staticCfg->numRangeBins - 1U;
    }
    if (maxRange >= staticCfg->numRangeBins)
    {
        maxRange = staticCfg->numRangeBins - 1U;
    }
    if (minRange > maxRange)
    {
        return MMWDEMO_DENSE_ERANGE_CFG;
    }

    header = (MmwDemo_DenseElevation_OutputHeader *)outBuffer;
    output = (MmwDemo_DenseElevation_OutputSample *)(outBuffer + sizeof(*header));
    memset(sectorCount, 0, sizeof(sectorCount));
    
    {
        uint32_t p;
        uint16_t rangesFftCount = 0U;
        memset(gDenseRangeFftDone, 0, sizeof(gDenseRangeFftDone));
        
        for (p = 0U; p < staticCfg->numCfarPeaks; p++)
        {
            uint16_t rngIdx = staticCfg->cfarRngDopSnrList[p].rangeIdx;
            uint16_t dopIdx = staticCfg->cfarRngDopSnrList[p].dopplerIdx;
            int16_t snr = staticCfg->cfarRngDopSnrList[p].snr;
            MmwDemo_DenseDopplerCandidate candidate;

            if (rngIdx < minRange || rngIdx > maxRange)
            {
                continue;
            }

            /* Complex Doppler symbols are regenerated only for occupied ranges.
             * One FFT per antenna supplies every selected Doppler candidate.
             * Hard-limited to MMWDEMO_DENSE_ELEV_MAX_DOPPLER_FFT_RANGES (16) to protect frame timing budget. */
            if (rngIdx < sizeof(gDenseRangeFftDone))
            {
                if (gDenseRangeFftDone[rngIdx] == 0U)
                {
                    if (rangesFftCount < MMWDEMO_DENSE_ELEV_MAX_DOPPLER_FFT_RANGES)
                    {
                        MmwDemo_DenseElevation_dopplerFftForRange(rngIdx, staticCfg);
                        gDenseRangeFftDone[rngIdx] = 1U;
                        rangesFftCount++;
                    }
                    else
                    {
                        /* Frame budget reached; skip regeneration for additional range bins */
                        continue;
                    }
                }
            }
            else
            {
                continue;
            }

            candidate.index = dopIdx;
            candidate.snrDb10 = (uint16_t)snr;
            MmwDemo_DenseElevation_processDoppler(
                rngIdx, &candidate, cfg, staticCfg,
                output, outputCapacity, &outputCount,
                sectorCount, &outputLimited);
        }
    }

    processingCycles = Cycleprofiler_getTimeStamp() - processingStartCycles;
    if (outputLimited != 0U)
    {
        flags |= MMWDEMO_DENSE_FLAG_OUTPUT_LIMITED;
    }
    if (staticCfg->staticClutterRemovalEnabled != 0U)
    {
        flags |= MMWDEMO_DENSE_FLAG_STATIC_CLUTTER_APPLIED;
    }

    memset(header, 0, sizeof(*header));
    header->magic = MMWDEMO_DENSE_ELEVATION_MAGIC;
    header->version = MMWDEMO_DENSE_ELEVATION_PROTOCOL_VERSION;
    header->headerSize = sizeof(*header);
    header->sampleSize = sizeof(*output);
    header->numSamples = outputCount;
    header->flags = flags;
    header->subFrameNumber = staticCfg->subFrameNum;
    header->rangeStart = minRange;
    header->rangeEnd = maxRange + 1U;
    header->numRangeBins = staticCfg->numRangeBins;
    header->numDopplerBins = staticCfg->numDopplerBins;
    header->numAzimuthBins = MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS;
    header->numElevationBins = MMWDEMO_DENSE_ELEV_NUM_ELEVATION_BINS;
    header->minAzimuthDeg10 = MmwDemo_DenseElevation_deg10(cfg->minAzimuthDeg);
    header->maxAzimuthDeg10 = MmwDemo_DenseElevation_deg10(cfg->maxAzimuthDeg);
    header->minElevationDeg10 = MmwDemo_DenseElevation_deg10(cfg->minElevationDeg);
    header->maxElevationDeg10 = MmwDemo_DenseElevation_deg10(cfg->maxElevationDeg);
    header->protocolStatus = 0U;
    header->processingCycles = processingCycles;
    header->radarFrameNumber = staticCfg->frameNumber;
    *outSize = sizeof(*header) + outputCount * sizeof(*output);
    return 0;
}

void MmwDemo_DenseElevation_reset(void)
{
    gDenseGeometrySignature = 0U;
    gDenseGeometryValid = 0U;
    gDenseTwiddleSize = 0U;
}
