/**
 * @file dense_elevation.c
 *
 * @brief Frame-synchronous navigation source extraction with continuous
 *        elevation filtering. TI CFAR is not used by this path.
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
#define DENSE_Q19_SHIFT                  (19U)
#define DENSE_MIN_POWER                  (1.0f)
#define DENSE_MAX_DOPPLER_CANDIDATES     (3U)
#define DENSE_MAX_AZIMUTH_PEAKS          (2U)
#define DENSE_DOPPLER_GUARD_BINS         (1U)
#define DENSE_AZIMUTH_GUARD_BINS         (2U)
#define DENSE_AZIMUTH_SECTORS            (8U)
/* Reject elevation phase when the normalized secondary-row power is more
 * than 16 dB below the reference row. This is a phase-quality check, not an
 * elevation-angle or occupancy threshold. */
#define DENSE_MIN_ELEVATION_ROW_RATIO    (0.025118864f)

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
    uint16_t magnitudeQ8;
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

#pragma DATA_SECTION(gDenseFftInput, ".l3ram");
#pragma DATA_ALIGN(gDenseFftInput, 8);
static far int16_t gDenseFftInput[2U * MMWDEMO_DENSE_ELEV_MAX_DOPPLER_BINS];

#pragma DATA_SECTION(gDenseFftTwiddle, ".l3ram");
#pragma DATA_ALIGN(gDenseFftTwiddle, 8);
static far int16_t gDenseFftTwiddle[2U * MMWDEMO_DENSE_ELEV_MAX_DOPPLER_BINS];

#pragma DATA_SECTION(gDenseDetRowScratch, ".l3ram");
#pragma DATA_ALIGN(gDenseDetRowScratch, 8);
static far uint16_t gDenseDetRowScratch[MMWDEMO_DENSE_ELEV_MAX_DOPPLER_BINS];

#pragma DATA_SECTION(gDenseAzPower, ".l3ram");
#pragma DATA_ALIGN(gDenseAzPower, 8);
static far float gDenseAzPower[MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS];

#pragma DATA_SECTION(gDenseAzSourceVQ15, ".l3ram");
#pragma DATA_ALIGN(gDenseAzSourceVQ15, 8);
static far int16_t gDenseAzSourceVQ15[MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS];

#pragma DATA_SECTION(gDenseAzValid, ".l3ram");
#pragma DATA_ALIGN(gDenseAzValid, 8);
static far uint8_t gDenseAzValid[MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS];

#pragma DATA_SECTION(gDenseOutputPriority, ".l3ram");
#pragma DATA_ALIGN(gDenseOutputPriority, 8);
static far uint32_t
    gDenseOutputPriority[MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES];

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
    if (signature == gDenseGeometrySignature)
    {
        return 0;
    }

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

static uint16_t MmwDemo_DenseElevation_medianQ8(
    const uint16_t *row,
    uint16_t count)
{
    uint16_t i;

    memcpy(gDenseDetRowScratch, row, count * sizeof(uint16_t));
    for (i = 1U; i < count; i++)
    {
        uint16_t key = gDenseDetRowScratch[i];
        int32_t j = (int32_t)i - 1;
        while ((j >= 0) && (gDenseDetRowScratch[j] > key))
        {
            gDenseDetRowScratch[j + 1] = gDenseDetRowScratch[j];
            j--;
        }
        gDenseDetRowScratch[j + 1] = key;
    }

    if ((count & 1U) != 0U)
    {
        return gDenseDetRowScratch[count >> 1U];
    }
    return (uint16_t)(((uint32_t)gDenseDetRowScratch[(count >> 1U) - 1U] +
                       (uint32_t)gDenseDetRowScratch[count >> 1U] + 1U) >> 1U);
}

static uint16_t MmwDemo_DenseElevation_thresholdQ8(
    float thresholdDb,
    uint16_t numVirtualAntennas)
{
    uint16_t fftAntennas = 1U;
    float threshold;

    while (fftAntennas < numVirtualAntennas)
    {
        fftAntennas <<= 1U;
    }
    /* TI dopplerproc output is Q8 average log2 magnitude. TI documents the
     * user-dB conversion as 256 * TdB / 6 * N/N'. */
    threshold = 256.0f * thresholdDb * (float)numVirtualAntennas /
                (6.0f * (float)fftAntennas);
    if (threshold > 65535.0f)
    {
        return 65535U;
    }
    return (uint16_t)(threshold + 0.5f);
}

static uint16_t MmwDemo_DenseElevation_deltaQ8ToDb10(
    uint16_t deltaQ8,
    uint16_t numVirtualAntennas)
{
    uint16_t fftAntennas = 1U;
    uint32_t value;

    while (fftAntennas < numVirtualAntennas)
    {
        fftAntennas <<= 1U;
    }
    value = ((uint32_t)deltaQ8 * 60U * fftAntennas +
             (128U * numVirtualAntennas)) /
            (256U * numVirtualAntennas);
    if (value > 65535U)
    {
        value = 65535U;
    }
    return (uint16_t)value;
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

static uint16_t MmwDemo_DenseElevation_selectDopplerCandidates(
    const uint16_t *detRow,
    uint16_t numDopplerBins,
    uint16_t numVirtualAntennas,
    float snrThresholdDb,
    MmwDemo_DenseDopplerCandidate *candidate)
{
    uint16_t noiseQ8 = MmwDemo_DenseElevation_medianQ8(detRow, numDopplerBins);
    uint16_t thresholdQ8 = MmwDemo_DenseElevation_thresholdQ8(
        snrThresholdDb, numVirtualAntennas);
    uint16_t count = 0U;
    uint16_t pass;

    for (pass = 0U; pass < DENSE_MAX_DOPPLER_CANDIDATES; pass++)
    {
        uint16_t bestIndex = 0U;
        uint16_t bestMagnitude = 0U;
        uint8_t found = 0U;
        uint16_t d;

        for (d = 0U; d < numDopplerBins; d++)
        {
            uint16_t previous = (d == 0U) ? (numDopplerBins - 1U) : (d - 1U);
            uint16_t next = (d + 1U == numDopplerBins) ? 0U : (d + 1U);
            uint32_t required = (uint32_t)noiseQ8 + thresholdQ8;
            uint16_t k;
            uint8_t separated = 1U;

            if (((uint32_t)detRow[d] < required) ||
                (detRow[d] < detRow[previous]) ||
                (detRow[d] < detRow[next]))
            {
                continue;
            }
            for (k = 0U; k < count; k++)
            {
                if (MmwDemo_DenseElevation_circularDistance(
                        d, candidate[k].index, numDopplerBins) <=
                    DENSE_DOPPLER_GUARD_BINS)
                {
                    separated = 0U;
                    break;
                }
            }
            if ((separated != 0U) &&
                ((found == 0U) || (detRow[d] > bestMagnitude)))
            {
                bestIndex = d;
                bestMagnitude = detRow[d];
                found = 1U;
            }
        }

        if (found == 0U)
        {
            break;
        }
        candidate[count].index = bestIndex;
        candidate[count].magnitudeQ8 = bestMagnitude;
        candidate[count].snrDb10 = MmwDemo_DenseElevation_deltaQ8ToDb10(
            (uint16_t)(bestMagnitude - noiseQ8), numVirtualAntennas);
        count++;
    }
    return count;
}

static int32_t MmwDemo_DenseElevation_quantizeElevation(float v)
{
    float scaled = v *
        ((float)MMWDEMO_DENSE_ELEV_NUM_ELEVATION_BINS * 0.5f);
    int32_t signedBin = (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f)
                                         : (int32_t)(scaled - 0.5f);

    if (signedBin >
        ((int32_t)MMWDEMO_DENSE_ELEV_NUM_ELEVATION_BINS / 2 - 1))
    {
        signedBin =
            (int32_t)MMWDEMO_DENSE_ELEV_NUM_ELEVATION_BINS / 2 - 1;
    }
    if (signedBin <
        -((int32_t)MMWDEMO_DENSE_ELEV_NUM_ELEVATION_BINS / 2))
    {
        signedBin =
            -((int32_t)MMWDEMO_DENSE_ELEV_NUM_ELEVATION_BINS / 2);
    }
    return signedBin;
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
    uint32_t ant;
    uint32_t a;
    uint32_t row;
    uint16_t peakIndex[DENSE_MAX_AZIMUTH_PEAKS];
    float peakPower[DENSE_MAX_AZIMUTH_PEAKS];
    uint16_t peakCount = 0U;

    memset(gDenseAzPower, 0, sizeof(gDenseAzPower));
    memset(gDenseAzValid, 0, sizeof(gDenseAzValid));
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
        float referenceNormalized;
        float elevationNormalized;
        float crossReal;
        float crossImag;
        float phase;
        float v;
        float elevationLimitScale;

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
        if ((referencePower < DENSE_MIN_POWER) ||
            (elevationPower < DENSE_MIN_POWER))
        {
            continue;
        }

        referenceNormalized = referencePower /
            ((float)gDenseRowAntCount[gDenseReferenceRow] *
             (float)gDenseRowAntCount[gDenseReferenceRow]);
        elevationNormalized = elevationPower /
            ((float)gDenseRowAntCount[gDenseElevationRow] *
             (float)gDenseRowAntCount[gDenseElevationRow]);
        if (elevationNormalized <
            referenceNormalized * DENSE_MIN_ELEVATION_ROW_RATIO)
        {
            continue;
        }

        /* P_reference * conj(P_elevation), matching TI AoAProc.
         * Antenna offsets (x, z) are in lambda/2 units.
         * Inter-row x offset already removed by azimuth steering.
         * Residual cross-row phase is \pi * gDenseVerticalBaseline * sin(elevation). */
        crossReal =
            rowSum[gDenseReferenceRow].real * rowSum[gDenseElevationRow].real +
            rowSum[gDenseReferenceRow].imag * rowSum[gDenseElevationRow].imag;
        crossImag =
            rowSum[gDenseReferenceRow].imag * rowSum[gDenseElevationRow].real -
            rowSum[gDenseReferenceRow].real * rowSum[gDenseElevationRow].imag;
        phase = atan2sp(crossImag, crossReal);
        v = phase / (DENSE_PI * (float)gDenseVerticalBaseline);

        /* This continuous source estimate is the authoritative elevation
         * filter. The 16-bin elevationIdx is never used for acceptance. */
        if ((v < minElevSin) || (v > maxElevSin) ||
            ((u * u + v * v) > 1.0f))
        {
            continue;
        }
        elevationLimitScale = sqrtf(1.0f - v * v);
        if ((u < elevationLimitScale * minAzSin) ||
            (u > elevationLimitScale * maxAzSin))
        {
            continue;
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
        float v = (float)gDenseAzSourceVQ15[azimuth] / 32767.0f;
        int32_t signedE = MmwDemo_DenseElevation_quantizeElevation(v);
        int32_t signedD =
            (candidate->index < (staticCfg->numDopplerBins / 2U))
                ? (int32_t)candidate->index
                : (int32_t)candidate->index -
                  (int32_t)staticCfg->numDopplerBins;
        MmwDemo_DenseElevation_OutputSample sample;

        sample.rangeIdx = rangeIdx;
        sample.azimuthIdx = (int8_t)signedA;
        sample.elevationIdx = (int8_t)signedE;
        sample.dopplerIdx = (int16_t)signedD;
        sample.snrDb10 = candidate->snrDb10;
        sample.sourceVQ15 = gDenseAzSourceVQ15[azimuth];
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
    uint32_t cycleBudget,
    uint8_t *outBuffer,
    uint32_t outBufferSize,
    uint16_t maxOutputSamples,
    uint32_t *outSize)
{
    MmwDemo_DenseElevation_OutputHeader *header;
    MmwDemo_DenseElevation_OutputSample *output;
    const uint16_t *detMatrix;
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
    uint32_t startCycles;
    uint32_t processingCycles;
    uint32_t range;
    uint16_t numVirtualAntennas;
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
        (cfg->snrThresholdDb != cfg->snrThresholdDb) ||
        (cfg->minElevationDeg < -90.0f) || (cfg->maxElevationDeg > 90.0f) ||
        (cfg->minAzimuthDeg < -90.0f) || (cfg->maxAzimuthDeg > 90.0f) ||
        (cfg->minElevationDeg > cfg->maxElevationDeg) ||
        (cfg->minAzimuthDeg > cfg->maxAzimuthDeg) ||
        (cfg->snrThresholdDb < 0.0f) || (cfg->snrThresholdDb > 80.0f))
    {
        return MMWDEMO_DENSE_EINVAL;
    }

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
    detMatrix = (const uint16_t *)staticCfg->detMatrix.data;
    numVirtualAntennas =
        (uint16_t)(staticCfg->numTxAntennas * staticCfg->numRxAntennas);
    memset(sectorCount, 0, sizeof(sectorCount));
    startCycles = Cycleprofiler_getTimeStamp();

    for (range = minRange; range <= maxRange; range++)
    {
        const uint16_t *detRow =
            &detMatrix[(uint32_t)range * staticCfg->numDopplerBins];
        MmwDemo_DenseDopplerCandidate
            candidate[DENSE_MAX_DOPPLER_CANDIDATES];
        uint16_t candidateCount =
            MmwDemo_DenseElevation_selectDopplerCandidates(
                detRow, staticCfg->numDopplerBins, numVirtualAntennas,
                cfg->snrThresholdDb, candidate);
        uint16_t k;

        if (candidateCount == 0U)
        {
            continue;
        }

        /* Complex Doppler symbols are regenerated only for occupied ranges.
         * One FFT per antenna supplies every selected Doppler candidate. */
        MmwDemo_DenseElevation_dopplerFftForRange(
            (uint16_t)range, staticCfg);
        for (k = 0U; k < candidateCount; k++)
        {
            MmwDemo_DenseElevation_processDoppler(
                (uint16_t)range, &candidate[k], cfg, staticCfg,
                output, outputCapacity, &outputCount,
                sectorCount, &outputLimited);
        }
    }

    processingCycles = Cycleprofiler_getTimeStamp() - startCycles;
    if (outputLimited != 0U)
    {
        flags |= MMWDEMO_DENSE_FLAG_OUTPUT_LIMITED;
    }
    if ((cycleBudget != 0U) && (processingCycles > cycleBudget))
    {
        flags |= MMWDEMO_DENSE_FLAG_TIME_BUDGET_EXCEEDED;
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
    gDenseTwiddleSize = 0U;
}
