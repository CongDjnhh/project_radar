/**
 * @file dense_elevation.h
 *
 * @brief Frame-synchronous navigation source cloud, independent of TI CFAR.
 */

#ifndef MMWDEMO_DENSE_ELEVATION_H
#define MMWDEMO_DENSE_ELEVATION_H

#include <stdint.h>
#include <stddef.h>
#include <ti/common/sys_common.h>
#include <ti/datapath/dpif/dpif_radarcube.h>
#include <ti/datapath/dpif/dpif_detmatrix.h>
#include <ti/board/antenna_geometry.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MMWDEMO_DENSE_ELEV_NUM_AZIMUTH_BINS       (64U)
#define MMWDEMO_DENSE_ELEV_NUM_ELEVATION_BINS     (16U)
#define MMWDEMO_DENSE_ELEV_MAX_VIRTUAL_ANTENNAS   \
    (SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL)
#define MMWDEMO_DENSE_ELEV_MAX_DOPPLER_BINS       (128U)
#define MMWDEMO_DENSE_ELEV_MAX_SUBFRAMES          (4U)
#define MMWDEMO_DENSE_ELEV_MAX_ROWS               (4U)
#define MMWDEMO_DENSE_ELEV_MAX_OUTPUT_SAMPLES     (1536U)

#define MMWDEMO_DENSE_ELEVATION_MAGIC             (0x44454C56U) /* DELV */
#define MMWDEMO_DENSE_ELEVATION_PROTOCOL_VERSION  (7U)

/* V7 is one complete radar frame, never a rolling or cross-frame sweep. */
#define MMWDEMO_DENSE_FLAG_CFAR_BYPASSED           (1U << 0)
#define MMWDEMO_DENSE_FLAG_DOPPLER_PREFILTER       (1U << 1)
#define MMWDEMO_DENSE_FLAG_RX_PHASE_COMPENSATED    (1U << 2)
#define MMWDEMO_DENSE_FLAG_TDM_COMPENSATED         (1U << 3)
#define MMWDEMO_DENSE_FLAG_FOV_FILTERED             (1U << 4)
#define MMWDEMO_DENSE_FLAG_FRAME_COMPLETE           (1U << 5)
#define MMWDEMO_DENSE_FLAG_OUTPUT_LIMITED           (1U << 6)
#define MMWDEMO_DENSE_FLAG_TIME_BUDGET_EXCEEDED     (1U << 7)
#define MMWDEMO_DENSE_FLAG_STATIC_CLUTTER_APPLIED   (1U << 8)
#define MMWDEMO_DENSE_FLAG_AZIMUTH_NMS              (1U << 9)
#define MMWDEMO_DENSE_REQUIRED_FLAGS \
    (MMWDEMO_DENSE_FLAG_CFAR_BYPASSED | \
     MMWDEMO_DENSE_FLAG_DOPPLER_PREFILTER | \
     MMWDEMO_DENSE_FLAG_RX_PHASE_COMPENSATED | \
     MMWDEMO_DENSE_FLAG_TDM_COMPENSATED | \
     MMWDEMO_DENSE_FLAG_FOV_FILTERED | \
     MMWDEMO_DENSE_FLAG_FRAME_COMPLETE | \
     MMWDEMO_DENSE_FLAG_AZIMUTH_NMS)

/* Negative DSP return codes. Positive magnitude is carried in
 * outputHeader.protocolStatus when processing fails. */
#define MMWDEMO_DENSE_EINVAL                         (-1)
#define MMWDEMO_DENSE_ERADARCUBE_NULL                (-2)
#define MMWDEMO_DENSE_ERADARCUBE_FORMAT              (-3)
#define MMWDEMO_DENSE_EANTENNA_CFG                   (-4)
#define MMWDEMO_DENSE_ERANGE_CFG                     (-5)
#define MMWDEMO_DENSE_EDOPPLER_CFG                   (-6)
#define MMWDEMO_DENSE_ERADARCUBE_SIZE                (-7)
#define MMWDEMO_DENSE_EGEOMETRY_ROWS                 (-8)
#define MMWDEMO_DENSE_EOUTPUT_CAPACITY               (-9)
#define MMWDEMO_DENSE_EDETMATRIX_NULL                (-10)
#define MMWDEMO_DENSE_EDETMATRIX_FORMAT              (-11)
#define MMWDEMO_DENSE_EDETMATRIX_SIZE                (-12)
#define MMWDEMO_DENSE_EUNSUPPORTED_MIMO               (-13)
#define MMWDEMO_DENSE_ETIME_BUDGET                    (-14)

typedef struct MmwDemo_DenseElevation_Cfg_t
{
    float minElevationDeg;
    float maxElevationDeg;
    float minAzimuthDeg;
    float maxAzimuthDeg;
    uint16_t minRangeBin;
    uint16_t maxRangeBin;
    float snrThresholdDb;
} MmwDemo_DenseElevation_Cfg;

/** One navigation source. sourceVQ15 is the continuous v=sin(elevation)
 * estimate used by the firmware FOV filter. elevationIdx is display-only. */
typedef struct MmwDemo_DenseElevation_OutputSample_t
{
    uint16_t rangeIdx;
    int8_t   azimuthIdx;   /* -32 .. +31, u = 2*index/64 */
    int8_t   elevationIdx; /*  -8 ..  +7, quantized compatibility value */
    int16_t  dopplerIdx;   /* signed Doppler bin */
    uint16_t snrDb10;      /* detMatrix peak over range-row median, 0.1 dB */
    int16_t  sourceVQ15;   /* continuous v=sin(elevation), Q15 */
    int16_t  powerDb10;    /* Raw beamformed digital power at the steered
                            * direction, 100*log10(power), in 0.1 dB units.
                            * It is not calibrated dBm and remains dependent on
                            * FFT/window/profile scaling. INT16_MIN means that
                            * the power was zero or non-finite. */
} MmwDemo_DenseElevation_OutputSample;

/** Header at the beginning of every TLV 11 payload. */
typedef struct MmwDemo_DenseElevation_OutputHeader_t
{
    uint32_t magic;
    uint16_t version;
    uint16_t headerSize;
    uint16_t sampleSize;
    uint16_t numSamples;
    uint16_t flags;
    uint16_t subFrameNumber;
    uint16_t rangeStart;
    uint16_t rangeEnd;       /* exclusive; entire interval is from one frame */
    uint16_t numRangeBins;
    uint16_t numDopplerBins;
    uint8_t  numAzimuthBins;
    uint8_t  numElevationBins;
    int16_t  minAzimuthDeg10;
    int16_t  maxAzimuthDeg10;
    int16_t  minElevationDeg10;
    int16_t  maxElevationDeg10;
    uint16_t protocolStatus; /* 0 success; positive magnitude of DSP error */
    uint32_t processingCycles;
    uint32_t radarFrameNumber;
} MmwDemo_DenseElevation_OutputHeader;

typedef struct MmwDemo_DenseElevation_StaticCfg_t
{
    DPIF_RadarCube radarCube;
    DPIF_DetMatrix detMatrix; /* Q8 log2 magnitude from TI Doppler HWA */
    ANTDEF_AntGeometry antDef;
    const cmplx16ImRe_t *rxChPhaseComp;
    const int32_t *dopplerWindow;
    uint16_t numRangeBins;
    uint16_t numDopplerChirps;
    uint16_t numDopplerBins;
    uint8_t  numTxAntennas;
    uint8_t  numRxAntennas;
    uint8_t  dopplerWindowIsSymmetric;
    uint8_t  staticClutterRemovalEnabled;
    uint8_t  isTdmMimoValid;
    uint8_t  subFrameNum;
    uint8_t  reserved[2];
    uint32_t frameNumber;
} MmwDemo_DenseElevation_StaticCfg;

/** Process every configured range in the current radar frame. Work is never
 * continued in a later frame. deadlineStartCycles/deadlineOffsetCycles define
 * one absolute frame-relative deadline shared by ObjectDetection, Nav and
 * Dense; cycleBudget is retained as Nav-entry telemetry for the output header.
 * A nonzero deadlineOffsetCycles returns MMWDEMO_DENSE_ETIME_BUDGET instead of
 * publishing partial data when that common deadline expires. */
int32_t MmwDemo_DenseElevation_process(
    const MmwDemo_DenseElevation_Cfg *cfg,
    const MmwDemo_DenseElevation_StaticCfg *staticCfg,
    uint32_t deadlineStartCycles,
    uint32_t deadlineOffsetCycles,
    uint32_t cycleBudget,
    uint8_t *outBuffer,
    uint32_t outBufferSize,
    uint16_t maxOutputSamples,
    uint32_t *outSize);

void MmwDemo_DenseElevation_reset(void);

/* Wire ABI checks: ARM and C674 must agree exactly. */
typedef char MmwDemo_DenseSampleSizeCheck[
    (sizeof(MmwDemo_DenseElevation_OutputSample) == 12U) ? 1 : -1];
typedef char MmwDemo_DenseSamplePowerOffsetCheck[
    (offsetof(MmwDemo_DenseElevation_OutputSample, powerDb10) == 10U) ? 1 : -1];
typedef char MmwDemo_DenseSampleRangeOffsetCheck[
    (offsetof(MmwDemo_DenseElevation_OutputSample, rangeIdx) == 0U) ? 1 : -1];
typedef char MmwDemo_DenseSampleAzimuthOffsetCheck[
    (offsetof(MmwDemo_DenseElevation_OutputSample, azimuthIdx) == 2U) ? 1 : -1];
typedef char MmwDemo_DenseSampleElevationOffsetCheck[
    (offsetof(MmwDemo_DenseElevation_OutputSample, elevationIdx) == 3U) ? 1 : -1];
typedef char MmwDemo_DenseSampleDopplerOffsetCheck[
    (offsetof(MmwDemo_DenseElevation_OutputSample, dopplerIdx) == 4U) ? 1 : -1];
typedef char MmwDemo_DenseSampleSnrOffsetCheck[
    (offsetof(MmwDemo_DenseElevation_OutputSample, snrDb10) == 6U) ? 1 : -1];
typedef char MmwDemo_DenseSampleSourceVOffsetCheck[
    (offsetof(MmwDemo_DenseElevation_OutputSample, sourceVQ15) == 8U) ? 1 : -1];
typedef char MmwDemo_DenseHeaderSizeCheck[
    (sizeof(MmwDemo_DenseElevation_OutputHeader) == 44U) ? 1 : -1];

#ifdef __cplusplus
}
#endif

#endif /* MMWDEMO_DENSE_ELEVATION_H */
