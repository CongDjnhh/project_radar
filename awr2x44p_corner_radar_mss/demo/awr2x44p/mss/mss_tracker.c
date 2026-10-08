/**
 *   @file  mss_tracker.c
 *
 *   @brief
 *      Consists of functions related to RANSAC and tracking module
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2020 - 2021 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Standard Include Files*/
#include <stdint.h>

/* mmwWave SDK Include Files*/
#include <ti/datapath/dpif/dpif_pointcloud.h>

/* MCU + SDK Files*/
#include <kernel/dpl/HeapP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/AddrTranslateP.h>

/* Demo Include Files*/
#define GTRACK_3D
#include "../../alg/gtrack/gtrack.h"
#include "../../datapath/dpc/objdethwaDDMA/objectdetection.h"
#include "../common/mmw_output.h"
#include "mmw_mss.h"

#define GTRACK_HEAP_MEM_SIZE (60 * 1024U)
#define INVDET_EPSILON       0.000001

volatile uint32_t gGtrackPointCloudConversionTime = 0;
volatile uint32_t gGtrackStepTime                 = 0;
volatile uint32_t gransacTime                     = 0;
volatile uint32_t gTransmitTime                   = 0;
volatile uint32_t gNumPoints                      = 0;
volatile uint32_t gNumFiltered                    = 0;
volatile uint32_t gNumTracks                      = 0;

GTRACK_measurementPoint gGtrackPointCloud[GTRACK_NUM_POINTS_MAX] __attribute__((aligned(32)));
GTRACK_measurementPoint gGtrackPointCloudFiltered[GTRACK_NUM_POINTS_MAX] __attribute__((aligned(32)));
GTRACK_targetDesc       gTargetDescr[GTRACK_NUM_TRACKS_MAX] __attribute__((aligned(32)));

uint16_t gIndxInlier[GTRACK_NUM_POINTS_MAX] __attribute__((aligned(32)));

uint8_t      gGtrackHeapMem[GTRACK_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));
HeapP_Object gGtrackHeapObj;

extern MmwDemo_MSS_MCB gMmw_MCB;

static uint32_t ransacFitPolynomial(GTRACK_measurementPoint *data, uint16_t *idxInlier, uint16_t numDetectedPts, uint16_t maxNumIterations, float threshold);

int32_t gtrack_heap_init(void)
{
    HeapP_construct(&gGtrackHeapObj, gGtrackHeapMem, GTRACK_HEAP_MEM_SIZE);

    return 0;
}

int32_t gtrack_ransac_proc(int numObj, DPIF_PointCloudCartesian *DPIFPointCloudPtr, DPIF_PointCloudSideInfo *DPIFSideInfoPtr, uint8_t *ransacFilterMask, MmwDemo_tracker_out *targetOutput, uint32_t *numTracks)
{
    int      numInliers;
    int      numFiltered;
    int      curIncludeSrc;
    int      curIncludeDst;
    uint16_t mNum     = 0;
    uint16_t tNum     = 0;
    uint8_t  presence = 0;

    uint32_t i;

    gNumPoints = numObj;

    // pointcloud conversion
    if (gMmw_MCB.trackerCfg.gtrackEnabled || gMmw_MCB.trackerCfg.ransacEnabled)
    {
        gGtrackPointCloudConversionTime = CycleCounterP_getCount32();

        int i;
        for (i = 0; i < numObj; i++)
        {
            float x, y, z, vel, snr, range, azim;
            x   = DPIFPointCloudPtr[i].x;
            y   = DPIFPointCloudPtr[i].y;
            z   = DPIFPointCloudPtr[i].z;
            vel = DPIFPointCloudPtr[i].velocity;

            snr = ((float)DPIFSideInfoPtr[i].snr) / 10.0;
            snr = powf(10, snr / 20);

            range = (float)sqrtf(x * x + y * y + z * z);
            azim  = atan2f(x, y);

            gGtrackPointCloud[i].vector.range   = range;
            gGtrackPointCloud[i].vector.azimuth = azim;
            gGtrackPointCloud[i].vector.elev    = 0;
            gGtrackPointCloud[i].vector.doppler = vel;
            gGtrackPointCloud[i].snr            = snr;
        }

        gGtrackPointCloudConversionTime = CycleCounterP_getCount32() - gGtrackPointCloudConversionTime;
    }


    // ransac enabled. Get filtered pointcloud
    if (gMmw_MCB.trackerCfg.ransacEnabled)
    {
        gransacTime = CycleCounterP_getCount32();

        mNum       = numObj;
        numInliers = ransacFitPolynomial(&gGtrackPointCloud[0], &gIndxInlier[0], mNum, gMmw_MCB.trackerCfg.ransacIterations, gMmw_MCB.trackerCfg.ransacThresh);

        // filter the pointcloud
        numFiltered   = 0;
        curIncludeSrc = 0; // next element to be included from source pointcloud
        curIncludeDst = 0; // index where next included element will be added to gGtrackPointCloudFiltered

        // set filter mask to 0
        memset(ransacFilterMask, 0, mNum * sizeof(uint8_t));

        for (int i = 0; i < numInliers; i++)
        {
            while (curIncludeSrc < gIndxInlier[i])
            {
                ransacFilterMask[curIncludeSrc]          = 1;
                gGtrackPointCloudFiltered[curIncludeDst] = gGtrackPointCloud[curIncludeSrc];
                curIncludeSrc++;
                curIncludeDst++;
                numFiltered++;
            }
            curIncludeSrc = gIndxInlier[i] + 1;
        }

        while (curIncludeSrc < mNum)
        { // add all remaining elements after last gIndxInlier to gGtrackPointCloudFiltered
            ransacFilterMask[curIncludeSrc]          = 1;
            gGtrackPointCloudFiltered[curIncludeDst] = gGtrackPointCloud[curIncludeSrc];
            curIncludeSrc++;
            curIncludeDst++;
            numFiltered++;
        }
        gNumFiltered = numFiltered;
        gransacTime  = CycleCounterP_getCount32() - gransacTime;
    }


    if (gMmw_MCB.trackerCfg.gtrackEnabled)
    {
        tNum = 0;

        gGtrackStepTime = CycleCounterP_getCount32();

        if (gMmw_MCB.trackerCfg.ransacEnabled)
        {
            mNum = numFiltered;
            gtrack_step(gMmw_MCB.trackerCfg.gHTrackModule, gGtrackPointCloudFiltered, 0, mNum, gTargetDescr, &tNum, 0, 0, &presence, NULL);
        }
        else
        {
            mNum = (uint16_t)numObj;
            gtrack_step(gMmw_MCB.trackerCfg.gHTrackModule, gGtrackPointCloud, 0, mNum, gTargetDescr, &tNum, 0, 0, &presence, NULL);
        }

        gGtrackStepTime = CycleCounterP_getCount32() - gGtrackStepTime;

        gNumTracks = tNum;
        *numTracks = tNum;

        for (i = 0; i < tNum; i++)
        {
            targetOutput[i].posVelAcc = *((GTRACK_state_vector_pos_vel_acc *)&gTargetDescr[i].S);
            targetOutput[i].state     = gTargetDescr[i].trackState;
        }
    }

    return 0;
}


/**
 *  @b Description
 *  @n
 *       This function computes the inverse of 3x3 matrix.
 *       Matrix is real, single precision floating point.
 *       Matrix is in row-major order
 *
 *  @param[in]  A
 *       Matrix A
 *  @param[out]  det_out
 *       det_out = determinant;
 *  @param[out]  inv
 *       inv = inverse(A);
 *
 *
 *  @retval
 *      errCode: -1 if determinant of given matrix is 0
 */

int ransacMatrixInv(const float *A, float *det_out, float *inv)
{
    float det;
    float invdet;

    det = A[0] * (A[4] * A[8] - A[7] * A[5]) -
        A[1] * (A[3] * A[8] - A[5] * A[6]) +
        A[2] * (A[3] * A[7] - A[4] * A[6]);

    if (det <= 0.0 + INVDET_EPSILON && det >= 0.0 - INVDET_EPSILON)
    {
        return -1;
    }

    invdet = 1.0 / det;
    // invdet = _rcpsp(det);
    // invdet = invdet * (2.0f - det * invdet);

    inv[0] = (A[4] * A[8] - A[7] * A[5]) * invdet;
    inv[1] = (A[2] * A[7] - A[1] * A[8]) * invdet;
    inv[2] = (A[1] * A[5] - A[2] * A[4]) * invdet;
    inv[3] = (A[5] * A[6] - A[3] * A[8]) * invdet;
    inv[4] = (A[0] * A[8] - A[2] * A[6]) * invdet;
    inv[5] = (A[3] * A[2] - A[0] * A[5]) * invdet;
    inv[6] = (A[3] * A[7] - A[6] * A[4]) * invdet;
    inv[7] = (A[6] * A[1] - A[0] * A[7]) * invdet;
    inv[8] = (A[0] * A[4] - A[3] * A[1]) * invdet;

    *det_out = det;

    return 0;
}


// xData -> azimuth, yData->radial velocity
// idxInlier to be excluded
static uint32_t ransacFitPolynomial(GTRACK_measurementPoint *data, uint16_t *idxInlier, uint16_t numDetectedPts, uint16_t maxNumIterations, float threshold)
{
    uint32_t idx, idxRandNum, idxIteration;
    uint32_t numPtsInlier, bestCountInlier;
    float    yFit, errFit;
    float    det;
    float    xData[3], yData[3];
    float    Amat[9]   = { 1, 0, 0, 1, 0, 0, 1, 0, 0 };
    float    P[3]      = { 0, 0, 0 }; // Current Polynomial model fit
    float    P_Best[3] = { 0, 0, 0 }; // Best Polynomial model fit
    float    invA[9];

    // Initializations
    bestCountInlier = 0;

    for (idxIteration = 0; idxIteration < maxNumIterations; idxIteration++)
    {

        // (A) Select random data points
        // DSP_urand32_init(seed, polynomial, state);

        for (idx = 0; idx < 3; idx++)
        {
            // idxRandNum = rand() % numDetectedPts;
            idxRandNum = rand() / (RAND_MAX + 1.0) * (numDetectedPts - 1);
            xData[idx] = data[idxRandNum].vector.azimuth;
            yData[idx] = data[idxRandNum].vector.doppler;
            // printf("%d \n", idxRandNum);
        }

        // (B) Fit a polynomial to randomly selected points
        Amat[1] = xData[0];
        Amat[2] = xData[0] * xData[0];
        Amat[4] = xData[1];
        Amat[5] = xData[1] * xData[1];
        Amat[7] = xData[2];
        Amat[8] = xData[2] * xData[2];

        if (ransacMatrixInv(Amat, &det, invA) != 0)
        {
            // determinant for this iteration was 0. skip this.
            continue;
        }

        // Compute the model parameters
        P[0] = invA[0] * yData[0] + invA[1] * yData[1] + invA[2] * yData[2];
        P[1] = invA[3] * yData[0] + invA[4] * yData[1] + invA[5] * yData[2];
        P[2] = invA[6] * yData[0] + invA[7] * yData[1] + invA[8] * yData[2];

        // (C) Check for all the data points with the model to Find the Outliers
        numPtsInlier = 0;
        for (idx = 0; idx < numDetectedPts; idx++)
        {
            yFit   = P[0] + P[1] * data[idx].vector.azimuth + P[2] * data[idx].vector.azimuth * data[idx].vector.azimuth;
            errFit = fabsf(yFit - data[idx].vector.doppler);
            if (errFit < threshold)
            {
                numPtsInlier++;
            }
        }

        // (D) Update the "BEST" model
        if (numPtsInlier > bestCountInlier)
        {
            bestCountInlier = numPtsInlier;
            P_Best[0]       = P[0];
            P_Best[1]       = P[1];
            P_Best[2]       = P[2];
            // printf("%d   %d\n", idxIteration, bestCountInlier);
        }
    }
    // (E) Use the Best Fit Model to find the inlier points;
    numPtsInlier = 0;
    for (idx = 0; idx < numDetectedPts; idx++)
    {
        yFit   = P_Best[0] + P_Best[1] * data[idx].vector.azimuth + P_Best[2] * data[idx].vector.azimuth * data[idx].vector.azimuth;
        errFit = fabsf(yFit - data[idx].vector.doppler);
        if (errFit < threshold)
        {
            idxInlier[numPtsInlier] = idx;
            numPtsInlier++;
        }
    }

    return numPtsInlier;
}

#if 0
void sendResultsExportedCommand(DPM_Buffer *ptrResult){
    int32_t retVal;
    DPC_ObjectDetection_ExecuteResult *dpcResults;
    DPC_ObjectDetection_ExecuteResultExportedInfo exportInfo;

    /* Validate DPC results buffer */
    DebugP_assert (ptrResult->size[0] == sizeof(DPC_ObjectDetection_ExecuteResult));

    /* Translate the address: */
    dpcResults = (DPC_ObjectDetection_ExecuteResult *) AddrTranslateP_getLocalAddr((uint32_t)ptrResult->ptrBuffer[0]);

        /*****************************************************************
     * Send notification to data path after results are handled
     *****************************************************************/
    /* Indicate result consumed and end of frame/sub-frame processing */
    exportInfo.subFrameIdx = dpcResults->subFrameIdx;
    retVal = DPM_ioctl (gMmw_MCB.objDetDpmHandle,
                        DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED,
                        &exportInfo,
                        sizeof (DPC_ObjectDetection_ExecuteResultExportedInfo));
    if (retVal < 0) {
        DebugP_log ("Error: DPM DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED failed [Error code %d]\n",
                    retVal);
        DebugP_assert(0);
    }
}
#endif
