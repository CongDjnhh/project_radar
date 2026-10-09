/**
 *   @file  mss_tracker.h
 *
 *   @brief
 *      Header file related to RANSAC and tracking module
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
#ifndef MMWDEMO_TRACKER_H
#define MMWDEMO_TRACKER_H

/* mmWave SDK Include Files*/
#include <ti/datapath/dpif/dpif_pointcloud.h>

/* Demo Include Files*/
#include "../common/mmw_output.h"
#include "../common/mmw_common.h"

int32_t gtrack_ransac_proc(int numObj, DPIF_PointCloudCartesian *DPIFPointCloudPtr, DPIF_PointCloudSideInfo *DPIFSideInfoPtr, uint8_t *ransacFilterMask, MmwDemo_tracker_out *targetOutput, uint32_t *numTracks);

#if 0
void sendResultsExportedCommand(DPM_Buffer *ptrResult);
#endif

int32_t gtrack_heap_init(void);
int32_t MmwDemo_trackerConfig(void);
void    MmwDemo_loadRansac(void);
int32_t MmwDemo_loadGtrackCfg(void);
int32_t MmwDemo_updateTrackerCfg(bool enable, uint16_t maxPoints, uint16_t maxTracks, float deltaT);
int32_t MmwDemo_updateRansacCfg(bool enable, uint16_t iterations, float thresh);

#endif // MMWDEMO_TRACKER_H
