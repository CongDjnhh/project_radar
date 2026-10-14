/**
 *   @file  mmw_common.h
 *
 *   @brief
 *      Defines DPM functions for inter-core communication.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2024 Texas Instruments, Inc.
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

#ifndef MMW_COMMON_H
#define MMW_COMMON_H

#include <drivers/ipc_rpmsg.h>
#include <ti/common/mmwave_error.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define DPM_MSS_BOOT_INFO_BIT_POS     (0U)
#define DPM_DSS_CM4_BOOT_INFO_BIT_POS (1U)
#define DPM_DSS_BOOT_INFO_BIT_POS     (2U)

#define DPM_EINVAL (MMWAVE_ERRNO_DPM_BASE - 1)

#define DPM_BUFFER_MAX_SIZE (512U)

#define MMWDEMO_DPC_COMMONCFG    0x11111111U
#define MMWDEMO_DPC_CFG          0x22222222U
#define MMWDEMO_DPC_ELEVEST_CFG  0x33333333U
#define MMWDEMO_DPC_START        0x44444444U
#define MMWDEMO_DPC_EXECUTE      0x55555555U
#define MMWDEMO_DPC_RESULT       0x66666666U
#define MMWDEMO_WAIT_FRM_DSS_CFG 0x77777777U
#define MMWDEMO_CALIBSAVEACK     0x88888888U
#define MMWDEMO_DSS_START_SENSOR 0x99999999U
#define MMWDEMO_DSS_STOP_SENSOR  0xAAAAAAAAU

    /**
     * @brief
     *  DPM Handle
     */
    typedef void *DPM_Handle;

    typedef void (*DPM_ReportFxn)(void *data, uint16_t dataLen);

    typedef struct DPM_InitCfg_t
    {
        uint16_t localEndPt;

        DPM_ReportFxn reportFxn;

        uint8_t setBitPos;
    } DPM_InitCfg;

    /**
     * @brief
     *  DPM Master control block
     *
     * @details
     *  The structure is used to hold all the relevant information required to
     *  execute the DPM module
     */
    typedef struct DPM_MCB_t
    {
        /* dpm init config */
        DPM_InitCfg dpmInitCfg;
        /**
         * @brief   Remote IPC Object: This is used by the IPC module to communicate
         * with the remote peer to exchange DPM IPC messages.
         */
        RPMessage_Object ipcMailboxObj;

    } DPM_MCB;

    DPM_MCB gDPMObj;

    extern uint16_t gRemoteCoreEndPt[CSL_CORE_ID_MAX];
    DPM_Handle      DPM_init(DPM_InitCfg *ptrInitCfg, int32_t *errCode);
    static void     mboxRemoteMailboxCallbackFxn(RPMessage_Object *obj,
                                                 void             *arg,
                                                 void             *data,
                                                 uint16_t          dataLen,
                                                 int32_t           crcStatus,
                                                 uint16_t          remoteCoreId,
                                                 uint16_t          remoteEndPt);

    int32_t Mmw_setLinkState(uint8_t bitPos, uint8_t val);
    int32_t DPM_synch(DPM_Handle handle, int32_t *errCode);
    int32_t DPM_send(DPM_Handle handle, void *arg, uint32_t argLen, uint16_t remoteCoreID);

#ifdef __cplusplus
}
#endif


#endif /* mmw_common.h */
