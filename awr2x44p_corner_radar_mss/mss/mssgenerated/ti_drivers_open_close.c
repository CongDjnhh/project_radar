/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/*
 * Auto generated file
 */

#include "ti_drivers_open_close.h"
#include <kernel/dpl/DebugP.h>

void Drivers_open(void)
{
    Drivers_edmaOpen();
    Drivers_qspiOpen();
    Drivers_mibspiOpen();
    Drivers_uartOpen();
}

void Drivers_close(void)
{
    Drivers_qspiClose();
    Drivers_mibspiClose();
    Drivers_uartClose();
    Drivers_edmaClose();
}

/*
 * QSPI
 */
/* QSPI Driver handles */
QSPI_Handle gQspiHandle[CONFIG_QSPI_NUM_INSTANCES];

/* QSPI Driver Parameters */
QSPI_Params gQspiParams[CONFIG_QSPI_NUM_INSTANCES] =
{
    {
        .edmaInst = -1,
    },
};

void Drivers_qspiOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_QSPI_NUM_INSTANCES; instCnt++)
    {
        gQspiHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_QSPI_NUM_INSTANCES; instCnt++)
    {
        gQspiHandle[instCnt] = QSPI_open(instCnt, &gQspiParams[instCnt]);
        if(NULL == gQspiHandle[instCnt])
        {
            DebugP_logError("QSPI open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_qspiClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_qspiClose(void)
{
    uint32_t instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_QSPI_NUM_INSTANCES; instCnt++)
    {
        if(gQspiHandle[instCnt] != NULL)
        {
            QSPI_close(gQspiHandle[instCnt]);
            gQspiHandle[instCnt] = NULL;
        }
    }

    return;
}

/*
 * EDMA
 */
/* EDMA Driver handles */
EDMA_Handle gEdmaHandle[CONFIG_EDMA_NUM_INSTANCES];

/* EDMA Driver Open Parameters */
EDMA_Params gEdmaParams[CONFIG_EDMA_NUM_INSTANCES] =
{
    {
        .intrEnable = TRUE,
        .errIntrEnable = FALSE,
    },
    {
        .intrEnable = TRUE,
        .errIntrEnable = FALSE,
    },
};

void Drivers_edmaOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        gEdmaHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        gEdmaHandle[instCnt] = EDMA_open(instCnt, &gEdmaParams[instCnt]);
        if(NULL == gEdmaHandle[instCnt])
        {
            DebugP_logError("EDMA open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_edmaClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_edmaClose(void)
{
    uint32_t instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_EDMA_NUM_INSTANCES; instCnt++)
    {
        if(gEdmaHandle[instCnt] != NULL)
        {
            EDMA_close(gEdmaHandle[instCnt]);
            gEdmaHandle[instCnt] = NULL;
        }
    }

    return;
}


/*
 * MIBSPI
 */

/* MIBSPI Driver handles */
MIBSPI_Handle gMibspiHandle[CONFIG_MIBSPI_NUM_INSTANCES];
/* MIBSPI Driver Open Parameters */
MIBSPI_OpenParams gMibspiOpenParams[CONFIG_MIBSPI_NUM_INSTANCES] =
{
    {
        .mode                        = MIBSPI_CONTROLLER,
        .transferMode                = MIBSPI_MODE_BLOCKING,
        .transferTimeout             = SystemP_WAIT_FOREVER,
        .transferCallbackFxn         = NULL,
        .iCountSupport               = FALSE,
        .dataSize                    = 8U,
        .frameFormat                 = MIBSPI_POL0_PHA1,
        .u.controllerParams.bitRate      = 1000000,
        .u.controllerParams.t2cDelay     = 0x5U,
        .u.controllerParams.c2tDelay     = 0x5U,
        .u.controllerParams.wDelay       = 0x0U,
        .u.controllerParams.numPeripherals    = 1,
        .u.controllerParams.peripheralProf    =
        {
            
            [0] =    
            {
                .chipSelect         = 0,
                .dmaReqLine         = 0,
                .ramBufLen          = (uint8_t)MIBSPI_RAM_MAX_ELEM, 
            },
        },
        .pinMode                    = MIBSPI_PINMODE_4PIN_CS,
        .shiftFormat                = MIBSPI_MSB_FIRST,
        .dmaEnable                  = TRUE,
        .edmaInst                   = CONFIG_EDMA1,
        .eccEnable                  = FALSE,
        .csHold                     = TRUE,
        .txDummyValue               = 0xFFFFU,
        .compatibilityMode          = FALSE,
    },
};

void Drivers_mibspiOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_MIBSPI_NUM_INSTANCES; instCnt++)
    {
        gMibspiHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }
    
    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_MIBSPI_NUM_INSTANCES; instCnt++)
    {
        gMibspiHandle[instCnt] = MIBSPI_open(instCnt, &gMibspiOpenParams[instCnt]);
        if(NULL == gMibspiHandle[instCnt])
        {
            DebugP_logError("MIBSPI open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_mibspiClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_mibspiClose(void)
{
    uint32_t instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_MIBSPI_NUM_INSTANCES; instCnt++)
    {
        if(gMibspiHandle[instCnt] != NULL)
        {
            MIBSPI_close(gMibspiHandle[instCnt]);
            gMibspiHandle[instCnt] = NULL;
        }
    }

    return;
}

/*
 * UART
 */

/* UART Driver handles */
UART_Handle gUartHandle[CONFIG_UART_NUM_INSTANCES];

/* UART Driver Parameters */
UART_Params gUartParams[CONFIG_UART_NUM_INSTANCES] =
{
    {
        .baudRate           = 3125000,
        .dataLength         = UART_LEN_8,
        .stopBits           = UART_STOPBITS_1,
        .parityType         = UART_PARITY_NONE,
        .readMode           = UART_TRANSFER_MODE_BLOCKING,
        .writeMode          = UART_TRANSFER_MODE_BLOCKING,
        .readCallbackFxn    = NULL,
        .writeCallbackFxn   = NULL,
        .transferMode       = UART_CONFIG_MODE_POLLED,
        .intrNum            = 55U,
        .intrPriority       = 4U,
        .edmaInst           = 0xFFFFFFFFU,
        .rxEvtNum           = 59U,
        .txEvtNum           = 60U,
    },
    {
        .baudRate           = 115200,
        .dataLength         = UART_LEN_8,
        .stopBits           = UART_STOPBITS_1,
        .parityType         = UART_PARITY_NONE,
        .readMode           = UART_TRANSFER_MODE_BLOCKING,
        .writeMode          = UART_TRANSFER_MODE_BLOCKING,
        .readCallbackFxn    = NULL,
        .writeCallbackFxn   = NULL,
        .transferMode       = UART_CONFIG_MODE_INTERRUPT,
        .intrNum            = 53U,
        .intrPriority       = 4U,
        .edmaInst           = 0xFFFFFFFFU,
        .rxEvtNum           = 57U,
        .txEvtNum           = 58U,
    },
};

void Drivers_uartOpen(void)
{
    uint32_t instCnt;
    int32_t  status = SystemP_SUCCESS;

    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        gUartHandle[instCnt] = NULL;   /* Init to NULL so that we can exit gracefully */
    }

    /* Open all instances */
    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        gUartHandle[instCnt] = UART_open(instCnt, &gUartParams[instCnt]);
        if(NULL == gUartHandle[instCnt])
        {
            DebugP_logError("UART open failed for instance %d !!!\r\n", instCnt);
            status = SystemP_FAILURE;
            break;
        }
    }

    if(SystemP_FAILURE == status)
    {
        Drivers_uartClose();   /* Exit gracefully */
    }

    return;
}

void Drivers_uartClose(void)
{
    uint32_t instCnt;

    /* Close all instances that are open */
    for(instCnt = 0U; instCnt < CONFIG_UART_NUM_INSTANCES; instCnt++)
    {
        if(gUartHandle[instCnt] != NULL)
        {
            UART_close(gUartHandle[instCnt]);
            gUartHandle[instCnt] = NULL;
        }
    }

    return;
}

