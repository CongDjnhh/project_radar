/**
 *   @file  mss_main.c
 *
 *   @brief
 *      This is the main file which implements the millimeter wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018-2024 Texas Instruments, Inc.
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

/** @mainpage Millimeter Wave (mmw) Demo for AWR294X
 * [TOC]
 *  @section intro_sec Introduction
 *
 *  @image html toplevel.png
 *
 *  The millimeter wave demo shows some of the capabilities of the AWR294X SOC
 *  using the drivers in the mmWave SDK (Software Development Kit).
 *  It allows user to specify the chirping profile and displays the detected
 *  objects and other information in real-time.
 *
 *  Following is a high level description of the features of this demo:
 *  - Be able to specify desired chirping profile through command line interface (CLI)
 *    on a UART port or through the TI Gallery App - **mmWave Demo Visualizer** -
 *    that allows user to provide a variety of profile configurations via the UART input port
 *    and displays the streamed detected output from another UART port in real-time,
 *    as seen in picture above.
 *  - Some sample profile configurations have been provided in the demo directory that can be
 *    used with CLI directly or via **mmWave Demo Visualizer** under following directory:
 *    @verbatim mmw/profiles @endverbatim
 *  - Do 1D, 2D, CFAR, Azimuth and Elevation processing and stream out velocity
 *    and three spatial coordinates (x,y,z) of the detected objects in real-time.
 *    The demo can also be configured to do 2D only detection (velocity and x,y coordinates).
 *  - Various display options besides object detection like Doppler-range heat map.
 *
 *  @section procChain Processing Chains
 *    Two processing chains are available for the AWR294X demo.
      Check @verbatim
      ti/datapath/dpc/objectdetection/objdethwa/docs/doxygen/html/index.html
      @endverbatim
      or
      @verbatim
      ti/datapath/dpc/objectdetection/objdethwaDDMA/docs/doxygen/html/index.html
      @endverbatim
 *    for details on the processing chains. The CLI arguments for each chain are available in
 *    the User Guide.
 *  - TDM (Time Division Multiplexing): Here, no two transmitters are active at the same time.
 *    At the receiver antennae, there is no ambiguity on the chirp received, and every received
 *    chirp can directly be mapped, without the need of a separate disambiguation, to the corresponding
 *    Tx antenna.  The antenna support for this chain is (AzimTx, ElevTx, Rx) = (2,1,4), (2,1,4),
 *    (2,0,4), (1,0,1), (1,0,2), (1,0,4) for AWR294X and additionally (3,1,4) and (3,0,4) for AWR2944.
 *  - DDM (Doppler Division Multiplexing): Here, all transmitters are active at the same time.
 *    At the receiver antennae, the received chirps need to be disambiguated (demodulation)
 *    to obtain samples corresponding to each Tx antenna. The antenna support for this chain is
 *    (AzimTx, ElevTx, Rx) = (2,1,4) for AWR2943 and (3,1,4) for AWR2944.
 *
 *   Feature | AWR2943-DDM | AWR2943-TDM | AWR2944-DDM |  AWR2944-TDM
 *  :----|:-------------------------|:---------------------------|:-----------------------------------|:-----------------------------------:
 *   Legacy Frame | Yes | Yes | Yes | Yes
 *   Advanced subframe | No | Yes | No | Yes
 *   LVDS Streaming | Yes | Yes | Yes | Yes
 *   Enet Streaming | No | Yes | No | Yes
 *   Range Bias Estimation/Compensation | No | Yes | No | Yes
 *   RX Gain/Phase Estimation/Compensation | Yes | Yes | Yes | Yes
 *
 *  @section limit Limitations
 *  - Because of UART speed limit (< 1 Mbps), the frame time is more restrictive.
 *    For example, for the azimuth and Doppler heat maps for 256 FFT range and
 *    16 point FFT Doppler, it takes about 200 ms to transmit.
 *  - For most boards, a range bias of few centimeters has been observed. User can estimate
 *    the range bias on their board and correct using the calibration procedure
 *    described in @ref Calibration_section.
 *
 *  @section systemFlow System Execution Flow
 *  The millimeter wave demo runs on R4F (MSS) and DSP(DSS). Following diagram shows the system
 *  execution flow
 *
 *  @image html system_flow.png "System Execution Flow"
 *
 *  @section tasks Software Tasks
 *    The demo consists of the following (FreeRTOS) tasks:
 *
 *    **MSS**
 *    - @ref MmwDemo_initTask. This task is created/launched by @ref main and is a
 *      one-time active task whose main functionality is to initialize drivers (\<driver\>_init),
 *      MMWave module (MMWave_init), DPM module (DPM_init), open UART and other drivers (SPI),
 *      and create/launch the following tasks
 *      (the @ref CLI_task is launched indirectly by calling @ref CLI_open).
 *    - @ref CLI_task. This command line interface task provides a simplified 'shell' interface
 *      which allows the configuration of the BSS via the mmWave interface (MMWave_config).
 *      It parses input CLI configuration commands like chirp profile and GUI configuration.
 *      When sensor start CLI command is parsed, all actions related to starting sensor and
 *      starting the processing the data path are taken.
 *      When sensor stop CLI command is parsed, all actions related to stopping the sensor and
 *      stopping the processing of the data path are taken
 *    - @ref MmwDemo_mmWaveCtrlTask. This task is used to provide an execution
 *      context for the mmWave control, it calls in an endless loop the MMWave_execute API.
 *    - @ref mmwDemo_mssDPMTask. This task is used to provide an execution context for
 *      DPM (Data Path Manager) execution, it calls in an endless loop. There is no DPC registered
 *      with DPM.
 *    - @ref mmwDemo_mssUartDataExportTask. This task is used to export the data on UART. This task
 *      pends for @ref UartExportSemHandle in an endless loop, which is posted when current frame
 *      processing and previous frame's uart transmission is completed.
 *
 *    **DSS**
 *    - @ref MmwDemo_initTask. This task is created/launched by @ref main and is a
 *      one-time active task whose main functionality is to initialize drivers (\<driver\>_init),
 *      DPM module (DPM_init), data path related drivers (EDMA, HWA), and
 *      create/launch the following tasks.
 *    - @ref MmwDemo_DPC_ObjectDetection_dpmTask. This task is used to provide an execution
 *      context for DPM (Data Path Manager) execution, it calls in an endless loop
 *      the DPM_execute API. In this context, all of the registered object detection
 *      DPC (Data Path Chain) APIs like configuration, control and execute will
 *      take place. In this task. When the DPC's execute API produces the detected objects and other
 *      results, they are reported to MSS where they are transmitted out of the UART port for display
 *      using the visualizer.
 *
 *  @section datapath Data Path
 *   @image html datapath_overall.png "Top Level Data Path Processing Chain"
 *   \n
 *   \n
 *   @image html datapath_overall_timing.png "Top Level Data Path Timing - TDM"
 *   \n
 *   @image html datapath_overall_timingDDM.png "Top Level Data Path Timing - DDM"
 *
 *   The data path processing consists of taking ADC samples as input and producing
 *   detected objects (point-cloud and other information) to be shipped out of
 *   UART port to the PC. The algorithm processing is realized using
 *   the DPM registered Object Detection DPC. The details of the processing in DPC
 *   can be seen from the following doxygen documentation based on the processing chain
 *   to be used:
 *   @verbatim
      ti/datapath/dpc/objectdetection/objdethwa/docs/doxygen/html/index.html
     @endverbatim

 *   or
     @verbatim
      ti/datapath/dpc/objectdetection/objdethwaDDMA/docs/doxygen/html/index.html
     @endverbatim
 *
 *  @section DDMP Phase shifters (DDM)
 *   ti/datapath/dpc/objectdetection/objdethwaDDMA/docs/doxygen/html/index.html can be checked for information
 *   on why phase shifts are needed in the DDMA processing chain and what their values will be.
 *   CLI ddmPhaseShiftAntOrder takes antennas indices in increasing order of phase shift value,
 *   if all of them were enabled For example, a value of {0,3,1,2} would mean that phase shifts for a particular chirp are
 *   in the following order of magnitude- tx0ChirpPhase < tx3ChirpPhase < tx1ChirpPhase < tx2ChirpPhase
 *   Even if the user does not intend to use all the tx antennas, the order should be programmed assuming
 *   that all the Tx were enabled. The phase shift values for the ones that are not enabled will be
 *   configured to 0 by the code. \n
 *
 *   Note that in the DDMA case, the elevation antenna(s) should always come at the end of this array.
 *   Basically, phaseShift(azimuth) < phaseShift(elevation) must be ensured. Hence, {0, 2, 3, 1} should be configured
 *   for AWR2944ETS, since Tx0, Tx2, Tx3 are azimuth antennas and Tx1 is elevation antenna.
 *   To understand antenna configuration for AWR294X, please refer to ti/board folder in the SDK.
 *   For AWR2943, the {0, 2, 1} is similarly configured.
 *
 *  @section antGeometryCfg Antenna Geometry Configuration (DDM)
 *  DDM Demo supports different antenna configurations configurable via CLI antGeometryCfg. By default, profiles provided
 *  in the demo are configured according to the AWR294X EVM. User can change the layout using the antGeometryCfg command.
 *  In this command, the row and column index of each antenna define the virtual antennas' physical location index (0, 1, 2, ...)
 *  in the elevation and azimuth domains, respectively, as illustrated in below figure.
 *  In other words, for the AWR2944 EVM antenna pattern, these index values indicate two elevation rows (specified by 0 and 1), and 12 azimuth columns (specified by
 *  0, 1, 2, ... 11) followed by the last two arguments representing the unit of cell spacing in azimuth and elevation dimension in units of $\lambda$.
 *
 *  @verbatim
    antGeometryCfg <Row(Tx0Rx0)> <Col(Tx0Rx0)> <Row(Tx0Rx1)> <Col(Tx0Rx1)> ... <Row(Tx0Rx[R-1])> <Col(Tx0Rx[R-1])> <Row(Tx1Rx0)> <Col(Tx1Rx0)> ... <Row(Tx[T-1]Rx[R-1])> <Col(Tx[T-1]Rx[R-1])>
    @endverbatim
 *  \n
 *
 *  @image html ETS_antenna.png "Virtual antenna pattern and corresponding antenna index for AWR2944 EVM"
 *
 *  For the antenna pattern shown in figure, CLI command should be given as
 *  @verbatim
    antGeometryCfg 1 0 1 1 1 2 1 3 0 2 0 3 0 4 0 5 1 4 1 5 1 6 1 7 1 8 1 9 1 10 1 11 0.5 0.8
    @endverbatim
 *
 *  From the entered antenna geometry, antenna rearrangement order and zero insertion mask in azimuth and elevation row
 *  is computed, required for azimuth and elevation angle estimation. Provided antenna calibration parameters are also
 *  rearranged based on the computed rearrangement order.
 *
 *  @section optimization DDMA Optimizations
 *  In order to reduce the processing time of DDMA chain, primarily following optimizations are done:
 *  - HWA/DMA/DSP parallelization.
 *  - EDMA Polling instead of Interrupts.
 *  - Using Linear transfers in EDMA instead of Transpose transfers.
 *  - Parallelizing AoA processing with the next frame.
 *  - Parallelizing UART data sending the next frame.
 *  - Reduction of number of Azimuth Bins from 48 to 32.
 *  - Disable of Range CFAR / Sum TX.
 *  - Usage of IDMA in DSP for transfer.
 *  - Optimization of DSP code of AoA processing.
 *  - Optimization of DSP Code of DDMA Demodulation.
 *  - Optimization of decompression stage for lesser number of range bins per compressed block.
 *
 *   @image html DDM_optimization1.png "Parallelizing AoA processing and UART data transfer with the next frame"
 *
 * As shown in the diagram, AoA uses DSP and 1D processing is done entirely on HWA, so both of them can be parallelized.
 * Similarly, UART TX run entirely on R5F, so it can be parallelized with the processes running on HWA and DSP.
 * So, after the parallel processing optimizations, AoA processing and UART transfer of the current frame can run in
 * parallel with the 1D and 2D processing of the next frame. Please note that the timing numbers show in above diagram
 * are for AWR2944 high range profile (profile_3d_3Azim_1ElevTx_DDMA_awr2944_highRange.cfg). Also, note that AoA timing
 * depends on the number of detected objects. AoA processing for 100 detected objects takes around 250us.
 *
 *  @section output Output information sent to host
 *      @subsection output_general Output Packet
 *      Output packets with the detection information are sent out every frame
 *      through the UART. Each packet consists of the header @ref MmwDemo_output_message_header_t
 *      and the number of TLV items containing various data information with
 *      types enumerated in @ref MmwDemo_output_message_type_e. The numerical values
 *      of the types can be found in @ref mmw_output.h. Each TLV
 *      item consists of type, length (@ref MmwDemo_output_message_tl_t) and payload information.
 *      The structure of the output packet is illustrated in the following figure.
 *      Since the length of the packet depends on the number of detected objects
 *      it can vary from frame to frame. The end of the packet is padded so that
 *      the total packet length is always multiple of 32 Bytes.
 *
 *      @image html output_packet_uart.png "Output packet structure sent to UART"
 *
 *      The following subsections describe the structure of each TLV.
 *
 *      @subsection tlv1 List of detected objects
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_DETECTED_POINTS)
 *
 *       Length: (Number of detected objects) x (size of @ref DPIF_PointCloudCartesian_t)
 *
 *       Value: Array of detected objects. The information of each detected object
 *       is as per the structure @ref DPIF_PointCloudCartesian_t. When the number of
 *       detected objects is zero, this TLV item is not sent. The maximum number of
 *       objects that can be detected in a sub-frame/frame is @ref DPC_OBJDET_MAX_NUM_OBJECTS.
 *
 *       The orientation of x,y and z axes relative to the sensor is as per the
 *       following figure.
 *
 *        @image html coordinate_geometry.png "Coordinate Geometry"
 *
 *       The whole detected objects TLV structure is illustrated in figure below.
 *
 *       @image html detected_objects_tlv.png "Detected objects TLV"
 *
 *      @subsection tlv2 Range profile
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_RANGE_PROFILE)
 *
 *       Length: (Range FFT size) x (size of uint16_t)
 *
 *       Value: Array of profile points at 0th Doppler (stationary objects).
 *       The points represent the sum of log2 magnitudes of received antennas
 *       expressed in Q9 format.
 *
 *      @subsection tlv3 Noise floor profile (sent only in TDM demo)
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_NOISE_PROFILE)
 *
 *       Length: (Range FFT size) x (size of uint16_t)
 *
 *       Value: This is the same format as range profile but the profile
 *       is at the maximum Doppler bin (maximum speed
 *       objects). In general for stationary scene, there would be no objects or clutter at
 *       maximum speed so the range profile at such speed represents the
 *       receiver noise floor.
 *
 *      @subsection tlv4 Azimuth static heatmap (sent only in TDM demo)
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP)
 *
 *       Length: (Range FFT size) x (Number of virtual antennas) (size of @ref cmplx16ImRe_t_)
 *
 *       Value: Array @ref DPU_AoAProcHWA_HW_Resources::azimuthStaticHeatMap. The antenna data
 *       are complex symbols, with imaginary first and real second in the following order:\n
 *       @verbatim
             Imag(ant 0, range 0), Real(ant 0, range 0),...,Imag(ant N-1, range 0),Real(ant N-1, range 0)
             ...
             Imag(ant 0, range R-1), Real(ant 0, range R-1),...,Imag(ant N-1, range R-1),Real(ant N-1, range R-1)
         @endverbatim
 *       Based on this data the static azimuth heat map is constructed by the GUI
 *       running on the host.
 *
 *      @subsection tlv5 Range/Doppler heatmap
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP)
 *
 *       Length: (Range FFT size) x (Doppler FFT size) (size of uint16_t)
 *
 *       Value: Detection matrix @ref DPIF_DetMatrix::data.
 *       The order is : \n
 *       @verbatim
            X(range bin 0, Doppler bin 0),...,X(range bin 0, Doppler bin D-1),
            ...
            X(range bin R-1, Doppler bin 0),...,X(range bin R-1, Doppler bin D-1)
         @endverbatim
 *
 *      @subsection tlv6 Stats information
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_STATS )
 *
 *       Length: (size of @ref MmwDemo_output_message_stats_t)
 *
 *       Value: Timing information as per @ref MmwDemo_output_message_stats_t.
 *       See timing diagram below related to the stats.
 *
 *      @image html processing_timing.png "Processing timing"
 *
 *       Note:
 *       -#  The @ref MmwDemo_output_message_stats_t::interChirpProcessingMargin is not
 *           computed (it is always set to 0). This is because there is no CPU involvement
 *           in the 1D processing (only HWA and EDMA are involved), and it is not possible to
 *           know how much margin is there in chirp processing without CPU being notified
 *           at every chirp when processing begins (chirp event) and when the HWA-EDMA
 *           computation ends. The CPU is intentionally kept free during 1D processing
 *           because a real application may use this time for doing some post-processing
 *           algorithm execution.
 *       -#  While the
 *           @ref MmwDemo_output_message_stats_t::interFrameProcessingTime reported
 *           will be of the current sub-frame/frame,
 *           the @ref MmwDemo_output_message_stats_t::interFrameProcessingMargin and
 *           @ref MmwDemo_output_message_stats_t::transmitOutputTime
 *           will be of the previous sub-frame (of the same
 *           @ref MmwDemo_output_message_header_t::subFrameNumber as that of the
 *           current sub-frame) or of the previous frame.
 *       -#  The @ref MmwDemo_output_message_stats_t::interFrameProcessingMargin excludes
 *           the UART transmission time (available as
 *           @ref MmwDemo_output_message_stats_t::transmitOutputTime). This is done
 *           intentionally to inform the user of a genuine inter-frame processing margin
 *           without being influenced by a slow transport like UART, this transport time
 *           can be significantly longer for example when streaming out debug information like
 *           heat maps. Also, in a real product deployment, higher speed interfaces (e.g LVDS)
 *           are likely to be used instead of UART. User can calculate the margin
 *           that includes transport overhead (say to determine the max frame rate
 *           that a particular demo configuration will allow) using the stats
 *           because they also contain the UART transmission time.
 *
 *      The CLI command \"guMonitor\" specifies which TLV element will
 *      be sent out within the output packet. The arguments of the CLI command are stored
 *      in the structure @ref MmwDemo_GuiMonSel_t.
 *
 *      @subsection tlv7 Side information of detected objects (sent only in TDM demo)
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO)
 *
 *       Length: (Number of detected objects) x (size of @ref DPIF_PointCloudSideInfo_t)
 *
 *       Value: Array of detected objects side information. The side information
 *       of each detected object is as per the structure @ref DPIF_PointCloudSideInfo_t).
 *       When the number of detected objects is zero, this TLV item is not sent.
 *
 *      @subsection tlv9 Temperature Stats
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS)
 *
 *       Length: (size of @ref MmwDemo_temperatureStats_t)
 *
 *       Value: Structure of detailed temperature report as obtained from Radar front end.
 *       @ref MmwDemo_temperatureStats_t::tempReportValid is set to return value of
 *       rlRfGetTemperatureReport. If @ref MmwDemo_temperatureStats_t::tempReportValid is 0,
 *       values in @ref MmwDemo_temperatureStats_t::temperatureReport are valid else they should
 *       be ignored. This TLV is sent along with Stats TLV described in @ref tlv6
 *
 *  @section Calibration_section Range Bias (only supported in TDM) and Rx Channel Gain/Phase Measurement and Compensation
 *
 *     Because of imperfections in antenna layouts on the board, RF delays in SOC, etc,
 *     there is need to calibrate the sensor to compensate for bias in the range estimation and
 *     receive channel gain and phase imperfections. The following figure illustrates
 *     the calibration procedure.
 *
 *     @anchor Figure_calibration_ladder_diagram
 *     @image html calibration_ladder_diagram.png "Calibration procedure ladder diagram"
 *
 *      The calibration procedure includes the following steps:
 *     -# Set a strong target like corner reflector at the distance of X meter
 *     (X less than 50 cm is not recommended) at boresight.
 *     -# Set the following command
 *     in the configuration profile in .../profiles/profile_calibration.cfg,
 *     to reflect the position X as follows:
 *     @verbatim
       measureRangeBiasAndRxChanPhase 1 X D
       @endverbatim
 *     where D (in meters) is the distance of window around X where the peak will be searched.
 *     The purpose of the search window is to allow the test environment from not being overly constrained
 *     say because it may not be possible to clear it of all reflectors that may be stronger than the one used
 *     for calibration. The window size is recommended to be at least the distance equivalent of a few range bins.
 *     One range bin for the calibration profile (profile_calibration.cfg) is about 5 cm.
 *     The first argument "1" is to enable the measurement. The stated configuration
 *     profile (.cfg) must be used otherwise the calibration may not work as expected
 *     (this profile ensures all transmit and receive antennas are engaged among
 *     other things needed for calibration).
 *     -# Start the sensor with the configuration file.
 *     -# In the configuration file, the measurement is enabled because of which
 *     the DPC will be configured to perform the measurement and generate
 *     the measurement result (@ref DPU_AoAProc_compRxChannelBiasCfg_t in TDM and @ref Measure_compRxChannelBiasCfg_t in DDM) in its
 *     result structure (@ref DPC_ObjectDetection_ExecuteResult_t::compRxChanBiasMeasurement),
 *     the measurement results are written out on the CLI port (@ref MmwDemo_measurementResultOutput)
 *     in the format below:
 *     @verbatim
       TDM: compRangeBiasAndRxChanPhase <rangeBias> <Re(0,0)> <Im(0,0)> <Re(0,1)> <Im(0,1)> ... <Re(0,R-1)> <Im(0,R-1)> <Re(1,0)> <Im(1,0)> ... <Re(T-1,R-1)> <Im(T-1,R-1)>
       @endverbatim
 *     \n
 *
 *     @verbatim
       DDM: compRxChanPhase <Im(0,0)> <Re0,0)> <Im(0,1)> <Re(0,1)> ... <Im(0,R-1)> <Re(0,R-1)> <Im(1,0)> <Re(1,0)> ... <Im(T-1,R-1)> <Re(T-1,R-1)>
       where TX antenna order is such that azimuth Tx antennas are followed by the elevation TX antennas.
       @endverbatim

       For details of how DPC performs the measurement, see the DPC documentation.
 *     -# The command printed out on the CLI now can be copied and pasted in any
 *     configuration file for correction purposes. This configuration will be
 *     passed to the DPC for the purpose  of applying compensation during angle
 *     computation, the details of this can be seen in the DPC documentation.
 *     If compensation is not desired,
 *     the following command should be given
 *     @verbatim
       TDM: compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0
       @endverbatim
 *     \n
 *
 *     @verbatim
       DDM: antennaCalibParams 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1
       @endverbatim
 *     Above sets the range bias to 0 and
 *     the phase coefficients to unity so that there is no correction. Note
 *     the two commands must always be given in any configuration file, typically
 *     the measure command will be disabled when the correction command is the
 *     desired one. It is recommended to use the measureRangeBiasAndRxChanPhase command only in the
 *     in the frame based chirps (dfeDataOutput=1) mode.
 *
 *  @section LVDSStreamingNotes Streaming data over LVDS
 *
 *    The LVDS streaming feature enables the streaming of HW data (ADC data)
 *    and/or user specific SW data through LVDS interface.
 *    The streaming is done mostly by the CBUFF and EDMA peripherals with minimal CPU intervention.
 *    The streaming is configured through the @ref MmwDemo_LvdsStreamCfg_t CLI command which
 *    allows control of HSI header, enable/disable of HW and SW data and data format choice for the HW data. Note
 *    that only HW data without HSI header is supported as of now.
 *    The choices for data formats for HW data are:
 *      - @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED
 *      - @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_ADC
 *
 *    When HW data LVDS streaming is enabled, the ADC data is streamed per
 *    chirp on every chirp event.
 *    -# For HW data, the inter-chirp duration should be sufficient to stream out the desired amount
 *       of data. For example, if the HW data-format is ADC and HSI header is enabled (note: HSI
 *       header sending is not enabled in this demo),
 *       then the total amount of data generated per chirp
 *       is:\n
 *        (numAdcSamples * numRxChannels * 4 (size of complex sample) +
 *       52 [sizeof(HSIDataCardHeader_t) + sizeof(HSISDKHeader_t)] )
 *       rounded up to multiples of 256 [=sizeof(HSIHeader_t)] bytes.\n
 *       The chirp time Tc in us = idle time + ramp end time in the profile configuration.
 *       For n-lane LVDS with each lane at a maximum of B Mbps,\n
 *       maximum number of bytes that can be send per chirp = Tc * n * B / 8 which
 *       should be greater than the total amount of data generated per chirp i.e\n
 *       Tc * n * B / 8 >= round-up(numAdcSamples * numRxChannels * 4 + 52, 256). \n
 *       E.g if n = 2, B = 600 Mbps, idle time = 7 us, ramp end time = 44 us, numAdcSamples = 512,
 *       numRxChannels = 4, then 7650 >= 8448 is violated so this configuration will not work.
 *       If the idle-time is doubled in the above example, then we have 8700 > 8448, so
 *       this configuration will work.
 *    -# The total amount of data to be transmitted in a HW or SW packet must be greater than the
 *       minimum required by CBUFF, which is 64 bytes or 32 CBUFF Units (this is the definition
 *       CBUFF_MIN_TRANSFER_SIZE_CBUFF_UNITS in the CBUFF driver implementation).
 *       If this threshold condition is violated, the CBUFF driver will return an error during
 *       configuration and the demo will generate a fatal exception as a result.
 *       When HSI header is enabled, the total transfer size is ensured to be at least
 *       256 bytes, which satisfies the minimum. If HSI header is disabled, for the HW session,
 *       this means that numAdcSamples * numRxChannels * 4 >= 64. Although mmwavelink
 *       allows minimum number of ADC samples to be 2, the demo is supported
 *       for numAdcSamples >= 64. So HSI header is not required to be enabled for HW only case.
 *
 *   @subsection lvdsImpl Implementation Notes
 *
 *    -# The LVDS implementation is mostly present in mmw_lvds_stream.h and
 *      mmw_lvds_stream.c with calls in mss_main.c. Additionally HSI
 *      clock initialization is done at first time sensor start using @ref MmwDemo_mssSetHsiClk. Also see the
 *      @ MmwDemo_BoardInit function for register configuration related to HSI clock.
 *    -# EDMA channel resources for CBUFF/LVDS are in the global resource file
 *      (mmw_res.h, see @ref resourceAlloc) along with other EDMA resource allocation.
 *      The user data header and two user payloads are configured as three user buffers in the CBUFF driver.
 *      Hence SW allocation for EDMA provides for three sets of EDMA resources as seen in
 *      the SW part (swSessionEDMAChannelTable[.]) of @ref MmwDemo_LVDSStream_EDMAInit.
 *    -# Although the CBUFF driver is configured for two sessions (hw and sw),
 *      at any time only one can be active. So depending on the LVDS CLI configuration
 *      and whether advanced frame or not, there is logic to activate/deactivate
 *      HW and SW sessions as necessary.
 *    -# The CBUFF session (HW/SW) configure-create and delete
 *      depends on whether or not re-configuration is required after the
 *      first time configuration.
 *      -# For HW session, re-configuration is done during sub-frame switching to
 *        re-configure for the next sub-frame but when there is no advanced frame
 *        (number of sub-frames = 1), the HW configuration does not need to
 *        change so HW session does not need to be re-created.
 *    -# SW-trigger-based streaming is not supported in the current release.
 *
 *    The following figure shows a timing diagram for the LVDS streaming
 *    (the figure is not to scale as actual durations will vary based on configuration).
 *    Note: SW streaming is not supported in the current version of the demo.
 *
 *      @image html lvdstiming.png "LVDS timing diagram"
 *
 *  @section EnetStreamingNotes Streaming data over Ethernet
 *
 *    This demo supports a simple use case of transferring detected objects data over Ethernet using
 *    TCP protocol and LwIP stack. The data transfer is done using a client-server based architecture
 *    with the EVM acting as the client and the PC acting as a server. This utility is a modified
 *    example of the TCPECHO app that comes packages as a part of LwIP in the PDK.
 *    For connection and usage details, please refer to the MMWAVE SDK User Guide.
 *
 *   @subsection enetImpl Implementation Notes
 *
 *    -# The enet streaming utility comes as a part of the files enet_cpswconfighandler.c, enet_stream.c
 *       enet_tcpclient.c and tcpserver.py files.
 *    -# enetTask in enet_stream.c is the main task that initializes all the components and tasks, and has an IP
 *       address assigned to the device through DHCP.
 *    -# Once the IP is assigned to the device, the user can run the server application tcpserver.py and wait for
 *       connection establishment.
 *    -# Command queryLocalIp can be used through CLI to return the local IP (IP address acquired by EVM). Refer to the
 *       User Guide for more details.
 *    -# The user will send the local IP through the enetStreamCfg command using CLI. Refer to the
 *       User Guide for more details. Once this configuration is done, the semaphore EnetCfgDoneSemHandle
 *       will be posted, and connection will be established (see tcpclient.c).
 *    -# The user will be able to see the object data getting printed on the console of tcpserver.py.
 *    -# Note that the communication port is pre-configured as "7".
 *    -# It must be noted that the LwIP stack requires 650 kB of extra memory (L3 RAM), and care must be taken to
 *       ensure that the demo L3 requirements do not result in higher L3 memory usage than what is available.
 *
 *  @section bypassCLI How to bypass CLI
 *
 *    Re-implement the file mmw_cli.c as follows:
 *
 *    -# @ref MmwDemo_CLIInit should just create a task with input taskPriority.
 *       Lets say the task is called "MmwDemo_sensorConfig_task".
 *    -# All other functions are not needed
 *    -# Implement the MmwDemo_sensorConfig_task as follows:
 *       - Fill gMmw_MCB.cfg.openCfg
 *       - Fill gMmw_MCB.cfg.ctrlCfg
 *       - Add profiles and chirps using @ref MMWave_addProfile and @ref MMWave_addChirp functions
 *       - Call @ref MmwDemo_CfgUpdate for every offset in @ref configStoreOffsets
 *         (MMWDEMO_xxx_OFFSET in mmw_mss.h)
 *       - Fill gMmw_MCB.objDetCommonCfg.preStartCommonCfg
 *       - Call @ref MmwDemo_openSensor
 *       - Call @ref MmwDemo_configSensor
 *       - Call @ref MmwDemo_startSensor (One can use helper function
 *         @ref MmwDemo_isAllCfgInPendingState to know if all dynamic config was provided)
 *    -# The user can also use the CLI_BYPASS API in the CLI library to directly bypass the CLI
 *       command send over UART.
 *
 *  @section resourceAlloc Hardware Resource Allocation
 *    The Object Detection DPC needs to configure the DPUs hardware resources (HWA, EDMA).
 *    Even though the hardware resources currently are only required to be allocated
 *    for this one and only DPC in the system, the resource partitioning is shown to be in
 *    the ownership of the demo. This is to illustrate the general
 *    case of resource allocation across more than one DPCs and/or demo's own
 *    processing that is post-DPC processing. This partitioning
 *    can be seen in the mmw_res.h file. This file is passed as a compiler command line
 *    define @verbatim "--define=APP_RESOURCE_FILE="<ti/demo/awr294x/mmw/mmw_resDDM.h>" @endverbatim
 *    or @verbatim "--define=APP_RESOURCE_FILE="<ti/demo/awr294x/mmw/mmw_resTDM.h>" @endverbatim
 *    in mmw_mss.mak and mmw_dss.mak when building the DPC sources as part of building the demo application and
 *    is referred in object detection DPC sources where needed as
 *    @verbatim #include APP_RESOURCE_FILE @endverbatim
 *
 *  @section demoDesignNotes Design Notes
 *
 *  Due to the limitation of DPM local queue size, for certain DPM functions such as @ref DPM_start, @ref DPM_stop and
 *  some of the DPC control through @ref DPM_ioctl, semaphores are used to sync between calling task and function
 *  @ref MmwDemo_DPC_ObjectDetection_reportFxn. So that it won't cause DPM crash because of running out of
 *  DPM local queues. The following diagram demonstrates the example calling flow for blocking @ref DPM_ioctl() function call.
 *  Non-blocking @ref DPM_ioctl is also shown for comparison.
 *
 *  @image html dpm_ioctl_handling.png "DPM_ioctl calling flow"
 *
 *  There are DPM report functions on both MSS and DSS for the same DPM_Report. However the sequence is not guaranteed between the
 *  two cores.
 *
 *  @section memoryUsage Memory Usage
 *  @subsection memUsageSummary Memory usage summary
 *    Please refer to the respective .map files in the demo folder for details on memory usage.
 *
 *  @section error Note on Error Codes
 *
 *    ------------------------------
 *    When demo runs into error conditions, an error code will be generated and printed out.
 *    Error code is defined as a negative integer. It comes from the following categories:
 *    - Drivers
 *    - Control modules
 *    - Data Processing Chain
 *    - Data Processing Unit
 *    - Demo
 *
 *    The error code is defined as (Module error code base - Module specific error code).\n
 *    The base error code for the above modules can be found in @ref mmwave_error.h\n
 *    The base error code for DPC and DPU can be found in @ref dp_error.h
 *
 *    Module specific error code is specified in the module's header file.
 *    Examples:
 *       - UART driver error code is defined in uart.h
 *       - DPC error code is defined in the dpc used in demo
 *
 *  @subsection mmwave_error mmWave module Error Code
 *    Error code from mmWave module is encoded in the following manner:
 *
 *    Bits(31::16)  |  Bits(15::2)   | Bits (1::0)
 *    :-------------|:----------------|:-------------:
 *    mmwave  error  | Subsystem error   | error level
 *
 *    - mmwave error is defined in mmwave.h \n
 *    - Subsystem error is returned from sub-system such as mmwavelink and mailbox driver. \n
 *    - Error level is referred as WARNING level and ERROR level.
 *    - mmWave exposes an API - MMWave_decodeError() that can be used in demo to decode error code
 *
 *  @subsection mmwave_error_example Example
 *    - Here is an example on how to parse the error code - "-40111"\n
 *        -# The error code is from module with error base "-40000", which indicates it is DPC error.\n
 *        -# By referring to @ref dp_error.h, base "-40100" is from HWA based objectdetection DPC.\n
 *        -# Then find the error code in objectdetection.h for error(-11) as DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE
 *
 *    - Another example is with mmWave control module: - "mmWave Config failed [Error code: -3109 Subsystem: 71]"\n
 *        -# The above message indicates the error is from module(-3100 ->mmwave) with error -9(MMWAVE_ECHIRPCFG)\n
 *        -# The subsystem(mmwavelink) error is 71(RL_RET_CODE_CHIRP_TX_ENA_1INVAL_IN) which can be found in mmwavelink.h
 *
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* MCU+SDK include files. */
#include <drivers/uart.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CycleCounterP.h>

/* Sysconfig include files*/
#include "mssgenerated/ti_drivers_config.h"
#include "mssgenerated/ti_board_config.h"
#include "mssgenerated/ti_drivers_open_close.h"
#include "mssgenerated/ti_board_open_close.h"

/* FreeRTOS include files. */
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>
#include "FreeRTOS.h"
#include "task.h"

/* mmWave SDK Include Files: */
#include <ti/common/syscommon.h>
#include <ti/common/mmwavesdk_version.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/mathutils/mathutils.h>
#include <ti/utils/testlogger/logger.h>
#include <ti/board/antenna_geometry.h>
#include <ti/datapath/dpif/dpif_pointcloud.h>

/* Demo Include Files */
#include "../common/mmw_config.h"
#include "../common/mmw_output.h"
#include "../../utils/mmwdemo_rfparser.h"
#include "../../utils/mmwdemo_adcconfig.h"
#include "../../utils/mmwdemo_monitor.h"
#include "../../utils/enet_stream.h"
#include "../../utils/mmwdemo_flash.h"
#include "../mmw_resDDM.h"
#include "mmw_mss.h"
#include "mss_tracker.h"
#include "../common/mmw_common.h"
#ifdef CAN_STREAM
#include "mss_mcan.h"
#endif
#include "../common/chirp_design_HECR.h"

/* Profiler Include Files */
#include <kernel/dpl/CycleCounterP.h>

/**
 * @brief Task Priority settings:
 * Mmwave task is at higher priority because of potential async messages from BSS
 * that need quick action in real-time.
 *
 * CLI task must be at a lower priority than object detection
 * dpm task priority because the dynamic CLI command handling in the objection detection
 * dpm task assumes CLI task is held back during this processing. The alternative
 * is to use a semaphore between the two tasks.
 */

/* Demo tasks should have priority higher than enet/lwip tasks */
#ifdef ENET_STREAM
#define MMWDEMO_CLI_TASK_PRIORITY         7
#define MMWDEMO_UART_EXPORT_TASK_PRIORITY 8
#define MMWDEMO_MMWAVE_CTRL_TASK_PRIORITY 9
#define MMWDEMO_MMWAVE_ENET_TASK_PRIORITY 1
#else
#define MMWDEMO_UART_EXPORT_TASK_PRIORITY 3
#define MMWDEMO_MMWAVE_CTRL_TASK_PRIORITY 5
#endif


#define DPC_OBJDET_INSTANCEID (0xFEEDFEED)
extern SemaphoreP_Object objDataSemaphoreHandle;
extern uint16_t          gRemoteCoreEndPt[CSL_CORE_ID_MAX];

/* These address offsets are in bytes, when configure address offset in hardware,
   these values will be converted to number of 128bits
   Buffer at offset 0x0U is reserved by BSS, hence offset starts from 0x200
 */
#define MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET 0x200U
#define MMW_DEMO_CQ_RXSAT_ADDR_OFFSET  0x400U

/* CQ data is at 16 bytes alignment for mulitple chirps */
#define MMW_DEMO_CQ_DATA_ALIGNMENT 16U


#define MAX_MOD_FREQ_DIVIDER_MANTISSA 127U

/* Variable to store detected object data for ethernet streaming */
MmwDemo_enetStreamObjData gEnetStreamObjData;
/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/
/* FreeRTOS Task declarations. */
#define MMWDEMO_INIT_TASK_PRI (1U)

#define MMWDEMO_INIT_TASK_STACK_SIZE             (4 * 1024U)
#define MMWDEMO_MMWAVE_CTRL_TASK_STACK_SIZE      (3 * 1024U)
#define MMWDEMO_UART_DATA_EXPORT_TASK_STACK_SIZE (4 * 1024U)
#ifdef ENET_STREAM
#define MMWDEMO_MMWAVE_ENET_TASK_STACK_SIZE (4 * 1024U)
#endif

/* Applictaon task stack variables */
StackType_t gAppMainTskStack[MMWDEMO_INIT_TASK_STACK_SIZE] __attribute__((aligned(32)));
StackType_t gMmwCtrlTskStack[MMWDEMO_MMWAVE_CTRL_TASK_STACK_SIZE] __attribute__((aligned(32)));
StackType_t gUartTskStack[MMWDEMO_UART_DATA_EXPORT_TASK_STACK_SIZE] __attribute__((aligned(32)));
#ifdef ENET_STREAM
StackType_t gMmwEnetTskStack[MMWDEMO_MMWAVE_ENET_TASK_STACK_SIZE] __attribute__((aligned(32)));
#endif

MmwDemo_tracker_out      gTargetOutput[GTRACK_NUM_TRACKS_MAX] __attribute__((aligned(32)));
DPIF_PointCloudCartesian gDPIFPointCloud[GTRACK_NUM_POINTS_MAX] __attribute__((aligned(32)));
DPIF_PointCloudSideInfo  gDPIFPointCloudSideInfo[GTRACK_NUM_POINTS_MAX] __attribute__((aligned(32)));
uint8_t                  gRansacFilterMask[GTRACK_NUM_POINTS_MAX] __attribute__((aligned(32)));
pointCloudRadialCompact  gPointCloudRadialCompactList[GTRACK_NUM_POINTS_MAX] __attribute__((aligned(32)));

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_MSS_MCB gMmw_MCB;
uint32_t        gMssDPCState = MMWDEMO_WAIT_FRM_DSS_CFG;

/* Calibration Data Save/Restore defines */
#define MMWDEMO_CALIB_FLASH_SIZE  4096
#define MMWDEMO_CALIB_STORE_MAGIC (0x7CB28DF9U)

// check
// MmwDemo_calibData gCalibDataStorage __attribute__((aligned(8)));

static void MmwDemo_checkEdmaErrors(void);

MmwDemo_calibData gCalibDataStoragetoDss __attribute__((aligned(8), section(".l3ram")));
MmwDemo_calibData gCalibDataStoragefromDss __attribute__((aligned(8)));

SemaphoreP_Object gCalibSaveSemHandle;

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

// extern void MmwDemo_CLIInit(uint8_t taskPriority);

#ifdef MMWDEMO_DDM
extern uint8_t                MmwDemo_RFParserPhaseShiftOrder[SYS_COMMON_NUM_TX_ANTENNAS];
extern MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg;
#endif

#ifdef TRACKER_ON_MSS
extern int32_t MmwDemo_trackerConfig();
#endif

#ifdef CAN_STREAM
#define MMWDEMO_HEADER          0xAU
#define MMWDEMO_PADDING         0xBU
#define CAN_MESSAGE_MMWDEMO_MAX 0x8U /*(messge type (6) + header (1) + padding(1))*/

#define CAN_MESSAGE_MMWDEMO_HEADER  0xC1
#define CAN_MESSAGE_MMWDEMO_PADDING 0xB1

#define CAN_MSGOBJ_HEADER  0xAU
#define CAN_MSGOBJ_PADDING 0xBU

typedef enum mmwDemo_can_message_type_e
{
    /*! @brief   List of detected points */
    CAN_MESSAGE_MMWDEMO_DETECTED_POINTS = 0xD1,

    /*! @brief   Range profile */
    CAN_MESSAGE_MMWDEMO_RANGE_PROFILE,

    /*! @brief   Noise floor profile */
    CAN_MESSAGE_MMWDEMO_NOISE_PROFILE,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    CAN_MESSAGE_MMWDEMO_AZIMUT_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    CAN_MESSAGE_MMWDEMO_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Stats information */
    CAN_MESSAGE_MMWDEMO_STATS,

    /*! @brief   List of detected points */
    CAN_MESSAGE_MMWDEMO_DETECTED_POINTS_SIDE_INFO,

    /*! @brief   Samples to calculate static azimuth/elevation heatmap,
                 (all virtual antennas exported) - unused in this demo */
    CAN_MESSAGE_MMWDEMO_AZIMUT_ELEVATION_STATIC_HEAT_MAP,

    /*! @brief   temperature stats from Radar front end */
    CAN_MESSAGE_MMWDEMO_TEMPERATURE_STATS,

    /*! @brief   tracker TLV */
    CAN_MESSAGE_MMWDEMO_TRACKER,

    /*! @brief   TLV for ransac filter mask */
    CAN_MESSAGE_MMWDEMO_RANSAC_FILTER_MASK,

    /*! @brief   TLV for compact pointcloud */
    CAN_MESSAGE_MMWDEMO_DETECTED_POINTS_COMPACT,

    /*! @brief   TLV for compact SNR list */
    CAN_MESSAGE_MMWDEMO_OUTPUT_SNR

} mmwDemo_can_message_type;
#endif
/**************************************************************************
 ************************* Millimeter Wave Demo Functions prototype *************
 **************************************************************************/

/* MMW demo functions for datapath operation */
void        MmwDemo_handleObjectDetResult(void);
void        MmwDemo_DPC_ObjectDetection_reportFxn(void *data, uint16_t dataLen);
static void MmwDemo_transmitProcessedOutput(
    UART_Handle                        uartHandle,
    DPC_ObjectDetection_ExecuteResult *result,
    DPIF_PointCloudCartesian          *objOut,
    DPIF_PointCloudSideInfo           *objOutSideInfo,
    pointCloudRadialCompact           *pointCloudRadialCompactList,
    MmwDemo_output_message_stats      *timingInfo,
    MmwDemo_tracker_out               *targetOut,
    size_t                             tNum,
    uint8_t                           *ransacFilterMask);

static void MmwDemo_measurementResultOutput(void *compRxChanCfg);

int32_t MmwDemo_DPM_ioctl_blocking(
    DPM_Handle handle,
    uint32_t   cmd,
    void      *arg,
    uint32_t   argLen);

/* Mmwave demo init functions */
void MmwDemo_initTask(void *args);
// static void MmwDemo_platformInit(MmwDemo_platformCfg *config);
static bool MmwDemo_BoardInit(void);

/* Calibration save/restore APIs */
static int32_t MmwDemo_calibInit(void);
int32_t        MmwDemo_calibSave(MmwDemo_calibDataHeader *ptrCalibDataHdr, MmwDemo_calibData *ptrCalibrationData);
int32_t        MmwDemo_calibRestore(MmwDemo_calibData *calibrationData);
static int32_t MmwDemo_calibDSSlinkRestore(void);
static int32_t MmwDemo_calibDSSlinkSave(void);

#ifdef CAN_STREAM
static mmwDemo_can_message_type Get_CanMessageIdentifier(MmwDemo_output_message_type type);
#endif

volatile uint32_t transmitStartTime = 0;
/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      Send assert information through CLI.
 */
void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression)
    {
        DebugP_log("Exception: %s, line %d.\n", file, line);
    }
}


/**************************************************************************
 ******************** Millimeter Wave Demo Results Transmit Functions *************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Sends the calibration Range Bias (TDM) and Rx Channel Gain/Phase Measurement
 *      and Compensation info through CLI
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_measurementResultOutput(void *compRxChanCfg)
{
    int32_t                       i;
    Measure_compRxChannelBiasCfg *result = (Measure_compRxChannelBiasCfg *)compRxChanCfg;
    // DebugP_log("compRxChanPhase (Im-Re) \n");

    /* Send the received DSS calibration info through CLI */
    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        DebugP_log("%.5f, ", (float)result->rxChPhaseComp[i].imag / 16384.);
        DebugP_log("%.5f, ", (float)result->rxChPhaseComp[i].real / 16384.);
    }
    DebugP_log("\n");
}


/** @brief Transmits detection data over UART
 *
 *    The following data is transmitted:
 *    1. Header (size = 32bytes), including "Magic word", (size = 8 bytes)
 *       and including the number of TLV items
 *    TLV Items:
 *    2. If detectedObjects flag is 1 or 2, DPIF_PointCloudCartesian structure containing
 *       X,Y,Z location and velocity for detected objects,
 *       size = sizeof(DPIF_PointCloudCartesian) * number of detected objects
 *    3. If detectedObjects flag is 1, DPIF_PointCloudSideInfo structure containing SNR
 *       and noise for detected objects,
 *       size = sizeof(DPIF_PointCloudCartesian) * number of detected objects
 *    4. If logMagRange flag is set,  rangeProfile,
 *       size = number of range bins * sizeof(uint16_t)
 *    5. If noiseProfile flag is set,  noiseProfile,
 *       size = number of range bins * sizeof(uint16_t)
 *    6. If rangeAzimuthHeatMap flag is set, the zero Doppler column of the
 *       range cubed matrix, size = number of Rx Azimuth virtual antennas *
 *       number of chirps per frame * sizeof(uint32_t)
 *    7. If rangeDopplerHeatMap flag is set, the log magnitude range-Doppler matrix,
 *       size = number of range bins * number of Doppler bins * sizeof(uint16_t)
 *    8. If statsInfo flag is set, the stats information
 *   @param[in] uartHandle   UART driver handle
 *   @param[in] result       Pointer to result from object detection DPC processing
 *   @param[in] timingInfo   Pointer to timing information provided from core that runs data path
 */
static void MmwDemo_transmitProcessedOutput(
    UART_Handle                        uartHandle,
    DPC_ObjectDetection_ExecuteResult *result,
    DPIF_PointCloudCartesian          *objOut,
    DPIF_PointCloudSideInfo           *objOutSideInfo,
    pointCloudRadialCompact           *pointCloudRadialCompactList,
    MmwDemo_output_message_stats      *timingInfo,
    MmwDemo_tracker_out               *targetOutput,
    size_t                             tNum,
    uint8_t                           *ransacFilterMask)
{
    MmwDemo_output_message_header header;
    MmwDemo_GuiMonSel            *pGuiMonSel;
    uint32_t                      tlvIdx = 0;
    uint32_t                      index;
    uint32_t                      numPaddingBytes;
    uint32_t                      packetLen;
    uint8_t                       padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
    MmwDemo_output_message_tl     tl[MMWDEMO_OUTPUT_MSG_MAX];
    uint16_t                     *detMatrix = (uint16_t *)result->detMatrix.data;
    DPC_ObjectDetection_Stats    *stats;
    UART_Transaction              trans;
#ifdef CAN_STREAM
    uint32_t numObjOut;
    uint32_t msgIdentifier;
#endif

    /* mmwavelink is running on DSS */
    uint16_t numDopFFTSubBins = gMmw_MCB.subFrameCfg[result->subFrameIdx].numDopplerBins / (mathUtils_countSetBits(gMmw_MCB.cfg.openCfg.chCfg.txChannelEn) + gMmw_MCB.numEmptySubBands);

    /* Get Gui Monitor configuration */
    pGuiMonSel = &gMmw_MCB.subFrameCfg[result->subFrameIdx].guiMonSel;

    UART_Transaction_init(&trans);

    /* Clear message header */
    memset((void *)&header, 0, sizeof(MmwDemo_output_message_header));

    /******************************************************************
       Send out data that is enabled, Since processing results are from DSP,
       address translation is needed for buffer pointers
    *******************************************************************/
    {
        detMatrix = (uint16_t *)AddrTranslateP_getLocalAddr((uint32_t)detMatrix);

        stats = (DPC_ObjectDetection_Stats *)AddrTranslateP_getLocalAddr((uint32_t)result->stats);
    }

    /* Header: */
#if defined(SOC_AWR2944)
    header.platform = 0x2944;
#elif defined(SOC_AWR2X44P)
    header.platform = 0x2E44;
#else
    header.platform = 0x2943;
#endif
    header.magicWord[0]   = 0x0102;
    header.magicWord[1]   = 0x0304;
    header.magicWord[2]   = 0x0506;
    header.magicWord[3]   = 0x0708;
    header.numDetectedObj = result->numObjOut;
    header.version        = MMWAVE_SDK_VERSION_BUILD |
        (MMWAVE_SDK_VERSION_BUGFIX << 8) |
        (MMWAVE_SDK_VERSION_MINOR << 16) |
        (MMWAVE_SDK_VERSION_MAJOR << 24);

    packetLen = sizeof(MmwDemo_output_message_header);
    if (((pGuiMonSel->detectedObjects == 1) || (pGuiMonSel->detectedObjects == 2)) &&
        (result->numObjOut > 0))
    {
        tl[tlvIdx].type   = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS;
        tl[tlvIdx].length = sizeof(DPIF_PointCloudCartesian) * result->numObjOut;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    /* Side info */
    if ((pGuiMonSel->detectedObjects == 1) && (result->numObjOut > 0))
    {
        tl[tlvIdx].type   = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO;
        tl[tlvIdx].length = sizeof(DPIF_PointCloudSideInfo) * result->numObjOut;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    /* Radial compact pointcloud */
    if ((pGuiMonSel->detectedObjects == 3) && (result->numObjOut > 0))
    {
        tl[tlvIdx].type   = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_COMPACT;
        tl[tlvIdx].length = sizeof(pointCloudRadialCompact) * result->numObjOut;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->logMagRange)
    {
        tl[tlvIdx].type   = MMWDEMO_OUTPUT_MSG_RANGE_PROFILE;
        tl[tlvIdx].length = sizeof(uint16_t) * gMmw_MCB.subFrameCfg[result->subFrameIdx].numRangeBins;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->noiseProfile)
    {
        tl[tlvIdx].type   = MMWDEMO_OUTPUT_MSG_NOISE_PROFILE;
        tl[tlvIdx].length = sizeof(uint16_t) * gMmw_MCB.subFrameCfg[result->subFrameIdx].numRangeBins;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->rangeDopplerHeatMap)
    {
        tl[tlvIdx].type   = MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP;
        tl[tlvIdx].length = gMmw_MCB.subFrameCfg[result->subFrameIdx].numRangeBins * numDopFFTSubBins * sizeof(uint16_t);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->statsInfo)
    {
        tl[tlvIdx].type   = MMWDEMO_OUTPUT_MSG_STATS;
        tl[tlvIdx].length = sizeof(MmwDemo_output_message_stats);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->trackers && tNum != 0)
    {
        // add tracker TLV
        tl[tlvIdx].type   = MMWDEMO_OUTPUT_MSG_TRACKER;
        tl[tlvIdx].length = tNum * sizeof(MmwDemo_tracker_out);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    if (pGuiMonSel->ransacFilterMask && gMmw_MCB.trackerCfg.ransacEnabled)
    {
        // add ransacFilterMask TLV
        tl[tlvIdx].type   = MMWDEMO_OUTPUT_MSG_RANSAC_FILTER_MASK;
        tl[tlvIdx].length = sizeof(uint8_t) * result->numObjOut;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }


    header.numTLVs = tlvIdx;
    /* Round up packet length to multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
    header.totalPacketLen = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN *
        ((packetLen + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - 1)) / MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
    header.timeCpuCycles  = 0; // Pmu_getCount(0);
    header.frameNumber    = stats->frameStartIntCounter;
    header.subFrameNumber = result->subFrameIdx;

    DebugP_logInfo("Platform = %d, Version = %d, NumObj = %d, numTLVs = %d\n", header.platform, header.version, header.numDetectedObj, header.numTLVs);

    CacheP_wbInv((void *)&header, sizeof(MmwDemo_output_message_header), CacheP_TYPE_ALLD);
#ifdef CAN_STREAM
    msgIdentifier = CAN_MESSAGE_MMWDEMO_HEADER;
    /**
     * Size of header is 40 Bytes but all the bytes can't be sent at a time.
     * Currently transferring 48 Bytes where last 8 Bytes are 0.
     * Another alternative is that 40 Bytes can be transferred as (20 Bytes + 20 Bytes), (32 Bytes + 8 Bytes), etc. Visualizer should be modified for this.
     */
    mcan_transfer(msgIdentifier, (uint8_t *)&header, sizeof(MmwDemo_output_message_header), MCAN_DATA_SIZE_48BYTES);
#else
    UART_Transaction_init(&trans);
    trans.buf   = (uint8_t *)&header;
    trans.count = sizeof(MmwDemo_output_message_header);
    UART_write(uartHandle, &trans);
#endif

    tlvIdx = 0;

    /* Send detected Objects */
    if ((pGuiMonSel->detectedObjects == 3) && (result->numObjOut > 0))
    {
#ifdef CAN_STREAM
        msgIdentifier = Get_CanMessageIdentifier((MmwDemo_output_message_type)tl[tlvIdx].type);
        mcan_transfer(msgIdentifier, (uint8_t *)&tl[tlvIdx], sizeof(MmwDemo_output_message_tl), MCAN_DATA_SIZE_8BYTES);
#else
        UART_Transaction_init(&trans);
        trans.buf   = &tl[tlvIdx];
        trans.count = sizeof(MmwDemo_output_message_tl);
        UART_write(uartHandle, &trans);
#endif

        /*Send array of objects */
#ifdef CAN_STREAM
        numObjOut = result->numObjOut;
        for (uint32_t i = 0; i < numObjOut; i++)
        {
            mcan_transfer(msgIdentifier, (uint8_t *)pointCloudRadialCompactList + i * sizeof(pointCloudRadialCompact), sizeof(pointCloudRadialCompact), MCAN_DATA_SIZE_8BYTES);
        }
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)pointCloudRadialCompactList;
        trans.count = sizeof(pointCloudRadialCompact) * result->numObjOut;
        UART_write(uartHandle, &trans);
#endif
        tlvIdx++;
    }

    /* Send detected Objects */
    if (((pGuiMonSel->detectedObjects == 1) || (pGuiMonSel->detectedObjects == 2)) &&
        (result->numObjOut > 0))
    {
#ifdef CAN_STREAM
        msgIdentifier = Get_CanMessageIdentifier((MmwDemo_output_message_type)tl[tlvIdx].type);
        mcan_transfer(msgIdentifier, (uint8_t *)&tl[tlvIdx], sizeof(MmwDemo_output_message_tl), MCAN_DATA_SIZE_8BYTES);
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)&tl[tlvIdx];
        trans.count = sizeof(MmwDemo_output_message_tl);
        UART_write(uartHandle, &trans);
#endif

        /*Send array of objects */
#ifdef CAN_STREAM
        numObjOut = result->numObjOut;
        for (uint32_t i = 0; i < numObjOut; i++)
        {
            mcan_transfer(msgIdentifier, (uint8_t *)objOut + i * sizeof(DPIF_PointCloudCartesian), sizeof(DPIF_PointCloudCartesian), MCAN_DATA_SIZE_16BYTES);
        }
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)objOut;
        trans.count = sizeof(DPIF_PointCloudCartesian) * result->numObjOut;
        UART_write(uartHandle, &trans);
#endif
        tlvIdx++;
    }

#ifdef ENET_STREAM
    if (gMmw_MCB.enetCfg.streamEnable)
    {
        gEnetStreamObjData.numObj = result->numObjOut;
        memcpy((void *)gEnetStreamObjData.objData, (void *)objOut, sizeof(DPIF_PointCloudCartesian) * gEnetStreamObjData.numObj);
        SemaphoreP_post(&objDataSemaphoreHandle);
    }
#endif

    /* Send detected Objects Side Info */
    if ((pGuiMonSel->detectedObjects == 1) && (result->numObjOut > 0))
    {
#ifdef CAN_STREAM
        msgIdentifier = Get_CanMessageIdentifier((MmwDemo_output_message_type)MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO);
        mcan_transfer(msgIdentifier, (uint8_t *)&tl[tlvIdx], sizeof(MmwDemo_output_message_tl), MCAN_DATA_SIZE_8BYTES);
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)&tl[tlvIdx];
        trans.count = sizeof(MmwDemo_output_message_tl);
        UART_write(uartHandle, &trans);
#endif

#ifdef CAN_STREAM
        for (uint32_t i = 0; i < numObjOut; i++)
        {
            mcan_transfer(msgIdentifier, (uint8_t *)objOutSideInfo + i * sizeof(DPIF_PointCloudSideInfo), sizeof(DPIF_PointCloudSideInfo), MCAN_DATA_SIZE_4BYTES);
        }
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)objOutSideInfo;
        trans.count = sizeof(DPIF_PointCloudSideInfo) * result->numObjOut;
        UART_write(uartHandle, &trans);
#endif
        tlvIdx++;
    }

    /* Send Range profile */
    if (pGuiMonSel->logMagRange)
    {
#ifdef CAN_STREAM
        msgIdentifier = Get_CanMessageIdentifier((MmwDemo_output_message_type)tl[tlvIdx].type);
        mcan_transfer(msgIdentifier, (uint8_t *)&tl[tlvIdx], sizeof(MmwDemo_output_message_tl), MCAN_DATA_SIZE_8BYTES);
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)&tl[tlvIdx];
        trans.count = sizeof(MmwDemo_output_message_tl);
        UART_write(uartHandle, &trans);
#endif

        for (index = 0; index < gMmw_MCB.subFrameCfg[result->subFrameIdx].numRangeBins; index++)
        {
#ifdef CAN_STREAM
            mcan_transfer(msgIdentifier, (uint8_t *)&detMatrix[index * numDopFFTSubBins], sizeof(uint16_t), MCAN_DATA_SIZE_2BYTES);
#else
            UART_Transaction_init(&trans);
            trans.buf   = (uint8_t *)&detMatrix[index * numDopFFTSubBins];
            trans.count = sizeof(uint16_t);
            UART_write(uartHandle, &trans);
#endif
        }
        tlvIdx++;
    }

    /* Send noise profile */
    if (pGuiMonSel->noiseProfile)
    {
        uint32_t maxDopIdx = gMmw_MCB.subFrameCfg[result->subFrameIdx].numRangeBins / 2 - 1;
#ifdef CAN_STREAM
        msgIdentifier = Get_CanMessageIdentifier((MmwDemo_output_message_type)tl[tlvIdx].type);
        mcan_transfer(msgIdentifier, (uint8_t *)&tl[tlvIdx], sizeof(MmwDemo_output_message_tl), MCAN_DATA_SIZE_8BYTES);
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)&tl[tlvIdx];
        trans.count = sizeof(MmwDemo_output_message_tl);
        UART_write(uartHandle, &trans);
#endif

        for (index = 0; index < gMmw_MCB.subFrameCfg[result->subFrameIdx].numRangeBins; index++)
        {
#ifdef CAN_STREAM
            mcan_transfer(msgIdentifier, (uint8_t *)&detMatrix[index * gMmw_MCB.subFrameCfg[result->subFrameIdx].numDopplerBins / (mathUtils_countSetBits(gMmw_MCB.cfg.openCfg.chCfg.txChannelEn) + gMmw_MCB.numEmptySubBands) + maxDopIdx], sizeof(uint16_t), MCAN_DATA_SIZE_2BYTES);
#else
            UART_Transaction_init(&trans);
            trans.buf   = (uint8_t *)&detMatrix[index * gMmw_MCB.subFrameCfg[result->subFrameIdx].numDopplerBins / (mathUtils_countSetBits(gMmw_MCB.cfg.openCfg.chCfg.txChannelEn) + gMmw_MCB.numEmptySubBands) + maxDopIdx];
            trans.count = sizeof(uint16_t);
            UART_write(uartHandle, &trans);
#endif
        }
        tlvIdx++;
    }

    /* Send data for range/Doppler heatmap */
    if (pGuiMonSel->rangeDopplerHeatMap == 1)
    {
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)&tl[tlvIdx];
        trans.count = sizeof(MmwDemo_output_message_tl);
        UART_write(uartHandle, &trans);

        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)detMatrix;
        trans.count = tl[tlvIdx].length;
        UART_write(uartHandle, &trans);
        tlvIdx++;
    }

    /* Send stats information */
    if (pGuiMonSel->statsInfo == 1)
    {
#ifdef CAN_STREAM
        msgIdentifier = Get_CanMessageIdentifier((MmwDemo_output_message_type)tl[tlvIdx].type);
        mcan_transfer(msgIdentifier, (uint8_t *)&tl[tlvIdx], sizeof(MmwDemo_output_message_tl), MCAN_DATA_SIZE_8BYTES);
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)&tl[tlvIdx];
        trans.count = sizeof(MmwDemo_output_message_tl);
        UART_write(uartHandle, &trans);
#endif

        /* Address translation is done when buffer is received*/
#ifdef CAN_STREAM
        mcan_transfer(msgIdentifier, (uint8_t *)timingInfo, tl[tlvIdx].length, MCAN_DATA_SIZE_24BYTES);
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)timingInfo;
        trans.count = tl[tlvIdx].length;
        UART_write(uartHandle, &trans);
#endif
        tlvIdx++;
    }

    /* Send data for tracker output */
    if (pGuiMonSel->trackers && (tNum != 0))
    {
#ifdef CAN_STREAM
        msgIdentifier = Get_CanMessageIdentifier((MmwDemo_output_message_type)tl[tlvIdx].type);
        mcan_transfer(msgIdentifier, (uint8_t *)&tl[tlvIdx], sizeof(MmwDemo_output_message_tl), MCAN_DATA_SIZE_8BYTES);
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)&tl[tlvIdx];
        trans.count = sizeof(MmwDemo_output_message_tl);
        UART_write(uartHandle, &trans);
#endif

#ifdef CAN_STREAM
        for (uint32_t i = 0; i < tNum; i++)
        {
            mcan_transfer(msgIdentifier, (uint8_t *)targetOutput + i * sizeof(MmwDemo_tracker_out), sizeof(MmwDemo_tracker_out), MCAN_DATA_SIZE_48BYTES);
        }
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)targetOutput;
        trans.count = tl[tlvIdx].length;
        UART_write(uartHandle, &trans);
#endif
        tlvIdx++;
    }

    /* Send data for ransac filter mask */
    if (pGuiMonSel->ransacFilterMask && gMmw_MCB.trackerCfg.ransacEnabled)
    {
#ifdef CAN_STREAM
        msgIdentifier = Get_CanMessageIdentifier((MmwDemo_output_message_type)tl[tlvIdx].type);
        mcan_transfer(msgIdentifier, (uint8_t *)&tl[tlvIdx], sizeof(MmwDemo_output_message_tl), MCAN_DATA_SIZE_8BYTES);
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)&tl[tlvIdx];
        trans.count = sizeof(MmwDemo_output_message_tl);
        UART_write(uartHandle, &trans);
#endif

#ifdef CAN_STREAM
        numObjOut = result->numObjOut;
        for (uint32_t i = 0; i < numObjOut; i++)
        {
            mcan_transfer(msgIdentifier, (uint8_t *)ransacFilterMask + i * sizeof(uint8_t), sizeof(uint8_t), MCAN_DATA_SIZE_1BYTES);
        }
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)ransacFilterMask;
        trans.count = tl[tlvIdx].length;
        UART_write(uartHandle, &trans);
#endif
        tlvIdx++;
    }

    /* Send padding bytes */
    numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (packetLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - 1));
    if (numPaddingBytes < MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
    {
#ifdef CAN_STREAM
        msgIdentifier = CAN_MESSAGE_MMWDEMO_PADDING;
        mcan_transfer(msgIdentifier, (uint8_t *)padding, numPaddingBytes, MCAN_DATA_SIZE_32BYTES);
#else
        UART_Transaction_init(&trans);
        trans.buf   = (uint8_t *)padding;
        trans.count = numPaddingBytes;
        UART_write(uartHandle, &trans);
#endif
    }
}

#ifdef CAN_STREAM
mmwDemo_can_message_type Get_CanMessageIdentifier(MmwDemo_output_message_type type)
{
    mmwDemo_can_message_type can_msgId;
    switch (type)
    {
        case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_DETECTED_POINTS;
            break;
        }
        case MMWDEMO_OUTPUT_MSG_RANGE_PROFILE:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_RANGE_PROFILE;
            break;
        }
        case MMWDEMO_OUTPUT_MSG_NOISE_PROFILE:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_NOISE_PROFILE;
            break;
        }
        case MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_AZIMUT_STATIC_HEAT_MAP;
            break;
        }
        case MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_RANGE_DOPPLER_HEAT_MAP;
            break;
        }
        case MMWDEMO_OUTPUT_MSG_STATS:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_STATS;
            break;
        }
        case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_DETECTED_POINTS_SIDE_INFO;
            break;
        }
        case MMWDEMO_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_AZIMUT_ELEVATION_STATIC_HEAT_MAP;
            break;
        }
        case MMWDEMO_OUTPUT_MSG_TEMPERATURE_STATS:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_TEMPERATURE_STATS;
            break;
        }
        case MMWDEMO_OUTPUT_MSG_TRACKER:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_TRACKER;
            break;
        }
        case MMWDEMO_OUTPUT_MSG_RANSAC_FILTER_MASK:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_RANSAC_FILTER_MASK;
            break;
        }
        case MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_COMPACT:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_DETECTED_POINTS_COMPACT;
            break;
        }
        default:
        {
            can_msgId = (mmwDemo_can_message_type)CAN_MESSAGE_MMWDEMO_MAX;
        }
    }
    return can_msgId;
}
#endif


/**************************************************************************
 ******************** Millimeter Wave Demo data path Functions *******************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Help function to make DPM_ioctl blocking until response is reported
 *
 *  @retval
 *      Success         -0
 *      Failed          <0
 */
int32_t MmwDemo_DPM_ioctl_blocking(
    DPM_Handle handle,
    uint32_t   cmd,
    void      *arg,
    uint32_t   argLen)
{
    int32_t retVal = 0;

    switch (cmd)
    {
        case DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG:
        case DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG:
            /* These are not supported in AWR2E44P as mmWaveLink runs only on DSS */
#if (defined(SOC_AWR2X44P) && defined(MMWAVELINK_ON_MSS))
            retVal = DPM_send(handle, (void *)arg, (uint16_t)argLen, CSL_CORE_ID_M4SS0_1);

#endif
            break;
#if (defined(SOC_AWR2X44P) && defined(MMWAVELINK_ON_MSS))
        case DPC_OBJDET_IOCTL__STATIC_PRE_START_ELEVEST_CFG:
            retVal = DPM_send(handle, (void *)arg, (uint16_t)argLen, CSL_CORE_ID_C66SS0);
            break;
#endif
        default:
            break;
    }


    if (retVal == 0)
    {
        /* Wait until ioctl completed */
        SemaphoreP_pend(&gMmw_MCB.DPMioctlSemHandle, SystemP_WAIT_FOREVER);
    }

    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      Utility function to convert the CFAR threshold
 *      from a CLI encoded dB value to a linear value
 *      as expected by the Doppler DPU
 *
 *  @param[in] codedCfarVal CFAR SNR threshold in dB as encoded in the CLI
 *
 *  @retval
 *      CFAR threshold in linear format
 */
static uint16_t MmwDemo_convertCfarToLinear(uint16_t codedCfarVal)
{
    uint16_t linearVal;
    float    dbVal, linVal;

    /* dbVal is a float value from 0-100dB. It needs to
    be converted to linear scale..
    First, recover float dbVal that was encoded in CLI. */
    dbVal = (float)(codedCfarVal / MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);

    /* Now convert it to linear value */
    linVal = log2(pow(10, (dbVal / 20.0))) * (1 << 11) + 0.5;

    linearVal = (uint16_t)linVal;
    return (linearVal);
}


/**
 *  @b Description
 *  @n
 *      DPM Registered Report Handler. The DPM Module uses this registered function to notify
 *      the application about DPM reports.
 *
 *  @param[in]  reportType
 *      Report Type
 *  @param[in]  instanceId
 *      Instance Identifier which generated the report
 *  @param[in]  errCode
 *      Error code if any.
 *  @param[in] arg0
 *      Argument 0 interpreted with the report type
 *  @param[in] arg1
 *      Argument 1 interpreted with the report type
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_DPC_ObjectDetection_reportFxn(void *data, uint16_t dataLen)
{
    uint32_t msg = *((uint32_t *)data);
    if (msg == MMWDEMO_DPC_COMMONCFG || msg == MMWDEMO_DPC_CFG)
    {
        SemaphoreP_post(&gMmw_MCB.DPMioctlSemHandle);
    }
#ifndef MMWAVELINK_ON_MSS
    else if (/* gMssDPCState == MMWDEMO_WAIT_FRM_DSS_CFG && */ dataLen == sizeof(mmwDemo_cfgToMss))
    {
        gMssDPCState        = MMWDEMO_DPC_RESULT;
        /* cfgFromDss ignored since MMWAVELINK_ON_MSS */
    }
    else if (dataLen == sizeof(MMWave_CalibrationData))
    {
        gMmw_MCB.cfg.calibrationDataCfg = *((MMWave_CalibrationData *)data);
        SemaphoreP_post(&gCalibSaveSemHandle);
    }
#endif
    else
    {
        gMmw_MCB.result = (DPC_ObjectDetection_ExecuteResult *)data;

        if (gMmw_MCB.stats.isLastFrameDataProcessed)

        {

            /* reset Frame data processed flag, set after full obj data is actually streamed out */

            gMmw_MCB.stats.isLastFrameDataProcessed = false;

            /* signal the UART task to transmit the data */

            SemaphoreP_post(&gMmw_MCB.UartExportSemHandle);
        }

        else
        {

            volatile uint32_t transmitTime = (CycleCounterP_getCount32() - transmitStartTime) / (SOC_getSelfCpuClk() / 1000000U);

            DebugP_log("UART processing not completed: numObjOut %d Time %d\n", ((DPC_ObjectDetection_ExecuteResult *)gMmw_MCB.result)->numObjOut, transmitTime);

            MmwDemo_debugAssert(0);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function to get next sub-frame index
 *
 *  @param[in] currentIndx Current sub-frame index
 *  @param[in] numSubFrames Number of sub-frames
 *
 *  @retval
 *      Index of next sub-frame.
 */
static uint8_t MmwDemo_getNextSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames)
{
    uint8_t nextIndx;

    if (currentIndx == (numSubFrames - 1))
    {
        nextIndx = 0;
    }
    else
    {
        nextIndx = currentIndx + 1;
    }
    return (nextIndx);
}

/**
 *  @b Description
 *  @n
 *      Utility function to get previous sub-frame index
 *
 *  @param[in] currentIndx Current sub-frame index
 *  @param[in] numSubFrames Number of sub-frames
 *
 *  @retval
 *      Index of previous sub-frame
 */
static uint8_t MmwDemo_getPrevSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames)
{
    uint8_t prevIndx;

    if (currentIndx == 0)
    {
        prevIndx = numSubFrames - 1;
    }
    else
    {
        prevIndx = currentIndx - 1;
    }
    return (prevIndx);
}

#ifdef LVDS_STREAM
#if HECR_LVDS_IS_SW_ENABLED /* Currently not tested */
/**
 *  @b Description
 *  @n
 *      Transmit user data over LVDS interface.
 *
 *  @param[in]  subFrameIndx Sub-frame index
 *  @param[in]  dpcResults   pointer to DPC result
 *
 */
void MmwDemo_transferLVDSUserData(uint8_t                            subFrameIndx,
                                  DPC_ObjectDetection_ExecuteResult *dpcResults)
{
    int32_t                    errCode;
    DPC_ObjectDetection_Stats *stats;
    DPIF_PointCloudCartesian  *objOut;
    DPIF_PointCloudSideInfo   *objOutSideInfo;

    stats = (DPC_ObjectDetection_Stats *)CSL_TranslateAddress((uint32_t)dpcResults->stats,
                                                              CSL_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                              &errCode);
    DebugP_assert(errCode == CSL_PASS);

    objOut = (DPIF_PointCloudCartesian *)CSL_TranslateAddress((uint32_t)dpcResults->objOut,
                                                              CSL_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                              &errCode);
    DebugP_assert(errCode == CSL_PASS);

    objOutSideInfo = (DPIF_PointCloudSideInfo *)CSL_TranslateAddress((uint32_t)dpcResults->objOutSideInfo,
                                                                     CSL_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                                     &errCode);
    DebugP_assert(errCode == CSL_PASS);

    /* Delete previous SW session if it exists. SW session is being
       reconfigured every frame because number of detected objects
       may change from frame to frame which implies that the size of
       the streamed data may change. */
    if (gMmw_MCB.lvdsStream.swSessionHandle != NULL)
    {
        MmwDemo_LVDSStreamDeleteSwSession();
    }

    /* Configure SW session for this subframe */
    if (MmwDemo_LVDSStreamSwConfig(dpcResults->numObjOut,
                                   objOut,
                                   objOutSideInfo) < 0)
    {
        DebugP_log("Failed LVDS stream SW configuration for sub-frame %d\n", subFrameIndx);
        MmwDemo_debugAssert(0);
        return;
    }

    /* Populate user data header that will be streamed out*/
    gMmw_MCB.lvdsStream.userDataHeader->frameNum    = stats->frameStartIntCounter;
    gMmw_MCB.lvdsStream.userDataHeader->detObjNum   = dpcResults->numObjOut;
    gMmw_MCB.lvdsStream.userDataHeader->subFrameNum = (uint16_t)dpcResults->subFrameIdx;

    /* If SW LVDS stream is enabled, start the session here. User data will immediately
       start to stream over LVDS.*/
    if (CBUFF_activateSession(gMmw_MCB.lvdsStream.swSessionHandle, &errCode) < 0)
    {
        DebugP_log("Failed to activate CBUFF session for LVDS stream SW. errCode=%d\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }
}
#endif
#endif

/**
 *  @b Description
 *  @n
 *      Function to handle frame processing results from DPC
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_handleObjectDetResult(void)
{
    DPC_ObjectDetection_ExecuteResult *dpcResults;
    MmwDemo_output_message_stats      *frameStats;
    volatile uint32_t                  startTime;
    uint8_t                            nextSubFrameIdx;
    uint8_t                            numSubFrames;
    uint8_t                            currSubFrameIdx;
    uint8_t                            prevSubFrameIdx;
    MmwDemo_SubFrameStats             *currSubFrameStats;
    MmwDemo_SubFrameStats             *prevSubFrameStats;

    /*****************************************************************
     * datapath has finished frame processing, results are reported
     *****************************************************************/

    /* Translate the address: */
    dpcResults = (gMmw_MCB.result);

    numSubFrames      = gMmw_MCB.objDetCommonCfg.preStartCommonCfg.numSubFrames;
    currSubFrameIdx   = dpcResults->subFrameIdx;
    prevSubFrameIdx   = MmwDemo_getPrevSubFrameIndx(currSubFrameIdx, numSubFrames);
    currSubFrameStats = &gMmw_MCB.subFrameStats[currSubFrameIdx];
    prevSubFrameStats = &gMmw_MCB.subFrameStats[prevSubFrameIdx];

    /*****************************************************************
     * Transmit results
     *****************************************************************/
    startTime = CycleCounterP_getCount32();

    /* streaming Range Bias out while mmWaveLink running on DSS, is not supported */
    if (gMmw_MCB.objDetCommonCfg.preStartCommonCfg.measureRxChannelBiasCfg.enabled == 1)
    {
        if (dpcResults->compRxChanBiasMeasurement != NULL)
        {
            dpcResults->compRxChanBiasMeasurement = (Measure_compRxChannelBiasCfg *)AddrTranslateP_getLocalAddr((uint32_t)dpcResults->compRxChanBiasMeasurement);
            MmwDemo_measurementResultOutput((void *)dpcResults->compRxChanBiasMeasurement);
        }
        else
        {
            /* DPC is not ready to ship the measurement results */
        }
    }

    /* Translate the address: */
    frameStats = (MmwDemo_output_message_stats *)AddrTranslateP_getLocalAddr((uint32_t)gMmw_MCB.ptrResult.ptrBuffer[1]);

    /* Update current frame stats */
    currSubFrameStats->outputStats.interFrameCPULoad          = frameStats->interFrameCPULoad;
    currSubFrameStats->outputStats.activeFrameCPULoad         = frameStats->activeFrameCPULoad;
    currSubFrameStats->outputStats.interChirpProcessingMargin = frameStats->interChirpProcessingMargin;
    currSubFrameStats->outputStats.interFrameProcessingTime   = frameStats->interFrameProcessingTime;
    prevSubFrameStats->outputStats.interFrameProcessingMargin = frameStats->interFrameProcessingMargin;
    currSubFrameStats->outputStats.interFrameProcessingMargin = currSubFrameStats->outputStats.interFrameProcessingMargin -
        (currSubFrameStats->pendingConfigProcTime + currSubFrameStats->subFramePreparationTime);
#ifdef LVDS_STREAM
    if (gMmw_MCB.subFrameCfg[currSubFrameIdx].lvdsStreamCfg.dataFmt !=
        MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
    {
        /* check Edma errors (which are considered fatal) for the current sub-frame's
         * h/w session that is expected to be completed by now */
        MmwDemo_checkEdmaErrors();

        /* Pend for completion of h/w session, generally this will not wait
         * because of time spent doing inter-frame processing is expected to
         * be bigger than the transmission of the h/w session */
        SemaphoreP_pend(&gMmw_MCB.lvdsStream.hwFrameDoneSemHandle, SystemP_WAIT_FOREVER);
    }
#if HECR_LVDS_IS_SW_ENABLED
    /* Transfer data on LVDS if s/w session is enabled for the current sub-frame */
    if (gMmw_MCB.subFrameCfg[currSubFrameIdx].lvdsStreamCfg.isSwEnabled == 1)
    {
        MmwDemo_transferLVDSUserData(currSubFrameIdx, dpcResults);
    }
#endif
#endif

    memcpy((void *)gDPIFPointCloud, AddrTranslateP_getLocalAddr((uint32_t)dpcResults->objOut), dpcResults->numObjOut * sizeof(DPIF_PointCloudCartesian));
    memcpy((void *)gDPIFPointCloudSideInfo, AddrTranslateP_getLocalAddr((uint32_t)dpcResults->objOutSideInfo), dpcResults->numObjOut * sizeof(DPIF_PointCloudSideInfo));
    memcpy((void *)gPointCloudRadialCompactList, AddrTranslateP_getLocalAddr((uint32_t)dpcResults->pointCloudRadialCompactList), dpcResults->numObjOut * sizeof(pointCloudRadialCompact));

    size_t numTracks = 0;
    gtrack_ransac_proc(dpcResults->numObjOut, gDPIFPointCloud, gDPIFPointCloudSideInfo, gRansacFilterMask, gTargetOutput, &numTracks);

    /* Transmit processing results for the frame */
    transmitStartTime = CycleCounterP_getCount32();
    MmwDemo_transmitProcessedOutput(gMmw_MCB.loggingUartHandle,
                                    dpcResults,
                                    gDPIFPointCloud,
                                    gDPIFPointCloudSideInfo,
                                    gPointCloudRadialCompactList,
                                    &currSubFrameStats->outputStats,
                                    gTargetOutput,
                                    numTracks,
                                    gRansacFilterMask);

#ifdef LVDS_STREAM
    /* Wait until s/w session is complete. We expect the LVDS transmission of
     * s/w session to be completed by now because the UART transmission above is slower.
     * Doing the wait immediately after starting the transmission above MmwDemo_transferLVDSUserData
     * will serialize the LVDS and UART transfers so it is better to do after UART
     * transmission (which is blocking call i.e UART transmission is completed when we exit
     * out of above MmwDemo_transmitProcessedOutput). Note we cannot replace below code
     * with check for previous sub-frame s/w session completion before the
     * MmwDemo_transferLVDSUserData above because we need to ensure that
     * current sub-frame/frame's contents are not being read during the
     * next sub-frame/frame transmission, presently the data that is being
     * transmitted is not double buffered to allow this */
    if (gMmw_MCB.subFrameCfg[currSubFrameIdx].lvdsStreamCfg.isSwEnabled == 1)
    {
        /* check Edma errors (which are considered fatal) for the s/w session
         * that is expected to be completed by now */
        MmwDemo_checkEdmaErrors();

        /* Pend completion of s/w session, no wait is expected here */
        SemaphoreP_pend(&gMmw_MCB.lvdsStream.swFrameDoneSemHandle, SystemP_WAIT_FOREVER);
    }
#endif

    /* Update current frame transmit time */
    currSubFrameStats->outputStats.transmitOutputTime = (CycleCounterP_getCount32() - startTime) / (SOC_getSelfCpuClk() / 1000000U); /* In micro seconds */

    /*****************************************************************
     * Handle dynamic pending configuration
     * For non-advanced frame case:
     *   process all pending dynamic config commands.
     * For advanced-frame case:
     *  Process next sub-frame related pending dynamic config commands.
     *  If the next sub-frame was the first sub-frame of the frame,
     *  then process common (sub-frame independent) pending dynamic config
     *  commands.
     *****************************************************************/
    startTime = CycleCounterP_getCount32();

    nextSubFrameIdx                          = MmwDemo_getNextSubFrameIndx(currSubFrameIdx, numSubFrames);
    (void)nextSubFrameIdx;
    currSubFrameStats->pendingConfigProcTime = (CycleCounterP_getCount32() - startTime) / (SOC_getSelfCpuClk() / 1000000U);

/* subFrame switch for LVDS streaming not supported in this version when mmWaveLink running on DSS */
#if (defined(MMWAVELINK_ON_MSS) && defined(SUBSYS_MSS))
    /*****************************************************************
     * Prepare for subFrame switch
     *****************************************************************/
    if (numSubFrames > 1)
    {
        MmwDemo_SubFrameCfg *nextSubFrameCfg; (void)nextSubFrameCfg;

        startTime = CycleCounterP_getCount32();

        nextSubFrameCfg = &gMmw_MCB.subFrameCfg[nextSubFrameIdx];

        currSubFrameStats->subFramePreparationTime = (CycleCounterP_getCount32() - startTime) / (SOC_getSelfCpuClk() / 1000000U);

#ifdef LVDS_STREAM
        /* Configure HW LVDS stream for this subframe? */
        if (nextSubFrameCfg->lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
        {
            /* check Edma errors (which are considered fatal) for any previous session, even
             * though we have checked for a previous s/w session, if s/w session weren't
             * enabled, then this will check for previous h/w session related Edma errors */
            MmwDemo_checkEdmaErrors();

            MmwDemo_configLVDSHwData(nextSubFrameIdx);
        }
        currSubFrameStats->subFramePreparationTime = (CycleCounterP_getCount32() - startTime) / (SOC_getSelfCpuClk() / 1000000U);
#endif
    }
    else
#endif
    {
        currSubFrameStats->subFramePreparationTime = 0;
    }

    /* set the Frame data processed flag to indicate that obj data is out successfully */
    gMmw_MCB.stats.isLastFrameDataProcessed = true;
}

/**
 *  @b Description
 *  @n
 *      UART Data Export Task which exports the detected objects and other processing chain outputs on UART.
 *
 *  @retval
 *      Not Applicable.
 */
void mmwDemo_mssUartDataExportTask(void *args)
{
    while (1)
    {
        /* Export the Data: */
        SemaphoreP_pend(&gMmw_MCB.UartExportSemHandle, SystemP_WAIT_FOREVER);
        MmwDemo_handleObjectDetResult();
    }
}

#ifdef ENET_STREAM
/**
 *  @b Description
 *  @n
 *      Function to declare when Ethernet Configuration is complete
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssEnetCfgDone(void)
{

    /* Post EnetCfgDone Semaphore to signal that the IP
    has been configured and connection can now be made */
    SemaphoreP_post(&gMmw_MCB.enetCfg.EnetCfgDoneSemHandle);

    return 0;
}
#endif


/**************************************************************************
 ******************** Millimeter Wave Demo init Functions ************************
 **************************************************************************/
#if 0 /* UART property is set by SysConfig Driver generated code, not required here */
/**
 *  @b Description
 *  @n
 *      Platform specific hardware initialization.
 *
 *  @param[in]  config     Platform initialization configuraiton
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_platformInit(MmwDemo_platformCfg *config)
{
    /* Initialize the DEMO configuration: */
    config->sysClockFrequency   = MSS_SYS_VCLK;
    config->loggingBaudRate     = 892857;
    config->commandBaudRate     = 115200;

}
#endif

#ifdef LVDS_STREAM
/**
 *  @b Description
 *  @n
 *      Checks for EDMA errors, used on devices where error interrupts are not connected
 *      to the CPU. Current use case is for LVDS, note it is not very useful to
 *      check for edma errors within the CBUFF session completion interrupts
 *      because they will not happen if edma had errors. So this API should be
 *      called at opportune places in the application code, typically at some time
 *      later than the triggering of the session when it is roughly expected that
 *      the session would have completed by that time.
 */
static void MmwDemo_checkEdmaErrors(void)
{
    bool     isAnyError = false;
    uint32_t baseAddr   = 0U;

    baseAddr = EDMA_getBaseAddr(gMmw_MCB.edmaHandle);
    DebugP_assert(baseAddr != 0);

    isAnyError = ((EDMA_getErrIntrStatus(baseAddr) != 0U) ||
                  (EDMA_errIntrHighStatusGet(baseAddr) != 0U));

    if (isAnyError == true)
    {
        DebugP_log("EDMA channel controller has errors, see gMmw_MCB.EDMA_errorInfo\n");
        MmwDemo_debugAssert(0);
    }
}
#endif

/**
 *  @b Description
 *  @n
 *      Calibration save/restore initialization
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_calibInit(void)
{
    int32_t     retVal = 0;
    rlVersion_t verArgs;

    /* Initialize verArgs */
    memset((void *)&verArgs, 0, sizeof(rlVersion_t));

    /* Calibration save/restore init */
    gMmw_MCB.calibCfg.sizeOfCalibDataStorage = sizeof(MmwDemo_calibData);
    gMmw_MCB.calibCfg.calibDataHdr.magic     = MMWDEMO_CALIB_STORE_MAGIC;
    memcpy((void *)&gMmw_MCB.calibCfg.calibDataHdr.linkVer, (void *)&verArgs.mmWaveLink, sizeof(rlSwVersionParam_t));
    memcpy((void *)&gMmw_MCB.calibCfg.calibDataHdr.radarSSVer, (void *)&verArgs.rf, sizeof(rlFwVersionParam_t));

    /* Check if Calibration data is over the Reserved storage */
    if (gMmw_MCB.calibCfg.sizeOfCalibDataStorage <= MMWDEMO_CALIB_FLASH_SIZE)
    {
        gMmw_MCB.calibCfg.calibDataHdr.hdrLen  = sizeof(MmwDemo_calibDataHeader);
        gMmw_MCB.calibCfg.calibDataHdr.dataLen = sizeof(MmwDemo_calibData) - sizeof(MmwDemo_calibDataHeader);

        /* Resets calibration data */
        memset((void *)&gCalibDataStoragetoDss, 0, sizeof(MmwDemo_calibData));
        memset((void *)&gCalibDataStoragefromDss, 0, sizeof(MmwDemo_calibData));

        retVal = mmwDemo_flashInit();
    }
    else
    {
        DebugP_log("Error: Calibration data size is bigger than reserved size\n");
        retVal = -1;
    }

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This function retrieves the calibration data from front end and saves it in flash.
 *
 *  @param[in]  ptrCalibDataHdr     	Pointer to Calibration data header
 *  @param[in]  ptrCalibrationData      Pointer to Calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_calibSave(MmwDemo_calibDataHeader *ptrCalibDataHdr, MmwDemo_calibData *ptrCalibrationData)
{
    uint32_t flashOffset;
    int32_t  retVal = 0;

    /* Calculate the read size in bytes */
    flashOffset = gMmw_MCB.calibCfg.flashOffset;

    /* Copy header  */
    memcpy((void *)&(ptrCalibrationData->calibDataHdr), ptrCalibDataHdr, sizeof(MmwDemo_calibDataHeader));

    /* Flash calibration data */
    retVal = mmwDemo_flashWrite(flashOffset, (uint8_t *)ptrCalibrationData, sizeof(MmwDemo_calibData));
    if (retVal < 0)
    {
        /* Flash Header failed */
        DebugP_log("Error: MmwDemo failed flashing calibration data with error[%d].\n", retVal);
    }
    return (retVal);
}


/**
 *  @b Description
 *  @n
 *      This function reads calibration data from flash and send it to front end through MMWave_open()
 *
 *  @param[in]  ptrCalibData     	Pointer to Calibration data
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t MmwDemo_calibRestore(MmwDemo_calibData *ptrCalibData)
{
    MmwDemo_calibDataHeader *pDataHdr;
    int32_t                  retVal = 0;
    uint32_t                 flashOffset;

    pDataHdr = &(ptrCalibData->calibDataHdr);

    /* Calculate the read size in bytes */
    flashOffset = gMmw_MCB.calibCfg.flashOffset;

    /* Read calibration data header */
    if (mmwDemo_flashRead(flashOffset, (uint8_t *)pDataHdr, sizeof(MmwDemo_calibData)) < 0)
    {
        /* Error: only one can be enable at at time */
        DebugP_log("Error: MmwDemo failed when reading calibration data from flash.\n");
        return -1;
    }

    /* Validate data header */
    if ((pDataHdr->magic != MMWDEMO_CALIB_STORE_MAGIC) ||
        (pDataHdr->hdrLen != gMmw_MCB.calibCfg.calibDataHdr.hdrLen) ||
        (pDataHdr->dataLen != gMmw_MCB.calibCfg.calibDataHdr.dataLen))
    {
        /* Header validation failed */
        DebugP_log("Error: MmwDemo calibration data header validation failed.\n");
        retVal = -1;
    }
    /* Matching mmwLink version:
         In this demo, we would like to save/restore with the matching mmwLink and RF FW version.
         However, this logic can be changed to use data saved from previous mmwLink and FW releases,
         as long as the data format of the calibration data matches.
     */
    else if (memcmp((void *)&pDataHdr->linkVer, (void *)&gMmw_MCB.calibCfg.calibDataHdr.linkVer, sizeof(rlSwVersionParam_t)) != 0)
    {
        DebugP_log("Error: MmwDemo failed mmwLink version validation when restoring calibration data.\n");
        retVal = -1;
    }
    else if (memcmp((void *)&pDataHdr->radarSSVer, (void *)&gMmw_MCB.calibCfg.calibDataHdr.radarSSVer, sizeof(rlFwVersionParam_t)) != 0)
    {
        DebugP_log("Error: MmwDemo failed RF FW version validation when restoring calibration data.\n");
        retVal = -1;
    }
    return (retVal);
}

#ifndef MMWAVELINK_ON_MSS
static int32_t MmwDemo_calibDSSlinkRestore(void)
{
    MMWave_CalibrationData  calibrationDataCfg;
    MMWave_CalibrationData *ptrCalibrationDataCfg;
    uint32_t                retval = 0;

    if (MmwDemo_calibRestore(&gCalibDataStoragetoDss) < 0)
    {
        DebugP_log("Error: MmwDemo failed restoring calibration data from flash.\n");
        return -1;
    }

    calibrationDataCfg.ptrCalibData           = &gCalibDataStoragetoDss.calibData;
    calibrationDataCfg.ptrPhaseShiftCalibData = &gCalibDataStoragetoDss.phaseShiftCalibData;
    ptrCalibrationDataCfg                     = &calibrationDataCfg;

    retval = DPM_send(gMmw_MCB.objDetDpmHandle, (void *)(ptrCalibrationDataCfg), sizeof(MMWave_CalibrationData), CSL_CORE_ID_C66SS0);
    if (retval != 0)
    {
        DebugP_log("Error: MmwDemo failed sending calibration data to DSS.\n");
        return -1;
    }
    return retval;
}

static int32_t MmwDemo_calibDSSlinkSave(void)
{
    uint32_t retVal = 0;
    uint32_t msg    = 0;

    gCalibDataStoragefromDss.calibData           = *gMmw_MCB.cfg.calibrationDataCfg.ptrCalibData;
    gCalibDataStoragefromDss.phaseShiftCalibData = *gMmw_MCB.cfg.calibrationDataCfg.ptrPhaseShiftCalibData;

    /* Save data in flash */
    retVal = MmwDemo_calibSave(&gMmw_MCB.calibCfg.calibDataHdr, &gCalibDataStoragefromDss);
    if (retVal < 0)
    {
        return retVal;
    }

    msg    = MMWDEMO_CALIBSAVEACK;
    retVal = DPM_send(gMmw_MCB.objDetDpmHandle, (void *)(&msg), sizeof(uint32_t), CSL_CORE_ID_C66SS0);
    DebugP_assert(0U == retVal);

    return retVal;
}
#endif

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */

void MmwDemo_initTask(void *args)
{
    int32_t     errCode;
    DPM_InitCfg dpmInitCfg;
    // DPC_ObjectDetection_InitParams      objDetInitParams;
    int32_t i;

    Drivers_open();
    Board_driversOpen();

    MmwDemo_BoardInit();

    /* Debug Message: */
    DebugP_log("**********************************************\n");
    DebugP_log("Debug: Launching the MMW Demo on MSS\n");
    DebugP_log("**********************************************\n");

    /* Debug Message: */
    DebugP_log("Debug: Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/

    /* Initialize Last Frame data Set flag for first frame. */
    gMmw_MCB.stats.isLastFrameDataProcessed = true;

#ifdef LVDS_STREAM
    gMmw_MCB.edmaHandle = gEdmaHandle[CONFIG_EDMA0];

    /* Initialize LVDS streaming components */
    if ((errCode = MmwDemo_LVDSStreamInit()) < 0)
    {
        DebugP_log("Error: MMWDemoDSS LVDS stream init failed with Error[%d]\n", errCode);
        return;
    }

    /* Configure Pad registers for LVDS. */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_LVDS_PAD_CTRL0, 0x0);
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_LVDS_PAD_CTRL1, 0x02000000);

    /*The delay below is needed only if the DCA1000EVM is being used to capture the data traces.
      This is needed because the DCA1000EVM FPGA needs the delay to lock to the
      bit clock before they can start capturing the data correctly. */
    ClockP_usleep(12 * 1000);
#endif

    /* initialize cq configs to invalid profile index to be able to detect
     * unconfigured state of these when monitors for them are enabled.
     */
    for (i = 0; i < RL_MAX_PROFILES_CNT; i++)
    {
        gMmw_MCB.cqSatMonCfg[i].profileIndx    = (RL_MAX_PROFILES_CNT + 1);
        gMmw_MCB.cqSigImgMonCfg[i].profileIndx = (RL_MAX_PROFILES_CNT + 1);
    }

    /* Platform specific configuration */
    /* UART property is set by SysConfig Driver generated code, not required here */
    // MmwDemo_platformInit(&gMmw_MCB.cfg.platformCfg);


    /* Open the UART Instance */
    gMmw_MCB.commandUartHandle = gUartHandle[CONFIG_UART0];
    if (gMmw_MCB.commandUartHandle == NULL)
    {
        MmwDemo_debugAssert(0);
        return;
    }

    /* Open the Logging UART Instance: */
    gMmw_MCB.loggingUartHandle = gUartHandle[CONFIG_UART1];
    if (gMmw_MCB.loggingUartHandle == NULL)
    {
        DebugP_log("Error: Unable to open the Logging UART Instance\n");
        MmwDemo_debugAssert(0);
        return;
    }

    DebugP_logInfo("Both UART instances opened\n");

#ifdef CAN_STREAM
    mcan_initialization();
#endif

    /* Create binary semaphores which is used to signal DPM_start/DPM_stop/DPM_ioctl is done
     * to the sensor management task. The signalling (SemaphoreP_post) will be done
     * from DPM registered report function (which will execute in the DPM execute task context). */
    SemaphoreP_constructBinary(&gMmw_MCB.DPMstartSemHandle, 0);
    SemaphoreP_constructBinary(&gMmw_MCB.DPMstopSemHandle, 0);
    SemaphoreP_constructBinary(&gMmw_MCB.DPMioctlSemHandle, 0);
    SemaphoreP_constructBinary(&gMmw_MCB.UartExportSemHandle, 0);

    /* Create binary semaphore to pend Main task, */
    SemaphoreP_constructBinary(&gMmw_MCB.demoInitTaskCompleteSemHandle, 0);

#ifdef ENET_STREAM
    /*****************************************************************************
     * Launch the mmWave enet task
     *****************************************************************************/
    /* Create Enet configuration done semaphore */
    SemaphoreP_constructBinary(&gMmw_MCB.enetCfg.EnetCfgDoneSemHandle, 0);

    gMmw_MCB.taskHandles.enetTask = xTaskCreateStatic(enetTask,
                                                      "enet_task",
                                                      MMWDEMO_MMWAVE_ENET_TASK_STACK_SIZE,
                                                      NULL,
                                                      MMWDEMO_MMWAVE_ENET_TASK_PRIORITY,
                                                      gMmwEnetTskStack,
                                                      &gMmw_MCB.taskHandles.enetTaskObj);

    configASSERT(gMmw_MCB.taskHandles.enetTask != NULL);
#endif

    /*****************************************************************************
     * Initialization of the DPM Module:
     *****************************************************************************/
    memset((void *)&dpmInitCfg, 0, sizeof(DPM_InitCfg));

    /* Setup the configuration: */
    dpmInitCfg.localEndPt = gRemoteCoreEndPt[CSL_CORE_ID_R5FSS0_0];
    dpmInitCfg.reportFxn  = MmwDemo_DPC_ObjectDetection_reportFxn;
    dpmInitCfg.setBitPos  = DPM_MSS_BOOT_INFO_BIT_POS;

    gtrack_heap_init();

    /* Initialize the DPM Module: */
    gMmw_MCB.objDetDpmHandle = DPM_init(&dpmInitCfg, &errCode);
    if (gMmw_MCB.objDetDpmHandle == NULL)
    {
        DebugP_log("Error: Unable to initialize the DPM Module [Error: %d]\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }
    int32_t syncStatusRefVal = CSL_FMKR(DPM_DSS_BOOT_INFO_BIT_POS, DPM_MSS_BOOT_INFO_BIT_POS, 7);

    /* Synchronization: This will synchronize the execution of the datapath module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = DPM_synch(gMmw_MCB.objDetDpmHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the framework */
            DebugP_log("Error: DPM Synchronization failed [Error code %d]\n", errCode);
            MmwDemo_debugAssert(0);
            return;
        }

        if (syncStatus == syncStatusRefVal)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        ClockP_usleep(1 * 1000U);
    }

    /* Calibration save/restore initialization */
    if (MmwDemo_calibInit() < 0)
    {
        DebugP_log("Error: Calibration data initialization failed \n");
        MmwDemo_debugAssert(0);
    }

    /* Launch the UART Data Export Task */
    gMmw_MCB.taskHandles.uartDataExportTask = xTaskCreateStatic(mmwDemo_mssUartDataExportTask,
                                                                "mmwdemo_uart_task",
                                                                MMWDEMO_UART_DATA_EXPORT_TASK_STACK_SIZE,
                                                                NULL,
                                                                MMWDEMO_UART_EXPORT_TASK_PRIORITY,
                                                                gUartTskStack,
                                                                &gMmw_MCB.taskHandles.uartDataExportTaskObj);

    configASSERT(gMmw_MCB.taskHandles.uartDataExportTask != NULL);

    /*****************************************************************************
     * Initialize the Profiler
     *****************************************************************************/
    CycleCounterP_reset();

#ifndef MMWAVELINK_ON_MSS
    /* Set the calibration parameters */
    gMmw_MCB.calibCfg.restoreEnable = FACTORY_CALIB_RESTORE_ENABLE;
    gMmw_MCB.calibCfg.saveEnable    = FACTORY_CALIB_SAVE_ENABLE;
    gMmw_MCB.calibCfg.flashOffset   = FACTORY_CALIB_FLASH_OFFSET;

    /*Restores calibration data and sends to DSS core*/
    if (gMmw_MCB.calibCfg.restoreEnable != 0)
    {
        MmwDemo_calibDSSlinkRestore();
    }
    /*Saves calibration data to flash*/
    if (gMmw_MCB.calibCfg.saveEnable != 0)
    {
        SemaphoreP_constructBinary(&gCalibSaveSemHandle, 0);

        SemaphoreP_pend(&gCalibSaveSemHandle, SystemP_WAIT_FOREVER);

        MmwDemo_calibDSSlinkSave();
    }
#endif

#ifdef TRACKER_ON_MSS
    /* tracker configuration*/
    MmwDemo_trackerConfig();
#endif

    /* Never return for this task. */
    SemaphoreP_pend(&gMmw_MCB.demoInitTaskCompleteSemHandle, SystemP_WAIT_FOREVER);

    /* The following line should never be reached. */
    DebugP_assertNoLog(0);
}

/**
 *  @b Description
 *  @n
 *      Performs Board Initialization
 *
 *  @retval
 *      Success -   true
 *  @retval
 *      Error   -   false
 */
static bool MmwDemo_BoardInit(void)
{
    /* Configure HSI Clock. */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL, 0x333);

    return true;
}

/**
 *  @b Description
 *  @n
 *      This function computes Modulation frequency divider
 *      (7 bit mantissa and 3 bit exponent) and Modulation depth
 *      (3 bit integer and 18 bit fraction) based on user provided inputs
 *      Modulation rate, Modulation depth in %
 *
 *  @param[in] refClk Pre-divided reference clock input for the ADPLL
 *  @param[in] dpllM ADPLL Feedback Multiplier
 *  @param[in] ptrdpllCfg Pointer to SSC Config (MmwDemo_spreadSpectrumConfig)
 *
 *  @retval
 *      FRACCTRL Register value
 *
 */

static uint32_t computeSscFactCtrlVal(const uint32_t refClk, const uint16_t dpllM, MmwDemo_spreadSpectrumConfig *ptrdpllCfg)
{
    float    modRateSel         = 0.0f;
    float    deltaMStep         = 0.0f;
    uint32_t delatMStepInt      = 0U;
    float    deltaMStepFrac     = 0.0f;
    uint32_t deltaMStepFracInt  = 0U;
    uint32_t modFreqDivExponent = 0U;
    uint32_t modFreqDivMantissa = 0U;

    modRateSel = (refClk * 1000.0f) / (4.0f * ptrdpllCfg->modRate);

    modFreqDivExponent = (uint32_t)(floor(modRateSel / MAX_MOD_FREQ_DIVIDER_MANTISSA));

    modFreqDivMantissa = (uint32_t)(floor(modRateSel / pow(2.0f, modFreqDivExponent)));

    ptrdpllCfg->modRate = refClk * 1000 / (4 * modFreqDivMantissa * (pow(2, modFreqDivExponent)));

    if (modFreqDivExponent <= 3U)
    {
        deltaMStep = (ptrdpllCfg->modDepth * dpllM) / (100.0f * modFreqDivMantissa * pow(2.0f, modFreqDivExponent));
    }
    else
    {
        deltaMStep = (ptrdpllCfg->modDepth * dpllM) / (100.0f * modFreqDivMantissa * 8.0f);
    }

    delatMStepInt = (uint32_t)(deltaMStep + 0.5f);

    deltaMStepFrac = deltaMStep - delatMStepInt;

    deltaMStepFracInt = (uint32_t)ceilf(deltaMStepFrac * (1U << 18U));

    ptrdpllCfg->modDepth = 100 * ((deltaMStepFrac / (1 << 18)) * modFreqDivMantissa * (1U << modFreqDivExponent) / dpllM);

    return (deltaMStepFracInt + (delatMStepInt * (1U << 18U)) +
            (modFreqDivMantissa * (1U << 21U)) + (modFreqDivExponent * (1U << 28U)) +
            (ptrdpllCfg->downSpread * (1U << 31U)));
}


/**
 *  @b Description
 *  @n
 *      Performs Spread Spectrum Configuration (SSC)
 *      SSC reduces the Electromagnetic Interference by spreading it out across frequencies instead of concentrating at a single frequency
 *
 *  @retval
 *      none
 */
void MMWDemo_configSSC(void)
{
    uint16_t dpllM  = 0U;
    uint16_t dpllN  = 0U;
    uint32_t finp   = 40U; /* XTAL */
    uint32_t refClk = 0U;

    CSL_mss_toprcmRegs *ptrMssTopRcmRegs = (CSL_mss_toprcmRegs *)CSL_MSS_TOPRCM_U_BASE;

    if (gMmw_MCB.coreAdpllSscCfg.isEnable)
    {
        dpllM = CSL_FEXT(ptrMssTopRcmRegs->PLL_CORE_MN2DIV,
                         MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_M);

        dpllN = CSL_FEXT(ptrMssTopRcmRegs->PLL_CORE_M2NDIV,
                         MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_N);

        /** refClk - Pre-divided reference clock input for the ADPLL
         * CLKINP/(N+1) where N is Pre divider and CLKINP is OSC Clock
         */
        refClk = finp / (dpllN + 1);

        ptrMssTopRcmRegs->PLL_CORE_FRACCTRL = computeSscFactCtrlVal(refClk, dpllM, &gMmw_MCB.coreAdpllSscCfg);

        CSL_FINS(ptrMssTopRcmRegs->PLL_CORE_CLKCTRL, MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_ENSSC, 1U);
    }

    if (gMmw_MCB.dspAdpllSscCfg.isEnable)
    {
        dpllM = CSL_FEXT(ptrMssTopRcmRegs->PLL_DSP_MN2DIV,
                         MSS_TOPRCM_PLL_DSP_MN2DIV_PLL_DSP_MN2DIV_M);

        dpllN = CSL_FEXT(ptrMssTopRcmRegs->PLL_DSP_M2NDIV,
                         MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_N);

        refClk = finp / (dpllN + 1);

        ptrMssTopRcmRegs->PLL_DSP_FRACCTRL = computeSscFactCtrlVal(refClk, dpllM, &gMmw_MCB.dspAdpllSscCfg);

        CSL_FINS(ptrMssTopRcmRegs->PLL_DSP_CLKCTRL, MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_ENSSC, 1U);
    }

    if (gMmw_MCB.perAdpllSscCfg.isEnable)
    {
        dpllM = CSL_FEXT(ptrMssTopRcmRegs->PLL_PER_MN2DIV,
                         MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_M);

        dpllN = CSL_FEXT(ptrMssTopRcmRegs->PLL_PER_M2NDIV,
                         MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_N);

        refClk = finp / (dpllN + 1);

        ptrMssTopRcmRegs->PLL_PER_FRACCTRL = computeSscFactCtrlVal(refClk, dpllM, &gMmw_MCB.perAdpllSscCfg);

        CSL_FINS(ptrMssTopRcmRegs->PLL_PER_CLKCTRL, MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_ENSSC, 1U);
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int32_t main(void)
{
    /* init SOC specific modules */
    System_init();
    Board_init();

    gMmw_MCB.taskHandles.initTask = xTaskCreateStatic(MmwDemo_initTask,
                                                      "mmwdemo_init_task",
                                                      MMWDEMO_INIT_TASK_STACK_SIZE,
                                                      NULL,
                                                      MMWDEMO_INIT_TASK_PRI,
                                                      gAppMainTskStack,
                                                      &gMmw_MCB.taskHandles.initTaskObj);
    configASSERT(gMmw_MCB.taskHandles.initTask != NULL);

    /* Start the scheduler to start the tasks executing. */
    vTaskStartScheduler();

    /* The following line should never be reached because vTaskStartScheduler()
    will only return if there was not enough FreeRTOS heap memory available to
    create the Idle and (if configured) Timer tasks.  Heap management, and
    techniques for trapping heap exhaustion, are described in the book text. */
    DebugP_assertNoLog(0);
}



