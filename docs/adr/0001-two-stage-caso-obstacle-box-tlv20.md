# ADR 0001: Two-Stage CASO-CFAR and Single Compact Bounding Box TLV

## Context
In the `oob_1843` radar demo on the IWR1843Boost, standard Cell-Averaging CFAR (CFAR-CA) in hardware causes target masking: strong reflectors (e.g. walls, large metallic obstacles) inflate the reference noise window and suppress adjacent weaker obstacles. Furthermore, streaming hundreds of raw 3D point cloud measurements (TLV 1) over UART consumes several kilobytes per frame, inducing latency, lowering FPS, and burdening downstream companion computers (Raspberry Pi) with clustering overhead.

## Decision
1. **Two-Stage Detection**:
   - **Stage 1 (Hardware)**: Configure HWA CFAR to use **`CFAR_CASO`** (Cell Averaging Smallest Of, mode 2) instead of CFAR-CA to eliminate masking of weak targets by adjacent strong reflectors without incurring C674x DSP overhead.
   - **Stage 2 (DSP Deep Module)**: Implement a spatial clustering engine on the C674x DSP that delineates contiguous range bins and azimuth angles into discrete 3D **`ObstacleBox`** structures (calculating physical depth $D_y$ from range bins, width $W_x$ from azimuth span, height $H_z$ from elevation, center position $(x, y, z)$, radial velocity, and peak SNR).
2. **Single Compact TLV 20 Output**:
   - Standardize **TLV 20** (`MMWDEMO_OUTPUT_MSG_NAVCANDIDATE_LIST`) as an array of 32-byte `ObstacleBox` structures.
   - Disable raw point cloud streaming (TLV 1) in `guiMonitor` to stream solely the compact Bounding Box TLV, shrinking frame size to $< 500$ bytes and maximizing frame rate for real-time obstacle avoidance on the Raspberry Pi.

## Considered Options
- **Software OS-CFAR on C674x DSP**: Rejected due to high CPU sorting cycle budget on 16,384 Range-Doppler cells risking frame deadline overruns.
- **Retaining TLV 1 raw points + TLV 20**: Rejected because dual streaming saturates UART bandwidth at 921,600 baud and wastes companion computer processing cycles.
