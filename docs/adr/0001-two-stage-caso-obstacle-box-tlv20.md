# ADR 0001: Two-Stage CASO-CFAR and Single Compact Bounding Box TLV

## Context
In the `oob_1843` radar demo on the IWR1843Boost, the standard HWA CFAR path can reject static returns at zero Doppler, starving downstream obstacle processing. Streaming hundreds of raw 3D point cloud measurements (TLV 1) over UART also consumes several kilobytes per frame, inducing latency and burdening downstream companion computers (Raspberry Pi) with clustering overhead.

## Decision
1. **Two-Stage Detection**:
   - **Stage 1 (C674x DSP)**: Apply range-domain **CASO-CFAR** (Cell Averaging Smallest Of) to the Doppler peak profile across the configured range span, retaining local peaks and nearby shoulders.
   - **Stage 2 (C674x DSP)**: Apply Doppler-domain CASO-CFAR to each accepted range, explicitly preserving significant zero-Doppler returns, then use Dense Elevation and spatial clustering to form 3D **`ObstacleBox`** structures.
2. **Single Compact TLV 20 Output**:
   - Standardize **TLV 20** (`MMWDEMO_OUTPUT_MSG_NAVCANDIDATE_LIST`) as an array of 32-byte `ObstacleBox` structures.
   - Disable raw point cloud streaming (TLV 1) in `guiMonitor` to stream solely the compact Bounding Box TLV, shrinking frame size to $< 500$ bytes and maximizing frame rate for real-time obstacle avoidance on the Raspberry Pi.

## Considered Options
- **Full-matrix software OS-CFAR on C674x DSP**: Rejected due to the sorting cost across 16,384 Range-Doppler cells and risk of frame deadline overruns; the bounded CASO scans avoid that cost.
- **Retaining TLV 1 raw points + TLV 20**: Rejected because dual streaming saturates UART bandwidth at 921,600 baud and wastes companion computer processing cycles.
