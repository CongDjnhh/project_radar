# Product Requirements Document (PRD): TI mmWave Firmware (CCS Theia)

## 1. Overview
This workspace hosts embedded firmware for TI mmWave Radar sensors:
- Target platforms: IWR1843 (MSS Cortex-R4F + DSS C674x DSP) and AWR2944P.
- Responsibilities:
  - Configuration parsing via CLI UART (`mmw_cli.c`).
  - Radar-cube signal processing (Range FFT, Doppler FFT, CFAR, AoA).
  - Dense elevation processing (`dense_elevation.c`, `dense_elevation.h`).
  - TLV packet serialization and streaming over Data UART (`mmw_output.h`, `mmw_lvds_stream.c`).

## 2. Test Oracle & Verification Command
- Compilation and syntax check in CCS Theia.
- Data structures and packet alignment verification against TI mmWave SDK protocol.

---

## 3. Tasks Checklist

### Milestone 1: Data Protocol & Header Integrity
- [x] **Task 1.1**: Verify packet header layout in `oob_1843_mss/local_inc/mmw_output.h` matches exactly 40 bytes with 8-byte Magic Word (`0x0102, 0x0304, 0x0506, 0x0708`).
  - *Acceptance Criteria:* `sizeof(MmwDemo_output_message_header)` is strictly 40 bytes.
- [x] **Task 1.2**: Verify TLV Type 1 (Point Cloud Cartesian) and TLV 7 (Side Info) packing alignments in MSS.
  - *Acceptance Criteria:* Point records are 16 bytes (float x, y, z, vel) and side info records are 4 bytes (snr, noise).

### Milestone 2: Dense Elevation & Signal Processing
- [x] **Task 2.1**: Audit `dense_elevation.c` and `dense_elevation.h` for azimuth/elevation phase calibration integrity.
  - *Acceptance Criteria:* Antenna virtual array coordinates align with IWR1843 antenna geometry.
- [x] **Task 2.2**: Audit CLI command parser in `mmw_cli.c` to guarantee `sensorStart` / `sensorStop` emit proper `Done` and prompt tokens.
  - *Acceptance Criteria:* CLI responses remain synchronized with `radar-pipeline` controller.
