# Original User Request

## 2026-10-14T04:04:23Z

This is a single self-contained fix and feature enhancement; keep it small and focused.
Requested team: Small, focused team (Firmware DSP Algorithm & Validation)

Refactor and validate the custom UAV obstacle perception pipeline on TI IWR1843BOOST (C674x DSP + R4F MSS): completely replace the TI DPU CFAR HWA with a native Two-Stage CASO-CFAR peak detector running on C674x DSP over the Detection Matrix (`detMatrix`), outputting 32-byte 3D Bounding Boxes (TLV 20) with full coverage for both static and moving obstacles.

Working directory: `C:\Users\Admin\workspace_ccstheia` (Firmware Projects) & `D:\Dinh\CT_Verse\Dev_code\radar-pipeline` (Toolchain)
Integrity mode: development

## Verification Resources
- Diagnostic verification script: `C:\Users\Admin\.gemini\antigravity\brain\04f17ed4-7a94-43a0-83b8-efe4bce5f0fb\scratch\run_controller_test.py`
- Live data capture tool: `D:\Dinh\CT_Verse\Dev_code\radar-pipeline\capture_experiment.py`
- Replay and evaluation script: `D:\Dinh\CT_Verse\Dev_code\radar-pipeline\compare_and_replay.py`
- Hardware targets: TI IWR1843BOOST on COM9 (CLI @ 115200) and COM8 (Data @ 921600).
- Build target output: `D:\Dinh\CT_Verse\Dev_code\radar-pipeline\binaries\algo_new_bbox32.bin`.

## Requirements

### R1. Native Two-Stage CASO-CFAR Engine on C674x DSP (Bypassing TI DPU CFAR)
- Remove/bypass dependency on TI's `DPU_CFARCAProcHWA`.
- Implement a dedicated C674x Two-Stage CASO-CFAR module within `oob_1843_dss` to scan `detMatrix` ($256 \times 32$ log-magnitude bins):
  - Stage 1 (Range Domain): Detect physical obstacle ranges across full chirp span ($0.15\text{ m} - 8.64\text{ m}$), resolving targets across all $256$ range bins even at 0 Doppler.
  - Stage 2 (Doppler Domain): Identify velocity peaks per valid range cell without starving static reflectors.
  - FOV: Full aperture coverage: Azimuth $\pm 90^\circ$, Elevation $\pm 90^\circ$ (continuous estimation with baseline unwrapping).
- Cap peak candidates to maintain DSP execution time under $4\text{ ms}$ per frame.

### R2. Continuous Elevation & Azimuth 3D Bounding Box Clustering (`nav_point_gen`)
- Perform 4D spatio-kinematic clustering (Range, Doppler, Azimuth, Elevation) to group contiguous energy bins into discrete 3D bounding boxes (`ObstacleBox`: x, y, z, width, depth, height, velocity, snr, boxId).
- Implement fail-open persistence in history confirmation so newly appearing obstacles are reported immediately on arrival without being wiped by intermittent dropped frames.

### R3. TLV 20 Wire Format & Live Toolchain Validation
- Output single TLV 20 (32 bytes per obstacle box, struct `<7f2h`) over UART Data Port (921600 baud).
- Verify end-to-end data replay and live capture via `compare_and_replay.py` and `capture_experiment.py`.
- Strict constraint: Do NOT modify TI SDK files in `C:\ti\...`. All custom code must reside inside `oob_1843_dss` and `oob_1843_mss`.

## Acceptance Criteria

### Detection & Packaging
- [ ] Radar streams valid TLV 20 packets on COM8 with `numDetectedObj > 0` when an obstacle is present in front of the radar.
- [ ] Each TLV 20 item is exactly 32 bytes (`<7f2h`), matching physical coordinates $(x, y, z)$, dimensions $(w_x, d_y, h_z)$, velocity and SNR.
- [ ] DSP cycle budget margin remains positive (`interFrameProcessingMargin > 0`).
- [ ] Multicore appimage compiles cleanly without warnings/errors and updates `algo_new_bbox32.bin`.


## 2026-10-14T04:21:56Z

[CRITICAL USER EXPERIMENT DATA & GROUND TRUTH]

The user just captured 300 frames on the physical IWR1843BOOST board with a static obstacle placed in the radar field of view:
- Capture file: `D:\Dinh\CT_Verse\Dev_code\radar-pipeline\data\experiments\new_trial_20261014_105925.pkl`
- Observation: Currently `detected_objects = 0` across all 300 frames.
- Key Discovery from Range Profile analysis:
  - In `range_profile` (TLV 2), there is a strong, distinct physical target peak at **Range Bin 49-51 (~2.06m - 2.14m, centered at bin 50 = 2.10m)** with magnitude > 51.0 dB (> 30 dB SNR above local noise floor).
  - The RF front-end, ADC buffers, 1D Range FFT, and 2D Doppler FFT `detMatrix` are 100% operational and seeing the target clearly.
  - The 0-object failure was purely due to TI CFAR HWA Doppler gating rejecting the 0-Doppler static peak, starving the downstream pipeline.

Requirements for your implementation:
1. Ensure the native Two-Stage CASO-CFAR engine on C674x DSP scans `detMatrix` across all 256 range bins so that strong static peaks like Bin 50 (2.10m, 0 Doppler) are immediately detected and passed as candidates.
2. Ensure `nav_point_gen.c`'s `emitObstacleBoxes` clusters this peak into a 32-byte `ObstacleBox` (`<7f2h`) with $y \approx 2.10\text{ m}$.
3. Verify using `compare_and_replay.py` on `new_trial_20261014_105925.pkl` or live capture that objects > 0.
4. Strictly preserve the constraint: DO NOT edit files in `C:\ti\...`. All code must be in `oob_1843_dss` and `oob_1843_mss`. Rebuild and update `D:\Dinh\CT_Verse\Dev_code\radar-pipeline\binaries\algo_new_bbox32.bin`.


## 2026-10-14T06:51:27Z

[USER ACTION COMPLETED] The user has successfully flashed `algo_new_bbox32.bin` into QSPI flash memory on the IWR1843BOOST board, reset the board, and set SOP jumpers into Functional Mode. Hardware is now executing the new Two-Stage CASO-CFAR + 3D Bounding Box firmware directly from flash on COM9 and COM8. Active live hardware capture `flashed_boot` has been triggered to validate cold-boot execution.


## 2026-10-14T07:02:30Z

The user has successfully flashed the new firmware binary `algo_new_bbox32.bin` to the IWR1843BOOST board QSPI flash, verified SOP jumpers are set to functional run mode (SOP2 open), and reset the board. COM8 and COM9 are active and ready. Proceed with running capture and test verification on hardware.

## 2026-10-14T07:30:41Z

[CRITICAL OPERATIONAL CONSTRAINT FROM USER]
Whenever the firmware is rebuilt and a new `.bin` file (`algo_new_bbox32.bin`) is generated:
1. The agent/team MUST STOP immediately and notify the user to flash the new `.bin` file into the IWR1843BOOST board.
2. DO NOT run any hardware tests (COM8/COM9, `capture_experiment.py`, `run_controller_test.py`) until the user explicitly confirms that the flash is completed and the board is reset into functional mode.
3. Only after the user confirms completion may the verification tests proceed.

## 2026-10-14T07:42:49Z

[USER ACTION COMPLETED - ROUND 2 FLASH]
The user has confirmed:
"Đã flash thành công firmware mới build và đã chuyển SOP về mode Functional thành công, bây giờ bạn có thể tiếp tục công việc"
The newly built `algo_new_bbox32.bin` (CRC32 `27a85109`, 339,460 bytes) is now flashed into QSPI flash memory, SOP jumpers are set to Functional Run Mode (SOP2 open), and the board has been reset. COM8 and COM9 are active and unblocked for hardware verification and Review Round 3.




## 2026-10-14T07:31:18Z

CRITICAL OPERATIONAL DIRECTIVE FROM USER:
Whenever firmware is rebuilt and a new multicore appimage `algo_new_bbox32.bin` is generated:
1. You MUST STOP execution immediately and report to user asking them to flash `algo_new_bbox32.bin` onto the physical IWR1843BOOST board.
2. DO NOT run any hardware tests (`capture_experiment.py`, `run_controller_test.py`, COM8/COM9 interaction) until the user explicitly confirms that the flash is completed and the board is reset into functional mode.
3. Only resume hardware testing after user confirmation.


## 2026-10-14T07:43:37Z

USER CONFIRMATION RECEIVED:
"Đã flash thành công firmware mới build và đã chuyển SOP về mode Functional thành công, bây giờ bạn có thể tiếp tục công việc"
The user has successfully flashed `algo_new_bbox32.bin` (CRC32 `27a85109`, 339,460 bytes) to the IWR1843BOOST board and reset the board into Functional Run Mode. COM8 and COM9 are active and unblocked.
Proceed with dispatching Review Round 3 and conducting hardware verification tests.
