# Glossary: AWR2944 High-End Corner Radar (HECR) Project

## Autonomous Boot
The legacy operational mode of the High-End Corner Radar (HECR) demo where radar RF frontend chirping, profile, frame configuration, and signal processing chains are initialized automatically at power-on from hardcoded C header values (`chirp_design_HECR.h`), without waiting for serial CLI configuration commands.

## Dynamic CLI Configuration
The target operational mode where the radar boots into an idle state, opens a command-line interface (CLI) on UART0 (`115200` baud), waits for a standard `.cfg` profile sent from a host terminal or Visualizer, and allows runtime parameter updates (CFAR thresholds, FOV limits, Tracker gating) via dynamic IOCTL commands while the radar is running or between frames without requiring re-flashing.

## DDMA (Doppler Division Multiple Access)
A MIMO radar modulation technique utilized by the AWR2944 hardware accelerator (HWA 2.1) and DSP where multiple transmit (TX) antennas emit chirps simultaneously with unique phase shifts per chirp, separating virtual antenna channels in the Doppler domain.

## HECR (High-End Corner Radar)
TI reference design application for 77GHz automotive corner radar running across three cores on the AWR2944: Cortex-R5F (MSS), C66x DSP (DSS), and Cortex-M4F (DPC/HWA coordinator).

## MSS (Master Subsystem - Cortex-R5F)
The Cortex-R5F core responsible for system initialization, UART CLI communication, GTRACK / RANSAC target tracking, TLV packet framing, and streaming output data at 3.125 Mbps.

## DSS (DSP Subsystem - TMS320C66x DSP)
The C66x DSP core responsible for Angle of Arrival (AoA) estimation, quadratic peak interpolation, 3D Point Cloud generation, and radar subsystem (BSS) link management via mmWaveLink.

## M4 / DPC (Data Path Component - Cortex-M4)
The Cortex-M4 core executing the low-level radar signal processing chain (Range FFT, Doppler FFT, DDMA decoding) leveraging HWA 2.1 (Hardware Accelerator) and EDMA.
