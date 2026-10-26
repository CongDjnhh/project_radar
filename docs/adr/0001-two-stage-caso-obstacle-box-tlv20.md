# ADR 0001: Full-frame HWA CASO and obstacle output

Status: accepted design, 2026-10-22. Hardware qualification is pending.

## Context

The IWR1843 ISK application consumes range-Doppler detections to produce bounded obstacle descriptions. Detection coverage, memory limits, UART serialization and result ownership determine the sustainable frame period. Counts after a capped HWA list do not prove that every raw detection survived.

## Decision

Use SDK HWA CASO in Doppler then range, intersecting the results and applying configured FOV. Zero Doppler remains eligible. Process every returned candidate in the configured range during the same frame, grouping by range so the resident FFT cannot be confused with another range.

Support 1,536 CFAR candidates, 3,072 dense/cluster inputs and 255 exported boxes. Reject unsupported FOV/dimension combinations at configuration. Cluster complete inputs before output selection; retain range/azimuth representation and report limits explicitly.

Export conservative XYZ bounding boxes as unchanged 32-byte TLV20 records. Add 48-byte versioned TLV21 for counts, limiting/uncertainty flags, packet wire time and tagged completed-export timing. Current-frame rank is not a persistent tracking identity. Classification remains a geometric heuristic.

Default mobile policy emits current observations only. Fixed-radar policy may hold last measured coordinates for at most 100 ms in frame time. History is separate per subframe and resets on lifecycle/configuration changes. No extrapolation uses an unverified Doppler sign.

Preserve SDK source, radar-cube/HWA/EDMA contracts and the result-export acknowledgement. Custom obstacle output with unbounded LVDS waits is rejected. MSS packaging depends on the current DSS image.

## Resource and validation consequences

The baseline is 3TX/4RX, 256 range bins, 16 Doppler bins and FOV 0.15–8.64 m, initially 20 Hz. Measure the complete frame ownership interval and select a period of at least measured maximum divided by 0.8, rounded upward to 1 ms and no shorter than 50 ms. Maximum UART output can require a slower rate; do not silently thin a frame to claim 20 Hz.

HWA intermediate banks clamp before FOV/intersection. Coverage remains unverified until source/reference fixtures and hardware pressure tests establish its limits. Static linker fit, offline tests, target execution, runtime allocation and sustained timing are distinct acceptance gates.

The [firmware validation report](../oob_1843_firmware_validation.md) defines wire offsets, flags, build evidence, target fixtures and hardware acceptance criteria.
