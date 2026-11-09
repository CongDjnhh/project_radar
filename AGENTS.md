# Agent Engineering Rules & Governance Standards

## 1. Epistemic Rigor & Anti-Sycophancy Protocol (Tư duy Phản biện & Dựa trên Bằng chứng)

1. **No Blind Agreement (Cấm đồng thuận mù quáng):**
   - Never agree with a user hypothesis, architectural proposal, or debugging assumption without first verifying it against the actual source code, hardware manuals, linker scripts, and SDK contracts.
   - If a user's request violates hardware constraints, real-time deadlines, memory budgets, ABI alignment, or protocol compatibility, the agent **MUST push back** with concrete evidence (file paths, line numbers, struct sizes, cycle/bandwidth math) before writing code.
2. **Evidence Classification:**
   - Every technical claim or design decision must distinguish between:
     - `[SOURCE]`: Verified directly in the active repository source, headers, or linker scripts.
     - `[SDK/HW-DOC]`: Verified in official SDK documentation, TRM, or register specifications for the exact silicon/SDK version.
     - `[DERIVED]`: Proven via mathematical derivation (memory layout math, link budget, timing/baudrate calculation).
     - `[INFERENCE]`: Unverified hypothesis requiring inspection or runtime measurement. Never present `[INFERENCE]` as `[SOURCE]`.

---

## 2. Hardware-Agnostic Discovery First (Không giả định cứng kiến trúc phần cứng)

Never hardcode assumptions from one radar platform onto another. Before proposing or modifying any code, explicitly identify the target hardware profile from project files (`.projectspec`, `.syscfg`, linker `.cmd`, makefile, or headers):

- **Core Topology & ISA:** Single-core (e.g., Cortex-M4F), Dual-core (e.g., Cortex-R4F + C674x), Tri-core (e.g., Cortex-R5F + C66x + Cortex-M4F), or Host/SoC processor.
- **SDK & RTOS Generation:** TI-RTOS (SYS/BIOS), FreeRTOS, NoRTOS, POSIX, or bare-metal; mmWave SDK 3.x vs MCU+ SDK 4.x vs Low-Power SDK 5.x.
- **Accelerator & Waveform Architecture:** HWA 1.x vs HWA 2.x; TDM-MIMO vs BPM-MIMO vs DDMA-MIMO; uncompressed vs BFP-compressed Radar Cube.
- **Inter-Core Communication (IPC) & Shared Memory:** Mailbox/DPM + HSRAM vs IPC Notify/RPMessage + Shared L2/L3 RAM.

---

## 3. Memory, IPC & ABI Contract Integrity

1. **Cross-Core ABI Guard:**
   - Any data structure shared across cores, processors, or external transport interfaces must use explicit fixed-width types (`uint8_t`, `int16_t`, `uint32_t`, `float`), explicit padding fields, and deterministic alignment.
   - Enforce compile-time size and alignment assertions (`_Static_assert` or `typedef char ...[(sizeof(T) == EXPECTED) ? 1 : -1]`) on both producer and consumer cores.
   - Always verify address translation requirements across cores/DMA domains (`SOC_translateAddress` or MPU/RAT mappings) and cache coherence (`CacheP_wb`, `CacheP_inv`) before accessing shared buffers.
2. **Buffer Headroom & Capacity Math:**
   - Never compute maximum element counts by dividing total shared buffer size solely by element size:
     $$\text{MaxElements} = \left\lfloor \frac{\text{TotalBufferBytes} - (\text{Headers} + \text{Metadata} + \text{AlignmentPadding} + \text{SafetyReserve})}{\text{sizeof(Element)}} \right\rfloor$$
   - Every buffer write path must enforce strict runtime bounds checking prior to memory copy or DMA transfer.

---

## 4. Real-Time & Bandwidth Determinism

1. **Frame Deadline & Cycle Budget:**
   - Any processing added to an active radar frame or inter-frame window must account for worst-case execution time (WCET) plus IPC/export overhead, ensuring total frame processing remains strictly within the configured frame/subframe period with safety margin.
2. **Transport & Wire-Time Budget:**
   - Before adding or expanding output packets (UART, CAN-FD, SPI, Ethernet/LVDS), verify wire transmission time against baudrate/bitrate and frame period:
     $$T_{\text{tx}} = \frac{\text{TotalPacketBytes} \times \text{BitsPerByteEncoding}}{\text{BaudRate}}$$
   - Ensure $T_{\text{dsp/mcu}} + T_{\text{tx}} + T_{\text{margin}} < T_{\text{frame}}$.

---

## 5. End-to-End Pipeline Co-Design (Firmware ↔ Transport ↔ Host/Visualizer)

1. **Producer-Consumer Synchronization:**
   - A change to any output packet format, TLV payload, CLI command, or state flag in Firmware is **incomplete** until all downstream consumers (Host parsers, Visualizers, telemetry tables, 3D/2D rendering layers, and configuration profiles) are audited and updated in lockstep.
2. **Order-Independent & Defensive Parsing:**
   - Host parsers and visualizers must not assume fixed arrival order of optional sub-packets/TLVs within a frame, must validate payload lengths against declared element sizes, and must properly update rendering bounding volumes/culling states when geometry changes dynamically.

---

## 6. Workspace Hygiene & Codebase Navigation

1. **Build Tree Purity:**
   - **Never** create auxiliary scripts (`.py`, `.sh`, `.ps1`, `.js`), temporary scratch files, or test artifacts inside firmware build project directories (e.g., `*_mss`, `*_dss`, CCS project roots) where IDE auto-discovery or makefiles may glob them or pollute version control. Store offline scripts strictly in dedicated tool/scratch directories outside the firmware project trees.
2. **Domain & Knowledge Graph Navigation:**
   - Consult `GLOSSARY-MAP.md`, per-domain `GLOSSARY.md` (`docs/domains/<context>/GLOSSARY.md`), and `docs/adr/` to align with domain terminology and architectural decisions.
   - When exploring C codebases (`oob_1843_dss`, `oob_1843_mss`, `awr2x44p`, `awr2944_hecr`), query `graphify-out/graph.json` or `graphify-out/GRAPH_REPORT.md` first to map symbol relationships and call hierarchies, **then always verify against the live source files** to guard against stale graph indices after recent edits.
3. **Issue Tracker & Triage:**
   - GitHub issues on `CongDjnhh/project_radar` (`docs/agents/issue-tracker.md`) using the canonical triage vocabulary (`needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix` in `docs/agents/triage-labels.md`).
