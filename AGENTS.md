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

---

## 7. Dynamic Radar Parameter Derivation & Zero Hardcoding Policy (Quy chuẩn Suy diễn Động & Triệt tiêu Magic Numbers)

1. **Zero Hardcoding & Zero Estimation Policy (Nguyên lý Bất biến):**
   - Mọi thuật toán DSP, CFAR, DBSCAN clustering, tracking gating, elevation processing, và surface generation **BẮT BUỘC PHẢI SUY DIỄN ĐỘNG** từ cấu hình sóng thực tế (`profileCfg`, `chirpCfg`, `frameCfg`) và hình học mảng ăng-ten (`antenna_geometry`).
   - **Tuyệt đối cấm** sử dụng số fix cứng (magic numbers) hoặc hằng số ước lượng kinh nghiệm (heuristics / fallback defaults) như `0.25f`, `0.08f * R`, `1.0f`, `12U`, `100000UL`, `dx/dy/dz = 0.5f`, `vRes fallback = 0.25f`, hoặc ma trận hiệp phương sai cố định.
   - Mọi magic number vi phạm được phân loại là **Lỗi kiến trúc nghiêm trọng (`SEV-1`)**. Cấm merge bất kỳ PR/commit nào chứa các giá trị này.

2. **Canonical Physics Derivation Chain [DERIVED] (Chuỗi Suy diễn Vật lý Sóng mmWave):**
   - **Tần số trung tâm & Bước sóng:**
     $$B = S \times T_{\text{ADC}} = S \times \frac{N_{\text{ADC}}}{F_s}, \quad f_c = f_0 + \frac{B}{2} + T_{\text{adcStart}} \cdot S, \quad \lambda = \frac{c}{f_c}$$
     *(Tuyệt đối không giả định cứng $\lambda = 3.9\text{ mm}$ hay $3.8\text{ mm}$).*
   - **Bước bin & Độ phân giải (Range & Doppler):**
     $$\delta r = \frac{c \cdot F_s}{2 S \cdot N_{\text{rangeFFT}}}, \quad \Delta R = \frac{c}{2 B}$$
     $$\delta v = \frac{\lambda}{2 \cdot N_{\text{Tx}} \cdot N_{\text{dopplerFFT}} \cdot T_{\text{chirp}}}, \quad \Delta v = \frac{\lambda}{2 \cdot N_{\text{Tx}} \cdot N_{\text{doppler}} \cdot T_{\text{chirp}}}$$
     $$v_{\max} = \frac{\lambda}{4 \cdot N_{\text{Tx}} \cdot T_{\text{chirp}}}, \quad T_{\text{frame}} = \text{frameCfg.framePeriodicity} \times 5 \times 10^{-6}\text{ s}$$
   - **Góc & Không gian ngang (Spatial & Angular):**
     $$\Delta u_{\text{azim}} = \frac{\lambda}{N_{\text{vAzim}} \cdot d_x} \approx \frac{2}{N_{\text{vAzim}}}, \quad \Delta x(R) = R \cdot \Delta u_{\text{azim}}, \quad \Delta z(R) = R \cdot \Delta u_{\text{elev}}$$

3. **Subsystem Governance Mandates:**
   - **Dense Elevation / 3D Processing:** Bước quét góc directional cosine phải bằng hoặc ước của $\Delta u_{\text{azim}}$, cấm dùng hằng số `0.08f`. Điều kiện liên kết vận tốc giữa các đỉnh phổ phải dựa trên khoảng cách vận tốc vật lý $|\Delta v_{\text{phys}}| = |\Delta \text{bin}| \cdot \delta v \le k \cdot \Delta v$, cấm so sánh bin cứng `> 12U`. Chiều cao $z = R \cdot \sin(\phi)$ phải dùng $\lambda(f_c)$ và $\delta r$ động.
   - **Adaptive CFAR:** Cửa sổ bảo vệ $N_{\text{guard}}$ phải bao trùm kích thước vật lý của mục tiêu: $N_{\text{guard,range}} = \lceil L_{\text{target,range}} / \delta r \rceil$. Hệ số nhân $\alpha = N_{\text{train}} (P_{\text{fa}}^{-1/N_{\text{train}}} - 1)$. Bù trừ tăng ích tích luỹ $G_{\text{int}} = 10 \log_{10}(N_{\text{vAnt}})$.
   - **DBSCAN Clustering:** Bán kính lân cận phải là ellipsoid bất đối xứng theo độ không đảm bảo đo lường: $\epsilon_r = k_r \cdot \delta r$, $\epsilon_x(R) = k_x \cdot R \cdot \Delta u_{\text{azim}}$, $\epsilon_v = k_v \cdot \delta v$. Cấm dùng công thức kinh nghiệm $\epsilon = 0.25 + 0.08R$. Cấm dùng fallback `vRes = 0.25f` — nếu thiếu tham số phải panic/assert ngay lập tức!
   - **Tracking Gating & Filter Covariance:** Chu kỳ lấy mẫu $dt = T_{\text{frame}}$ lấy từ `frameCfg`. Hiệp phương sai đo lường $\sigma^2$ phải tỷ lệ nghịch với SNR theo Cramér-Rao Lower Bounds: $\sigma_r^2 = (\delta r)^2 / (2 \cdot \text{SNR}_{\text{lin}})$. Cổng gating phải dùng khoảng cách Mahalanobis $d_M^2 \le \chi^2_m(1 - P_G)$.
   - **Surface & Obstacle Generation:** Kích thước lưới voxel $dx \ge \delta r$. Thời gian lưu ảnh (hold/persistence) phải tính theo số chu kỳ frame: $N_{\text{hold}} = \lceil T_{\text{hold\_sec}} / T_{\text{frame}} \rceil$, cấm dùng microsecond cố định `#define SURFACE_HOLD_US 100000UL`.

4. **Inter-Core ABI Contract (`MmwDemo_DerivedPhysicsParams`):**
   - MSS RF Parser (`mmwdemo_rfparser.c`) là Single Source of Truth (SSoT), bắt buộc tính toán và đóng gói toàn bộ thông số dẫn xuất vật lý vào struct cố định:
     ```c
     typedef struct MmwDemo_DerivedPhysicsParams_t {
         float    rangeStep;            /* [m/bin] Kích thước bin cự ly FFT */
         float    dopplerStep;          /* [m/s/bin] Kích thước bin vận tốc Doppler FFT */
         float    rangeResolution;      /* [m] Độ phân giải cự ly vật lý delta_R = c / (2*B) */
         float    dopplerResolution;    /* [m/s] Độ phân giải vận tốc vật lý delta_v */
         float    wavelength;           /* [m] Bước sóng lambda = c / fc */
         float    centerFreq;           /* [Hz] Tần số trung tâm fc */
         float    bandwidth;            /* [Hz] Băng thông quét hiệu dụng */
         float    chirpInterval;        /* [s] Chu kỳ chirp */
         float    framePeriodSec;       /* [s] Chu kỳ frame */
         float    vMax;                 /* [m/s] Vận tốc tối đa */
         float    rMax;                 /* [m] Cự ly tối đa */
         float    azimuthBeamwidthRad;  /* [rad] Búp sóng Rayleigh Azimuth */
         float    elevationBeamwidthRad;/* [rad] Búp sóng Rayleigh Elevation */
         uint16_t numVirtualAntAzim;     /* Số ăng-ten ảo Azimuth */
         uint16_t numVirtualAntElev;     /* Số ăng-ten ảo Elevation */
         uint16_t reserved[2];          /* Padding 32-bit */
     } MmwDemo_DerivedPhysicsParams;
     _Static_assert(sizeof(MmwDemo_DerivedPhysicsParams) == 60, "ABI size mismatch");
     ```
   - DSS phải nhận struct này qua HSRAM trước khi chạy data path và áp dụng kiểm tra nghiêm ngặt:
     `DebugP_assert((physics->rangeStep > 0.0f) && (physics->dopplerStep > 0.0f) && (physics->wavelength > 0.0f) && (physics->framePeriodSec > 0.0f));`
     Cấm mọi hành vi fallback hoặc gán giá trị mặc định.
