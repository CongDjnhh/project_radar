# IWR1843 Out-of-Box firmware

Workspace chứa hai project phối hợp để tạo firmware cho IWR1843:

- `oob_1843_dss`: chương trình C674x/DSS xử lý tín hiệu và chuỗi phát hiện bằng HWA.
- `oob_1843_mss`: chương trình R4F/MSS điều khiển radar, CLI, UART/LVDS và đóng gói firmware đa lõi.

## Môi trường đã kiểm chứng

- TI mmWave SDK `03.06.02.00-LTS` tại `C:/TI/mmwave_sdk_03_06_02_00-LTS`.
- Code Composer Studio `21.0.1` tại `C:/TI/ccs2101`.
- XDCtools `3.50.08.24_core`, SYS/BIOS `6.73.01.01`.
- TI C6000 compiler `8.3.3`, TI ARM compiler `16.9.6.LTS`.

Các đường dẫn công cụ trong makefile của thư mục build được CCS sinh theo máy. Khi chuyển máy, nên import lại hai project vào CCS và để CCS tái tạo makefile.

## Thứ tự build

Luôn build DSS trước MSS vì bước hậu xử lý MSS cần file RPRC của DSS. Import bản sao vào workspace build riêng để giữ nguyên artifact và thay đổi đang có trong source workspace. Chỉ chạy hai lệnh import một lần cho workspace build mới; khi sửa source sau đó phải đồng bộ bản sao trước khi build.

```powershell
$cli = 'C:/TI/ccs2101/ccs/eclipse/ccs-server-cli.bat'
$source = 'C:/Users/Admin/workspace_ccstheia'
$build = "$source/.build-surface"
& $cli '-workspace' $build '-application' 'projectImport' '-ccs.location' "$source/oob_1843_dss" '-ccs.copyIntoWorkspace'
& $cli '-workspace' $build '-application' 'projectImport' '-ccs.location' "$source/oob_1843_mss" '-ccs.copyIntoWorkspace'
& $cli '-workspace' $build '-application' 'projectBuild' '-ccs.projects' 'oob_1843_dss' '-ccs.configuration' 'isk' '-ccs.buildType' 'full' '-ccs.listProblems'
& $cli '-workspace' $build '-application' 'projectBuild' '-ccs.projects' 'oob_1843_mss' '-ccs.configuration' 'isk' '-ccs.buildType' 'full' '-ccs.listProblems'
```

Firmware dùng để flash là:

```text
.build-surface/oob_1843_mss/isk/oob_1843_isk.bin
```

Ảnh hợp lệ phải có magic `MSTR`, ba input MSS/BSS/DSS và core ID lần lượt là `0x35510000`, `0xB5510000`, `0xD5510000`. Kiểm tra cả CRC64, CRC32 và DSS RPRC thực tế được nhúng:

```powershell
& C:/TI/ccs2101/ccs/ccs_base/DebugServer/ca-modules/tools/python3.14/python.exe oob_1843_dss/tests/validate_surface.py --image .build-surface/oob_1843_mss/isk/oob_1843_isk.bin --dss .build-surface/oob_1843_dss/isk/oob_1843_dss_isk.bin
```

Không dùng artifact có tiền tố `out_of_box_1843_` hoặc coi binary trong thư mục build nguồn là kết quả của workspace build riêng. MSS `makefile.defs` giữ dependency vào DSS RPRC để thay đổi chỉ ở DSS vẫn kích hoạt đóng gói MSS.

## Cấu hình board

- `isk`: đã build thành công; chưa nghiệm thu thuật toán và timing trên phần cứng.
- `aop`: có trong metadata CCS nhưng chưa được build/nghiệm thu trong đợt này; chọn đồng nhất cấu hình cho cả hai lõi và xác minh lại antenna/profile trước khi sử dụng.

Profile nghiệm thu là `oob_1843_dss/profile_caso.cfg`: ISK 3TX/4RX, 256 range bins, 16 Doppler bins, chu kỳ ban đầu 50 ms. HWA CFAR → DenseElevation → clustering xét toàn bộ ứng viên hợp lệ trong dung lượng 1.536 CFAR/3.072 mẫu. TLV20 giữ record 32 byte; TLV21 phiên bản 1 cung cấp trạng thái frame 48 byte. `surfaceMotionCfg <subFrameIdx> <mode>` chỉ nhận khi sensor dừng: `0` chuyển động, `1` cố định, `-1` chọn mọi subframe.

[Báo cáo thiết kế, TLV/CLI, budget và nghiệm thu](docs/oob_1843_firmware_validation.md) ghi rõ map, hash ảnh, fixture và phần còn cần đo. Chạy kiểm thử offline bằng `validate_surface.py` không có tham số. Bản tự kiểm thử riêng nằm tại `.build-surface/oob_1843_dss/isk-selftest/oob_1843_isk_selftest.bin`; đây không phải ảnh đo timing sản phẩm.

Chỉ thao tác board khi người dùng xác nhận đã kết nối/flash/reset. Gửi cấu hình trước `sensorStart`; thay TX count/mask cần reset board; chỉ một tiến trình được mở UART dữ liệu. Nghiệm thu cần tối thiểu 10.000 phép đo export cho mỗi chế độ và kiểm tra stack/heap/pool thực tế. Build thành công không chứng minh 20 Hz hoặc không mất ứng viên trong HWA.

`objdetrangehwa.c` là processing-chain MSS cũ không được đăng ký bởi ứng dụng hiện tại. File được giữ làm mã tham khảo nhưng bị loại khỏi cả hai cấu hình build để tránh tạo object chết và gây nhầm lẫn với processing-chain DSS đang hoạt động.

## Cảnh báo C674x `#10370-D`

Project DSP legacy `out_of_box_1843_dss` chia `.text` qua nhiều vùng L2, vì vậy linker có thể tạo trampoline từ code AoA tới `__push_rts`/`__pop_rts`. Đây là diagnostic đã được TI xác nhận là dự kiến trên các DSP xWR và không nên xử lý bằng `--disable_push_pop`, vì tùy chọn đó làm tăng code size trên toàn ứng dụng.

`c674x_linker.cmd` giữ nguyên tối ưu push/pop và chỉ suppress diagnostic `10370`. Không suppress các warning linker khác. Tham khảo [TI mmWave SDK FAQ](https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/856028/faq-software-related-queries-based-on-mmwave-sdk) và [giải thích của TI về push/pop](https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/638908/awr1642boost-compiling-warming-for-the-mmwave-sdk-1-1-0-2-10370-d-dsp).
