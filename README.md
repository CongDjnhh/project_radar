# IWR1843 Out-of-Box firmware

Workspace chứa hai project phối hợp để tạo firmware cho IWR1843:

- `oob_1843_dss`: chương trình C674x/DSS xử lý tín hiệu và chuỗi phát hiện bằng HWA.
- `oob_1843_mss`: chương trình R4F/MSS điều khiển radar, CLI, UART/LVDS và đóng gói firmware đa lõi.

## Môi trường đã kiểm chứng

- TI mmWave SDK `03.06.02.00-LTS` tại `C:/TI/mmwave_sdk_03_06_02_00-LTS`.
- Code Composer Studio tại `C:/TI/ccs2002`.
- XDCtools `3.50.08.24_core`, SYS/BIOS `6.73.01.01`.
- TI C6000 compiler `8.3.3`, TI ARM compiler `16.9.6.LTS`.

Các đường dẫn công cụ trong makefile của thư mục build được CCS sinh theo máy. Khi chuyển máy, nên import lại hai project vào CCS và để CCS tái tạo makefile.

## Thứ tự build

Luôn build DSS trước MSS vì bước hậu xử lý MSS cần file RPRC của DSS.

```powershell
$env:PATH = 'C:\TI\ccs2002\ccs\utils\bin;' + $env:PATH

Set-Location oob_1843_dss\isk
gmake clean all

Set-Location ..\..\oob_1843_mss\isk
gmake clean all
```

Firmware dùng để flash là:

```text
oob_1843_mss/isk/oob_1843_isk.bin
```

Ảnh hợp lệ phải có magic `MSTR`, ba input MSS/BSS/DSS và core ID lần lượt là `0x35510000`, `0xB5510000`, `0xD5510000`. Không dùng các artifact cũ có tiền tố `out_of_box_1843_`.

## Cấu hình board

- `isk`: cấu hình đã được clean-build và kiểm chứng đầy đủ.
- `aop`: cấu hình antenna-on-package có trong metadata CCS; chọn đồng nhất cấu hình `aop` cho cả DSS và MSS khi sử dụng board AOP.

`objdetrangehwa.c` là processing-chain MSS cũ không được đăng ký bởi ứng dụng hiện tại. File được giữ làm mã tham khảo nhưng bị loại khỏi cả hai cấu hình build để tránh tạo object chết và gây nhầm lẫn với processing-chain DSS đang hoạt động.

## Cảnh báo C674x `#10370-D`

Project DSP legacy `out_of_box_1843_dss` chia `.text` qua nhiều vùng L2, vì vậy linker có thể tạo trampoline từ code AoA tới `__push_rts`/`__pop_rts`. Đây là diagnostic đã được TI xác nhận là dự kiến trên các DSP xWR và không nên xử lý bằng `--disable_push_pop`, vì tùy chọn đó làm tăng code size trên toàn ứng dụng.

`c674x_linker.cmd` giữ nguyên tối ưu push/pop và chỉ suppress diagnostic `10370`. Không suppress các warning linker khác. Tham khảo [TI mmWave SDK FAQ](https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/856028/faq-software-related-queries-based-on-mmwave-sdk) và [giải thích của TI về push/pop](https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/638908/awr1642boost-compiling-warming-for-the-mmwave-sdk-1-1-0-2-10370-d-dsp).
