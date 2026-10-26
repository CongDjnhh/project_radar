# IWR1843 Out-of-Box Radar

Firmware radar hai lõi tạo quan sát vật cản từ phép đo range–Doppler. [Báo cáo thiết kế và nghiệm thu](../../oob_1843_firmware_validation.md) quy định interface, dung lượng và bằng chứng kiểm chứng.

## Language

**MSS**:
Lõi điều khiển radar và xuất kết quả của hệ thống IWR1843.

**DSS**:
Lõi xử lí tín hiệu và tạo quan sát vật cản của hệ thống IWR1843.

**DPC / DPU**:
DPC là chuỗi xử lí radar; DPU là đơn vị xử lí một công đoạn trong chuỗi đó.

**HWA / EDMA**:
HWA là bộ tăng tốc tín hiệu radar; EDMA là bộ chuyển dữ liệu giữa các vùng nhớ và ngoại vi phục vụ chuỗi xử lí.

**Radar cube**:
Tập mẫu radar phức theo anten, chirp và range của một phép đo.

**CASO**:
Phương pháp CFAR lấy ước lượng nhiễu nhỏ hơn từ hai cửa sổ tham chiếu. Doppler bằng không vẫn là ứng viên phát hiện hợp lệ.

**Full-frame coverage**:
Độ phủ xử lí toàn miền FOV được cấu hình trong mỗi frame, không luân phiên miền giữa các frame. Danh sách trả về từ một tầng có giới hạn không tự chứng minh độ phủ này.
_Avoid_: coverage đã xác nhận chỉ vì build thành công

**DenseElevation**:
Phép ước lượng hỗ trợ góc azimuth/elevation từ các ứng viên range–Doppler.

**Angular support**:
Miền hướng đo biểu diễn bởi direction cosine u/v và độ rộng của chúng, không phải một điểm hình học chính xác.

**SurfaceGen / ObstacleBox**:
SurfaceGen là phép gom các quan sát liên kết thành cụm; ObstacleBox là hộp XYZ bao hỗ trợ đo của một cụm vật cản.
_Avoid_: waypoint, lệnh tránh vật cản

**Frame rank**:
Thứ hạng box trong frame hiện tại, không mang danh tính xuyên frame.
_Avoid_: track ID ổn định

**TLV20**:
Bản tin các box vật cản của frame.

**TLV21**:
Bản tin trạng thái xử lí, giới hạn dung lượng, độ tin cậy coverage và timing của frame.

**Mobile mode**:
Chế độ radar chuyển động, chỉ xuất vật cản đo trong frame hiện tại.

**Fixed mode**:
Chế độ radar cố định, có thể giữ tọa độ đo cuối trong thời hạn persistence khi mất detection.

**Persistence**:
Việc giữ quan sát còn hạn của cùng subframe, không phải dự đoán chuyển động hay tracking.

**RESULT_EXPORTED**:
Xác nhận hoàn tất sử dụng kết quả frame, giải phóng quyền sở hữu để chuẩn bị phép đo tiếp theo.

**Completed export interval**:
Khoảng thời gian từ frame start đến hoàn tất xuất kết quả và công việc chuẩn bị bắt buộc cho phép đo tiếp theo.
_Avoid_: chỉ thời gian UART, chỉ thời gian thuật toán

**Suspected saturation**:
Tình trạng bộ đếm chạm dung lượng, có khả năng mất ứng viên nhưng chưa xác định chính xác lượng mất.
_Avoid_: số detection đã mất được chứng minh
