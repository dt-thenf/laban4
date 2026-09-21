# laban4

La bàn đeo người có **bù nghiêng**, **tự hiệu chuẩn trực tiếp trên ESP32** sử dụng **GY-85**.

Bộ cảm biến GY-85 dùng trong firmware:

- ADXL345 — gia tốc kế 3 trục, I2C `0x53`
- ITG-3205 / ITG-3200 — con quay hồi chuyển 3 trục, I2C `0x68`
- HMC5883L — từ kế 3 trục, I2C `0x1E`

Repo này được xây dựng mới từ đầu, không phụ thuộc các repo la bàn cũ và không cần chương trình hiệu chuẩn trên máy tính.

## Hệ trục của người đeo

Toàn bộ dự án dùng một hệ trục thống nhất:

~~~text
                       +X
                  hướng lên đầu
                        ^
                        |
          +Y <----------+
       sang tay trái    |
                        |
              +Z hướng vào trong cơ thể

          -Z = HƯỚNG TIẾN / HƯỚNG DI CHUYỂN
~~~

Vector hướng cần lấy góc la bàn được cố định trong code:

~~~text
forward_body = [0, 0, -1]
~~~

Vì vậy heading được tính theo **-Z**, không phải mặc định theo cặp trục X/Y của HMC5883L.

## Vì sao vẫn xác định đúng hướng khi module bị nghiêng

Cách tính đơn giản kiểu `atan2(My, Mx)` chỉ phù hợp khi module gần nằm ngang. Khi module nghiêng, thành phần từ trường theo phương thẳng đứng sẽ làm sai góc.

`laban4` sử dụng cách tính theo vector:

1. Ước lượng vector **hướng lên / trọng lực** bằng ADXL345 và ổn định bằng gyro ITG-3205.
2. Hiệu chỉnh HMC5883L để giảm sai số hard-iron và soft-iron.
3. Chiếu vector từ trường đã hiệu chỉnh lên mặt phẳng vuông góc với trọng lực để tìm hướng Bắc từ.
4. Chiếu vector hướng tiến **-Z** lên cùng mặt phẳng ngang.
5. Tính góc có dấu giữa hướng Bắc và hướng tiến.
6. Lọc góc theo đường tròn để không bị nhảy khi chuyển từ 359° sang 0°.
7. Cộng độ từ thiên nếu cần quy đổi sang Bắc thật.

Cách này không phụ thuộc trực tiếp vào roll/pitch/yaw Euler trong phép tính heading chính.

Có một giới hạn vật lý không thể loại bỏ: nếu **-Z gần thẳng đứng**, hình chiếu ngang của hướng tiến gần bằng 0 nên góc phương vị của chính vector này không còn xác định tốt. Firmware sẽ báo `HDG=HOLD` thay vì xuất một góc ngẫu nhiên.

## Hiệu chuẩn hoàn toàn trên module

Không cần Python, MATLAB, file CSV hay công cụ hiệu chuẩn trên PC.

Toàn bộ quá trình hiệu chuẩn được thực hiện qua **Arduino Serial Monitor** và kết quả được lưu vào **NVS của ESP32**.

### Hiệu chuẩn từ kế

Firmware thu các mẫu 3D của HMC5883L và tự fit ellipsoid ngay trên ESP32:

~~~text
(x-c)^T Q (x-c) = 1
~~~

Từ đó firmware xác định:

- tâm lệch hard-iron `c`
- ma trận hiệu chỉnh soft-iron 3x3
- thành phần sai lệch chéo giữa các trục
- sai số RMS của phép fit
- condition number để đánh giá chất lượng hiệu chuẩn

Ma trận thu được biến ellipsoid đo được trở lại gần hình cầu đơn vị.

### Hiệu chuẩn gia tốc kế

ADXL345 được hiệu chuẩn theo mô hình ellipsoid 3D tương tự. Trong quá trình lấy mẫu, firmware loại bỏ những mẫu có gia tốc động quá lớn để tránh làm sai việc xác định 1 g.

### Hiệu chuẩn gyro

ITG-3205 được lấy trung bình zero-rate bias khi module đứng yên. Firmware cũng tính độ lệch chuẩn và từ chối lưu nếu phát hiện module bị di chuyển quá nhiều trong lúc hiệu chuẩn.

## Đấu nối ESP32

Khuyến nghị dùng mức 3.3 V:

| GY-85 | ESP32 |
|---|---|
| 3.3V | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

Có thể đổi chân ở đầu file `laban4.ino`:

~~~cpp
#define LABAN4_SDA 21
#define LABAN4_SCL 22
~~~

Firmware chạy I2C ở 400 kHz.

## Nạp bằng Arduino IDE

1. Cài ESP32 Arduino core.
2. Mở `laban4.ino`.
3. Chọn đúng board ESP32 và cổng COM.
4. Upload firmware.
5. Mở Serial Monitor ở **115200 baud**, bật gửi ký tự xuống dòng.

Firmware không cần thư viện cảm biến bên thứ ba. Chỉ sử dụng các thành phần có sẵn của ESP32 Arduino core:

- `Arduino.h`
- `Wire.h`
- `Preferences.h`

## Khởi động lần đầu

Gõ:

~~~text
scan
~~~

Kết quả mong đợi:

~~~text
0x1E HMC5883L
0x53 ADXL345
0x68 ITG3205
~~~

Sau đó chạy hiệu chuẩn đầy đủ:

~~~text
cal all
~~~

Quy trình:

1. Giữ module **hoàn toàn đứng yên** trong bước hiệu chuẩn gyro.
2. Khi Serial chuyển sang hiệu chuẩn 3D, xoay module chậm qua càng nhiều tư thế càng tốt trong khoảng 40 giây.
3. Thực hiện chuyển động số 8 rộng.
4. Lần lượt đưa cả hai chiều ±X, ±Y, ±Z lên trên/xuống dưới để phủ đủ không gian 3D.
5. Tránh bàn thép, nam châm, loa, động cơ, biến áp, pin lớn và dây dòng cao.

Nếu chất lượng fit đạt yêu cầu, dữ liệu sẽ tự lưu vào NVS.

Kiểm tra bằng:

~~~text
status
~~~

## Các lệnh Serial Monitor

Tên lệnh được giữ bằng tiếng Anh để ngắn và ổn định:

| Lệnh | Chức năng |
|---|---|
| `help` | hiện danh sách lệnh |
| `status` | xem trạng thái hiệu chuẩn, ma trận, mapping và heading |
| `scan` | quét thiết bị I2C |
| `raw` | in một mẫu dữ liệu thô và dữ liệu đã hiệu chỉnh |
| `heading` | in heading hiện tại một lần |
| `stream on` / `stream off` | bật/tắt xuất heading liên tục |
| `rate 10` | đặt tốc độ xuất từ 1 đến 50 dòng/giây |
| `decl 0.0` | đặt độ từ thiên, phía Đông là số dương |
| `cal gyro` | hiệu chuẩn bias gyro khi đứng yên |
| `cal accel` | hiệu chuẩn ellipsoid gia tốc kế trong 30 giây |
| `cal mag` | hiệu chuẩn hard/soft-iron từ kế trong 40 giây |
| `cal all` | chạy toàn bộ gyro + accel + mag |
| `map show` | xem ánh xạ trục cảm biến sang hệ trục cơ thể |
| `map accel +x +y +z` | đặt ánh xạ trục gia tốc kế |
| `map gyro +x +y +z` | đặt ánh xạ trục gyro |
| `map mag +x +y +z` | đặt ánh xạ trục từ kế |
| `factory reset` | xóa hiệu chuẩn/cài đặt và về mặc định |

## Ánh xạ trục cảm biến sang hệ trục cơ thể

Mặc định firmware giả sử trục cảm biến cùng chiều với trục module:

~~~text
body X = +sensor X
body Y = +sensor Y
body Z = +sensor Z
~~~

Biểu diễn bằng:

~~~text
+x +y +z
~~~

Nếu bo GY-85 thực tế có cảm biến được hàn khác hướng, không cần sửa source. Có thể đổi trực tiếp qua Serial Monitor.

Ví dụ:

~~~text
map mag +y -x +z
~~~

Nghĩa là:

~~~text
body X = +mag Y
body Y = -mag X
body Z = +mag Z
~~~

Mỗi trục X/Y/Z phải xuất hiện đúng một lần. Sau khi đổi mapping nên chạy lại hiệu chuẩn của cảm biến tương ứng.

## Đặt độ từ thiên

La bàn từ mặc định chỉ hướng Bắc từ. Muốn quy đổi sang Bắc thật:

~~~text
decl <độ>
~~~

Quy ước:

- độ từ thiên phía Đông: số dương
- độ từ thiên phía Tây: số âm

Ví dụ minh họa:

~~~text
decl 1.25
~~~

Không nên dùng cố định giá trị ví dụ này. Hãy dùng độ từ thiên phù hợp với vị trí và thời điểm thực tế.

## Ví dụ dữ liệu xuất ra

~~~text
HUONG=123.42 do ESE  tho=124.01  A=1.004  M=0.996  ngangTien=0.931  GAM
~~~

Ý nghĩa:

- `HUONG`: heading sau lọc
- `tho`: heading chưa qua lọc tròn
- `A`: độ lớn vector gia tốc đã hiệu chỉnh, gần 1 khi gần tĩnh
- `M`: độ lớn vector từ trường đã hiệu chỉnh, gần 1 sau khi calibration tốt
- `ngangTien`: độ lớn hình chiếu ngang của hướng tiến -Z
- `GAM`: đã có hiệu chuẩn Gyro + Accelerometer + Magnetometer

Nếu `ngangTien` tiến gần 0 thì -Z đang gần thẳng đứng và heading của hướng này trở nên kém xác định.

## Cách kiểm tra sau hiệu chuẩn

Sau khi chạy `cal all`:

1. Đưa module ra xa kim loại.
2. Hướng **-Z** về một hướng Bắc tham chiếu đáng tin cậy.
3. Ghi lại heading.
4. Nghiêng/roll module nhưng vẫn giữ hình chiếu ngang của **-Z** theo cùng hướng.
5. Heading phải giữ tương đối ổn định.
6. Lặp lại với Đông, Nam và Tây.
7. Nếu heading thay đổi mạnh khi nghiêng, kiểm tra lại giá trị A, M, mapping trục và môi trường từ trước khi chỉnh hệ số lọc.

## Giới hạn cần lưu ý

- HMC5883L là cảm biến cũ; nhiều module giá rẻ có thể dùng clone hoặc chip thay thế.
- Firmware này yêu cầu thiết bị tương thích HMC5883L ở địa chỉ `0x1E`. QMC5883L thường ở `0x0D` dùng register map khác và không được tự động coi là HMC5883L.
- Kim loại sắt từ ở gần có thể làm sai la bàn dù calibration đã tốt.
- Nên calibration khi GY-85 đã được lắp vào đúng cụm wearable cuối cùng, vì vít, pin và dây dẫn có thể làm thay đổi từ trường.
- Gia tốc tuyến tính mạnh làm sai tạm thời vector trọng lực từ accelerometer. Gyro giúp giảm ảnh hưởng nhưng GY-85 đời cũ không thể đạt hiệu năng động như IMU điện thoại hiện đại đã được nhà sản xuất hiệu chuẩn.
- Khi chính hướng tiến -Z gần thẳng đứng, azimuth của vector đó về mặt toán học không xác định tốt.

## Tài liệu kỹ thuật tham khảo

Thuật toán dựa trên mô hình vật lý eCompass đã được công bố:

- NXP/Freescale AN4248 — *Implementing a Tilt-Compensated eCompass using Accelerometer and Magnetometer Sensors*  
  https://www.nxp.com/docs/en/application-note/AN4248.pdf
- NXP/Freescale AN4246 — *Calibrating an eCompass in the Presence of Hard- and Soft-Iron Interference*  
  https://www.nxp.com/docs/en/application-note/AN4246.pdf
- Analog Devices — ADXL345 Data Sheet  
  https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
- TDK/InvenSense — ITG-3200 Product Specification  
  https://invensense.tdk.com/wp-content/uploads/2015/02/ITG-3200-Datasheet.pdf
- Honeywell — HMC5883L Data Sheet  
  https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf

Xem thêm `docs/ALGORITHM.md` để đọc chi tiết các phương trình được triển khai trong firmware.
