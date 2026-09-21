# Ghi chú thuật toán

Tài liệu này mô tả các lựa chọn thuật toán trong `laban4`. Bộ tính heading khi chạy được thiết kế theo **vector 3D**, không phụ thuộc trực tiếp vào góc Euler roll/pitch/yaw.

## 1. Hệ tọa độ

Hệ trục cơ thể được cố định như sau:

- +X: hướng lên đầu người đeo
- +Y: hướng sang tay trái người đeo
- +Z: hướng vào trong cơ thể
- -Z: hướng tiến cần xác định góc phương vị

Vector hướng tiến là:

~~~text
f = [0, 0, -1]^T
~~~

Dữ liệu của từng cảm biến được hiệu chỉnh trong hệ trục riêng trước, sau đó mới được ánh xạ sang hệ trục cơ thể bằng một phép hoán vị trục có dấu.

## 2. Hiệu chuẩn gia tốc kế và từ kế

Một vector lý tưởng có độ lớn không đổi sẽ tạo thành hình cầu khi quay cảm biến qua mọi hướng. Sai số offset, scale và tương tác chéo giữa các trục làm đám mây mẫu biến thành ellipsoid lệch tâm.

Firmware fit dạng đại số:

~~~text
x^T A x + b^T x = 1
~~~

trong đó A là ma trận đối xứng.

Vector ẩn gồm 9 hệ số:

~~~text
[Axx, Ayy, Azz, Axy, Axz, Ayz, bx, by, bz]
~~~

Với mỗi mẫu, hàng đặc trưng là:

~~~text
[x^2, y^2, z^2, 2xy, 2xz, 2yz, x, y, z]
~~~

Firmware cộng dồn hệ phương trình chuẩn trực tiếp trên ESP32 rồi giải hệ 9x9 bằng Gauss-Jordan có chọn pivot.

Tâm ellipsoid:

~~~text
c = -0.5 A^-1 b
~~~

Sau phép tịnh tiến x = y + c:

~~~text
y^T Q y = 1

Q = A / (1 + c^T A c)
~~~

Để ellipsoid có ý nghĩa vật lý, Q phải xác định dương.

Phân rã trị riêng của ma trận đối xứng 3x3:

~~~text
Q = V D V^T
~~~

Ma trận hiệu chỉnh:

~~~text
S = V sqrt(D) V^T
~~~

Mẫu sau hiệu chỉnh:

~~~text
x_hieuchinh = S (x_tho - c)
~~~

Đối với từ kế, phép biến đổi này đồng thời bù:

- lệch tâm hard-iron
- sai lệch scale do soft-iron
- tương tác chéo đối xứng giữa các trục

Firmware từ chối kết quả fit nếu:

- số mẫu quá ít
- dữ liệu không phủ đủ cả ba trục
- hệ phương trình suy biến
- Q không xác định dương
- sai số RMS sau hiệu chỉnh quá lớn
- condition number quá cao

## 3. Hiệu chuẩn con quay hồi chuyển

ITG-3205 được cấu hình ở thang ±2000 độ/giây.

Độ nhạy danh định dùng trong firmware:

~~~text
14.375 LSB / (độ/giây)
~~~

Module phải đứng yên trong khi thu 500 mẫu, tương đương khoảng 5 giây.

Với mỗi trục:

~~~text
bias = trung_binh(raw)
noise = do_lech_chuan(raw) / 14.375
~~~

Nếu nhiễu vượt ngưỡng cho phép, firmware coi module đã bị di chuyển và không lưu bias đó.

## 4. Ước lượng vector trọng lực / hướng lên

Gia tốc kế cho tham chiếu tuyệt đối theo trọng lực khi hệ gần tĩnh, nhưng khi người đeo chuyển động nó còn chứa gia tốc tuyến tính.

Gyro cho động học quay ngắn hạn tốt nhưng nếu tích phân lâu sẽ bị trôi.

Vì vậy `laban4` dùng bộ lọc bổ sung theo vector.

Gọi `u` là vector hướng lên của thế giới biểu diễn trong hệ trục cơ thể và `omega` là vận tốc góc đã hiệu chỉnh.

Một vector cố định trong thế giới khi quan sát từ vật đang quay tuân theo:

~~~text
du/dt = -omega x u
      =  u x omega
~~~

Bước lan truyền bằng gyro:

~~~text
u <- normalize(u + (u x omega) dt)
~~~

Sau đó hướng gia tốc kế đã chuẩn hóa được trộn dần trở lại vào `u`.

Khi độ lớn gia tốc lệch xa 1 g, trọng số hiệu chỉnh từ accelerometer bị giảm vì đó là dấu hiệu có gia tốc tuyến tính.

## 5. Vector Bắc có bù nghiêng

Gọi `m` là vector từ trường đã hiệu chỉnh và `u` là vector hướng lên đơn vị.

Loại bỏ thành phần từ trường theo phương thẳng đứng:

~~~text
n_h = m - u (m dot u)
n   = normalize(n_h)
~~~

`n` là hướng Bắc từ nằm trên mặt phẳng ngang, biểu diễn trong hệ trục cơ thể.

Làm tương tự với vector hướng tiến `f`:

~~~text
f_h = f - u (f dot u)
f_p = normalize(f_h)
~~~

Nếu độ lớn `f_h` gần bằng 0, nghĩa là hướng tiến -Z gần thẳng đứng. Khi đó azimuth của chính vector này trở nên kém xác định về mặt toán học, nên firmware giữ trạng thái heading không hợp lệ thay vì tạo số ngẫu nhiên.

## 6. Tính heading

Với hệ cơ sở ngang kiểu ENU:

~~~text
east = normalize(north x up)
~~~

Azimuth từ của hướng tiến:

~~~text
heading_mag = atan2(f_p dot east,
                    f_p dot north)
~~~

Kết quả được đưa về khoảng 0..360 độ.

Heading theo Bắc thật:

~~~text
heading_true = heading_mag + declination
~~~

Firmware dùng quy ước độ từ thiên phía Đông là số dương.

## 7. Lọc góc theo đường tròn

Bộ lọc thấp thông thường trên số đo góc sẽ sai quanh hướng Bắc. Ví dụ trung bình số học của 359° và 1° là 180°, trong khi hướng thực nằm gần 0°.

`laban4` biến góc thành điểm trên đường tròn đơn vị:

~~~text
cx = cos(heading)
sy = sin(heading)
~~~

Hai thành phần này được lọc thấp, chuẩn hóa lại rồi đổi ngược về heading bằng `atan2`.

Khi tốc độ quay từ gyro tăng, bộ lọc được cho đáp ứng nhanh hơn.

## 8. Kiểm tra nhiễu từ

Sau khi hiệu chuẩn ellipsoid thành công, độ lớn vector từ kế đã hiệu chỉnh phải gần 1.

Trong lúc chạy, firmware loại bỏ những mẫu có độ lớn từ trường lệch quá xa phạm vi cho phép trước khi cập nhật heading.

Kiểm tra này chỉ phát hiện được nhiễu mạnh theo độ lớn. Một nguồn từ trường cục bộ vẫn có thể làm xoay vector mà không thay đổi nhiều độ lớn, vì vậy vị trí lắp cảm biến và việc thử nghiệm thực tế vẫn rất quan trọng.

## 9. Vì sao không cần công cụ hiệu chuẩn trên máy tính

Mục tiêu thiết kế là người dùng có thể hiệu chuẩn tại hiện trường chỉ với:

- ESP32
- GY-85 đang kết nối
- Arduino Serial Monitor

Do đó toàn bộ các bước sau đều chạy trực tiếp trên ESP32:

- thu mẫu
- fit ellipsoid
- kiểm tra chất lượng dữ liệu
- tính ma trận hiệu chỉnh
- lưu NVS

Không cần Python, MATLAB hoặc chương trình hiệu chuẩn ngoài.

## Tài liệu tham khảo

NXP/Freescale AN4248:  
https://www.nxp.com/docs/en/application-note/AN4248.pdf

NXP/Freescale AN4246:  
https://www.nxp.com/docs/en/application-note/AN4246.pdf

Analog Devices ADXL345 Data Sheet:  
https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf

TDK/InvenSense ITG-3200 Product Specification:  
https://invensense.tdk.com/wp-content/uploads/2015/02/ITG-3200-Datasheet.pdf

Honeywell HMC5883L Data Sheet:  
https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
