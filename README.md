# 🤖 Dual-ESP32 Autonomous Maze Solver & Warehouse Robot

Dự án Robot tự hành giải mê cung và hỗ trợ vận chuyển hàng hóa dựa trên kiến trúc phần cứng phân tán **Dual-ESP32 (Master-Worker)**. Robot sử dụng hệ thống định vị hỗn hợp **Dead Reckoning (Encoder + IMU MPU6050)** kết hợp ô lưới (Grid-based Cell Navigation) để khám phá sa bàn, nhận diện tường biên bằng vạch line đen và phát hiện hàng hóa bằng cảm biến siêu âm.

---

## 📌 1. Tổng quan kiến trúc hệ thống (Master-Worker Architecture)

Hệ thống được chia thành 2 vi điều khiển ESP32 giao tiếp với nhau qua giao thức **UART (Serial2)** nhằm tối ưu hóa hiệu năng, tránh nghẽn cổ chai giữa việc xử lý giao diện mạng cấp cao và thời gian thực của động cơ:

### 🎛️ ESP32 Master (Gateway & Sensor Hub)
* **Nhiệm vụ chính:** Đóng vai trò là trung tâm đầu não cấp cao.
* **Chức năng:**
  * Phát mạng **Wi-Fi Access Point (AP)** tích hợp Web Server giao diện Dashboard thời gian thực cho phép điều khiển và cấu hình từ xa qua điện thoại/máy tính.
  * Đọc và xử lý trực tiếp cảm biến quán tính **MPU6050 (IMU)** tính toán góc Yaw và duy trì thuật toán khóa hướng (**Locked Heading**).
  * Quản lý trực tiếp cụm **5 mắt cảm biến hồng ngoại (IR)** quét tường biên/vạch line sa bàn và cảm biến siêu âm **HC-SR05** chuyên dụng nhận diện vật thể/hàng hóa.
  * Hiển thị thông số Telemetry và trạng thái hệ thống lên màn hình **OLED 0.96" (I2C)**.

### ⚙️ ESP32 Worker (Motor & Encoder Execution Unit)
* **Nhiệm vụ chính:** Chấp hành điều khiển động cơ thời gian thực.
* **Chức năng:**
  * Điều khiển cầu H **TB6612FNG** quản lý tốc độ và chiều quay của 2 động cơ giảm tốc GA25-370.
  * Đọc xung ngắt (Interrupt) từ **Dual Encoders** để đo quãng đường di chuyển chính xác theo từng ô lưới ($30 \times 30\text{ cm}$).
  * Thực thi các lệnh di chuyển phức tạp từ Master: Tiến thẳng liên tục, Tiến chuẩn 1 ô có cơ chế lùi bù trừ tâm (**Micro-reverse**), và Quay vòng kín (**Closed-loop Turn**).

---

## 🚀 2. Tính năng & Thuật toán cốt lõi (Active Features & Algorithms)

1. **Heading-Locked Straight Navigation (Bám hướng thời gian thực):**
   * Triệt tiêu hiện tượng đi vòng cung của robot bánh xích/bánh lốp bằng cách lấy góc mốc Yaw ban đầu, liên tục tính toán sai số hướng và bù trừ trực tiếp vào xung PWM của 2 bánh.
2. **Bi-directional Closed-Loop Turn & Alignment:**
   * Thuật toán quay góc vuông ($90^\circ, 180^\circ$) sử dụng PID vòng kín với MPU6050, tự động hãm tốc tiệm cận khi gần đến góc mục tiêu để triệt tiêu quán tính cơ khí.
3. **Grid-Cell Dead Reckoning & Micro-Reverse:**
   * Điều hướng theo ô lưới chuẩn $30\text{ cm}$. Khi đi đủ số xung encoder quy định cho một ô (`TICKS_PER_CELL`), robot tự động kích hoạt chu trình lùi nhẹ (Micro-reverse) để đưa trục quay về đúng tâm ô, chuẩn bị cho các hướng rẽ tiếp theo không bị cạ tường.
4. **Web Dashboard Telemetry:**
   * Giám sát trực tiếp trạng thái Yaw, Locked Heading, Sai số bù hướng, dữ liệu cảm biến IR/US và tọa độ Encoder qua trang quản trị Web cục bộ.

---

## pinout 3. Sơ đồ kết nối phần cứng (Pin Mapping)

### 🧩 ESP32 Master Pinout
| Linh kiện / Module | Chân kết nối trên ESP32 Master | Chức năng chi tiết |
| :--- | :--- | :--- |
| **MPU6050 / OLED** | GPIO 21 (SDA), GPIO 22 (SCL) | Bus I2C giao tiếp cảm biến IMU và màn hình |
| **Serial2 (UART)** | GPIO 16 (RX2), GPIO 17 (TX2) | Giao tiếp truyền tin với ESP32 Worker |
| **Cụm 5 Mắt IR** | GPIO 32, 33, 25, 26, 27 | Đọc tín hiệu số (Digital) quét tường biên sa bàn |
| **HC-SR05 Sensor** | Trig: GPIO 13, Echo: GPIO 14 | Đo khoảng cách phát hiện hàng hóa/vật thể |

### 🛠️ ESP32 Worker Pinout
| Linh kiện / Module | Chân kết nối trên ESP32 Worker | Chức năng chi tiết |
| :--- | :--- | :--- |
| **Serial2 (UART)** | GPIO 16 (RX2), GPIO 17 (TX2) | Nhận lệnh điều khiển từ Master |
| **TB6612FNG Driver** | PWMA: 4, AIN1: 13, AIN2: 27, STBY: 26 | Điều khiển động cơ trái (Lưu ý: AIN2 ở GPIO 27 an toàn khi boot) |
| **TB6612FNG Driver** | PWMB: 23, BIN1: 33, BIN2: 19 | Điều khiển động cơ phải |
| **Encoder Trái (X)** | GPIO 25 (A), GPIO 32 (B) | Đọc xung ngắt tốc độ cao bánh trái |
| **Encoder Phải (Y)** | GPIO 34 (A), GPIO 35 (B) | Đọc xung ngắt tốc độ cao bánh phải |

---

## 📂 4. Cấu trúc thư mục dự án

```text
📦 Maze-Solver-Robot-Dual-ESP32
 ┣ 📂 firmware
 ┃ ┣ 📜 ESP32_Master_2026-11-17.ino    # Mã nguồn trung tâm (Web, MPU6050, OLED, Sensors)
   ┗ 📜 ESP32_Worker_2026-11-17.ino    # Mã nguồn chấp hành (Motors, Encoders, PID execution)
 ┣ 📜 Maze_Robot_Version_Log.txt       # Nhật ký quản lý phiên bản và tài liệu kỹ thuật
 ┗ 📜 README.md                        # Tài liệu giới thiệu dự án