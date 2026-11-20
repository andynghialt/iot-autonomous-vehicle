# **🤖 Dual-ESP32 Autonomous Maze Solver & Warehouse Robot (4-DOF Arm)**
Dự án Robot tự hành giải mê cung và hỗ trợ vận chuyển hàng hóa tích hợp cánh tay gắp 4 bậc tự do (**4-DOF Robotic Arm**), xây dựng trên kiến trúc phần cứng phân tán **Dual-ESP32 (Master-Worker)**. Robot sử dụng hệ thống định vị hỗn hợp **Dead Reckoning (Encoder + IMU MPU6050)** kết hợp ô lưới (Grid-based Cell Navigation) để khám phá sa bàn, nhận diện vạch biên đen bằng cảm biến hồng ngoại và phát hiện vật thể/hàng hóa bằng cảm biến siêu âm.
## **📌 1. Tổng quan kiến trúc hệ thống (Master-Worker Architecture)**
Hệ thống được chia thành 2 vi điều khiển ESP32 giao tiếp với nhau qua giao thức **UART (Serial2)** nhằm tách bạch việc xử lý giao diện mạng/cảm biến quán tính với điều khiển động cơ và servo thời gian thực:
### **🎛️ ESP32 Master (Gateway & Sensor Hub)**
- **Nhiệm vụ chính:** Đóng vai trò trung tâm điều phối và giao diện điều khiển từ xa.
- **Chức năng:**
  - Phát mạng **Wi-Fi Access Point (AP)** (Maze\_Solver\_AP) tích hợp Web Server giao diện **2 Tab Menu** (Điều khiển xe & Cánh tay 4 bậc) cho phép điều khiển và giám sát từ xa.
  - Đọc và xử lý trực tiếp cảm biến quán tính **MPU6050 (IMU)** tính toán góc Yaw, duy trì **Locked Heading** và lọc nhiễu tĩnh (GYRO\_DEADBAND).
  - Quản lý trực tiếp cụm **5 mắt cảm biến hồng ngoại (IR)** (nền đen = 1, nền trắng = 0) và cảm biến siêu âm **HC-SR05** chuyên dụng nhận diện vật thể/hàng hóa.
  - Hiển thị thông số Telemetry và trạng thái hệ thống lên màn hình **OLED (I2C** 0x3C\*\*)\*\*.
### **⚙️ ESP32 Worker (Motor, Encoder & 4-DOF Arm Unit)**
- **Nhiệm vụ chính:** Chấp hành điều khiển động cơ bánh xe và cụm 4 servo cánh tay.
- **Chức năng:**
  - Điều khiển cầu H **TB6612FNG** quản lý tốc độ và chiều quay của 2 động cơ giảm tốc GA25-370.
  - Đọc xung ngắt (Interrupt) từ **Dual Encoders** để đo quãng đường di chuyển theo từng ô lưới (30 cm).
  - Thực thi các lệnh di chuyển từ Master: Tiến chuẩn 1 ô có cơ chế lùi bù trừ tâm (**Micro-reverse**), Tiến thẳng liên tục, Quay góc kín (**Closed-loop Turn**), và điều khiển cụm **4 Servo** (không attach lúc khởi động, tự động ngắt xung sau 0.5s để chống kẹt trục).
## **🚀 2. Tính năng & Thuật toán cốt lõi (Active Features & Algorithms)**
1. **Heading-Locked Straight Navigation (Bám hướng thời gian thực):**
   1. Giữ mốc góc Yaw ban đầu (lockedHeading), tính toán sai số hướng liên tục và bù trừ trực tiếp vào xung PWM của 2 bánh để hạn chế đi lệch hướng.
1. **Closed-Loop Turn & Alignment:**
   1. Quay góc ($90^\circ, 180^\circ$) và căn chỉnh về gốc dựa trên phản hồi góc Yaw từ MPU6050.
1. **Grid-Cell Dead Reckoning & Micro-Reverse:**
   1. Điều hướng theo ô lưới chuẩn 30 cm. Khi đạt ngưỡng xung encoder (TICKS\_PER\_CELL), robot thực hiện lùi nhẹ (**Micro-reverse**) rồi dừng để đưa trục quay về tâm ô.
1. **2-Tab Web Control & Safe Servo Execution:**
   1. **Tab 1 (Điều khiển xe):** Tiến 1 ô (30cm), Đi thẳng liên tục, Quay trái/phải 90°, Quay đầu 180°, Reset Yaw & Locked = 0°, Stop & căn chỉnh gốc, tinh chỉnh Scale X/Y.
   1. **Tab 2 (Cánh tay 4 bậc - 2 nút / khớp):**
      1. **S1 (Tay gắp SG90):** Chạy từ từ (sweep) tới 90° hoặc 0° để có lực đóng/mở đều.
      1. **S2 & S3 (2 khớp hông SG90):** Hỗ trợ bấm giữ/nhấp nhích góc, bước góc nhanh (-15°, -30°, -60°, -90° / +15°, +30°, +60°, +90°) và chạy đồng bộ ngược dấu (**Cùng đưa tới / Cùng thu về**) cho 2 khớp đối diện.
      1. **S4 (Đế xoay MG996R 360°):** Điều khiển chiều/tốc độ quanh mốc dừng 90°, nhả nút tự dừng (90).
      1. **Chế độ đặc biệt (ARMSP):** Đưa S2 lên 180° (ngắt xung sau 0.5s), sau đó gọi 90° **3 lần liên tiếp** (mỗi lần 0.5s và có ngắt xung).
      1. **Bảo vệ cơ khí & nguồn:** Không xuất xung servo khi khởi động, tự ngắt xung (detach) sau 0.5s mỗi thao tác và có nút ngắt xung thủ công trên Web.
## **⚡ 3. Lưu ý cấp nguồn (Power Isolation)**
- **Tách nhánh nguồn riêng cho Servo:** Cấp nguồn riêng cho cụm 4 servo qua mạch hạ áp Buck dòng cao (như **XL4016**, chỉnh khoảng 5V–6V) kèm tụ lọc bù dòng, **nối chung** GND với toàn bộ hệ thống. Không cấp tải servo qua AMS1117 để tránh sụt áp gây mất Wi-Fi/brownout trên ESP32 Master.
## **📌 4. Sơ đồ kết nối phần cứng (Pin Mapping)**
### **🧩 ESP32 Master Pinout**

|**Linh kiện / Module**|**Chân kết nối trên ESP32 Master**|**Chức năng chi tiết**|
| :- | :- | :- |
|**MPU6050 / OLED**|GPIO 21 (SDA), GPIO 22 (SCL)|Bus I2C giao tiếp cảm biến IMU và màn hình OLED (0x3C)|
|**Serial2 (UART)**|GPIO 16 (RX2), GPIO 17 (TX2)|Giao tiếp truyền tin (115200 bps) với ESP32 Worker|
|**Cụm 5 Mắt IR**|GPIO 32, 33, 25, 26, 27|Đọc tín hiệu số quét vạch biên sa bàn (Đen = 1, Trắng = 0)|
|**HC-SR05 Sensor**|Trig: GPIO 13, Echo: GPIO 14|Đo khoảng cách phát hiện vật thể/hàng hóa|
### **🛠️ ESP32 Worker Pinout**

|**Linh kiện / Module**|**Chân kết nối trên ESP32 Worker**|**Chức năng chi tiết**|
| :- | :- | :- |
|**Serial2 (UART)**|GPIO 16 (RX2), GPIO 17 (TX2)|Nhận lệnh điều khiển từ Master|
|**TB6612FNG Driver**|PWMA: 4, AIN1: 13, AIN2: 27, STBY: 26|Điều khiển động cơ trái|
|**TB6612FNG Driver**|PWMB: 23, BIN1: 33, BIN2: 19|Điều khiển động cơ phải|
|**Encoder Trái (X)**|GPIO 25 (A), GPIO 32 (B)|Đọc xung ngắt bánh trái|
|**Encoder Phải (Y)**|GPIO 34 (A), GPIO 35 (B)|Đọc xung ngắt bánh phải|
|**Servo S1 (Tay gắp)**|GPIO 22|SG90 tay gắp (chạy từ từ 90° / 0°)|
|**Servo S2 (Hông 1)**|GPIO 18|SG90 khớp hông 1|
|**Servo S3 (Hông 2)**|GPIO 21|SG90 khớp hông 2|
|**Servo S4 (Đế xoay)**|GPIO 5|MG996R 360° đế xoay (90 = dừng, trên/dưới 90 = quay)|
## **📂 5. Cấu trúc thư mục dự án**
Plaintext

📦 Maze-Solver-Robot-Dual-ESP32

` `┣ 📂 firmware

` `┃ ┣ 📜 ESP32\_Master\_2026-11-20.ino    # Mã nguồn trung tâm (2-Tab Web UI, MPU6050, OLED, Sensors)

` `┃ ┗ 📜 ESP32\_Worker\_2026-11-20.ino    # Mã nguồn chấp hành (Motors, Encoders, 4-DOF Servo & 0.5s auto-detach)

` `┣ 📜 Maze\_Robot\_Version\_Log.txt       # Nhật ký quản lý phiên bản và tài liệu kỹ thuật

` `┗ 📜 README.md                        # Tài liệu giới thiệu dự án
