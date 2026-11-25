# Kiến Trúc Tối Ưu Hóa (Phiên Bản Không Dây): Xe Tự Hành Ackermann 10 km/h Dùng Raspberry Pi 4 + FGO SLAM (`GTSAM iSAM2`)

> [!NOTE]
> Kiến trúc này giữ nguyên yêu cầu **điều khiển không dây với Raspberry Pi 4 (4GB) đặt trên xe** và **Laptop RTX 3050 đặt ở trạm mặt đất**, đồng thời tối ưu hóa triệt để việc phân tải tính toán, đồng bộ thời gian thực cho `Motion Deskewing` ở $10\text{ km/h}$ ($2.78\text{ m/s}$), và khớp 100% với **khung xe Ackermann 3 Encoder + 3 cầu H `HI216` kế thừa từ [K2022_SYSTEM_ARCHITECTURE.md](./K2022_SYSTEM_ARCHITECTURE.md)**.

---

## 1. Tại Sao Phải Tối Ưu Lại Kiến Trúc 3 Lớp Không Dây Này?

Khi tách Laptop RTX 3050 xuống đất và đặt Raspberry Pi trên xe, có **3 điểm nghẽn chí mạng** ở vận tốc $2.78\text{ m/s}$ ($10\text{ km/h}$) nếu ghép nối theo cách thông thường:
1. **Bỏ ESP32 cầu nối Wi-Fi:** Vì Raspberry Pi đã nằm ngay trên xe cạnh STM32F4, truyền dữ liệu từ STM32 qua ESP32 (Wi-Fi) rồi mới tới Pi/Laptop sẽ gây trễ ngẫu nhiên ($5-80\text{ ms}$) và rớt gói. Thay vào đó, **nối dây trực tiếp `STM32F4 <--USB-UART 921600 baud--> Raspberry Pi`** (bỏ hẳn ESP32).
2. **Đưa `Motion Deskewing` xuống chạy ngay trên Raspberry Pi:** Nếu gửi `LaserScan` thô và `IMU/Odom` qua Wi-Fi về Laptop rồi Laptop mới gỡ méo, độ trễ Wi-Fi lệch nhau giữa 2 luồng sẽ làm sai lệch vị trí tia laser tới vài chục cm. Vì cả `YDLIDAR S2` lẫn `STM32 (3 Encoder + IMU BNO085)` đều cắm dây thẳng vào Pi, **Raspberry Pi giữ chung 1 đồng hồ hệ thống (`CLOCK_MONOTONIC`)** nên phải chạy `laser_filters` + `Motion Deskewing` ngay tại chỗ, rồi mới bắn đám mây điểm đã nắn thẳng qua Wi-Fi 5GHz về Laptop!
3. **Watchdog phanh khẩn cấp $200\text{ ms}$ ngay trên Pi & STM32:** Ở tốc độ $2.78\text{ m/s}$, chỉ cần mất sóng Wi-Fi $0.5\text{ s}$ là xe đã trôi đi $1.4\text{ m}$. Raspberry Pi và STM32 bắt buộc phải có bộ đếm Watchdog độc lập: quá $200\text{ ms}$ không nhận được nhịp tim (`/cmd_vel` / Heartbeat) từ Laptop thì tự động phanh dừng xe (`v_ref = 0`).

---

## 2. Sơ Đồ 1: Kiến Trúc Phần Cứng 3 Lớp Không Dây Đã Tối Ưu Hóa

```mermaid
flowchart LR
    subgraph ONBOARD_CAR ["PHẦN CỨNG TRÊN XE ACKERMANN (K2022 + PI)"]
        direction TB
        subgraph BASE_HW ["Khung Xe Có Sẵn (K2022)"]
            PWR["Nguồn công suất khoang sau<br/>+ Mạch hạ áp 5V/5A (nuôi Pi) & 3.3V"]
            DRV["3x Mạch Cầu H HI216<br/>(Lái trước, Sau Trái, Sau Phải)"]
            M_STR["Động cơ Lái + ENC_0 (δ)<br/>L = 0.65m, ±30°"]
            M_RL["Động cơ Sau Trái + ENC_1 (v_L)"]
            M_RR["Động cơ Sau Phải + ENC_2 (v_R)"]
            PWR ==> DRV ==> M_STR & M_RL & M_RR
        end

        subgraph L1_STM ["Lớp 1: STM32F4 (Hard Real-Time 100-200Hz)"]
            IMU["IMU BNO085 / BNO086<br/>(6-DOF Game Rotation + Linear Accel)<br/>Đặt đệm chống rung, xa cầu H"]
            STM["STM32F407<br/>- Hardware Timer Encoder (TIM1, TIM2, TIM3)<br/>- PID 100Hz (TIM8) + ZUPT + Slew Limiter<br/>- Failsafe tự ngắt PWM khi mất UART > 200ms"]
            M_STR & M_RL & M_RR -->|"Pha A/B (74HC14D)"| STM
            STM -->|"PWM + DIR+/DIR-"| DRV
            IMU -->|"SPI 200Hz (hoặc UART-RVC 100Hz)"| STM
        end

        subgraph L2_PI ["Lớp 2: Raspberry Pi 4 (4GB) Trên Xe (Edge Sync & Deskew)"]
            LIDAR["YDLIDAR S2 (10Hz)<br/>+ Mái che V-groove + Trụ cao su"]
            CAM["Camera RGB (USB)<br/>(Nén JPEG gửi Laptop)"]
            RADAR["(Tùy chọn) Radar mmWave<br/>Phanh cứng trực tiếp tại chỗ"]
            PI["Raspberry Pi 4 4GB (ROS 2 Humble / CycloneDDS)<br/>1. Chung đồng hồ CLOCK_MONOTONIC cho LiDAR + STM32<br/>2. Chạy laser_filters + Dynamic Yaw Blending<br/>3. Chạy Motion Deskewing tại biên (nắn méo 27.8cm)<br/>4. Watchdog 200ms cắt tốc độ khi rớt Wi-Fi"]
            
            STM <-->|"Cáp USB-UART (921600 baud)<br/>(Bỏ ESP32 trung gian)"| PI
            LIDAR & CAM & RADAR -->|"Cáp USB trực tiếp"| PI
        end
    end

    subgraph GROUND_STATION ["TRẠM MẶT ĐẤT (BASE STATION)"]
        LAP["Laptop RTX 3050 (ROS 2 Humble Master)<br/>1. YOLOv8 TensorRT (từ ảnh nén JPEG)<br/>2. Multi-Res CSM + GTSAM iSAM2 (Slip-State FGO)<br/>3. Bản đồ 1600x1600 + WFD explore_lite_node<br/>4. Điều khiển Ackermann (A-GMPC / MPPI) bù trượt S_k"]
    end

    PI <==>|"Wi-Fi 5GHz (Dedicated Router / Hotspot)<br/>Gửi: Deskewed Scan + Compressed Image + Synced Odom<br/>Nhận: /cmd_vel (v_ref, ω_ref) + Heartbeat"| LAP

    classDef kept fill:#d1fae5,stroke:#059669,stroke-width:2px,color:#064e3b;
    classDef edge fill:#fef3c7,stroke:#d97706,stroke-width:2px,color:#78350f;
    classDef master fill:#dbeafe,stroke:#2563eb,stroke-width:2px,color:#1e3a8a;
    class PWR,DRV,M_STR,M_RL,M_RR kept;
    class IMU,STM,LIDAR,CAM,RADAR,PI edge;
    class LAP master;
```

---

## 3. Sơ Đồ 2: Phân Tải Thuật Toán 4 Tầng Trên Kiến Trúc Không Dây (Pi $\leftrightarrow$ Laptop RTX 3050)

```mermaid
flowchart TB
    subgraph T1 ["TẦNG 1: STM32F4 TRÊN XE (100Hz - 200Hz)"]
        E_READ["Đọc 3 Encoder Ackermann (δ, N_L, N_R)<br/>& IMU BNO085 (q_imu, ω_z, a_lin_x)"]
        ZUPT_CHK{"PWM == 0 &<br/>ΔN_L = ΔN_R = 0?"}
        ZUPT_ON["Kích hoạt ZUPT:<br/>vx = vy = ωz = 0"]
        KIN_ACK["Động học Ackermann (L=0.65m):<br/>v_x = (v_L + v_R)/2<br/>ω_steer = (v_x / 0.65) * tan(δ)<br/>ω_diff = (v_R - v_L) / W"]
        E_READ --> ZUPT_CHK
        ZUPT_CHK -->|"Có"| ZUPT_ON
        ZUPT_CHK -->|"Không"| KIN_ACK
    end

    subgraph T2 ["TẦNG 2: RASPBERRY PI 4 TRÊN XE (TIỀN XỬ LÝ TẠI BIÊN & BẢO VỆ)"]
        SYNC["Đồng bộ Timestamp chung (Pi Clock)<br/>Lưu Buffer Odom + IMU 200Hz"]
        BLEND["Dynamic Yaw Blending:<br/>S_ω = |ω_steer - ω_imu|<br/>w_odom = 0.35 * exp(-S_ω² / 2σ_ω²)"]
        FILT["laser_filters:<br/>ROR/Speckle + Shadow + [0.15, 5.0]m"]
        DESKEW["Motion Deskewing (ngay trên Pi!):<br/>Nội suy vị trí từng tia laser YDLIDAR S2<br/>Xuất PointCloud2 đã nắn méo"]
        WDG{"Watchdog Wi-Fi:<br/>Mất lệnh > 200ms?"}
        
        ZUPT_ON & KIN_ACK -->|"USB-UART 921600"| SYNC --> BLEND --> DESKEW
        FILT --> DESKEW
    end

    subgraph WIFI_LINK ["MẠNG KHÔNG DÂY WI-FI 5GHz (CycloneDDS / QoS Best-Effort cho Sensor)"]
        TX_UP["Uplink (Xe -> Laptop):<br/>/scan_deskewed (PointCloud2)<br/>/odom_synced + /imu_raw<br/>/camera/image_raw/compressed"]
        TX_DN["Downlink (Laptop -> Xe):<br/>/cmd_vel (v_ref, ω_ref) ở 30Hz<br/>Heartbeat kiểm tra kết nối"]
    end

    subgraph T3 ["TẦNG 3: BACK-END FGO TRÊN LAPTOP RTX 3050"]
        YOLO["YOLOv8 TensorRT (GPU):<br/>Xóa điểm thuộc Người/Xe khỏi /scan_deskewed"]
        CSM["Multi-Resolution CSM + Gaussian 3x3:<br/>Thô (0.04m, 0.025rad) -> Tinh (0.005m, 0.005rad)"]
        DEG["Degeneracy Check (Eigen Cov):<br/>λ_min < 0.25 => deg_scale = 3.5"]
        SLIP["Tính số dư trượt Ackermann:<br/>r_a = |a_odom,x - a_lin_imu,x|<br/>r_ω = |ω_steer - ω_imu,z|, a_lat = |v_x * ω_imu,z|"]
        ISAM["GTSAM iSAM2 (25Hz):<br/>- PriorFactor(X_0)<br/>- Odom + NHC (v_y=0) + Slip-Adaptive<br/>- CSM + Huber(1.345)<br/>- Ternary Kinematic-Slip Factor (X_k-1, X_k, S_k)<br/>- Loop Closure (≥35 KF, ≥6m, d≤1.8m, hit≥0.72)"]
        
        YOLO --> CSM --> ISAM
        DEG & SLIP --> ISAM
    end

    subgraph T4 ["TẦNG 4: BẢN ĐỒ, FRONTIER & ĐIỀU KHIỂN TRÊN LAPTOP RTX 3050"]
        GRID["Occupancy Grid 1600x1600 (80x80m, res 0.05m)<br/>Bresenham Log-Odds (-0.40/+0.85), ngắt |ω_z|>0.85 rad/s"]
        WFD["Wavefront Frontier (explore_lite_node.cpp)<br/>Utility = 1.2*Size - 1.5*Dist - 0.35*|Δθ|, Blacklist 0.75m"]
        CTRL["Ackermann Controller (A-GMPC / Nav2 MPPI)<br/>Nhận bù trượt S_k = [κ, β, α_ω] -> xuất /cmd_vel"]
        
        ISAM -->|"TF map->odom (25Hz)"| GRID --> WFD --> CTRL
        ISAM -->|"Pose + Slip State S_k"| CTRL
    end

    DESKEW --> TX_UP --> YOLO
    SYNC --> TX_UP --> SLIP
    CTRL --> TX_DN --> WDG
    WDG -->|"Có (Phanh khẩn)"| STOP["Gửi v_ref = 0 xuống STM32"]
    WDG -->|"Không (Bình thường)"| PASS["Chuyển (v_ref, ω_ref) xuống STM32 PID"]
```

---

## 4. Tối Ưu Hóa Mạng Wi-Fi 5GHz & Phân Bổ Băng Thông Giữa Pi và Laptop

Để chạy xe ở tốc độ **$10\text{ km/h}$ ($2.78\text{ m/s}$)** điều khiển qua Wi-Fi mà không bị nghẽn mạng hay trễ lệnh điều khiển, cần thiết lập **4 quy tắc bắt buộc trên Raspberry Pi & Laptop**:

1. **Dùng Router Wi-Fi 5GHz chuyên dụng (hoặc Hotspot 5GHz cố định kênh) — KHÔNG dùng mạng Wi-Fi chung:**
   - Sân thực nghiệm ngoài trời có nhiều nhiễu băng tần 2.4GHz. Hãy cắm một **Router Wi-Fi 5GHz (802.11ac) ngay tại trạm Laptop** (dùng nguồn dự phòng/pin sạc laptop), cấp IP tĩnh cho Pi và Laptop.
2. **Đổi RMW sang `rmw_cyclonedds_cpp` (CycloneDDS) và tách QoS:**
   - Với các topic dữ liệu liên tục (`/scan_deskewed`, `/camera/image_raw/compressed`, `/odom_synced`): đặt QoS là **`Best Effort` + `KeepLast(1)`** để nếu một gói bị rớt trên không trung, mạng sẽ bỏ qua luôn và nhận gói mới nhất thay vì cố truyền lại làm dồn ứ hàng đợi (Bufferbloat).
   - Với các topic quan trọng (`/map`, `/tf_static`, Action `/navigate_to_pose`): giữ `Reliable`.
3. **Nén ảnh Camera ngay trên Raspberry Pi trước khi gửi sang Laptop cho YOLOv8:**
   - Không bao giờ bắn ảnh `image_raw` chưa nén ($640\times 480 \times 30\text{ fps} \approx 220\text{ Mbps}$) qua Wi-Fi!
   - Trên Pi, dùng `image_transport` xuất **`/camera/image_raw/compressed` (JPEG Quality 60–70, 15–20 fps)** — băng thông tụt xuống chỉ còn **~3–5 Mbps**, dư sức cho GPU RTX 3050 giải nén và chạy `YOLOv8 TensorRT` trong $< 5\text{ ms}$.
4. **Cơ chế an toàn 2 tầng khi rớt mạng (Dual Watchdog):**
   - **Watchdog trên Raspberry Pi:** Nếu không nhận được lệnh `/cmd_vel` hoặc gói `heartbeat` từ Laptop trong **$200\text{ ms}$** $\rightarrow$ Pi chủ động gửi `v_ref = 0` xuống STM32 (và tự kích hoạt phanh mềm nếu có Radar mmWave phát hiện vật cản dưới $1.5\text{ m}$).
   - **Watchdog trên STM32F4:** Nếu mất tín hiệu UART từ Raspberry Pi quá **$200\text{ ms}$** $\rightarrow$ STM32 đặt `PWM = 0`, reset bộ tích phân PID (như cơ chế gốc của K2022 ở Hình 4.12).

---

## 5. Chọn IMU Phù Hợp Nhất Với Cấu Hình `STM32 + Raspberry Pi`

| Phương án đấu nối IMU | Mã IMU đề xuất | Cách đi dây & đồng bộ thời gian | Đánh giá cho bài toán 10 km/h |
| :--- | :--- | :--- | :--- |
| **Phương án A (Khuyên dùng nhất):** Gắn vào `STM32F4` để gộp chung gói tin với 3 Encoder | **`BNO085` hoặc `BNO086`** (PCB đen, chế độ 6-DOF `Game Rotation Vector` + `Linear Accel`) | Nối qua **SPI (200Hz)** hoặc **UART-RVC (100Hz)** vào `STM32F4`. STM32 đóng chung 1 gói `{enc_steer, enc_L, enc_R, q_imu, gyro_z, lin_acc_x}` bắn qua USB-UART 921600 lên Pi. | **Đồng bộ tuyệt đối** giữa Encoder và IMU ngay ở cấp phần cứng, trên Pi chỉ cần nghe đúng 1 cổng serial của STM32. |
| **Phương án B:** Gắn thẳng vào cổng USB của `Raspberry Pi` | **`Wheeltec N100`** hoặc **`WT901C-TTL` / `HWT905-USB`** (chế độ 6-Axis) | Cắm thẳng USB vào Raspberry Pi cùng với YDLIDAR S2. Pi chạy driver ROS 2 `fdilink_ahrs` (200Hz–400Hz) và đồng bộ thời gian bằng `Pi Monotonic Clock`. | **Cực dễ làm phần mềm**, không phải code driver SPI cho IMU trên STM32, độ ổn định Yaw công nghiệp, chỉ tốn thêm 1 cổng USB trên Pi. |
| **Các module nên né tránh** | `BNO055`, `BWT61CL`, `WT901C485` | `BNO055` khóa max 100Hz và tự calibration ngầm; `BWT61CL` thực chất là MPU6050 qua Bluetooth, trôi nhiệt độ; `WT901C485` dùng Modbus RS485 không tối ưu bằng TTL/SPI. | Dễ gây trễ hoặc lệch yaw khi xe chạy $2.78\text{ m/s}$ và rung xóc ngoài trời. |

> [!WARNING]
> **Lưu ý về nguồn nuôi trên xe khi gắn thêm Raspberry Pi:**
> Mạch hạ áp `LM2596S-5.0` cũ (nếu làm theo sơ đồ K2022) chỉ chịu tối đa $3\text{ A}$ và dễ bị nhiễu nhiệt/sụt áp khi dùng nuôi **Raspberry Pi 4**. Do đó, hãy cấp cho Raspberry Pi một **mạch Buck DC-DC 5V/5A riêng biệt (như XL4015 tách riêng hoặc module UBEC 5V 5A-6A)** nối trực tiếp từ ắc quy khoang sau, **tuyệt đối không lấy chung nhánh nguồn 5V với cầu H hay động cơ lái** để tránh Pi bị sập nguồn (Under-voltage Reset) giữa lúc xe đang chạy $10\text{ km/h}$!
