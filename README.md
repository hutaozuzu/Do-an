# Đồ Án Xe Tự Hành Ackermann (Kế thừa khung xe K2022 $\rightarrow$ FGO SLAM 10 km/h)

Kho lưu trữ tài liệu, thiết kế phần cứng tham chiếu từ khóa trước (`K2022`) và kiến trúc hệ thống xe tự hành Ackermann điều khiển không dây dùng **Factor Graph Optimization (`GTSAM iSAM2`)**.

## Tài Liệu Kiến Trúc

- **[K2022_SYSTEM_ARCHITECTURE.md](./K2022_SYSTEM_ARCHITECTURE.md)**: Sơ đồ hóa toàn bộ hệ thống đồ án khóa trước trong thư mục [K2022](./K2022) (Altium schematic/PCB, pinout STM32F4, firmware PID 100Hz, ROS 2 Humble, `A-GMPC`) và phân tích phần cứng kế thừa còn giữ trên xe (Khung Ackermann $L=0.65\text{ m}$, 3 động cơ DC, 3 encoder, 3 mạch cầu H `HI216`, khối nguồn).
- **[FGO_OPTIMIZED_ARCHITECTURE.md](./FGO_OPTIMIZED_ARCHITECTURE.md)**: Kiến trúc tối ưu hóa 3 lớp không dây cho đề tài mới ở tốc độ $10\text{ km/h}$ ($2.78\text{ m/s}$):
  - **Lớp 1 (STM32F4):** Đọc 3 Encoder + IMU (`BNO085`), PID 100Hz, ZUPT, nối dây `USB-UART 921600` lên Pi.
  - **Lớp 2 (Raspberry Pi 4 4GB trên xe):** Đồng bộ `CLOCK_MONOTONIC`, chạy `laser_filters`, `Dynamic Yaw Blending`, `Motion Deskewing` tại biên, Watchdog $200\text{ ms}$, nén ảnh JPEG qua Wi-Fi 5GHz (`CycloneDDS`).
  - **Lớp 3 (Laptop RTX 3050 ở trạm):** `YOLOv8 TensorRT`, `Multi-Res CSM`, `GTSAM iSAM2` (kèm `Slip-State Factor`), Occupancy Grid $1600 \times 1600$ ($80 \times 80\text{ m}$), `Wavefront Frontier`, và bộ điều khiển Ackermann bù trượt.

## Cấu Trúc Thư Mục

- `K2022/` — Báo cáo `DATN.pdf`, schematic/PCB Altium (`MainBoard_03_2023.pdf`, `Sheet1.SchDoc`, BOM, DRC) của khóa trước.
- `Bản sao của Slam__.docx` — Tài liệu tham khảo SLAM/FGO.
- `K2022_SYSTEM_ARCHITECTURE.md` — Sơ đồ hóa hệ thống K2022 & đánh giá phần cứng kế thừa.
- `FGO_OPTIMIZED_ARCHITECTURE.md` — Kiến trúc tối ưu FGO SLAM không dây.
