# Flight Controller

這份專案目前已整理成以 **Blue Pill STM32F103C8 + MPU6050 + ELRS/CRSF** 為核心的 **Quad X 多旋翼飛控實驗平台**。

目前重點功能：
- MPU6050 姿態量測與自動校正
- ELRS / CRSF 遙控輸入
- GUI 姿態與遙測監看
- `Quad X` 四軸混控
- `PA0~PA3` 四路 ESC PWM 輸出
- ARM / DISARM / Failsafe / Motor Idle
- 開機狀態機與 RC timeout 保護
- 基礎 cascaded PID 與濾波鏈
- ST-LINK 燒錄與單步除錯

## 目前硬體配置

- 主控板：STM32F103C8T6 Blue Pill
- IMU：MPU6050
- 遙控接收：ELRS / CRSF
- ESC 輸出：4 路 PWM
- GUI / Telemetry：UART

## Blue Pill 接線

| 功能 | 腳位 | 說明 |
|---|---|---|
| MPU6050 SDA | `PB9` | I2C 資料 |
| MPU6050 SCL | `PB8` | I2C 時脈 |
| ELRS RX | `PB11` | 接收機 TX -> Blue Pill RX |
| ELRS TX | `PB10` | Blue Pill TX -> 接收機 RX |
| Debug / GUI RX | `PA10` | USBTTL / telemetry radio TX -> PA10 |
| Debug / GUI TX | `PA9` | PA9 -> USBTTL / telemetry radio RX |
| ESC M1 | `PA0` | Front Left |
| ESC M2 | `PA1` | Front Right |
| ESC M3 | `PA2` | Rear Right |
| ESC M4 | `PA3` | Rear Left |
| AUX PWM | `PA6` | 50Hz 輔助輸出 |
| Status LED | `PC13` | 板載 LED |

## Quad X 馬達定義

目前混控定義如下：

- `M1 = Front Left`
- `M2 = Front Right`
- `M3 = Rear Right`
- `M4 = Rear Left`

## Quad X Mixer

自動模式下，飛控會先算出：
- `Throttle`
- `Roll`
- `Pitch`
- `Yaw`

再混成四顆馬達輸出：

```text
M1 = Throttle + Pitch + Roll - Yaw
M2 = Throttle + Pitch - Roll + Yaw
M3 = Throttle - Pitch - Roll - Yaw
M4 = Throttle - Pitch + Roll + Yaw
```

也就是：
- `Throttle`：四顆一起加減
- `Roll / Pitch / Yaw`：疊加到各顆馬達，形成姿態控制修正

## 懸停測試判讀表

| 你手上做的動作 | 正常修正方向 | 應該變大的馬達 | 應該變小的馬達 |
|---|---|---|---|
| 右邊壓低 | 往左拉回水平 | `M1`, `M4` | `M2`, `M3` |
| 左邊壓低 | 往右拉回水平 | `M2`, `M3` | `M1`, `M4` |
| 機頭壓低 | 把機頭抬回來 | `M1`, `M2` | `M3`, `M4` |
| 機尾壓低 | 把機尾抬回來 | `M3`, `M4` | `M1`, `M2` |

## PWM 輸出

目前 ESC 輸出是：
- `PA0~PA3`
- `400Hz`
- `1000us ~ 2000us`

其中：
- `1000us` = 最低
- `1500us` = 中間
- `2000us` = 最高

`PA6` 保留為 `50Hz` AUX PWM。

## 飛控安全骨架

目前已加入：

- `BOOT -> DISARMED -> ARMED -> FAILSAFE` 狀態機
- `ARM / DISARM` 指令
- 上電開機等待與 IMU ready 後才允許進入正常狀態
- `throttle low` 才允許 ARM
- `RC timeout` 後自動進入 `FAILSAFE`
- `FAILSAFE` 延時後自動 `DISARM`
- `ARMED` 且低油門時會保持 `motor idle`

GUI 可直接送出：

- `ARM`
- `DISARM`
- `CALIBRATE`
- `RESET_YAW`

## 控制器與濾波

目前控制架構：

- 外環：`Roll / Pitch` 角度環
- 內環：`Roll / Pitch / Yaw` 角速度環
- `Quad X` mixer 輸出到 `M1~M4`

目前已加入：

- 分軸 PID 參數
- rate loop anti-windup
- mixer 飽和 / 輸出限制
- `gyro low-pass`
- `accel low-pass`
- `D-term low-pass`
- `RC smoothing`
- 可選 `gyro notch` 骨架（預設關閉）

## GUI 功能

`tools/gui_telemetry.py` 目前可顯示：
- 人工地平儀
- IMU 詳細資料
- RC 原始輸入
- PID 輸出
- `M1~M4` PWM 微秒值
- Quad X mixer 對照

也提供：
- IMU 校正
- 重置 Yaw
- ARM / DISARM
- CSV 記錄
- 手動四馬達油門 debug
- 懸停穩定測試模式（固定油門 + 自動水平修正）
- 工程測試按鈕（PWM / PA0 / RAW TIM2）

## 建置與燒錄

```bash
pio run -e bluepill_f103c8
pio run -e bluepill_f103c8 -t upload
```

## GUI 執行

```bash
python tools/gui_telemetry.py
```

預設 telemetry / GUI baud rate：

```text
57600
```

## 專案目前定位

這份專案原本較接近「通用 RC 控制通道 / 固定翼風格輸出」，目前已整理成適合 **Quad X 多旋翼** bring-up、量測、GUI 觀察與基礎控制驗證的版本。
