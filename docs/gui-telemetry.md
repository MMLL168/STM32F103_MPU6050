# GUI 遙測說明

## 功能

- 顯示人工地平儀
- 顯示 IMU / RC / PID 即時數值
- 可送出 `校正 IMU` 命令
- 可送出 `重置 Yaw` 命令
- 可將遙測資料記錄成 CSV

## 啟動方式

```powershell
python tools\gui_telemetry.py
```

## 使用步驟

1. 選擇正確的 COM 埠。
2. 確認 baud rate 為 `115200`。
3. 按 `連線`。
4. 若需要重新做 MPU6050 靜止校正，按 `校正 IMU`。
5. 若需要將當前航向角歸零，按 `重置 Yaw`。
6. 若要保存遙測資料，按 `開始記錄 CSV`，選擇輸出檔案路徑。
7. 完成後可按 `停止記錄 CSV`。

## CSV 欄位

1. `timestamp`
2. `ax_g`
3. `ay_g`
4. `az_g`
5. `gx_dps`
6. `gy_dps`
7. `gz_dps`
8. `roll_deg`
9. `pitch_deg`
10. `yaw_deg`
11. `rc1`
12. `rc2`
13. `rc3`
14. `rc4`
15. `rc5`
16. `pid_roll_out`
17. `pid_pitch_out`
18. `pid_yaw_out`

## 韌體命令

GUI 會透過同一個 debug UART 送出以下命令：

- `CALIBRATE`
- `RESET_YAW`

韌體收到後會在序列視窗回傳狀態訊息，可直接在 GUI 左下角 log 區觀察。
