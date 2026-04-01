import csv
import math
import queue
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import tkinter as tk
from tkinter import filedialog, ttk

import serial
from serial.tools import list_ports


CSV_FIELD_NAMES = [
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "rc1",
    "rc2",
    "rc3",
    "rc4",
    "rc5",
    "pid_roll_out",
    "pid_pitch_out",
    "pid_yaw_out",
]


@dataclass
class TelemetryState:
    ax_g: float = 0.0
    ay_g: float = 0.0
    az_g: float = 0.0
    gx_dps: float = 0.0
    gy_dps: float = 0.0
    gz_dps: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    rc1: float = 0.0
    rc2: float = 0.0
    rc3: float = 0.0
    rc4: float = 0.0
    rc5: float = 0.0
    pid_roll_out: float = 0.0
    pid_pitch_out: float = 0.0
    pid_yaw_out: float = 0.0
    status_lines: list[str] = field(default_factory=list)
    last_update_monotonic: float = 0.0


class SerialReader(threading.Thread):
    def __init__(self, port: str, baudrate: int, output_queue: queue.Queue):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.output_queue = output_queue
        self.stop_event = threading.Event()
        self.serial_handle: Optional[serial.Serial] = None
        self.write_lock = threading.Lock()

    def run(self) -> None:
        try:
            self.serial_handle = serial.Serial(self.port, self.baudrate, timeout=0.2)
            self.output_queue.put(("status", f"已連線到 {self.port} @ {self.baudrate}"))

            while not self.stop_event.is_set():
                raw_line = self.serial_handle.readline()
                if not raw_line:
                    continue

                line = raw_line.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                self.output_queue.put(("line", line))

        except Exception as exc:  # noqa: BLE001
            self.output_queue.put(("status", f"序列埠錯誤: {exc}"))
        finally:
            if self.serial_handle is not None and self.serial_handle.is_open:
                self.serial_handle.close()
            self.output_queue.put(("closed", self.port))

    def send_command(self, command: str) -> bool:
        if self.serial_handle is None or not self.serial_handle.is_open:
            return False

        payload = (command.strip() + "\n").encode("ascii", errors="ignore")
        with self.write_lock:
            self.serial_handle.write(payload)
            self.serial_handle.flush()
        return True

    def stop(self) -> None:
        self.stop_event.set()


class HorizonWidget(tk.Canvas):
    def __init__(self, master):
        super().__init__(master, width=360, height=360, bg="#0f172a", highlightthickness=0)
        self.center_x = 180
        self.center_y = 180
        self.radius = 150

    def redraw(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        self.delete("all")

        self.create_oval(
            self.center_x - self.radius,
            self.center_y - self.radius,
            self.center_x + self.radius,
            self.center_y + self.radius,
            outline="#e2e8f0",
            width=2,
        )

        pitch_shift = max(-45.0, min(45.0, pitch_deg)) * 3.0
        roll_rad = math.radians(roll_deg)

        def rotate_point(x: float, y: float) -> tuple[float, float]:
            dx = x - self.center_x
            dy = y - self.center_y
            return (
                self.center_x + (dx * math.cos(roll_rad) - dy * math.sin(roll_rad)),
                self.center_y + (dx * math.sin(roll_rad) + dy * math.cos(roll_rad)),
            )

        sky_rect = [
            self.center_x - self.radius * 1.5,
            self.center_y - self.radius * 1.5 - pitch_shift,
            self.center_x + self.radius * 1.5,
            self.center_y - pitch_shift,
        ]
        ground_rect = [
            self.center_x - self.radius * 1.5,
            self.center_y - pitch_shift,
            self.center_x + self.radius * 1.5,
            self.center_y + self.radius * 1.5 - pitch_shift,
        ]

        sky_points = [
            rotate_point(sky_rect[0], sky_rect[1]),
            rotate_point(sky_rect[2], sky_rect[1]),
            rotate_point(sky_rect[2], sky_rect[3]),
            rotate_point(sky_rect[0], sky_rect[3]),
        ]
        ground_points = [
            rotate_point(ground_rect[0], ground_rect[1]),
            rotate_point(ground_rect[2], ground_rect[1]),
            rotate_point(ground_rect[2], ground_rect[3]),
            rotate_point(ground_rect[0], ground_rect[3]),
        ]

        self.create_polygon(*sum(sky_points, ()), fill="#38bdf8", outline="")
        self.create_polygon(*sum(ground_points, ()), fill="#b45309", outline="")

        left_horizon = rotate_point(self.center_x - self.radius * 1.3, self.center_y - pitch_shift)
        right_horizon = rotate_point(self.center_x + self.radius * 1.3, self.center_y - pitch_shift)
        self.create_line(*left_horizon, *right_horizon, fill="#f8fafc", width=3)

        for offset in (-20, -10, 10, 20):
            y = self.center_y - pitch_shift - offset * 3
            p1 = rotate_point(self.center_x - 30, y)
            p2 = rotate_point(self.center_x + 30, y)
            self.create_line(*p1, *p2, fill="#e2e8f0", width=1)

        self.create_arc(
            self.center_x - self.radius,
            self.center_y - self.radius,
            self.center_x + self.radius,
            self.center_y + self.radius,
            start=30,
            extent=120,
            style=tk.ARC,
            outline="#fbbf24",
            width=2,
        )

        self.create_line(self.center_x - 45, self.center_y, self.center_x - 12, self.center_y, fill="#f8fafc", width=4)
        self.create_line(self.center_x + 12, self.center_y, self.center_x + 45, self.center_y, fill="#f8fafc", width=4)
        self.create_line(self.center_x, self.center_y - 10, self.center_x, self.center_y + 10, fill="#f8fafc", width=3)

        self.create_text(
            self.center_x,
            24,
            text=f"Roll {roll_deg:6.2f} deg   Pitch {pitch_deg:6.2f} deg   Yaw {yaw_deg:6.2f} deg",
            fill="#f8fafc",
            font=("Consolas", 12, "bold"),
        )


class TelemetryApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Flight Controller Telemetry")
        self.root.geometry("1220x800")
        self.root.configure(bg="#111827")

        self.telemetry = TelemetryState()
        self.message_queue: queue.Queue = queue.Queue()
        self.reader: Optional[SerialReader] = None
        self.csv_file: Optional[object] = None
        self.csv_writer: Optional[csv.writer] = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.connection_var = tk.StringVar(value="尚未連線")
        self.last_update_var = tk.StringVar(value="尚未收到資料")
        self.csv_path_var = tk.StringVar(value="未啟用 CSV 記錄")

        self.value_vars: dict[str, tk.StringVar] = {name: tk.StringVar(value="0.00") for name in CSV_FIELD_NAMES}

        self._build_layout()
        self.refresh_ports()
        self.root.after(50, self.process_queue)
        self.root.after(500, self.update_freshness)

    def _build_layout(self) -> None:
        top_bar = ttk.Frame(self.root, padding=10)
        top_bar.pack(fill=tk.X)

        ttk.Button(top_bar, text="重新整理埠", command=self.refresh_ports).pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(top_bar, textvariable=self.port_var, width=14, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=8)
        ttk.Label(top_bar, text="Baud").pack(side=tk.LEFT)
        ttk.Entry(top_bar, textvariable=self.baud_var, width=8).pack(side=tk.LEFT, padx=8)
        ttk.Button(top_bar, text="連線", command=self.connect).pack(side=tk.LEFT)
        ttk.Button(top_bar, text="中斷", command=self.disconnect).pack(side=tk.LEFT, padx=8)
        ttk.Button(top_bar, text="校正 IMU", command=self.calibrate_imu).pack(side=tk.LEFT)
        ttk.Button(top_bar, text="重置 Yaw", command=self.reset_yaw).pack(side=tk.LEFT, padx=8)
        ttk.Button(top_bar, text="開始記錄 CSV", command=self.start_csv_logging).pack(side=tk.LEFT)
        ttk.Button(top_bar, text="停止記錄 CSV", command=self.stop_csv_logging).pack(side=tk.LEFT, padx=8)
        ttk.Label(top_bar, textvariable=self.connection_var).pack(side=tk.LEFT, padx=12)
        ttk.Label(top_bar, textvariable=self.last_update_var).pack(side=tk.RIGHT)

        csv_bar = ttk.Frame(self.root, padding=(10, 0, 10, 10))
        csv_bar.pack(fill=tk.X)
        ttk.Label(csv_bar, text="CSV:").pack(side=tk.LEFT)
        ttk.Label(csv_bar, textvariable=self.csv_path_var).pack(side=tk.LEFT, padx=8)

        content = ttk.Frame(self.root, padding=10)
        content.pack(fill=tk.BOTH, expand=True)
        content.columnconfigure(0, weight=3)
        content.columnconfigure(1, weight=2)
        content.rowconfigure(0, weight=1)

        left = ttk.Frame(content)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        left.rowconfigure(1, weight=1)

        self.horizon = HorizonWidget(left)
        self.horizon.pack(fill=tk.BOTH, expand=False)

        self.log_text = tk.Text(left, height=18, bg="#020617", fg="#e2e8f0", font=("Consolas", 10))
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        self.log_text.configure(state=tk.DISABLED)

        right = ttk.Frame(content)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.columnconfigure(1, weight=1)

        self._add_group(right, "IMU", ["ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "roll_deg", "pitch_deg", "yaw_deg"], 0, 0)
        self._add_group(right, "RC", ["rc1", "rc2", "rc3", "rc4", "rc5"], 0, 1)
        self._add_group(right, "PID", ["pid_roll_out", "pid_pitch_out", "pid_yaw_out"], 1, 0)

    def _add_group(self, master, title: str, fields: list[str], row: int, column: int) -> None:
        frame = ttk.LabelFrame(master, text=title, padding=10)
        frame.grid(row=row, column=column, sticky="nsew", padx=5, pady=5)
        for idx, field_name in enumerate(fields):
            ttk.Label(frame, text=field_name).grid(row=idx, column=0, sticky="w")
            ttk.Label(frame, textvariable=self.value_vars[field_name], font=("Consolas", 11)).grid(row=idx, column=1, sticky="e", padx=(12, 0))

    def refresh_ports(self) -> None:
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def connect(self) -> None:
        self.disconnect()
        if not self.port_var.get():
            self.connection_var.set("請先選擇 COM 埠")
            return

        try:
            baudrate = int(self.baud_var.get())
        except ValueError:
            self.connection_var.set("Baud rate 格式錯誤")
            return

        self.reader = SerialReader(self.port_var.get(), baudrate, self.message_queue)
        self.reader.start()
        self.connection_var.set(f"連線中 {self.port_var.get()} ...")

    def disconnect(self) -> None:
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        self.connection_var.set("尚未連線")

    def send_command(self, command: str, description: str) -> None:
        if self.reader is None or not self.reader.send_command(command):
            self.append_log(f"無法送出命令: {description}，請先連線")
            return

        self.append_log(f">>> {command}")

    def calibrate_imu(self) -> None:
        self.send_command("CALIBRATE", "校正 IMU")

    def reset_yaw(self) -> None:
        self.send_command("RESET_YAW", "重置 Yaw")

    def start_csv_logging(self) -> None:
        if self.csv_writer is not None:
            self.append_log("CSV 記錄已經啟用")
            return

        default_name = time.strftime("telemetry_%Y%m%d_%H%M%S.csv")
        path = filedialog.asksaveasfilename(
            title="選擇 CSV 輸出檔案",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV 檔案", "*.csv"), ("所有檔案", "*.*")],
        )
        if not path:
            return

        csv_path = Path(path)
        self.csv_file = csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp"] + CSV_FIELD_NAMES)
        self.csv_file.flush()
        self.csv_path_var.set(str(csv_path))
        self.append_log(f"開始記錄 CSV: {csv_path}")

    def stop_csv_logging(self) -> None:
        if self.csv_file is not None:
            self.csv_file.close()
        self.csv_file = None
        self.csv_writer = None
        self.csv_path_var.set("未啟用 CSV 記錄")
        self.append_log("已停止 CSV 記錄")

    def process_queue(self) -> None:
        while True:
            try:
                event_type, payload = self.message_queue.get_nowait()
            except queue.Empty:
                break

            if event_type == "status":
                self.connection_var.set(payload)
                self.append_log(payload)
            elif event_type == "line":
                self.handle_line(payload)
            elif event_type == "closed":
                if self.connection_var.get().startswith("連線中") or self.connection_var.get().startswith("已連線"):
                    self.connection_var.set(f"{payload} 已斷線")

        self.root.after(50, self.process_queue)

    def handle_line(self, line: str) -> None:
        values = self.try_parse_csv(line)
        if values is not None:
            for name, value in zip(CSV_FIELD_NAMES, values):
                setattr(self.telemetry, name, value)
                self.value_vars[name].set(f"{value:.2f}")
            self.telemetry.last_update_monotonic = time.monotonic()
            self.horizon.redraw(self.telemetry.roll_deg, self.telemetry.pitch_deg, self.telemetry.yaw_deg)
            self.write_csv_row(values)
            return

        self.telemetry.status_lines.append(line)
        self.telemetry.status_lines = self.telemetry.status_lines[-120:]
        self.append_log(line)

    def try_parse_csv(self, line: str) -> Optional[list[float]]:
        if "," not in line or ":" in line:
            return None

        parts = [part.strip() for part in line.split(",")]
        if len(parts) != len(CSV_FIELD_NAMES):
            return None

        try:
            return [float(part) for part in parts]
        except ValueError:
            return None

    def write_csv_row(self, values: list[float]) -> None:
        if self.csv_writer is None or self.csv_file is None:
            return

        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        self.csv_writer.writerow([timestamp] + values)
        self.csv_file.flush()

    def append_log(self, line: str) -> None:
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, line + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def update_freshness(self) -> None:
        if self.telemetry.last_update_monotonic <= 0.0:
            self.last_update_var.set("尚未收到資料")
        else:
            age = time.monotonic() - self.telemetry.last_update_monotonic
            self.last_update_var.set(f"資料延遲 {age:.2f}s")
        self.root.after(500, self.update_freshness)


def main() -> None:
    root = tk.Tk()
    style = ttk.Style(root)
    style.theme_use("clam")
    app = TelemetryApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.stop_csv_logging(), app.disconnect(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
