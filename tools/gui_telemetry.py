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
    "motor1_us",
    "motor2_us",
    "motor3_us",
    "motor4_us",
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
    motor1_us: float = 0.0
    motor2_us: float = 0.0
    motor3_us: float = 0.0
    motor4_us: float = 0.0
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
        self.center_x = 180.0
        self.center_y = 180.0
        self.radius = 150.0

    def redraw(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> None:
        self.delete("all")

        canvas_width = max(self.winfo_width(), 360)
        canvas_height = max(self.winfo_height(), 360)
        self.center_x = canvas_width / 2.0
        self.center_y = canvas_height / 2.0
        self.radius = min(canvas_width, canvas_height) * 0.42

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


class StickWidget(tk.Canvas):
    def __init__(self, master, title: str, x_label: str, y_label: str, throttle_mode: bool = False):
        super().__init__(master, width=220, height=260, bg="#0f172a", highlightthickness=0)
        self.title = title
        self.x_label = x_label
        self.y_label = y_label
        self.throttle_mode = throttle_mode

    def redraw(self, x_percent: float, y_percent: float) -> None:
        self.delete("all")

        width = max(self.winfo_width(), 220)
        height = max(self.winfo_height(), 260)
        pad_x = 22
        pad_top = 32
        pad_bottom = 18
        box_left = pad_x
        box_top = pad_top
        box_right = width - pad_x
        box_bottom = height - pad_bottom

        self.create_text(width / 2, 16, text=self.title, fill="#f8fafc", font=("Consolas", 11, "bold"))
        self.create_rectangle(box_left, box_top, box_right, box_bottom, outline="#94a3b8", width=2)

        center_x = (box_left + box_right) / 2
        center_y = (box_top + box_bottom) / 2
        self.create_line(center_x, box_top, center_x, box_bottom, fill="#334155", dash=(4, 4))
        self.create_line(box_left, center_y, box_right, center_y, fill="#334155", dash=(4, 4))

        if self.throttle_mode:
            normalized_y = max(0.0, min(1.0, y_percent / 100.0))
            knob_y = box_bottom - normalized_y * (box_bottom - box_top)
        else:
            normalized_y = max(-1.0, min(1.0, (y_percent - 50.0) / 50.0))
            knob_y = center_y - normalized_y * ((box_bottom - box_top) / 2)

        normalized_x = max(-1.0, min(1.0, (x_percent - 50.0) / 50.0))
        knob_x = center_x + normalized_x * ((box_right - box_left) / 2)

        self.create_oval(knob_x - 18, knob_y - 18, knob_x + 18, knob_y + 18, fill="#38bdf8", outline="#e2e8f0", width=2)
        self.create_text(center_x, box_bottom + 10, text=f"{self.x_label}: {x_percent:5.1f}%   {self.y_label}: {y_percent:5.1f}%", fill="#cbd5e1", font=("Consolas", 10))


class TelemetryApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Flight Controller Telemetry")
        self.root.geometry("1380x860")
        self.root.configure(bg="#111827")

        self.telemetry = TelemetryState()
        self.message_queue: queue.Queue = queue.Queue()
        self.reader: Optional[SerialReader] = None
        self.csv_file: Optional[object] = None
        self.csv_writer: Optional[csv.writer] = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="57600")
        self.connection_var = tk.StringVar(value="尚未連線")
        self.last_update_var = tk.StringVar(value="等待 IMU/遙測資料")
        self.csv_path_var = tk.StringVar(value="未啟用 CSV 記錄")
        self.control_mode_var = tk.StringVar(value="auto")
        self.manual_throttle_var = tk.IntVar(value=0)
        self.manual_motor_vars = [tk.IntVar(value=0) for _ in range(4)]
        self.manual_slider_updating = False
        self.pwm_test_pulse_var = tk.IntVar(value=1500)
        self.pwm_test_duty_var = tk.IntVar(value=50)

        self.value_vars: dict[str, tk.StringVar] = {name: tk.StringVar(value="--") for name in CSV_FIELD_NAMES}

        self._build_layout()
        self.refresh_ports()
        self.update_manual_controls()
        self.update_manual_throttle_label()
        self.reset_telemetry_view()
        self.root.bind("<Configure>", self.on_root_resize)
        self.root.after(50, self.process_queue)
        self.root.after(500, self.update_freshness)

    def _build_layout(self) -> None:
        outer = ttk.Frame(self.root)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.rowconfigure(0, weight=1)
        outer.columnconfigure(0, weight=1)

        self.scroll_canvas = tk.Canvas(outer, bg="#111827", highlightthickness=0)
        self.scroll_canvas.grid(row=0, column=0, sticky="nsew")
        self.v_scrollbar = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=self.scroll_canvas.yview)
        self.v_scrollbar.grid(row=0, column=1, sticky="ns")
        self.h_scrollbar = ttk.Scrollbar(outer, orient=tk.HORIZONTAL, command=self.scroll_canvas.xview)
        self.h_scrollbar.grid(row=1, column=0, sticky="ew")
        self.scroll_canvas.configure(yscrollcommand=self.v_scrollbar.set, xscrollcommand=self.h_scrollbar.set)

        self.main_frame = ttk.Frame(self.scroll_canvas, padding=0)
        self.main_frame_window = self.scroll_canvas.create_window((0, 0), window=self.main_frame, anchor="nw")
        self.main_frame.bind("<Configure>", self.on_main_frame_configure)
        self.scroll_canvas.bind("<Configure>", self.on_canvas_configure)

        top_bar = ttk.Frame(self.main_frame, padding=10)
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
        ttk.Button(top_bar, text="方波400 測試 ON", command=self.enable_pwm_square_test).pack(side=tk.LEFT)
        ttk.Button(top_bar, text="RAW TIM2 ON", command=self.enable_raw_tim2_test).pack(side=tk.LEFT, padx=(4, 0))
        ttk.Button(top_bar, text="ESC400 測試 ON", command=self.enable_pwm_esc_test).pack(side=tk.LEFT, padx=8)
        ttk.Button(top_bar, text="PWM 測試 OFF", command=self.disable_pwm_test).pack(side=tk.LEFT)
        ttk.Button(top_bar, text="PA0 HIGH", command=self.enable_pa0_high_test).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(top_bar, text="PA0 LOW", command=self.enable_pa0_low_test).pack(side=tk.LEFT, padx=4)
        ttk.Button(top_bar, text="PA0 1Hz", command=self.enable_pa0_blink_test).pack(side=tk.LEFT, padx=4)
        ttk.Button(top_bar, text="PA0 OFF", command=self.disable_pin_test).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Label(top_bar, text="Duty%").pack(side=tk.LEFT, padx=(8, 0))
        ttk.Entry(top_bar, textvariable=self.pwm_test_duty_var, width=4).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Label(top_bar, text="脈寬us").pack(side=tk.LEFT)
        ttk.Entry(top_bar, textvariable=self.pwm_test_pulse_var, width=6).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Button(top_bar, text="開始記錄 CSV", command=self.start_csv_logging).pack(side=tk.LEFT)
        ttk.Button(top_bar, text="停止記錄 CSV", command=self.stop_csv_logging).pack(side=tk.LEFT, padx=8)
        ttk.Label(top_bar, textvariable=self.connection_var).pack(side=tk.LEFT, padx=12)
        ttk.Label(top_bar, textvariable=self.last_update_var).pack(side=tk.RIGHT)

        csv_bar = ttk.Frame(self.main_frame, padding=(10, 0, 10, 10))
        csv_bar.pack(fill=tk.X)
        ttk.Label(csv_bar, text="CSV:").pack(side=tk.LEFT)
        ttk.Label(csv_bar, textvariable=self.csv_path_var).pack(side=tk.LEFT, padx=8)
        ttk.Label(csv_bar, text="模式:").pack(side=tk.LEFT, padx=(20, 4))
        ttk.Radiobutton(csv_bar, text="自動", value="auto", variable=self.control_mode_var, command=self.on_control_mode_changed).pack(side=tk.LEFT)
        ttk.Radiobutton(csv_bar, text="手動", value="manual", variable=self.control_mode_var, command=self.on_control_mode_changed).pack(side=tk.LEFT, padx=(4, 0))

        content = ttk.Frame(self.main_frame, padding=10)
        content.pack(fill=tk.BOTH, expand=True)
        content.columnconfigure(0, weight=3)
        content.columnconfigure(1, weight=3)
        content.rowconfigure(0, weight=1)

        left = ttk.Frame(content)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        left.rowconfigure(1, weight=1)

        self.horizon = HorizonWidget(left)
        self.horizon.pack(fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(left, height=18, bg="#020617", fg="#e2e8f0", font=("Consolas", 10))
        self.log_text.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        self.log_text.configure(state=tk.DISABLED)

        right = ttk.Frame(content)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.columnconfigure(1, weight=1)
        right.columnconfigure(2, weight=1)
        right.rowconfigure(0, weight=3)
        right.rowconfigure(1, weight=4)
        right.rowconfigure(2, weight=2)

        radio = ttk.LabelFrame(right, text="遙控器控制面板", padding=10)
        radio.grid(row=0, column=0, columnspan=3, sticky="nsew", padx=5, pady=5)
        radio.columnconfigure(0, weight=1)
        radio.columnconfigure(1, weight=1)
        radio.rowconfigure(1, weight=1)
        ttk.Label(
            radio,
            text="Quad X 控制量: rc1=Roll  rc2=Pitch  rc3=Throttle  rc4=Yaw；輸出層已混控成 M1~M4。",
        ).grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 8))

        self.left_stick = StickWidget(radio, "左搖桿", "Yaw", "Throttle", throttle_mode=True)
        self.left_stick.grid(row=1, column=0, sticky="nsew", padx=(0, 8))
        self.right_stick = StickWidget(radio, "右搖桿", "Roll", "Pitch")
        self.right_stick.grid(row=1, column=1, sticky="nsew", padx=(8, 0))

        self.manual_frame = ttk.LabelFrame(right, text="四馬達手動油門 Debug", padding=10)
        self.manual_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        self.manual_frame.columnconfigure(0, weight=1)
        self.manual_mode_label = ttk.Label(self.manual_frame, text="AUTO")
        self.manual_mode_label.pack(pady=(0, 8))
        self.manual_sliders_frame = ttk.Frame(self.manual_frame)
        self.manual_sliders_frame.pack(fill=tk.BOTH, expand=True)
        for column in range(5):
            self.manual_sliders_frame.columnconfigure(column, weight=1)

        common_frame = ttk.Frame(self.manual_sliders_frame)
        common_frame.grid(row=0, column=0, padx=(0, 6), sticky="n")
        ttk.Label(common_frame, text="共同").pack()
        self.manual_throttle_scale = tk.Scale(
            common_frame,
            from_=100,
            to=0,
            orient=tk.VERTICAL,
            length=220,
            width=24,
            sliderlength=28,
            resolution=1,
            variable=self.manual_throttle_var,
            command=self.on_manual_throttle_changed,
            state=tk.DISABLED,
            showvalue=False,
            bg="#dbe4f0",
            fg="#0f172a",
            troughcolor="#1e293b",
            activebackground="#38bdf8",
            highlightbackground="#94a3b8",
            highlightcolor="#38bdf8",
            highlightthickness=1,
            bd=2,
            relief=tk.RAISED,
            sliderrelief=tk.RAISED,
        )
        self.manual_throttle_scale.pack()
        self.manual_throttle_value_label = ttk.Label(common_frame, text="0%\n1000us", justify=tk.CENTER)
        self.manual_throttle_value_label.pack(pady=(4, 0))

        self.manual_motor_scales: list[tk.Scale] = []
        self.manual_motor_value_labels: list[ttk.Label] = []
        for index in range(4):
            motor_frame = ttk.Frame(self.manual_sliders_frame)
            motor_frame.grid(row=0, column=index + 1, padx=6, sticky="n")
            ttk.Label(motor_frame, text=f"M{index + 1}").pack()
            scale = tk.Scale(
                motor_frame,
                from_=100,
                to=0,
                orient=tk.VERTICAL,
                length=220,
                width=24,
                sliderlength=28,
                resolution=1,
                variable=self.manual_motor_vars[index],
                command=lambda _value, motor_index=index: self.on_manual_motor_changed(motor_index),
                state=tk.DISABLED,
                showvalue=False,
                bg="#dbe4f0",
                fg="#0f172a",
                troughcolor="#1e293b",
                activebackground="#38bdf8",
                highlightbackground="#94a3b8",
                highlightcolor="#38bdf8",
                highlightthickness=1,
                bd=2,
                relief=tk.RAISED,
                sliderrelief=tk.RAISED,
            )
            scale.pack()
            self.manual_motor_scales.append(scale)
            label = ttk.Label(motor_frame, text="0%\n1000us", justify=tk.CENTER)
            label.pack(pady=(4, 0))
            self.manual_motor_value_labels.append(label)
        self.manual_hint_label = ttk.Label(self.manual_frame, text="共同滑桿會同步設定 PA0/PA1/PA2/PA3；你也可以再用 M1~M4 個別微調每一路。", wraplength=300, justify=tk.LEFT)
        self.manual_hint_label.pack(pady=(8, 0), anchor="w")

        self._add_group(
            right,
            "IMU",
            ["ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "roll_deg", "pitch_deg", "yaw_deg"],
            1,
            1,
        )
        self._add_group(right, "PID", ["pid_roll_out", "pid_pitch_out", "pid_yaw_out"], 1, 2)
        self._add_group(
            right,
            "輸出 PWM",
            ["motor1_us", "motor2_us", "motor3_us", "motor4_us"],
            2,
            0,
            columnspan=2,
        )
        self._add_group(right, "RC 原始輸入", ["rc1", "rc2", "rc3", "rc4", "rc5"], 2, 2)

    def _add_group(self, master, title: str, fields: list[str], row: int, column: int, columnspan: int = 1) -> None:
        frame = ttk.LabelFrame(master, text=title, padding=10)
        frame.grid(row=row, column=column, columnspan=columnspan, sticky="nsew", padx=5, pady=5)
        for idx, field_name in enumerate(fields):
            ttk.Label(frame, text=field_name).grid(row=idx, column=0, sticky="w")
            ttk.Label(frame, textvariable=self.value_vars[field_name], font=("Consolas", 11)).grid(row=idx, column=1, sticky="e", padx=(12, 0))

    def reset_telemetry_view(self) -> None:
        self.telemetry = TelemetryState()
        for name in CSV_FIELD_NAMES:
            self.value_vars[name].set("--")
        self.horizon.redraw(0.0, 0.0, 0.0)
        self.left_stick.redraw(50.0, 0.0)
        self.right_stick.redraw(50.0, 50.0)
        self.manual_slider_updating = True
        self.manual_throttle_var.set(0)
        for motor_var in self.manual_motor_vars:
            motor_var.set(0)
        self.manual_slider_updating = False
        self.update_manual_throttle_label()
        self.update_responsive_layout()

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

        self.reset_telemetry_view()
        self.last_update_var.set("等待 IMU/遙測資料")
        self.reader = SerialReader(self.port_var.get(), baudrate, self.message_queue)
        self.reader.start()
        self.connection_var.set(f"連線中 {self.port_var.get()} ...")

    def disconnect(self) -> None:
        if self.reader is not None:
            self.reader.stop()
            self.reader = None
        self.connection_var.set("尚未連線")
        self.last_update_var.set("等待 IMU/遙測資料")
        self.control_mode_var.set("auto")
        self.update_manual_controls()
        self.reset_telemetry_view()

    def send_command(self, command: str, description: str) -> None:
        if self.reader is None or not self.reader.send_command(command):
            self.append_log(f"無法送出命令: {description}，請先連線")
            return

        self.append_log(f">>> {command}")

    def calibrate_imu(self) -> None:
        self.last_update_var.set("IMU 校正中，請保持靜止")
        self.send_command("CALIBRATE", "校正 IMU")

    def reset_yaw(self) -> None:
        self.send_command("RESET_YAW", "重置 Yaw")

    def enable_pwm_square_test(self) -> None:
        duty_percent = max(0, min(100, self.pwm_test_duty_var.get()))
        self.pwm_test_duty_var.set(duty_percent)
        self.send_command(f"PWMTESTDUTY {duty_percent}", "設定 PWM400 方波 Duty")
        self.send_command("PWMTEST SQUARE ON", "啟用 PWM400 方波測試模式")

    def enable_pwm_esc_test(self) -> None:
        pulse_us = max(1000, min(2000, self.pwm_test_pulse_var.get()))
        self.pwm_test_pulse_var.set(pulse_us)
        self.send_command(f"PWMTESTUS {pulse_us}", "設定 PWM400 測試脈寬")
        self.send_command("PWMTEST ESC ON", "啟用 ESC400 脈波測試模式")

    def enable_raw_tim2_test(self) -> None:
        self.send_command("PWMTEST RAWTIM2 ON", "啟用 TIM2 CH1 原始 400Hz 方波測試")

    def disable_pwm_test(self) -> None:
        self.send_command("PWMTEST OFF", "停用 PWM400 測試模式")

    def enable_pa0_high_test(self) -> None:
        self.send_command("PINTEST PA0 HIGH", "啟用 PA0 High 腳位身份測試")

    def enable_pa0_low_test(self) -> None:
        self.send_command("PINTEST PA0 LOW", "啟用 PA0 Low 腳位身份測試")

    def enable_pa0_blink_test(self) -> None:
        self.send_command("PINTEST PA0 BLINK", "啟用 PA0 1Hz 腳位身份測試")

    def disable_pin_test(self) -> None:
        self.send_command("PINTEST OFF", "停用 PA0 腳位身份測試")

    def on_control_mode_changed(self) -> None:
        mode = self.control_mode_var.get()
        self.update_manual_controls()
        if mode == "manual":
            self.send_command("MODE MANUAL", "切換手動模式")
            self.send_manual_throttle_command()
        else:
            self.manual_slider_updating = True
            self.manual_throttle_var.set(0)
            for motor_var in self.manual_motor_vars:
                motor_var.set(0)
            self.manual_slider_updating = False
            self.update_manual_throttle_label()
            self.send_command("MODE AUTO", "切換自動模式")

    def update_manual_controls(self) -> None:
        manual_enabled = (self.control_mode_var.get() == "manual")
        self.manual_mode_label.configure(text="MANUAL" if manual_enabled else "AUTO")
        self.manual_throttle_scale.configure(state=tk.NORMAL)
        for scale in self.manual_motor_scales:
            scale.configure(state=tk.NORMAL)

    def update_manual_throttle_label(self) -> None:
        throttle_percent = self.manual_throttle_var.get()
        throttle_us = 1000 + int((throttle_percent / 100.0) * 1000.0)
        self.manual_throttle_value_label.configure(text=f"{throttle_percent}%\n{throttle_us}us")
        for index, motor_var in enumerate(self.manual_motor_vars):
            motor_percent = motor_var.get()
            motor_us = 1000 + int((motor_percent / 100.0) * 1000.0)
            self.manual_motor_value_labels[index].configure(text=f"{motor_percent}%\n{motor_us}us")

    def send_manual_throttle_command(self) -> None:
        throttle_percent = self.manual_throttle_var.get()
        self.send_command(f"THROTTLE {throttle_percent}", "設定手動油門")

    def on_manual_throttle_changed(self, _value: str) -> None:
        if self.manual_slider_updating:
            return
        self.manual_slider_updating = True
        throttle_percent = self.manual_throttle_var.get()
        for motor_var in self.manual_motor_vars:
            motor_var.set(throttle_percent)
        self.manual_slider_updating = False
        self.update_manual_throttle_label()
        if self.control_mode_var.get() == "manual":
            self.send_manual_throttle_command()
            for index in range(4):
                self.send_manual_motor_command(index)

    def send_manual_motor_command(self, motor_index: int) -> None:
        motor_percent = self.manual_motor_vars[motor_index].get()
        self.send_command(f"MOTOR{motor_index + 1} {motor_percent}", f"設定 M{motor_index + 1} 油門")

    def on_manual_motor_changed(self, motor_index: int) -> None:
        if self.manual_slider_updating:
            return
        self.update_manual_throttle_label()
        if self.control_mode_var.get() == "manual":
            self.send_manual_motor_command(motor_index)

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
                    self.last_update_var.set("等待 IMU/遙測資料")

        self.root.after(50, self.process_queue)

    def handle_line(self, line: str) -> None:
        values = self.try_parse_csv(line)
        if values is not None:
            for name, value in zip(CSV_FIELD_NAMES, values):
                setattr(self.telemetry, name, value)
                self.value_vars[name].set(f"{value:.2f}")
            self.telemetry.last_update_monotonic = time.monotonic()
            self.horizon.redraw(self.telemetry.roll_deg, self.telemetry.pitch_deg, self.telemetry.yaw_deg)
            self.update_stick_views()
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
            values = [float(part) for part in parts]
        except ValueError:
            return None

        all_zero = all(abs(value) < 1e-6 for value in values)
        if all_zero:
            self.last_update_var.set("等待 IMU/遙測資料")
            return None

        return values

    def update_stick_views(self) -> None:
        roll_percent = self.rc_to_percent(self.telemetry.rc1, default=50.0)
        pitch_percent = self.rc_to_percent(self.telemetry.rc2, default=50.0)
        throttle_percent = self.rc_to_percent(self.telemetry.rc3, default=0.0)
        yaw_percent = self.rc_to_percent(self.telemetry.rc4, default=50.0)

        self.left_stick.redraw(yaw_percent, throttle_percent)
        self.right_stick.redraw(roll_percent, 100.0 - pitch_percent)

    @staticmethod
    def rc_to_percent(rc_value: float, default: float) -> float:
        if rc_value < 800.0 or rc_value > 2200.0:
            return default
        return max(0.0, min(100.0, (rc_value - 1000.0) * 0.1))

    @staticmethod
    def us_to_percent(pulse_us: float, default: float) -> float:
        if pulse_us <= 0.0:
            return default
        return max(0.0, min(100.0, (pulse_us - 1000.0) * 0.1))

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

    def on_main_frame_configure(self, _event) -> None:
        self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all"))

    def on_canvas_configure(self, event) -> None:
        min_width = 1380
        window_width = max(event.width, min_width)
        self.scroll_canvas.itemconfigure(self.main_frame_window, width=window_width)
        self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all"))

    def on_root_resize(self, _event) -> None:
        self.update_responsive_layout()

    def update_responsive_layout(self) -> None:
        if not hasattr(self, "manual_frame"):
            return

        manual_height = max(self.manual_frame.winfo_height(), 260)
        slider_length = max(120, min(260, manual_height - 120))
        for scale in [self.manual_throttle_scale, *self.manual_motor_scales]:
            scale.configure(length=slider_length)

        hint_wrap = max(240, min(420, self.manual_frame.winfo_width() - 30))
        self.manual_hint_label.configure(wraplength=hint_wrap)

    def update_freshness(self) -> None:
        if self.telemetry.last_update_monotonic <= 0.0:
            self.last_update_var.set("等待 IMU/遙測資料")
        else:
            age = time.monotonic() - self.telemetry.last_update_monotonic
            if age > 2.0:
                self.last_update_var.set("等待 IMU/遙測資料")
            else:
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
