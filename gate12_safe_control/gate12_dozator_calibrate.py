#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Minimal Gate12 dispenser servo calibrator.

Controls one ESP32 gate servo through the existing gate12 safe firmware.
Movement happens only while a left/right button is held, or while patrol mode is
explicitly enabled.
"""

from __future__ import annotations

import json
import re
import time
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


APP_DIR = Path(__file__).resolve().parent
CONFIG_PATH = APP_DIR / "gate12_dozator_calibrate.json"
BAUD = 115200
CENTER_US = 1500
DEFAULT_CLOSED_US = 1270
DEFAULT_EJECT_US = 1846

DEFAULTS = {
    "port": "COM6",
    "pos_us": CENTER_US,
    "zero_us": CENTER_US,
    "left_us": DEFAULT_CLOSED_US,
    "right_us": DEFAULT_EJECT_US,
    "closed_us": DEFAULT_CLOSED_US,
    "eject_us": DEFAULT_EJECT_US,
    "guard_min_us": 700,
    "guard_max_us": 2300,
    "speed_us_s": 1000,
    "patrol_pause_s": 0.4,
    "invert": False,
    "hold_torque": False,
}

INFO_RE = re.compile(r"\bpos=(\d+)\b.*\btarget=(\d+)\b")
OK_POS_RE = re.compile(r"OK (?:STOP|SET)\s+(?:\d+\s+)?(\d+)$")


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def load_config() -> dict:
    cfg = DEFAULTS.copy()
    if CONFIG_PATH.exists():
        try:
            data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            cfg.update({k: data[k] for k in cfg if k in data})
        except Exception:
            pass
    return cfg


def save_config(cfg: dict) -> None:
    CONFIG_PATH.write_text(json.dumps(cfg, ensure_ascii=False, indent=2), encoding="utf-8")


def list_ports() -> list[str]:
    if serial is None:
        return []
    ports = list(serial.tools.list_ports.comports())
    scored = []
    for p in ports:
        text = f"{p.device} {p.description or ''}".lower()
        score = 0
        if any(k in text for k in ("ch340", "cp210", "ch910", "usb-serial", "uart", "esp32")):
            score -= 10
        scored.append((score, p.device))
    return [p for _, p in sorted(scored)]


class SerialLink:
    def __init__(self) -> None:
        self.ser = None

    @property
    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    def open(self, port: str) -> None:
        if serial is None:
            raise RuntimeError("Нет pyserial. Установи: pip install pyserial")
        self.close()
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = BAUD
        ser.timeout = 0.02
        ser.write_timeout = 0.5
        ser.dtr = False
        ser.rts = False
        ser.open()
        self.ser = ser

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def write(self, line: str) -> None:
        if not self.is_open:
            raise RuntimeError("COM-порт не подключен")
        self.ser.write((line.strip() + "\n").encode("ascii", errors="ignore"))
        self.ser.flush()

    def read_lines(self) -> list[str]:
        if not self.is_open:
            return []
        out = []
        while True:
            raw = self.ser.readline()
            if not raw:
                break
            out.append(raw.decode("utf-8", errors="replace").strip())
        return out


class App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.cfg = load_config()
        self.link = SerialLink()
        self.connected = False
        self.holding = False
        self.patrol = False
        self.patrol_side = "left"
        self.next_patrol_at = 0.0
        self.last_info_at = 0.0

        self.port_var = tk.StringVar(value=str(self.cfg["port"]))
        self.status_var = tk.StringVar(value="не подключено")
        self.pos_var = tk.IntVar(value=int(self.cfg["pos_us"]))
        self.zero_var = tk.IntVar(value=CENTER_US)
        self.left_var = tk.IntVar(value=int(self.cfg["left_us"]))
        self.right_var = tk.IntVar(value=int(self.cfg["right_us"]))
        self.closed_var = tk.IntVar(value=int(self.cfg["closed_us"]))
        self.eject_var = tk.IntVar(value=int(self.cfg["eject_us"]))
        self.guard_min_var = tk.IntVar(value=int(self.cfg["guard_min_us"]))
        self.guard_max_var = tk.IntVar(value=int(self.cfg["guard_max_us"]))
        self.speed_var = tk.IntVar(value=int(self.cfg["speed_us_s"]))
        self.pause_var = tk.DoubleVar(value=float(self.cfg["patrol_pause_s"]))
        self.invert_var = tk.BooleanVar(value=bool(self.cfg["invert"]))
        self.hold_torque_var = tk.BooleanVar(value=bool(self.cfg["hold_torque"]))

        self._build_ui()
        self._refresh_ports()
        self._tick()

    def _build_ui(self) -> None:
        self.root.title("Дозатор банок - калибровка серво")
        self.root.geometry("760x680")
        self.root.minsize(720, 640)
        self.root.configure(bg="#101418")

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#101418")
        style.configure("Panel.TFrame", background="#171d23")
        style.configure("TLabel", background="#101418", foreground="#e8edf2", font=("Segoe UI", 10))
        style.configure("Muted.TLabel", background="#101418", foreground="#95a3b3", font=("Segoe UI", 9))
        style.configure("Panel.TLabel", background="#171d23", foreground="#e8edf2", font=("Segoe UI", 10))
        style.configure("TButton", font=("Segoe UI", 10), padding=8)
        style.configure("Big.TButton", font=("Segoe UI", 16, "bold"), padding=18)
        style.configure("Stop.TButton", font=("Segoe UI", 13, "bold"), padding=12)

        page = ttk.Frame(self.root, padding=16)
        page.pack(fill="both", expand=True)

        ttk.Label(page, text="Дозатор банок: калибровка серво", font=("Segoe UI", 16, "bold")).pack(anchor="w")
        ttk.Label(page, textvariable=self.status_var, style="Muted.TLabel").pack(anchor="w", pady=(2, 12))

        top = ttk.Frame(page, style="Panel.TFrame", padding=10)
        top.pack(fill="x", pady=(0, 12))
        ttk.Label(top, text="Порт", style="Panel.TLabel").grid(row=0, column=0, sticky="w")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, values=[], width=12)
        self.port_box.grid(row=0, column=1, padx=8, sticky="w")
        ttk.Button(top, text="Обновить", command=self._refresh_ports).grid(row=0, column=2, padx=(0, 8))
        ttk.Button(top, text="Подключить", command=self._connect).grid(row=0, column=3, padx=(0, 8))
        ttk.Button(top, text="Отключить сигнал", command=self._detach).grid(row=0, column=4)
        top.columnconfigure(5, weight=1)

        pos_panel = ttk.Frame(page, style="Panel.TFrame", padding=14)
        pos_panel.pack(fill="x", pady=(0, 12))
        ttk.Label(pos_panel, text="Текущее положение", style="Panel.TLabel").pack(anchor="w")
        self.pos_label = ttk.Label(
            pos_panel,
            text=f"{self.pos_var.get()} мкс",
            style="Panel.TLabel",
            font=("Segoe UI", 36, "bold"),
        )
        self.pos_label.pack(anchor="center", pady=(2, 2))

        zero_panel = ttk.Frame(page, style="Panel.TFrame", padding=10)
        zero_panel.pack(fill="x", pady=(0, 12))
        ttk.Button(zero_panel, text="0 / центр 1500", command=self._go_zero).grid(row=0, column=0, sticky="ew", padx=(0, 8))
        ttk.Label(zero_panel, text="ноль фиксирован: 1500 мкс", style="Panel.TLabel").grid(row=0, column=1, sticky="w", padx=(0, 12))
        ttk.Button(zero_panel, text="Края симметрично от 0", command=self._make_edges_symmetric).grid(row=0, column=2, sticky="ew")
        zero_panel.columnconfigure(0, weight=1)
        zero_panel.columnconfigure(2, weight=1)

        preset_panel = ttk.Frame(page, style="Panel.TFrame", padding=10)
        preset_panel.pack(fill="x", pady=(0, 12))
        ttk.Button(preset_panel, text="В закрыто / ожидание", command=self._go_closed).grid(row=0, column=0, sticky="ew", padx=(0, 8))
        ttk.Entry(preset_panel, textvariable=self.closed_var, width=8, justify="center").grid(row=0, column=1, padx=(0, 14))
        ttk.Button(preset_panel, text="Закрыто = текущее", command=self._save_closed).grid(row=0, column=2, sticky="ew", padx=(0, 18))
        ttk.Button(preset_panel, text="В выдачу", command=self._go_eject).grid(row=0, column=3, sticky="ew", padx=(0, 8))
        ttk.Entry(preset_panel, textvariable=self.eject_var, width=8, justify="center").grid(row=0, column=4, padx=(0, 14))
        ttk.Button(preset_panel, text="Выдача = текущее", command=self._save_eject).grid(row=0, column=5, sticky="ew")
        preset_panel.columnconfigure(0, weight=1)
        preset_panel.columnconfigure(2, weight=1)
        preset_panel.columnconfigure(3, weight=1)
        preset_panel.columnconfigure(5, weight=1)

        speed_panel = ttk.Frame(page, style="Panel.TFrame", padding=10)
        speed_panel.pack(fill="x", pady=(0, 12))
        ttk.Label(speed_panel, text="Скорость", style="Panel.TLabel").grid(row=0, column=0, sticky="w")
        self.speed_scale = tk.Scale(
            speed_panel,
            from_=5,
            to=1000,
            orient="horizontal",
            resolution=5,
            showvalue=False,
            variable=self.speed_var,
            command=lambda _value: self._speed_changed(),
            bg="#171d23",
            fg="#e8edf2",
            troughcolor="#303842",
            highlightthickness=0,
            activebackground="#78b7ff",
        )
        self.speed_scale.grid(row=0, column=1, sticky="ew", padx=10)
        self.speed_label = ttk.Label(speed_panel, text=f"{self.speed_var.get()} мкс/с", style="Panel.TLabel")
        self.speed_label.grid(row=0, column=2, sticky="e")
        ttk.Checkbutton(
            speed_panel,
            text="держать усилие после стопа (греет)",
            variable=self.hold_torque_var,
            command=self._hold_torque_changed,
        ).grid(row=1, column=0, columnspan=3, sticky="w", pady=(8, 0))
        speed_panel.columnconfigure(1, weight=1)

        move = ttk.Frame(page, style="Panel.TFrame", padding=10)
        move.pack(fill="x", pady=(0, 12))
        self.left_btn = ttk.Button(move, text="держать ←", style="Big.TButton")
        self.left_btn.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        self.stop_btn = ttk.Button(move, text="СТОП", style="Stop.TButton", command=self._stop)
        self.stop_btn.grid(row=0, column=1, sticky="nsew", padx=8)
        self.right_btn = ttk.Button(move, text="держать →", style="Big.TButton")
        self.right_btn.grid(row=0, column=2, sticky="nsew", padx=(8, 0))
        self.left_btn.bind("<ButtonPress-1>", lambda _event: self._hold_start("left"))
        self.left_btn.bind("<ButtonRelease-1>", lambda _event: self._hold_stop())
        self.right_btn.bind("<ButtonPress-1>", lambda _event: self._hold_start("right"))
        self.right_btn.bind("<ButtonRelease-1>", lambda _event: self._hold_stop())
        for col in range(3):
            move.columnconfigure(col, weight=1)

        edges = ttk.Frame(page, style="Panel.TFrame", padding=10)
        edges.pack(fill="x", pady=(0, 12))
        ttk.Button(edges, text="Левый край = текущее", command=self._save_left).grid(row=0, column=0, sticky="ew", padx=(0, 8))
        ttk.Entry(edges, textvariable=self.left_var, width=8, justify="center").grid(row=0, column=1, padx=(0, 18))
        ttk.Button(edges, text="Правый край = текущее", command=self._save_right).grid(row=0, column=2, sticky="ew", padx=(0, 8))
        ttk.Entry(edges, textvariable=self.right_var, width=8, justify="center").grid(row=0, column=3, padx=(0, 18))
        ttk.Checkbutton(edges, text="инвертировать ←/→", variable=self.invert_var, command=self._save_all).grid(row=0, column=4, sticky="e")
        edges.columnconfigure(0, weight=1)
        edges.columnconfigure(2, weight=1)

        patrol = ttk.Frame(page, style="Panel.TFrame", padding=10)
        patrol.pack(fill="x", pady=(0, 12))
        self.patrol_btn = ttk.Button(patrol, text="Патруль старт", command=self._toggle_patrol)
        self.patrol_btn.grid(row=0, column=0, sticky="ew", padx=(0, 10))
        ttk.Label(patrol, text="пауза у края, сек", style="Panel.TLabel").grid(row=0, column=1, sticky="e")
        ttk.Entry(patrol, textvariable=self.pause_var, width=6, justify="center").grid(row=0, column=2, padx=(8, 18))
        ttk.Label(patrol, text="защита мин", style="Panel.TLabel").grid(row=0, column=3, sticky="e")
        ttk.Entry(patrol, textvariable=self.guard_min_var, width=7, justify="center").grid(row=0, column=4, padx=(8, 12))
        ttk.Label(patrol, text="макс", style="Panel.TLabel").grid(row=0, column=5, sticky="e")
        ttk.Entry(patrol, textvariable=self.guard_max_var, width=7, justify="center").grid(row=0, column=6, padx=(8, 0))
        patrol.columnconfigure(0, weight=1)

        self.result_label = ttk.Label(page, text="", style="Muted.TLabel", font=("Consolas", 12))
        self.result_label.pack(anchor="w")
        self._update_result()

    def _refresh_ports(self) -> None:
        ports = list_ports()
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _connect(self) -> bool:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Порт", "Выбери COM-порт ESP32.")
            return False
        try:
            self.link.open(port)
        except Exception as exc:
            self.status_var.set(f"ошибка: {exc}")
            return False
        self.connected = True
        self.status_var.set(f"подключено: {port}; движения нет, пока не держишь кнопку")
        self._send_limits()
        self._send_speed()
        self._send_hold()
        self._send("DETACH")
        self._send("INFO")
        self._save_all()
        return True

    def _ensure_connected(self) -> bool:
        if self.link.is_open:
            return True
        return self._connect()

    def _send(self, line: str) -> None:
        if not self.link.is_open:
            return
        try:
            self.link.write(line)
        except Exception as exc:
            self.status_var.set(f"serial ошибка: {exc}")
            self.connected = False

    def _send_limits(self) -> None:
        lo = int(self.guard_min_var.get())
        hi = int(self.guard_max_var.get())
        if lo >= hi:
            lo, hi = 700, 2300
        self._send(f"LIM {lo} {hi}")

    def _send_speed(self) -> None:
        speed = int(self.speed_var.get())
        self.speed_label.configure(text=f"{speed} мкс/с")
        self._send(f"RATE {speed}")

    def _send_hold(self) -> None:
        self._send(f"HOLD {1 if self.hold_torque_var.get() else 0}")

    def _go_to(self, us: int, label: str) -> None:
        if not self._ensure_connected():
            return
        target = clamp(int(us), int(self.guard_min_var.get()), int(self.guard_max_var.get()))
        self.holding = False
        self.patrol = False
        self.patrol_btn.configure(text="Патруль старт")
        self._send_limits()
        self._send_speed()
        self._send_hold()
        self._send(f"SET {target}")
        self.status_var.set(f"идёт: {label} = {target} мкс")
        self._save_all()

    def _speed_changed(self) -> None:
        self._send_speed()
        self._save_all()

    def _hold_torque_changed(self) -> None:
        self._send_hold()
        self._save_all()

    def _logical_dir(self, side: str) -> int:
        direction = -1 if side == "left" else 1
        if self.invert_var.get():
            direction = -direction
        return direction

    def _hold_start(self, side: str) -> None:
        self.patrol = False
        self.patrol_btn.configure(text="Патруль старт")
        if not self._ensure_connected():
            return
        self._send_limits()
        self._send_speed()
        self._send_hold()
        self.holding = True
        self.status_var.set("движение: отпусти кнопку у нужного края")
        self._send(f"JOG {self._logical_dir(side)}")

    def _hold_stop(self) -> None:
        if not self.holding:
            return
        self.holding = False
        self._send("STOP")
        self._send("INFO")
        if self.hold_torque_var.get():
            self.status_var.set("остановлено; серво держит усилие")
        else:
            self.status_var.set("остановлено; сигнал снимется, можно сохранить край")

    def _stop(self) -> None:
        self.holding = False
        self.patrol = False
        self.patrol_btn.configure(text="Патруль старт")
        if self.link.is_open:
            self._send("STOP")
            self._send("INFO")
        self.status_var.set("стоп")

    def _detach(self) -> None:
        self._stop()
        self._send("DETACH")
        self.status_var.set("сигнал серво отключен")

    def _go_zero(self) -> None:
        self.zero_var.set(CENTER_US)
        self._go_to(CENTER_US, "0 / центр")

    def _go_closed(self) -> None:
        self._go_to(int(self.closed_var.get()), "закрыто")

    def _go_eject(self) -> None:
        self._go_to(int(self.eject_var.get()), "выдача")

    def _save_closed(self) -> None:
        self.closed_var.set(int(self.pos_var.get()))
        self.left_var.set(int(self.pos_var.get()))
        self._save_all()
        self._update_result()

    def _save_eject(self) -> None:
        self.eject_var.set(int(self.pos_var.get()))
        self.right_var.set(int(self.pos_var.get()))
        self._save_all()
        self._update_result()

    def _make_edges_symmetric(self) -> None:
        self.zero_var.set(CENTER_US)
        zero = CENTER_US
        left = int(self.left_var.get())
        right = int(self.right_var.get())
        guard_min = int(self.guard_min_var.get())
        guard_max = int(self.guard_max_var.get())
        span = min(abs(zero - left), abs(right - zero), zero - guard_min, guard_max - zero)
        if span < 10:
            messagebox.showwarning("Симметрия", "Маленький ход. Сначала выстави левый/правый край или проверь ноль.")
            return
        self.left_var.set(zero - span)
        self.right_var.set(zero + span)
        self._save_all()
        self._update_result()
        self.status_var.set(f"края симметричны: {zero - span} / {zero + span}")

    def _save_left(self) -> None:
        self.left_var.set(int(self.pos_var.get()))
        self.closed_var.set(int(self.pos_var.get()))
        self._save_all()
        self._update_result()

    def _save_right(self) -> None:
        self.right_var.set(int(self.pos_var.get()))
        self.eject_var.set(int(self.pos_var.get()))
        self._save_all()
        self._update_result()

    def _toggle_patrol(self) -> None:
        if self.patrol:
            self._stop()
            return
        if not self._ensure_connected():
            return
        left = int(self.left_var.get())
        right = int(self.right_var.get())
        if abs(left - right) < 20:
            messagebox.showwarning("Патруль", "Сначала сохрани левый и правый край.")
            return
        self._send_limits()
        self._send_speed()
        self._send_hold()
        self.patrol = True
        self.patrol_side = "left"
        self.patrol_btn.configure(text="Патруль стоп")
        self.status_var.set("патруль в сохранённых пределах")
        self._patrol_send_next()

    def _patrol_send_next(self) -> None:
        target = int(self.left_var.get()) if self.patrol_side == "left" else int(self.right_var.get())
        self._send(f"SET {target}")
        distance = abs(int(self.pos_var.get()) - target)
        speed = max(20, int(self.speed_var.get()))
        pause = max(0.0, float(self.pause_var.get()))
        self.next_patrol_at = time.time() + distance / speed + pause + 0.2
        self.patrol_side = "right" if self.patrol_side == "left" else "left"

    def _parse_line(self, line: str) -> None:
        match = INFO_RE.search(line)
        if match:
            self._set_pos(int(match.group(1)))
            return
        match = OK_POS_RE.search(line)
        if match:
            self._set_pos(int(match.group(1)))

    def _set_pos(self, pos: int) -> None:
        self.pos_var.set(pos)
        self.pos_label.configure(text=f"{pos} мкс")
        self._save_all()
        self._update_result()

    def _tick(self) -> None:
        if self.link.is_open:
            for line in self.link.read_lines():
                self._parse_line(line)

            now = time.time()
            if now - self.last_info_at > 0.35:
                self._send("INFO")
                self.last_info_at = now

            if self.patrol and now >= self.next_patrol_at:
                self._patrol_send_next()

        self.root.after(80, self._tick)

    def _update_result(self) -> None:
        self.result_label.configure(
            text=f"Дозатор: CLOSED={int(self.closed_var.get())}  EJECT={int(self.eject_var.get())}  ZERO={CENTER_US}  SPEED={int(self.speed_var.get())}"
        )

    def _save_all(self) -> None:
        self.cfg.update(
            port=self.port_var.get().strip(),
            pos_us=int(self.pos_var.get()),
            zero_us=int(self.zero_var.get()),
            left_us=int(self.left_var.get()),
            right_us=int(self.right_var.get()),
            closed_us=int(self.closed_var.get()),
            eject_us=int(self.eject_var.get()),
            guard_min_us=int(self.guard_min_var.get()),
            guard_max_us=int(self.guard_max_var.get()),
            speed_us_s=int(self.speed_var.get()),
            patrol_pause_s=float(self.pause_var.get()),
            invert=bool(self.invert_var.get()),
            hold_torque=bool(self.hold_torque_var.get()),
        )
        save_config(self.cfg)

    def close(self) -> None:
        try:
            self._stop()
        finally:
            self.link.close()


def main() -> None:
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.close(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
