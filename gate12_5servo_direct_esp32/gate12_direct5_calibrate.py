#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Guided hold-to-move calibration for five direct ESP32 servos."""

from __future__ import annotations

import json
import re
import time
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

import serial
import serial.tools.list_ports


APP_DIR = Path(__file__).resolve().parent
CONFIG_PATH = APP_DIR / "gate12_direct5_app.json"
BACKUP_PATH = APP_DIR / "gate12_direct5_calibration_backup.json"
BAUD = 115200
PINS = ("D13 / GPIO13", "D14 / GPIO14", "D27 / GPIO27", "D26 / GPIO26", "D25 / GPIO25")
NUM_SERVOS = len(PINS)
CAL_MIN_US = 700
CAL_MAX_US = 2300
DEFAULT_US = 1500
DEFAULT_HOME_US = 817   # 30 degrees on the 500..2400 us servo scale
DEFAULT_EJECT_US = 1661 # 110 degrees on the 500..2400 us servo scale
DEFAULT_JOG_RATE = 120
TEST_RATE = 500
POWER_MAX_MS = 3000


def default_config() -> dict:
    return {
        "port": "COM6",
        "selected_servo": 0,
        "cooldown_s": 3.0,
        "eject_hold_ms": 900,
        "power_max_ms": 3000,
        "hold_torque": False,
        "servos": [
            {
                "name": f"Дорожка {i + 1}",
                "enabled": True,
                "closed_us": DEFAULT_HOME_US,
                "eject_us": DEFAULT_EJECT_US,
                "speed_us_s": TEST_RATE,
                "guard_min_us": 757,
                "guard_max_us": 1721,
                "up_sign": 1,
                "home_fixed": False,
                "eject_fixed": False,
            }
            for i in range(NUM_SERVOS)
        ],
    }


def load_config() -> dict:
    cfg = default_config()
    if not CONFIG_PATH.exists():
        return cfg
    try:
        saved = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        for key in ("port", "selected_servo", "cooldown_s", "eject_hold_ms", "power_max_ms", "hold_torque"):
            if key in saved:
                cfg[key] = saved[key]
        for idx, item in enumerate(saved.get("servos", [])[:NUM_SERVOS]):
            if isinstance(item, dict):
                cfg["servos"][idx].update(item)
    except Exception:
        pass
    return cfg


def save_config(cfg: dict) -> None:
    payload = json.dumps(cfg, ensure_ascii=False, indent=2)
    CONFIG_PATH.write_text(payload, encoding="utf-8")
    BACKUP_PATH.write_text(payload, encoding="utf-8")


def available_ports() -> list[str]:
    ports = list(serial.tools.list_ports.comports())
    ports.sort(key=lambda p: (0 if any(x in (p.description or "").lower() for x in ("ch340", "cp210", "usb")) else 1, p.device))
    return [p.device for p in ports]


class SerialLink:
    def __init__(self) -> None:
        self.ser: serial.Serial | None = None
        self.firmware_info = "legacy"
        self.supports_cycle_id = False

    @property
    def open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    def connect(self, port: str) -> None:
        self.close()
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = BAUD
        ser.timeout = 0.01
        ser.write_timeout = 0.5
        ser.dtr = False
        ser.rts = False
        ser.open()
        self.ser = ser

        # Merely opening a COM port is not enough: an older sketch can use the
        # same port while silently ignoring every five-channel command. Require
        # the signature reply from gate12_direct5_safe before enabling the UI.
        time.sleep(0.7)
        ser.reset_input_buffer()
        ser.write(b"PING\n")
        ser.flush()
        deadline = time.monotonic() + 1.5
        replies: list[str] = []
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            replies.append(line)
            if line == "PONG":
                ser.write(b"VERSION\n")
                ser.flush()
                version_deadline = time.monotonic() + 0.6
                while time.monotonic() < version_deadline:
                    version_raw = ser.readline()
                    if not version_raw:
                        continue
                    version_line = version_raw.decode("utf-8", errors="replace").strip()
                    if version_line.startswith("VERSION "):
                        self.firmware_info = version_line
                        self.supports_cycle_id = "cycle_id=1" in version_line
                        break
                return

        self.close()
        detail = "; ".join(replies[-3:]) if replies else "нет ответа"
        raise RuntimeError(
            f"{port} открыт, но безопасная прошивка пяти дозаторов не отвечает "
            f"({detail}). Сначала прошей gate12_direct5_safe."
        )

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.firmware_info = "legacy"
        self.supports_cycle_id = False

    def send(self, command: str) -> None:
        if not self.open:
            raise RuntimeError("COM-порт не подключён")
        assert self.ser is not None
        self.ser.write((command.strip() + "\n").encode("ascii"))
        self.ser.flush()

    def lines(self) -> list[str]:
        if not self.open:
            return []
        assert self.ser is not None
        result = []
        while True:
            raw = self.ser.readline()
            if not raw:
                break
            result.append(raw.decode("utf-8", errors="replace").strip())
        return result


class Calibrator:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.cfg = load_config()
        self.link = SerialLink()
        self.current_us = DEFAULT_US
        self.jogging = False
        self.jog_channel: int | None = None
        self.last_info = 0.0

        self.channel_var = tk.IntVar(value=max(1, min(NUM_SERVOS, int(self.cfg.get("selected_servo", 0)) + 1)))
        self.port_var = tk.StringVar(value=str(self.cfg.get("port", "COM6")))
        self.rate_var = tk.IntVar(value=DEFAULT_JOG_RATE)
        self.position_var = tk.StringVar(value=f"{DEFAULT_US} мкс")
        self.status_var = tk.StringVar(value="подключение…")
        self.pin_var = tk.StringVar()
        self.home_var = tk.StringVar()
        self.eject_var = tk.StringVar()
        self.direction_var = tk.StringVar()

        self._build()
        self._load_channel_labels()
        self._refresh_ports()
        self.root.after(250, self._autoconnect)
        self.root.after(50, self._tick)

    def channel(self) -> int:
        return max(0, min(NUM_SERVOS - 1, int(self.channel_var.get()) - 1))

    def servo_cfg(self) -> dict:
        return self.cfg["servos"][self.channel()]

    def _build(self) -> None:
        self.root.title("Калибровка пяти дозаторов — удерживай кнопку")
        self.root.geometry("980x790")
        self.root.minsize(900, 720)
        self.root.configure(bg="#10161d")

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#10161d")
        style.configure("Card.TFrame", background="#18212b")
        style.configure("TLabel", background="#10161d", foreground="#edf4fb", font=("Segoe UI", 11))
        style.configure("Card.TLabel", background="#18212b", foreground="#edf4fb", font=("Segoe UI", 11))
        style.configure("Muted.TLabel", background="#10161d", foreground="#9aabba", font=("Segoe UI", 10))
        style.configure("TButton", font=("Segoe UI", 11, "bold"), padding=10)
        style.configure("Move.TButton", font=("Segoe UI", 17, "bold"), padding=24)
        style.configure("Mark.TButton", font=("Segoe UI", 13, "bold"), padding=15)
        style.configure("Cycle.TButton", font=("Segoe UI", 15, "bold"), padding=18)

        page = ttk.Frame(self.root, padding=18)
        page.pack(fill="both", expand=True)

        top = ttk.Frame(page)
        top.pack(fill="x")
        ttk.Label(top, text="КАЛИБРОВКА ДОЗАТОРОВ", font=("Segoe UI", 22, "bold")).pack(side="left")
        ttk.Label(top, textvariable=self.status_var, style="Muted.TLabel").pack(side="right")

        select = ttk.Frame(page, style="Card.TFrame", padding=14)
        select.pack(fill="x", pady=(14, 12))
        ttk.Label(select, text="Дорожка", style="Card.TLabel").grid(row=0, column=0, sticky="w")
        channel_box = ttk.Spinbox(select, from_=1, to=NUM_SERVOS, textvariable=self.channel_var, width=5, command=self._channel_changed, font=("Segoe UI", 14, "bold"))
        channel_box.grid(row=0, column=1, padx=(8, 22))
        channel_box.bind("<Return>", lambda _e: self._channel_changed())
        ttk.Label(select, text="Порт ESP32:", style="Card.TLabel").grid(row=0, column=2, sticky="e")
        ttk.Label(select, textvariable=self.pin_var, style="Card.TLabel", font=("Consolas", 14, "bold")).grid(row=0, column=3, padx=(8, 24))
        ttk.Label(select, text="COM", style="Card.TLabel").grid(row=0, column=4)
        self.port_box = ttk.Combobox(select, textvariable=self.port_var, width=10, state="readonly")
        self.port_box.grid(row=0, column=5, padx=8)
        ttk.Button(select, text="Подключить", command=self._connect).grid(row=0, column=6)
        select.columnconfigure(3, weight=1)

        pos = ttk.Frame(page, style="Card.TFrame", padding=14)
        pos.pack(fill="x", pady=(0, 12))
        center_row = ttk.Frame(pos, style="Card.TFrame")
        center_row.pack(fill="x", pady=(0, 10))
        ttk.Button(
            center_row,
            text="1. СБРОСИТЬ КАЛИБРОВКУ И ВЫСТАВИТЬ ЦЕНТР 1500 мкс\nТОЛЬКО СО СНЯТОЙ КАЧАЛКОЙ И ТЯГАМИ",
            style="Mark.TButton",
            command=self._center_servo,
        ).pack(side="left", fill="x", expand=True, padx=(0, 7))
        ttk.Button(
            center_row,
            text="СНЯТЬ СИГНАЛ\nОТПУСТИТЬ СЕРВУ",
            command=self._release_servo,
        ).pack(side="left", fill="both", padx=(7, 0))
        ttk.Label(pos, text="ТЕКУЩЕЕ ПОЛОЖЕНИЕ", style="Card.TLabel").pack()
        ttk.Label(pos, textvariable=self.position_var, style="Card.TLabel", font=("Consolas", 32, "bold")).pack(pady=(2, 4))
        ttk.Label(pos, text="Держи кнопку — движется. Отпусти — останавливается и снимает усилие.", style="Card.TLabel").pack()

        moves = ttk.Frame(page)
        moves.pack(fill="x", pady=(0, 10))
        self.down_btn = ttk.Button(moves, text="▼  ДЛИННАЯ РУКА ВНИЗ\nHOME / ЗАКРЫТО", style="Move.TButton")
        self.down_btn.pack(side="left", fill="x", expand=True, padx=(0, 7))
        self.up_btn = ttk.Button(moves, text="▲  ДЛИННАЯ РУКА ВВЕРХ\nВЫДАЧА / ОТКРЫТО", style="Move.TButton")
        self.up_btn.pack(side="left", fill="x", expand=True, padx=(7, 0))
        self.down_btn.bind("<ButtonPress-1>", lambda _e: self._start_jog("down"))
        self.down_btn.bind("<ButtonRelease-1>", self._stop_jog)
        self.up_btn.bind("<ButtonPress-1>", lambda _e: self._start_jog("up"))
        self.up_btn.bind("<ButtonRelease-1>", self._stop_jog)
        self.root.bind_all("<ButtonRelease-1>", self._stop_jog, add="+")

        controls = ttk.Frame(page)
        controls.pack(fill="x", pady=(0, 12))
        ttk.Button(controls, text="↔ ПОМЕНЯТЬ НАПРАВЛЕНИЯ КНОПОК", command=self._reverse_direction).pack(side="left", fill="x", expand=True, padx=(0, 7))
        ttk.Label(controls, text="Скорость калибровки", style="Muted.TLabel").pack(side="left", padx=(16, 8))
        ttk.Scale(controls, from_=30, to=300, variable=self.rate_var, orient="horizontal", length=200).pack(side="left")
        ttk.Label(controls, textvariable=self.direction_var, style="Muted.TLabel").pack(side="right", padx=(12, 0))

        marks = ttk.Frame(page)
        marks.pack(fill="x", pady=(0, 12))
        ttk.Button(marks, text="ЗАПОМНИТЬ КАК HOME\nдлинная рука внизу", style="Mark.TButton", command=self._mark_home).pack(side="left", fill="x", expand=True, padx=(0, 7))
        ttk.Button(marks, text="ЗАПОМНИТЬ КАК ВЫДАЧУ\nдлинная рука вверху", style="Mark.TButton", command=self._mark_eject).pack(side="left", fill="x", expand=True, padx=(7, 0))

        saved = ttk.Frame(page, style="Card.TFrame", padding=12)
        saved.pack(fill="x", pady=(0, 12))
        ttk.Label(saved, textvariable=self.home_var, style="Card.TLabel").pack(side="left", expand=True)
        ttk.Label(saved, textvariable=self.eject_var, style="Card.TLabel").pack(side="left", expand=True)

        tests = ttk.Frame(page)
        tests.pack(fill="x")
        ttk.Button(tests, text="Перейти в HOME", command=self._go_home).pack(side="left", fill="x", expand=True, padx=(0, 6))
        ttk.Button(tests, text="Перейти в выдачу", command=self._go_eject).pack(side="left", fill="x", expand=True, padx=6)
        self.cycle_btn = ttk.Button(tests, text="ПРОВЕРИТЬ ПОЛНЫЙ ЦИКЛ", style="Cycle.TButton", command=self._test_cycle)
        self.cycle_btn.pack(side="left", fill="x", expand=True, padx=(6, 0))

        ttk.Label(page, text="Esc — аварийный STOP всех пяти каналов. Сначала калибруй без банки.", style="Muted.TLabel").pack(anchor="w", pady=(14, 0))
        self.root.bind("<Escape>", lambda _e: self._stop_all())
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _refresh_ports(self) -> None:
        ports = available_ports()
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _autoconnect(self) -> None:
        self._connect(show_error=False)

    def _connect(self, show_error: bool = True) -> None:
        try:
            self.link.connect(self.port_var.get())
            self.cfg["port"] = self.port_var.get()
            save_config(self.cfg)
            self.status_var.set(f"подключено: {self.port_var.get()}")
            self._prepare_channel()
            self._sync_saved_calibrations()
        except Exception as exc:
            self.status_var.set(f"нет связи: {exc}")
            if show_error:
                messagebox.showerror("COM-порт", str(exc))

    def _send(self, command: str) -> None:
        if not self.link.open:
            self._connect()
        self.link.send(command)

    def _prepare_channel(self) -> None:
        ch = self.channel()
        self._send(f"CH {ch}")
        self._send(f"LIM {ch} {CAL_MIN_US} {CAL_MAX_US}")
        self._send(f"RATE {ch} {int(self.rate_var.get())}")
        self._send(f"HOLD {ch} 0")
        self._send(f"POWERMAX {ch} {POWER_MAX_MS}")
        self._send(f"INFO")

    def _sync_saved_calibrations(self) -> None:
        """Copy every completed PC calibration into ESP32 non-volatile storage."""
        for ch, servo in enumerate(self.cfg["servos"]):
            if servo.get("home_fixed") and servo.get("eject_fixed"):
                home = int(servo.get("closed_us", DEFAULT_US))
                eject = int(servo.get("eject_us", DEFAULT_US))
                if abs(home - eject) >= 20:
                    self.link.send(f"CAL {ch} {home} {eject}")

    def _channel_changed(self) -> None:
        self._stop_all()
        self.cfg["selected_servo"] = self.channel()
        save_config(self.cfg)
        self._load_channel_labels()
        if self.link.open:
            self._prepare_channel()

    def _load_channel_labels(self) -> None:
        ch = self.channel()
        servo = self.servo_cfg()
        self.pin_var.set(PINS[ch])
        home = int(servo.get("closed_us", DEFAULT_US))
        eject = int(servo.get("eject_us", DEFAULT_US))
        self.home_var.set(f"HOME: {home} мкс" + ("  ✓" if servo.get("home_fixed") else "  — не зафиксирован"))
        self.eject_var.set(f"ВЫДАЧА: {eject} мкс" + ("  ✓" if servo.get("eject_fixed") else "  — не зафиксирована"))
        self.direction_var.set("направления обычные" if int(servo.get("up_sign", 1)) > 0 else "направления поменяны")
        calibrated = bool(servo.get("home_fixed") and servo.get("eject_fixed") and abs(home - eject) >= 20)
        self.cycle_btn.state(["!disabled"] if calibrated else ["disabled"])

    def _start_jog(self, semantic_direction: str) -> None:
        if self.jogging:
            return
        ch = self.channel()
        up_sign = 1 if int(self.servo_cfg().get("up_sign", 1)) >= 0 else -1
        sign = up_sign if semantic_direction == "up" else -up_sign
        try:
            self._send(f"CH {ch}")
            self._send(f"LIM {ch} {CAL_MIN_US} {CAL_MAX_US}")
            self._send(f"RATE {ch} {max(30, min(300, int(self.rate_var.get())))}")
            self._send(f"POWERMAX {ch} {POWER_MAX_MS}")
            self._send(f"JOG {ch} {sign}")
            self.jogging = True
            self.jog_channel = ch
            label = "вверх / выдача" if semantic_direction == "up" else "вниз / HOME"
            self.status_var.set(f"дорожка {ch + 1}: движение {label}")
        except Exception as exc:
            self.status_var.set(f"ошибка: {exc}")

    def _stop_jog(self, _event=None) -> None:
        if not self.jogging:
            return
        ch = self.jog_channel if self.jog_channel is not None else self.channel()
        try:
            self._send(f"JOG {ch} 0")
            self._send(f"INFO")
            self.status_var.set(f"дорожка {ch + 1}: остановлено")
        except Exception as exc:
            self.status_var.set(f"ошибка остановки: {exc}")
        self.jogging = False
        self.jog_channel = None

    def _reverse_direction(self) -> None:
        self._stop_jog()
        servo = self.servo_cfg()
        servo["up_sign"] = -1 if int(servo.get("up_sign", 1)) > 0 else 1
        save_config(self.cfg)
        self._load_channel_labels()
        self.status_var.set("назначения кнопок вверх/вниз поменяны")

    def _center_servo(self) -> None:
        ch = self.channel()
        if not messagebox.askyesno(
            "Проверка безопасности",
            f"Дорожка {ch + 1}, {PINS[ch]}.\n\n"
            "Качалка/шестерёнка и тяги действительно сняты с вала сервы?\n\n"
            "После подтверждения старая калибровка этой дорожки будет удалена, "
            "а вал повернётся в центр 1500 мкс.",
        ):
            return
        try:
            self._stop_all()
            servo = self.servo_cfg()
            servo["closed_us"] = DEFAULT_US
            servo["eject_us"] = DEFAULT_US
            servo["guard_min_us"] = 1450
            servo["guard_max_us"] = 1550
            servo["home_fixed"] = False
            servo["eject_fixed"] = False
            servo["up_sign"] = 1
            save_config(self.cfg)

            self._send(f"CLEARCAL {ch}")
            self._send(f"LIM {ch} {CAL_MIN_US} {CAL_MAX_US}")
            self._send(f"RATE {ch} 300")
            self._send(f"POWERMAX {ch} 15000")
            self._send(f"HOLD {ch} 1")
            self._send(f"SET {ch} {DEFAULT_US}")
            self.current_us = DEFAULT_US
            self.position_var.set(f"{DEFAULT_US} мкс")
            self._load_channel_labels()
            self.status_var.set("центр 1500: установи качалку в механический HOME")
        except Exception as exc:
            self.status_var.set(f"ошибка центрирования: {exc}")
            messagebox.showerror("Центрирование", str(exc))

    def _release_servo(self) -> None:
        ch = self.channel()
        try:
            self._send(f"HOLD {ch} 0")
            self._send(f"STOP {ch}")
            self._send(f"DETACH {ch}")
            self.status_var.set(f"дорожка {ch + 1}: сигнал снят, серва отпущена")
        except Exception as exc:
            self.status_var.set(f"ошибка снятия сигнала: {exc}")

    def _mark_home(self) -> None:
        self._stop_jog()
        servo = self.servo_cfg()
        servo["closed_us"] = int(self.current_us)
        servo["home_fixed"] = True
        self._save_guard_if_complete(servo)
        save_config(self.cfg)
        self._persist_current_calibration()
        self._load_channel_labels()
        self.status_var.set(f"HOME зафиксирован: {self.current_us} мкс")

    def _mark_eject(self) -> None:
        self._stop_jog()
        servo = self.servo_cfg()
        servo["eject_us"] = int(self.current_us)
        servo["eject_fixed"] = True
        self._save_guard_if_complete(servo)
        save_config(self.cfg)
        self._persist_current_calibration()
        self._load_channel_labels()
        self.status_var.set(f"выдача зафиксирована: {self.current_us} мкс")

    @staticmethod
    def _save_guard_if_complete(servo: dict) -> None:
        if not (servo.get("home_fixed") and servo.get("eject_fixed")):
            return
        home = int(servo["closed_us"])
        eject = int(servo["eject_us"])
        servo["guard_min_us"] = max(700, min(home, eject) - 60)
        servo["guard_max_us"] = min(2300, max(home, eject) + 60)

    def _persist_current_calibration(self) -> None:
        servo = self.servo_cfg()
        if not (servo.get("home_fixed") and servo.get("eject_fixed")):
            return
        home = int(servo["closed_us"])
        eject = int(servo["eject_us"])
        if abs(home - eject) >= 20 and self.link.open:
            self.link.send(f"CAL {self.channel()} {home} {eject}")

    def _go_position(self, key: str, label: str) -> None:
        servo = self.servo_cfg()
        fixed_key = "home_fixed" if key == "closed_us" else "eject_fixed"
        if not servo.get(fixed_key):
            messagebox.showwarning("Не зафиксировано", f"Сначала зафиксируй положение «{label}».")
            return
        ch = self.channel()
        try:
            self._send(f"LIM {ch} {CAL_MIN_US} {CAL_MAX_US}")
            self._send(f"RATE {ch} {TEST_RATE}")
            self._send(f"POWERMAX {ch} 5000")
            self._send(f"SET {ch} {int(servo[key])}")
            self.status_var.set(f"дорожка {ch + 1}: переход в {label}")
        except Exception as exc:
            self.status_var.set(f"ошибка: {exc}")

    def _go_home(self) -> None:
        self._go_position("closed_us", "HOME")

    def _go_eject(self) -> None:
        self._go_position("eject_us", "выдачу")

    def _test_cycle(self) -> None:
        servo = self.servo_cfg()
        home = int(servo.get("closed_us", DEFAULT_US))
        eject = int(servo.get("eject_us", DEFAULT_US))
        if not (servo.get("home_fixed") and servo.get("eject_fixed") and abs(home - eject) >= 20):
            messagebox.showwarning("Калибровка", "Сначала зафиксируй HOME и выдачу.")
            return
        ch = self.channel()
        try:
            self._send(f"LIM {ch} {CAL_MIN_US} {CAL_MAX_US}")
            self._send(f"RATE {ch} {TEST_RATE}")
            self._send(f"POWERMAX {ch} 6000")
            self._send(f"CYCLE {ch} {home} {eject} 600")
            self.status_var.set(f"дорожка {ch + 1}: полный цикл")
        except Exception as exc:
            self.status_var.set(f"ошибка цикла: {exc}")

    def _stop_all(self) -> None:
        self.jogging = False
        self.jog_channel = None
        if not self.link.open:
            self.status_var.set("STOP")
            return
        try:
            for ch in range(NUM_SERVOS):
                self.link.send(f"STOP {ch}")
                self.link.send(f"DETACH {ch}")
            self.status_var.set("STOP: сигнал снят со всех каналов")
        except Exception as exc:
            self.status_var.set(f"ошибка STOP: {exc}")

    def _tick(self) -> None:
        now = time.time()
        if self.link.open:
            try:
                for line in self.link.lines():
                    match = re.search(r"INFO servo=(\d+).*?pos=(\d+)", line)
                    if match and int(match.group(1)) == self.channel():
                        self.current_us = int(match.group(2))
                        self.position_var.set(f"{self.current_us} мкс")
                    elif line.startswith("SAFE DETACH"):
                        self.jogging = False
                        self.jog_channel = None
                        self.status_var.set("автостоп безопасности — отпусти кнопку")
                    elif line.startswith("DONE"):
                        self.status_var.set("полный цикл завершён")
                if now - self.last_info > 0.12:
                    self.link.send("INFO")
                    self.last_info = now
            except Exception as exc:
                self.status_var.set(f"связь потеряна: {exc}")
                self.link.close()
        self.root.after(45, self._tick)

    def close(self) -> None:
        try:
            self._stop_all()
        finally:
            self.link.close()
            self.root.destroy()


def main() -> None:
    root = tk.Tk()
    Calibrator(root)
    root.mainloop()


if __name__ == "__main__":
    main()
