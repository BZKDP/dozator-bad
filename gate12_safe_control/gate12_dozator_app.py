#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Operator app for the Gate12 can dispenser."""

from __future__ import annotations

import json
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
CONFIG_PATH = APP_DIR / "gate12_dozator_app.json"
BAUD = 115200

NUM_SERVOS = 10
DEFAULT_CLOSED_US = 1270
DEFAULT_EJECT_US = 1846
DEFAULT_SPEED_US_S = 1000
DEFAULT_COOLDOWN_S = 3.0
DEFAULT_EJECT_HOLD_MS = 600
DEFAULT_POWER_MAX_MS = 3000

DEFAULTS = {
    "port": "COM6",
    "selected_servo": 0,
    "guard_min_us": 700,
    "guard_max_us": 2300,
    "cooldown_s": DEFAULT_COOLDOWN_S,
    "eject_hold_ms": DEFAULT_EJECT_HOLD_MS,
    "power_max_ms": DEFAULT_POWER_MAX_MS,
    "hold_torque": False,
    "servos": [
        {
            "name": f"Дорожка {i + 1}",
            "enabled": i == 0,
            "closed_us": DEFAULT_CLOSED_US,
            "eject_us": DEFAULT_EJECT_US,
            "speed_us_s": DEFAULT_SPEED_US_S,
        }
        for i in range(NUM_SERVOS)
    ],
}


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def load_config() -> dict:
    cfg = json.loads(json.dumps(DEFAULTS, ensure_ascii=False))
    if CONFIG_PATH.exists():
        try:
            data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            cfg.update({k: data[k] for k in cfg if k != "servos" and k in data})
            if isinstance(data.get("servos"), list):
                for idx, saved in enumerate(data["servos"][:NUM_SERVOS]):
                    if isinstance(saved, dict):
                        cfg["servos"][idx].update({k: saved[k] for k in cfg["servos"][idx] if k in saved})
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
        lines = []
        while True:
            raw = self.ser.readline()
            if not raw:
                break
            lines.append(raw.decode("utf-8", errors="replace").strip())
        return lines


class App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.cfg = load_config()
        self.link = SerialLink()
        self.busy_until = 0.0
        self.cooldown_until = 0.0
        self.safety_stop_at = 0.0
        self.last_info_at = 0.0
        self.last_dispense_at = 0.0
        self.dispense_count = 0

        self.port_var = tk.StringVar(value=str(self.cfg["port"]))
        self.status_var = tk.StringVar(value="не подключено")
        self.count_var = tk.StringVar(value="0")
        self.selected_var = tk.IntVar(value=int(self.cfg["selected_servo"]) + 1)
        self.closed_var = tk.IntVar()
        self.eject_var = tk.IntVar()
        self.speed_var = tk.IntVar()
        self.cooldown_var = tk.DoubleVar(value=float(self.cfg["cooldown_s"]))
        self.hold_ms_var = tk.IntVar(value=int(self.cfg["eject_hold_ms"]))
        self.power_max_var = tk.IntVar(value=int(self.cfg.get("power_max_ms", DEFAULT_POWER_MAX_MS)))
        self.guard_min_var = tk.IntVar(value=int(self.cfg["guard_min_us"]))
        self.guard_max_var = tk.IntVar(value=int(self.cfg["guard_max_us"]))
        self.hold_torque_var = tk.BooleanVar(value=bool(self.cfg["hold_torque"]))

        self._build_ui()
        self._refresh_ports()
        self._load_selected_servo()
        self._tick()
        self.root.after(300, self._try_autoconnect)

    def _build_ui(self) -> None:
        self.root.title("Дозатор банок")
        self.root.geometry("760x660")
        self.root.minsize(700, 600)
        self.root.configure(bg="#0f1419")

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#0f1419")
        style.configure("Panel.TFrame", background="#171d23")
        style.configure("TLabel", background="#0f1419", foreground="#eef4fb", font=("Segoe UI", 11))
        style.configure("Muted.TLabel", background="#0f1419", foreground="#95a3b3", font=("Segoe UI", 10))
        style.configure("Panel.TLabel", background="#171d23", foreground="#eef4fb", font=("Segoe UI", 10))
        style.configure("TButton", font=("Segoe UI", 11), padding=9)
        style.configure("Big.TButton", font=("Segoe UI", 24, "bold"), padding=30)
        style.configure("Stop.TButton", font=("Segoe UI", 15, "bold"), padding=14)

        page = ttk.Frame(self.root, padding=16)
        page.pack(fill="both", expand=True)

        top = ttk.Frame(page)
        top.pack(fill="x")
        ttk.Label(top, text="Дозатор банок", font=("Segoe UI", 20, "bold")).pack(side="left")
        ttk.Label(top, textvariable=self.status_var, style="Muted.TLabel").pack(side="right")

        main = ttk.Frame(page, style="Panel.TFrame", padding=16)
        main.pack(fill="x", pady=(14, 12))

        self.dispense_btn = ttk.Button(main, text="ВЫДАТЬ 1 БАНКУ\nSpace", style="Big.TButton", command=self._dispense)
        self.dispense_btn.pack(fill="x", ipady=10)

        row = ttk.Frame(main, style="Panel.TFrame")
        row.pack(fill="x", pady=(14, 0))
        ttk.Button(row, text="СТОП / снять усилие", style="Stop.TButton", command=self._emergency_stop).pack(side="left", fill="x", expand=True, padx=(0, 8))
        ttk.Button(row, text="В закрыто", command=self._go_closed).pack(side="left", fill="x", expand=True, padx=(8, 0))

        stats = ttk.Frame(page, style="Panel.TFrame", padding=12)
        stats.pack(fill="x", pady=(0, 12))
        ttk.Label(stats, text="Канал", style="Panel.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(stats, from_=1, to=NUM_SERVOS, textvariable=self.selected_var, width=5, command=self._selected_changed).grid(row=0, column=1, sticky="w", padx=(8, 20))
        ttk.Label(stats, text="Выдано", style="Panel.TLabel").grid(row=0, column=2, sticky="w")
        ttk.Label(stats, textvariable=self.count_var, style="Panel.TLabel", font=("Segoe UI", 15, "bold")).grid(row=0, column=3, sticky="w", padx=(8, 20))
        ttk.Button(stats, text="Подключить", command=self._connect).grid(row=0, column=4, sticky="e")
        stats.columnconfigure(5, weight=1)

        settings = ttk.Frame(page, style="Panel.TFrame", padding=12)
        settings.pack(fill="x")
        ttk.Label(settings, text="Порт", style="Panel.TLabel").grid(row=0, column=0, sticky="w")
        self.port_box = ttk.Combobox(settings, textvariable=self.port_var, values=[], width=12)
        self.port_box.grid(row=0, column=1, padx=(8, 20), sticky="w")
        ttk.Button(settings, text="Обновить", command=self._refresh_ports).grid(row=0, column=2, sticky="w")

        ttk.Label(settings, text="Закрыто", style="Panel.TLabel").grid(row=1, column=0, sticky="w", pady=(12, 0))
        ttk.Entry(settings, textvariable=self.closed_var, width=9, justify="center").grid(row=1, column=1, sticky="w", padx=(8, 20), pady=(12, 0))
        ttk.Label(settings, text="Выдача", style="Panel.TLabel").grid(row=1, column=2, sticky="w", pady=(12, 0))
        ttk.Entry(settings, textvariable=self.eject_var, width=9, justify="center").grid(row=1, column=3, sticky="w", padx=(8, 20), pady=(12, 0))
        ttk.Label(settings, text="Скорость", style="Panel.TLabel").grid(row=1, column=4, sticky="w", pady=(12, 0))
        ttk.Entry(settings, textvariable=self.speed_var, width=9, justify="center").grid(row=1, column=5, sticky="w", padx=(8, 0), pady=(12, 0))

        ttk.Label(settings, text="Cooldown, сек", style="Panel.TLabel").grid(row=2, column=0, sticky="w", pady=(12, 0))
        ttk.Entry(settings, textvariable=self.cooldown_var, width=9, justify="center").grid(row=2, column=1, sticky="w", padx=(8, 20), pady=(12, 0))
        ttk.Label(settings, text="Пауза выдачи, мс", style="Panel.TLabel").grid(row=2, column=2, sticky="w", pady=(12, 0))
        ttk.Entry(settings, textvariable=self.hold_ms_var, width=9, justify="center").grid(row=2, column=3, sticky="w", padx=(8, 20), pady=(12, 0))
        ttk.Checkbutton(settings, text="держать усилие после цикла (греет)", variable=self.hold_torque_var, command=self._save_from_ui).grid(row=2, column=4, columnspan=2, sticky="w", pady=(12, 0))

        ttk.Label(settings, text="Автостоп, мс", style="Panel.TLabel").grid(row=3, column=0, sticky="w", pady=(12, 0))
        ttk.Entry(settings, textvariable=self.power_max_var, width=9, justify="center").grid(row=3, column=1, sticky="w", padx=(8, 20), pady=(12, 0))
        ttk.Label(settings, text="Защита мин", style="Panel.TLabel").grid(row=3, column=2, sticky="w", pady=(12, 0))
        ttk.Entry(settings, textvariable=self.guard_min_var, width=9, justify="center").grid(row=3, column=3, sticky="w", padx=(8, 20), pady=(12, 0))
        ttk.Label(settings, text="Защита макс", style="Panel.TLabel").grid(row=3, column=4, sticky="w", pady=(12, 0))
        ttk.Entry(settings, textvariable=self.guard_max_var, width=9, justify="center").grid(row=3, column=5, sticky="w", padx=(8, 0), pady=(12, 0))
        ttk.Button(settings, text="Сохранить настройки", command=self._save_from_ui).grid(row=4, column=4, columnspan=2, sticky="ew", pady=(12, 0))

        hint = ttk.Label(
            page,
            text="Space: выдать одну банку. Esc: стоп. Повторное нажатие блокируется cooldown-ом и расчётным временем цикла.",
            style="Muted.TLabel",
        )
        hint.pack(anchor="w", pady=(12, 0))

        self.root.bind("<space>", lambda _event: self._dispense())
        self.root.bind("<Escape>", lambda _event: self._emergency_stop())
        self.root.bind("<Return>", lambda _event: self._dispense())

    def _refresh_ports(self) -> None:
        ports = list_ports()
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _try_autoconnect(self) -> None:
        if not self.link.is_open and self.port_var.get():
            self._connect(show_errors=False)

    def _connect(self, show_errors: bool = True) -> bool:
        port = self.port_var.get().strip()
        if not port:
            if show_errors:
                messagebox.showwarning("Порт", "Выбери COM-порт ESP32.")
            return False
        try:
            self.link.open(port)
        except Exception as exc:
            self.status_var.set(f"нет связи: {exc}")
            if show_errors:
                messagebox.showwarning("Порт", f"Не подключился к {port}: {exc}")
            return False
        self.status_var.set(f"подключено: {port}")
        self._apply_channel_settings()
        self._send("INFO")
        self._save_from_ui()
        return True

    def _send(self, line: str) -> None:
        if not self.link.is_open:
            raise RuntimeError("COM-порт не подключен")
        self.link.write(line)

    def _selected_index(self) -> int:
        return clamp(int(self.selected_var.get()) - 1, 0, NUM_SERVOS - 1)

    def _load_selected_servo(self) -> None:
        servo = self.cfg["servos"][self._selected_index()]
        self.closed_var.set(int(servo["closed_us"]))
        self.eject_var.set(int(servo["eject_us"]))
        self.speed_var.set(int(servo["speed_us_s"]))

    def _selected_changed(self) -> None:
        self._save_from_ui()
        self._load_selected_servo()
        if self.link.is_open:
            self._apply_channel_settings()

    def _save_from_ui(self) -> None:
        idx = self._selected_index()
        servo = self.cfg["servos"][idx]
        servo["closed_us"] = int(self.closed_var.get())
        servo["eject_us"] = int(self.eject_var.get())
        servo["speed_us_s"] = int(self.speed_var.get())
        self.cfg["selected_servo"] = idx
        self.cfg["port"] = self.port_var.get().strip()
        self.cfg["guard_min_us"] = int(self.guard_min_var.get())
        self.cfg["guard_max_us"] = int(self.guard_max_var.get())
        self.cfg["cooldown_s"] = float(self.cooldown_var.get())
        self.cfg["eject_hold_ms"] = int(self.hold_ms_var.get())
        self.cfg["power_max_ms"] = int(self.power_max_var.get())
        self.cfg["hold_torque"] = bool(self.hold_torque_var.get())
        save_config(self.cfg)

    def _apply_channel_settings(self) -> None:
        idx = self._selected_index()
        self._save_from_ui()
        lo = int(self.guard_min_var.get())
        hi = int(self.guard_max_var.get())
        if lo >= hi:
            lo, hi = 700, 2300
        speed = clamp(int(self.speed_var.get()), 5, 2000)
        power_max = clamp(int(self.power_max_var.get()), 0, 600000)
        hold = 1 if self.hold_torque_var.get() else 0
        self._send(f"CH {idx}")
        self._send(f"LIM {idx} {lo} {hi}")
        self._send(f"RATE {idx} {speed}")
        self._send(f"HOLD {idx} {hold}")
        self._send(f"POWERMAX {idx} {power_max}")

    def _cycle_seconds(self) -> float:
        speed = max(5, int(self.speed_var.get()))
        distance = abs(int(self.eject_var.get()) - int(self.closed_var.get()))
        return (distance * 2 / speed) + int(self.hold_ms_var.get()) / 1000 + 0.7

    def _dispense(self) -> None:
        now = time.time()
        if now < self.busy_until or now < self.cooldown_until:
            return
        if not self.link.is_open and not self._connect(show_errors=False):
            self.status_var.set("нет связи с ESP32")
            return

        try:
            self._apply_channel_settings()
            idx = self._selected_index()
            closed = int(self.closed_var.get())
            eject = int(self.eject_var.get())
            hold_ms = int(self.hold_ms_var.get())
            if abs(eject - closed) < 20:
                messagebox.showwarning("Настройки", "Позиции закрыто и выдача слишком близко.")
                return
            self._send(f"CYCLE {idx} {closed} {eject} {hold_ms}")
        except Exception as exc:
            self.status_var.set(f"ошибка: {exc}")
            return

        self.last_dispense_at = now
        self.busy_until = now + self._cycle_seconds()
        power_max_s = max(0, int(self.power_max_var.get())) / 1000.0
        self.cooldown_until = now + max(float(self.cooldown_var.get()), DEFAULT_COOLDOWN_S, power_max_s)
        self.safety_stop_at = now + power_max_s if power_max_s > 0 else 0.0
        self.dispense_count += 1
        self.count_var.set(str(self.dispense_count))
        self.status_var.set("выдача...")

    def _go_closed(self) -> None:
        if not self.link.is_open and not self._connect(show_errors=False):
            self.status_var.set("нет связи с ESP32")
            return
        try:
            self._apply_channel_settings()
            idx = self._selected_index()
            self._send(f"SET {idx} {int(self.closed_var.get())}")
            self.status_var.set("идёт в закрыто")
        except Exception as exc:
            self.status_var.set(f"ошибка: {exc}")

    def _emergency_stop(self) -> None:
        if not self.link.is_open:
            self.status_var.set("стоп")
            return
        try:
            idx = self._selected_index()
            self._send(f"STOP {idx}")
            self._send(f"DETACH {idx}")
        except Exception as exc:
            self.status_var.set(f"ошибка стопа: {exc}")
            return
        self.busy_until = 0
        self.cooldown_until = time.time() + 1.0
        self.safety_stop_at = 0.0
        self.status_var.set("стоп; сигнал снят")

    def _tick(self) -> None:
        now = time.time()
        if self.link.is_open:
            try:
                for line in self.link.read_lines():
                    if line.startswith("DONE"):
                        self.status_var.set("готово")
                    elif line.startswith("INFO") and now >= self.busy_until:
                        self.status_var.set("готово")
            except Exception as exc:
                self.status_var.set(f"связь потеряна: {exc}")
                self.link.close()

            if self.safety_stop_at and now >= self.safety_stop_at:
                try:
                    idx = self._selected_index()
                    self._send(f"STOP {idx}")
                    self._send(f"DETACH {idx}")
                    self.status_var.set("автостоп: сигнал снят")
                except Exception as exc:
                    self.status_var.set(f"ошибка автостопа: {exc}")
                self.safety_stop_at = 0.0

            if now - self.last_info_at > 0.5:
                try:
                    self._send("INFO")
                except Exception:
                    pass
                self.last_info_at = now

        locked = now < self.busy_until or now < self.cooldown_until
        self.dispense_btn.state(["disabled"] if locked else ["!disabled"])
        self.root.after(80, self._tick)

    def close(self) -> None:
        try:
            if self.link.is_open:
                idx = self._selected_index()
                self._send(f"STOP {idx}")
                self._send(f"DETACH {idx}")
        except Exception:
            pass
        self.link.close()


def main() -> None:
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.close(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
