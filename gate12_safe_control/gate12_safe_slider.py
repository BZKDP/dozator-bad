#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Safe PC controller for one Gate12 gear+rack dispenser servo.

It never sweeps the servo automatically. Connecting to the ESP32 does not move
the servo; movement starts only after "Включить серво", slider/step commands, or
the explicit one-can cycle button.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # UI can still open and show the install hint.
    serial = None


APP_DIR = Path(__file__).resolve().parent
CONFIG_PATH = APP_DIR / "gate12_safe_config.json"
BAUD = 115200

DEFAULTS = {
    "port": "",
    "min_us": 1450,
    "max_us": 1550,
    "current_us": 1500,
    "home_us": 1450,
    "drop_us": 1550,
    "hold_ms": 450,
    "rate_us_s": 300,
}


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def load_config() -> dict:
    if not CONFIG_PATH.exists():
        return DEFAULTS.copy()
    try:
        data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except Exception:
        return DEFAULTS.copy()
    cfg = DEFAULTS.copy()
    cfg.update({k: data[k] for k in cfg.keys() if k in data})
    return cfg


def save_config(cfg: dict) -> None:
    CONFIG_PATH.write_text(
        json.dumps(cfg, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def list_ports() -> list[str]:
    if serial is None:
        return []
    ports = list(serial.tools.list_ports.comports())
    ordered = []
    for p in ports:
        text = f"{p.device}  {p.description or ''}".lower()
        score = 0
        if any(k in text for k in ("cp210", "ch340", "ch910", "silicon", "usb-serial", "uart", "esp32")):
            score -= 10
        ordered.append((score, p.device))
    return [p for _, p in sorted(ordered)]


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
        payload = (line.strip() + "\n").encode("ascii", errors="ignore")
        self.ser.write(payload)
        self.ser.flush()

    def read_lines(self) -> list[str]:
        if not self.is_open:
            return []
        out = []
        while True:
            try:
                raw = self.ser.readline()
            except Exception:
                break
            if not raw:
                break
            out.append(raw.decode("utf-8", errors="replace").strip())
        return out


class App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.cfg = load_config()
        self.link = SerialLink()
        self.armed = False
        self.pending_slider_job = None

        self.root.title("Gate12: безопасный ползунок MG90S")
        self.root.geometry("620x560")
        self.root.minsize(580, 520)
        self.root.configure(bg="#101418")

        self.port_var = tk.StringVar(value=self.cfg["port"])
        self.status_var = tk.StringVar(value="не подключено")
        self.pos_var = tk.IntVar(value=int(self.cfg["current_us"]))
        self.min_var = tk.IntVar(value=int(self.cfg["min_us"]))
        self.max_var = tk.IntVar(value=int(self.cfg["max_us"]))
        self.home_var = tk.IntVar(value=int(self.cfg["home_us"]))
        self.drop_var = tk.IntVar(value=int(self.cfg["drop_us"]))
        self.hold_var = tk.IntVar(value=int(self.cfg["hold_ms"]))
        self.rate_var = tk.IntVar(value=int(self.cfg["rate_us_s"]))
        self.live_var = tk.BooleanVar(value=True)

        self._build_ui()
        self._refresh_ports()
        self._apply_scale_range()
        self._poll_serial()

    def _build_ui(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#101418")
        style.configure("Panel.TFrame", background="#171d23")
        style.configure("TLabel", background="#101418", foreground="#e8edf2", font=("Segoe UI", 10))
        style.configure("Muted.TLabel", background="#101418", foreground="#8c98a4", font=("Segoe UI", 9))
        style.configure("Panel.TLabel", background="#171d23", foreground="#e8edf2", font=("Segoe UI", 10))
        style.configure("TButton", font=("Segoe UI", 10), padding=7)
        style.configure("Danger.TButton", font=("Segoe UI", 10, "bold"), padding=7)

        root = ttk.Frame(self.root, padding=16)
        root.pack(fill="both", expand=True)

        ttk.Label(root, text="Gate12: MG90S через ESP32", font=("Segoe UI", 15, "bold")).pack(anchor="w")
        ttk.Label(root, textvariable=self.status_var, style="Muted.TLabel").pack(anchor="w", pady=(2, 12))

        top = ttk.Frame(root, style="Panel.TFrame", padding=10)
        top.pack(fill="x", pady=(0, 12))
        ttk.Label(top, text="COM-порт", style="Panel.TLabel").grid(row=0, column=0, sticky="w")
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, values=[], width=18)
        self.port_box.grid(row=0, column=1, sticky="ew", padx=8)
        ttk.Button(top, text="Обновить", command=self._refresh_ports).grid(row=0, column=2, padx=(0, 8))
        ttk.Button(top, text="Подключить", command=self._connect).grid(row=0, column=3)
        top.columnconfigure(1, weight=1)

        limits = ttk.Frame(root, style="Panel.TFrame", padding=10)
        limits.pack(fill="x", pady=(0, 12))
        ttk.Label(limits, text="Лимит слева, мкс", style="Panel.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Entry(limits, textvariable=self.min_var, width=8).grid(row=0, column=1, padx=(6, 16))
        ttk.Label(limits, text="Лимит справа, мкс", style="Panel.TLabel").grid(row=0, column=2, sticky="w")
        ttk.Entry(limits, textvariable=self.max_var, width=8).grid(row=0, column=3, padx=(6, 16))
        ttk.Label(limits, text="Скорость", style="Panel.TLabel").grid(row=0, column=4, sticky="w")
        ttk.Entry(limits, textvariable=self.rate_var, width=8).grid(row=0, column=5, padx=(6, 16))
        ttk.Button(limits, text="Применить", command=self._apply_limits).grid(row=0, column=6, sticky="e")
        limits.columnconfigure(6, weight=1)

        slider_panel = ttk.Frame(root, style="Panel.TFrame", padding=10)
        slider_panel.pack(fill="x", pady=(0, 12))
        head = ttk.Frame(slider_panel, style="Panel.TFrame")
        head.pack(fill="x")
        ttk.Label(head, text="Положение, мкс", style="Panel.TLabel", font=("Segoe UI", 12, "bold")).pack(side="left")
        self.value_label = ttk.Label(head, text=str(self.pos_var.get()), style="Panel.TLabel", font=("Segoe UI", 12, "bold"))
        self.value_label.pack(side="right")

        self.scale = tk.Scale(
            slider_panel,
            from_=self.min_var.get(),
            to=self.max_var.get(),
            orient="horizontal",
            resolution=1,
            showvalue=False,
            variable=self.pos_var,
            command=self._slider_changed,
            length=520,
            bg="#171d23",
            fg="#e8edf2",
            troughcolor="#303842",
            highlightthickness=0,
            activebackground="#78b7ff",
        )
        self.scale.pack(fill="x", pady=(8, 8))

        step_row = ttk.Frame(slider_panel, style="Panel.TFrame")
        step_row.pack(fill="x")
        for text, delta in (("-50", -50), ("-10", -10), ("-2", -2), ("+2", 2), ("+10", 10), ("+50", 50)):
            ttk.Button(step_row, text=text, command=lambda d=delta: self._step(d)).pack(side="left", padx=(0, 6))
        ttk.Checkbutton(step_row, text="ползунок сразу шлёт", variable=self.live_var).pack(side="right")

        action_row = ttk.Frame(root, style="Panel.TFrame", padding=10)
        action_row.pack(fill="x", pady=(0, 12))
        ttk.Button(action_row, text="Включить серво на этом положении", command=self._arm).pack(side="left", padx=(0, 8))
        ttk.Button(action_row, text="Стоп", command=self._stop, style="Danger.TButton").pack(side="left", padx=(0, 8))
        ttk.Button(action_row, text="Отключить сигнал", command=self._detach).pack(side="left")

        cycle = ttk.Frame(root, style="Panel.TFrame", padding=10)
        cycle.pack(fill="x", pady=(0, 12))
        ttk.Label(cycle, text="Покой", style="Panel.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Entry(cycle, textvariable=self.home_var, width=8).grid(row=0, column=1, padx=(6, 8))
        ttk.Button(cycle, text="Покой = текущее", command=self._save_home).grid(row=0, column=2, padx=(0, 16))
        ttk.Label(cycle, text="Выдача", style="Panel.TLabel").grid(row=0, column=3, sticky="w")
        ttk.Entry(cycle, textvariable=self.drop_var, width=8).grid(row=0, column=4, padx=(6, 8))
        ttk.Button(cycle, text="Выдача = текущее", command=self._save_drop).grid(row=0, column=5)
        ttk.Button(cycle, text="Поменять", command=self._swap_points).grid(row=1, column=0, pady=(10, 0), sticky="w")
        ttk.Label(cycle, text="Пауза, мс", style="Panel.TLabel").grid(row=1, column=1, pady=(10, 0), sticky="e")
        ttk.Entry(cycle, textvariable=self.hold_var, width=8).grid(row=1, column=2, pady=(10, 0), sticky="w")
        ttk.Button(cycle, text="Выдать 1 банку", command=self._dispense_one).grid(row=1, column=5, pady=(10, 0), sticky="e")
        cycle.columnconfigure(5, weight=1)

        self.log = tk.Text(root, height=7, bg="#0b0f13", fg="#d7dde4", insertbackground="#d7dde4",
                           relief="flat", font=("Consolas", 9), wrap="word")
        self.log.pack(fill="both", expand=True)
        self._log("Подключение само не двигает серво. Движение только после включения или кнопок.")

    def _refresh_ports(self) -> None:
        ports = list_ports()
        self.port_box["values"] = ports
        if not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    def _connect(self) -> None:
        port = self.port_var.get().split()[0].strip()
        if not port:
            messagebox.showwarning("COM-порт", "Не выбран COM-порт ESP32.")
            return
        try:
            self.link.open(port)
        except Exception as exc:
            self.status_var.set(f"ошибка подключения: {exc}")
            self._log(f"ERR connect: {exc}")
            return

        self.cfg["port"] = port
        save_config(self.cfg)
        self.status_var.set(f"подключено: {port}; серво пока не включено")
        self._send(f"LIM {self.min_var.get()} {self.max_var.get()}")
        self._send(f"RATE {self.rate_var.get()}")
        self._send("PING")

    def _apply_scale_range(self) -> bool:
        try:
            lo = int(self.min_var.get())
            hi = int(self.max_var.get())
        except Exception:
            messagebox.showwarning("Лимиты", "Лимиты должны быть числами.")
            return False
        if lo < 700 or hi > 2300 or hi - lo < 20:
            messagebox.showwarning("Лимиты", "Диапазон должен быть внутри 700..2300 мкс и шире 20 мкс.")
            return False
        pos = clamp(int(self.pos_var.get()), lo, hi)
        self.scale.configure(from_=lo, to=hi)
        self.pos_var.set(pos)
        self.value_label.configure(text=str(pos))
        return True

    def _apply_limits(self) -> None:
        if not self._apply_scale_range():
            return
        self.cfg.update(
            min_us=int(self.min_var.get()),
            max_us=int(self.max_var.get()),
            current_us=int(self.pos_var.get()),
            rate_us_s=int(self.rate_var.get()),
        )
        save_config(self.cfg)
        self._send(f"LIM {self.min_var.get()} {self.max_var.get()}")
        self._send(f"RATE {self.rate_var.get()}")

    def _slider_changed(self, value: str) -> None:
        pos = int(float(value))
        self.value_label.configure(text=str(pos))
        self.cfg["current_us"] = pos
        save_config(self.cfg)
        if not (self.armed and self.live_var.get() and self.link.is_open):
            return
        if self.pending_slider_job is not None:
            self.root.after_cancel(self.pending_slider_job)
        self.pending_slider_job = self.root.after(45, lambda: self._send_current_pos())

    def _send_current_pos(self) -> None:
        self.pending_slider_job = None
        self._send(f"SET {self.pos_var.get()}")

    def _step(self, delta: int) -> None:
        lo, hi = int(self.min_var.get()), int(self.max_var.get())
        self.pos_var.set(clamp(int(self.pos_var.get()) + delta, lo, hi))
        self._slider_changed(str(self.pos_var.get()))

    def _arm(self) -> None:
        if not self.link.is_open:
            self._connect()
            if not self.link.is_open:
                return
        self.armed = True
        self.status_var.set(f"серво включено; позиция {self.pos_var.get()} мкс")
        self._send(f"SET {self.pos_var.get()}")

    def _stop(self) -> None:
        self.armed = False
        self._send("STOP")
        self.status_var.set("стоп; ползунок временно не шлёт команды")

    def _detach(self) -> None:
        self.armed = False
        self._send("DETACH")
        self.status_var.set("сигнал серво отключен")

    def _save_home(self) -> None:
        self.home_var.set(int(self.pos_var.get()))
        self.cfg["home_us"] = int(self.home_var.get())
        save_config(self.cfg)

    def _save_drop(self) -> None:
        self.drop_var.set(int(self.pos_var.get()))
        self.cfg["drop_us"] = int(self.drop_var.get())
        save_config(self.cfg)

    def _swap_points(self) -> None:
        home = int(self.home_var.get())
        drop = int(self.drop_var.get())
        self.home_var.set(drop)
        self.drop_var.set(home)
        self.cfg["home_us"] = int(self.home_var.get())
        self.cfg["drop_us"] = int(self.drop_var.get())
        save_config(self.cfg)

    def _dispense_one(self) -> None:
        if not self.link.is_open:
            self._connect()
            if not self.link.is_open:
                return
        self.armed = True
        home = clamp(int(self.home_var.get()), int(self.min_var.get()), int(self.max_var.get()))
        drop = clamp(int(self.drop_var.get()), int(self.min_var.get()), int(self.max_var.get()))
        hold = clamp(int(self.hold_var.get()), 50, 5000)
        if abs(home - drop) < 10:
            messagebox.showwarning("Ход", "Точки покоя и выдачи почти одинаковые.")
            return
        self.cfg.update(home_us=home, drop_us=drop, hold_ms=hold)
        save_config(self.cfg)
        self._send(f"CYCLE {home} {drop} {hold}")

    def _send(self, line: str) -> None:
        if not self.link.is_open:
            self._log(f"не отправлено, нет связи: {line}")
            return
        try:
            self.link.write(line)
            self._log(f"> {line}")
        except Exception as exc:
            self.status_var.set(f"ошибка serial: {exc}")
            self._log(f"ERR serial: {exc}")

    def _poll_serial(self) -> None:
        for line in self.link.read_lines():
            if line:
                self._log(f"< {line}")
        self.root.after(80, self._poll_serial)

    def _log(self, line: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.log.insert("end", f"{ts} {line}\n")
        self.log.see("end")

    def close(self) -> None:
        self.link.close()


def main() -> None:
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.close(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
