#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Integrated five-lane dispenser dashboard and guided calibrator."""

from __future__ import annotations

import ctypes
import os
import re
import time
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

from gate12_direct5_calibrate import (
    CAL_MAX_US,
    CAL_MIN_US,
    DEFAULT_EJECT_US,
    DEFAULT_HOME_US,
    DEFAULT_US,
    NUM_SERVOS,
    PINS,
    SerialLink,
    available_ports,
    load_config,
    save_config,
)


INFO_RE = re.compile(r"INFO servo=(\d+).*?pos=(\d+)")
DONE_RE = re.compile(r"DONE\s+(\d+)(?:\s+token=(\d+))?")
SAFE_DETACH_RE = re.compile(r"SAFE DETACH\s+(\d+)")
ERROR_RE = re.compile(r"ERR(?:\s+(.*))?$")
TEST_RATE = 500
CAL_RATE = 120
SERVO_MIN_US = 500
SERVO_MAX_US = 2400
GROUP_PAUSE_S = 0.45
RUNTIME_LOG = Path(__file__).resolve().with_name("gate12_runtime.log")
_INSTANCE_MUTEX = None


def acquire_single_instance() -> bool:
    """Prevent two operator panels from fighting over the same COM port."""
    global _INSTANCE_MUTEX
    if os.name != "nt":
        return True
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    kernel32.CreateMutexW.argtypes = [ctypes.c_void_p, ctypes.c_bool, ctypes.c_wchar_p]
    kernel32.CreateMutexW.restype = ctypes.c_void_p
    kernel32.CloseHandle.argtypes = [ctypes.c_void_p]
    kernel32.CloseHandle.restype = ctypes.c_bool
    ctypes.set_last_error(0)
    handle = kernel32.CreateMutexW(None, False, "Local\\Gate12Direct5OperatorPanel")
    if not handle:
        return True
    if ctypes.get_last_error() == 183:  # ERROR_ALREADY_EXISTS
        kernel32.CloseHandle(ctypes.c_void_p(handle))
        return False
    _INSTANCE_MUTEX = (kernel32, handle)
    return True


def release_single_instance() -> None:
    global _INSTANCE_MUTEX
    if _INSTANCE_MUTEX is None:
        return
    kernel32, handle = _INSTANCE_MUTEX
    kernel32.CloseHandle(ctypes.c_void_p(handle))
    _INSTANCE_MUTEX = None


def us_to_degrees(us: int) -> int:
    """Show PWM positions on the standard 0..180 degree servo scale."""
    value = round((int(us) - SERVO_MIN_US) * 180 / (SERVO_MAX_US - SERVO_MIN_US))
    return max(0, min(180, value))


def degrees_to_us(degrees: int) -> int:
    angle = max(0, min(180, int(degrees)))
    return round(SERVO_MIN_US + angle * (SERVO_MAX_US - SERVO_MIN_US) / 180)


CAL_MIN_DEG = us_to_degrees(CAL_MIN_US)
CAL_MAX_DEG = us_to_degrees(CAL_MAX_US)


def calibrated(servo: dict) -> bool:
    return bool(
        servo.get("home_fixed")
        and servo.get("eject_fixed")
        and abs(int(servo.get("closed_us", DEFAULT_US)) - int(servo.get("eject_us", DEFAULT_US))) >= 20
    )


class FullApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.cfg = load_config()
        self.link = SerialLink()
        self.positions = [int(s.get("closed_us", DEFAULT_US)) for s in self.cfg["servos"]]
        self.name_vars: list[tk.StringVar] = []
        self.state_vars: list[tk.StringVar] = []
        self.value_vars: list[tk.StringVar] = []
        self.count_vars: list[tk.StringVar] = []
        self.speed_vars: list[tk.IntVar] = []
        self.speed_text_vars: list[tk.StringVar] = []
        self.speed_scales: list[tk.Scale] = []
        self.selected_vars: list[tk.BooleanVar] = []
        self.selection_checkbuttons: list[tk.Checkbutton] = []
        self.dispense_buttons: list[tk.Button] = []
        self.home_buttons: list[tk.Button] = []
        self.calibrate_buttons: list[tk.Button] = []
        self.calibration_dialog: CalibrationDialog | None = None
        self.calibrating = False
        self.pending_channels: set[int] = set()
        self.busy_deadlines: dict[int, float] = {}
        self.pending_tokens: dict[int, int | None] = {}
        self.next_cycle_token = 1
        self.group_queue: list[int] = []
        self.group_total = 0
        self.group_done = 0
        self.next_group_at = 0.0
        self.calibration_test_active = False
        self.last_info = 0.0
        self.speed_save_job: str | None = None

        self.port_var = tk.StringVar(value=str(self.cfg.get("port", "COM6")))
        self.connection_var = tk.StringVar(value="подключение…")
        self.activity_var = tk.StringVar(value="Система готовится")

        self._build()
        self._refresh_ports()
        self._refresh_cards()
        self.root.after(250, self._autoconnect)
        self.root.after(60, self._tick)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build(self) -> None:
        self.root.title("Дозатор БАДов — 5 дорожек")
        self.root.geometry("1500x820")
        self.root.minsize(1320, 730)
        self.root.configure(bg="#0d141b")

        header = tk.Frame(self.root, bg="#0d141b", padx=20, pady=16)
        header.pack(fill="x")
        tk.Label(header, text="ДОЗАТОР БАДОВ", bg="#0d141b", fg="#f3f7fb", font=("Segoe UI", 25, "bold")).pack(side="left")
        tk.Label(header, textvariable=self.connection_var, bg="#0d141b", fg="#91a5b8", font=("Segoe UI", 11)).pack(side="right")

        toolbar = tk.Frame(self.root, bg="#17222d", padx=16, pady=12)
        toolbar.pack(fill="x", padx=18)
        tk.Label(toolbar, text="COM-порт", bg="#17222d", fg="white", font=("Segoe UI", 11)).pack(side="left")
        self.port_box = ttk.Combobox(toolbar, textvariable=self.port_var, width=10, state="readonly")
        self.port_box.pack(side="left", padx=(8, 8))
        ttk.Button(toolbar, text="Подключить", command=self._connect).pack(side="left")
        ttk.Button(toolbar, text="Сохранить названия", command=self._save_names).pack(side="left", padx=(16, 8))
        self.dispense_selected_button = tk.Button(
            toolbar,
            text="ВЫБЕРИ ДОРОЖКИ",
            command=self.dispense_selected,
            bg="#46535f",
            fg="#bec9d2",
            disabledforeground="#8795a1",
            activebackground="#0f6936",
            activeforeground="white",
            relief="flat",
            padx=18,
            pady=9,
            font=("Segoe UI", 11, "bold"),
            state="disabled",
        )
        self.dispense_selected_button.pack(side="left", padx=(8, 8))
        tk.Button(
            toolbar,
            text="СТОП ВСЁ / СНЯТЬ СИГНАЛ",
            command=self.stop_all,
            bg="#b51d25",
            fg="white",
            activebackground="#8d1118",
            activeforeground="white",
            relief="flat",
            padx=18,
            pady=9,
            font=("Segoe UI", 11, "bold"),
        ).pack(side="right")

        tk.Label(
            self.root,
            text="Одна банка — зелёная кнопка в строке. Несколько за одно нажатие — отметь чекбоксы; автоматика надёжно выдаст их по очереди.",
            bg="#0d141b",
            fg="#9fb0bf",
            font=("Segoe UI", 11),
        ).pack(anchor="w", padx=22, pady=(12, 7))

        cards = tk.Frame(self.root, bg="#0d141b", padx=18)
        cards.pack(fill="both", expand=True)

        for ch in range(NUM_SERVOS):
            card = tk.Frame(cards, bg="#18232e", highlightthickness=1, highlightbackground="#2a3947", padx=12, pady=9)
            card.pack(fill="x", pady=5)
            card.grid_columnconfigure(1, weight=1)

            tk.Label(card, text=str(ch + 1), bg="#1169c9", fg="white", width=3, font=("Segoe UI", 20, "bold")).grid(row=0, column=0, rowspan=3, sticky="ns", padx=(0, 12))

            name_var = tk.StringVar(value=str(self.cfg["servos"][ch].get("name", f"БАД {ch + 1}")))
            self.name_vars.append(name_var)
            tk.Label(card, text="Название БАДа", bg="#18232e", fg="#8fa3b5", font=("Segoe UI", 9)).grid(row=0, column=1, sticky="sw")
            entry = tk.Entry(card, textvariable=name_var, bg="#f4f6f8", fg="#17212b", relief="flat", font=("Segoe UI", 13, "bold"), width=22)
            entry.grid(row=1, column=1, sticky="ew", padx=(0, 14))
            entry.bind("<Return>", lambda _e: self._save_names())

            tk.Label(card, text=PINS[ch], bg="#18232e", fg="#69aefa", font=("Consolas", 12, "bold"), width=15).grid(row=0, column=2, rowspan=2, padx=8)

            state_var = tk.StringVar()
            value_var = tk.StringVar()
            count_var = tk.StringVar()
            speed_var = tk.IntVar(value=max(200, min(1200, int(self.cfg["servos"][ch].get("speed_us_s", TEST_RATE)))))
            speed_text_var = tk.StringVar()
            selected_var = tk.BooleanVar(value=False)
            self.state_vars.append(state_var)
            self.value_vars.append(value_var)
            self.count_vars.append(count_var)
            self.speed_vars.append(speed_var)
            self.speed_text_vars.append(speed_text_var)
            self.selected_vars.append(selected_var)
            tk.Label(card, textvariable=state_var, bg="#18232e", fg="#6fd58a", font=("Segoe UI", 11, "bold"), width=22).grid(row=0, column=3)
            tk.Label(card, textvariable=value_var, bg="#18232e", fg="#d8e2eb", font=("Consolas", 10), width=24).grid(row=1, column=3)
            tk.Label(card, textvariable=count_var, bg="#18232e", fg="#f1c75b", font=("Segoe UI", 10, "bold"), width=11).grid(row=0, column=4, rowspan=2)

            select_cb = tk.Checkbutton(
                card,
                text="ВЫБРАТЬ",
                variable=selected_var,
                command=self._selection_changed,
                bg="#18232e",
                fg="#dce6ee",
                activebackground="#18232e",
                activeforeground="white",
                selectcolor="#1169c9",
                disabledforeground="#647482",
                font=("Segoe UI", 9, "bold"),
                padx=4,
            )
            select_cb.grid(row=0, column=5, rowspan=3, padx=4, sticky="nsew")

            calibrate_btn = tk.Button(card, text="НАСТРОЙКА /\nКАЛИБРОВКА", command=lambda c=ch: self.open_calibration(c), bg="#495a68", fg="white", relief="flat", padx=10, pady=12, font=("Segoe UI", 10, "bold"))
            calibrate_btn.grid(row=0, column=6, rowspan=3, padx=5, sticky="nsew")
            home_btn = tk.Button(card, text="В HOME", command=lambda c=ch: self.go_home(c), bg="#334451", fg="white", relief="flat", padx=10, pady=12, font=("Segoe UI", 10, "bold"))
            home_btn.grid(row=0, column=7, rowspan=3, padx=5, sticky="nsew")
            dispense_btn = tk.Button(card, command=lambda c=ch: self.dispense(c), bg="#168847", fg="white", activebackground="#0f6936", activeforeground="white", relief="flat", width=21, padx=12, pady=12, font=("Segoe UI", 11, "bold"))
            dispense_btn.grid(row=0, column=8, rowspan=3, padx=(5, 0), sticky="nsew")

            speed_row = tk.Frame(card, bg="#18232e")
            speed_row.grid(row=2, column=1, columnspan=4, sticky="ew", pady=(7, 0), padx=(0, 8))
            tk.Label(speed_row, textvariable=speed_text_var, bg="#18232e", fg="#dce6ee", font=("Segoe UI", 10, "bold"), width=21, anchor="w").pack(side="left")
            tk.Label(speed_row, text="медленно", bg="#18232e", fg="#8295a6", font=("Segoe UI", 9)).pack(side="left", padx=(4, 5))
            speed_scale = tk.Scale(
                speed_row,
                from_=200,
                to=1200,
                resolution=100,
                orient="horizontal",
                showvalue=False,
                length=280,
                variable=speed_var,
                command=lambda value, c=ch: self._speed_changed(c, value),
                bg="#18232e",
                fg="white",
                troughcolor="#344653",
                activebackground="#69aefa",
                highlightthickness=0,
                bd=0,
            )
            speed_scale.pack(side="left", fill="x", expand=True)
            tk.Label(speed_row, text="быстро", bg="#18232e", fg="#8295a6", font=("Segoe UI", 9)).pack(side="left", padx=(5, 0))
            self.speed_scales.append(speed_scale)
            self.selection_checkbuttons.append(select_cb)
            self.calibrate_buttons.append(calibrate_btn)
            self.home_buttons.append(home_btn)
            self.dispense_buttons.append(dispense_btn)

        footer = tk.Frame(self.root, bg="#111b24", padx=20, pady=12)
        footer.pack(fill="x", padx=18, pady=(8, 16))
        tk.Label(footer, textvariable=self.activity_var, bg="#111b24", fg="#e5edf4", font=("Segoe UI", 12, "bold")).pack(side="left")
        tk.Label(footer, text="Esc — аварийный STOP", bg="#111b24", fg="#93a5b4", font=("Segoe UI", 10)).pack(side="right")
        self.root.bind("<Escape>", lambda _e: self.stop_all())

    def _refresh_ports(self) -> None:
        ports = available_ports()
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _autoconnect(self) -> None:
        self._connect(show_error=False)

    def _connect(self, show_error: bool = True) -> bool:
        try:
            self.link.connect(self.port_var.get())
            self.cfg["port"] = self.port_var.get()
            save_config(self.cfg)
            mode = "защищённый протокол v2" if self.link.supports_cycle_id else "совместимый протокол v1"
            self.connection_var.set(f"подключено: {self.port_var.get()} · {mode}")
            self._log_event(
                f"CONNECTED port={self.port_var.get()} mode={mode} firmware={self.link.firmware_info}"
            )
            self.activity_var.set("Готово. Выбери БАД и нажми «Выдать банку»")
            # Reconnection may happen after USB/PC failed in the middle of a
            # cycle. Establish a known de-energised state before syncing values.
            for ch in range(NUM_SERVOS):
                self.link.send(f"STOP {ch}")
                self.link.send(f"DETACH {ch}")
            self.sync_calibrations()
            self.link.send("INFO ALL")
            return True
        except Exception as exc:
            self.connection_var.set(f"нет связи: {exc}")
            if show_error:
                messagebox.showerror("COM-порт", str(exc))
            return False

    def ensure_connection(self) -> bool:
        return self.link.open or self._connect()

    def sync_calibrations(self) -> None:
        if not self.link.open:
            return
        for ch, servo in enumerate(self.cfg["servos"]):
            if calibrated(servo):
                self.link.send(f"CAL {ch} {int(servo['closed_us'])} {int(servo['eject_us'])}")

    def _save_names(self) -> None:
        for ch, var in enumerate(self.name_vars):
            name = var.get().strip() or f"БАД {ch + 1}"
            self.cfg["servos"][ch]["name"] = name
            self.cfg["servos"][ch]["speed_us_s"] = int(self.speed_vars[ch].get())
            var.set(name)
        save_config(self.cfg)
        self.activity_var.set("Названия сохранены")

    def _speed_changed(self, ch: int, value: str) -> None:
        speed = max(200, min(1200, int(float(value))))
        self.cfg["servos"][ch]["speed_us_s"] = speed
        self.speed_text_vars[ch].set(f"Скорость: {speed}")
        if self.speed_save_job is not None:
            self.root.after_cancel(self.speed_save_job)
        self.speed_save_job = self.root.after(350, self._save_speed_settings)

    def _save_speed_settings(self) -> None:
        self.speed_save_job = None
        for ch, var in enumerate(self.speed_vars):
            self.cfg["servos"][ch]["speed_us_s"] = int(var.get())
        save_config(self.cfg)
        self.activity_var.set("Скорость дорожек сохранена")

    def selected_channels(self) -> list[int]:
        return [ch for ch, variable in enumerate(self.selected_vars) if variable.get()]

    def _selection_changed(self) -> None:
        self._refresh_cards()
        channels = self.selected_channels()
        if channels:
            self.activity_var.set("Выбраны дорожки: " + ", ".join(str(ch + 1) for ch in channels))
        else:
            self.activity_var.set("Отметь чекбоксы дорожек для групповой выдачи")

    def _is_busy(self) -> bool:
        return bool(self.pending_channels) or self.group_total > 0 or self.calibrating

    def _log_event(self, message: str) -> None:
        try:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            with RUNTIME_LOG.open("a", encoding="utf-8") as log_file:
                log_file.write(f"{timestamp} {message}\n")
        except OSError:
            pass

    def _refresh_cards(self) -> None:
        busy = self._is_busy()
        for ch, servo in enumerate(self.cfg["servos"]):
            ready = calibrated(servo)
            if not ready and self.selected_vars[ch].get():
                self.selected_vars[ch].set(False)
            self.state_vars[ch].set("✓ ГОТОВ К РАБОТЕ" if ready else "● НУЖНА КАЛИБРОВКА")
            self.value_vars[ch].set(
                f"HOME {us_to_degrees(int(servo.get('closed_us', DEFAULT_US)))}°  /  "
                f"ВЫДАЧА {us_to_degrees(int(servo.get('eject_us', DEFAULT_US)))}°"
                if ready
                else "HOME —  /  ВЫДАЧА —"
            )
            self.count_vars[ch].set(f"Выдано: {int(servo.get('dispense_count', 0))}")
            speed = max(200, min(1200, int(servo.get("speed_us_s", TEST_RATE))))
            if self.speed_vars[ch].get() != speed:
                self.speed_vars[ch].set(speed)
            self.speed_text_vars[ch].set(f"Скорость: {speed}")
            self.dispense_buttons[ch].configure(
                text="ВЫДАТЬ БАНКУ" if ready else "НАСТРОИТЬ И ЗАПУСТИТЬ",
                bg="#168847" if ready else "#a86b14",
                state="disabled" if busy else "normal",
            )
            self.home_buttons[ch].configure(state="normal" if ready and not busy else "disabled")
            self.calibrate_buttons[ch].configure(state="disabled" if busy else "normal")
            self.speed_scales[ch].configure(state="disabled" if busy else "normal")
            self.selection_checkbuttons[ch].configure(state="normal" if ready and not busy else "disabled")

        selected = self.selected_channels()
        group_ready = bool(selected) and all(calibrated(self.cfg["servos"][ch]) for ch in selected) and not busy
        self.dispense_selected_button.configure(
            text=f"ВЫДАТЬ ВЫБРАННЫЕ: {len(selected)}" if selected else "ВЫБЕРИ ДОРОЖКИ",
            bg="#168847" if group_ready else "#46535f",
            fg="white" if group_ready else "#bec9d2",
            state="normal" if group_ready else "disabled",
        )

    def open_calibration(self, ch: int) -> None:
        if self.pending_channels or self.group_total > 0:
            messagebox.showwarning("Дозатор занят", "Сначала дождись завершения текущей выдачи.")
            return
        if self.calibration_dialog and self.calibration_dialog.window.winfo_exists():
            self.calibration_dialog.window.lift()
            return
        if not self.ensure_connection():
            return
        self._save_names()
        # Entering calibration must never leave another channel energised.
        self.stop_all()
        self.calibrating = True
        self.cfg["selected_servo"] = ch
        save_config(self.cfg)
        self.calibration_dialog = CalibrationDialog(self, ch)
        self._refresh_cards()

    def calibration_closed(self) -> None:
        self.calibrating = False
        self.calibration_dialog = None
        self._refresh_cards()
        self.activity_var.set("Калибровка сохранена. Дорожка готова к работе.")

    def dispense(self, ch: int) -> None:
        servo = self.cfg["servos"][ch]
        if not calibrated(servo):
            if messagebox.askyesno(
                "Нужна калибровка",
                f"Дорожка {ch + 1} ({PINS[ch]}) ещё не настроена.\n\nОткрыть её калибровку сейчас?",
            ):
                self.open_calibration(ch)
            return
        if self._is_busy():
            return
        if not self.ensure_connection():
            return
        self._save_names()
        try:
            self._start_dispense(ch)
            self.activity_var.set(f"Выдача: {servo.get('name', f'Дорожка {ch + 1}')}…")
            self._refresh_cards()
        except Exception as exc:
            self.activity_var.set(f"Ошибка выдачи: {exc}")
            self.stop_all()

    def dispense_selected(self) -> None:
        channels = self.selected_channels()
        if not channels or self._is_busy():
            return
        missing = [ch for ch in channels if not calibrated(self.cfg["servos"][ch])]
        if missing:
            messagebox.showwarning(
                "Нужна калибровка",
                "Сначала откалибруй дорожки: " + ", ".join(str(ch + 1) for ch in missing),
            )
            return
        if not self.ensure_connection():
            return
        self._save_names()
        self.group_queue = list(channels)
        self.group_total = len(channels)
        self.group_done = 0
        self.next_group_at = 0.0
        self._log_event("GROUP START lanes=" + ",".join(str(ch + 1) for ch in channels))
        self._start_next_group()
        self._refresh_cards()

    def _start_next_group(self) -> None:
        if self.group_total <= 0 or self.pending_channels or not self.group_queue:
            return
        ch = self.group_queue.pop(0)
        try:
            self._start_dispense(ch)
            self.activity_var.set(
                f"Групповая выдача {self.group_done + 1}/{self.group_total}: дорожка {ch + 1}…"
            )
        except Exception as exc:
            self._log_event(f"GROUP ERROR lane={ch + 1} error={exc}")
            self.activity_var.set(f"Ошибка групповой выдачи на дорожке {ch + 1}: {exc}")
            self.stop_all()

    def _start_dispense(self, ch: int) -> None:
        servo = self.cfg["servos"][ch]
        home = int(servo["closed_us"])
        eject = int(servo["eject_us"])
        speed = max(100, min(1500, int(servo.get("speed_us_s", TEST_RATE))))
        hold_ms = max(100, min(2500, int(self.cfg.get("eject_hold_ms", 900))))
        guard_min = max(700, int(servo.get("guard_min_us", min(home, eject) - 60)))
        guard_max = min(2300, int(servo.get("guard_max_us", max(home, eject) + 60)))
        duration = abs(eject - home) * 2 / speed + hold_ms / 1000 + 1.3
        power_max = max(4000, min(15000, int(duration * 1000 + 1000)))
        # Calibration is synchronised on connect and whenever the operator saves
        # it. Re-sending CAL before every can used to commit identical values to
        # ESP32 NVS repeatedly, so a normal cycle only sends volatile settings.
        self.link.send(f"LIM {ch} {guard_min} {guard_max}")
        self.link.send(f"RATE {ch} {speed}")
        self.link.send(f"HOLD {ch} 0")
        self.link.send(f"POWERMAX {ch} {power_max}")
        token: int | None = None
        if getattr(self.link, "supports_cycle_id", False):
            token = self.next_cycle_token
            self.next_cycle_token = 1 if token >= 2_000_000_000 else token + 1
            self.link.send(f"CYCLEID {ch} {home} {eject} {hold_ms} {token}")
        else:
            self.link.send(f"CYCLE {ch} {home} {eject} {hold_ms}")
        self.pending_channels.add(ch)
        self.pending_tokens[ch] = token
        self.busy_deadlines[ch] = time.monotonic() + duration
        self._log_event(
            f"CYCLE lane={ch + 1} token={token or 'legacy'} home={home} eject={eject} "
            f"speed={speed} hold_ms={hold_ms}"
        )

    def go_home(self, ch: int) -> None:
        servo = self.cfg["servos"][ch]
        if not calibrated(servo) or self._is_busy():
            return
        if not self.ensure_connection():
            return
        try:
            self.link.send(f"LIM {ch} {int(servo['guard_min_us'])} {int(servo['guard_max_us'])}")
            self.link.send(f"RATE {ch} {TEST_RATE}")
            self.link.send(f"HOLD {ch} 0")
            self.link.send(f"POWERMAX {ch} 5000")
            self.link.send(f"SET {ch} {int(servo['closed_us'])}")
            self.activity_var.set(f"Дорожка {ch + 1}: переход в HOME")
        except Exception as exc:
            self.activity_var.set(f"Ошибка HOME: {exc}")

    def stop_all(self) -> None:
        self.pending_channels.clear()
        self.pending_tokens.clear()
        self.busy_deadlines.clear()
        self.group_queue.clear()
        self.group_total = 0
        self.group_done = 0
        self.next_group_at = 0.0
        self.calibration_test_active = False
        if self.link.open:
            try:
                for ch in range(NUM_SERVOS):
                    self.link.send(f"STOP {ch}")
                    self.link.send(f"DETACH {ch}")
            except Exception:
                pass
        self.activity_var.set("STOP: управляющие сигналы сняты со всех дорожек")
        self._log_event("STOP ALL")
        self._refresh_cards()

    def _tick(self) -> None:
        now = time.monotonic()
        if self.link.open:
            try:
                for line in self.link.lines():
                    info = INFO_RE.search(line)
                    if info:
                        ch = int(info.group(1))
                        if 0 <= ch < NUM_SERVOS:
                            self.positions[ch] = int(info.group(2))
                            if self.calibration_dialog and self.calibration_dialog.channel == ch:
                                self.calibration_dialog.set_position(self.positions[ch])
                    done = DONE_RE.match(line)
                    if done:
                        ch = int(done.group(1))
                        received_token = int(done.group(2)) if done.group(2) else None
                        if ch in self.pending_channels:
                            expected_token = self.pending_tokens.get(ch)
                            if expected_token is not None and received_token != expected_token:
                                self._log_event(
                                    f"STALE DONE lane={ch + 1} expected={expected_token} "
                                    f"received={received_token} ignored"
                                )
                                continue
                            servo = self.cfg["servos"][ch]
                            if not self.calibration_test_active:
                                servo["dispense_count"] = int(servo.get("dispense_count", 0)) + 1
                                save_config(self.cfg)
                            self.pending_channels.discard(ch)
                            self.pending_tokens.pop(ch, None)
                            self.busy_deadlines.pop(ch, None)
                            self._log_event(f"DONE lane={ch + 1}")
                            if self.group_total > 0:
                                self.group_done += 1
                                if self.group_queue:
                                    self.next_group_at = now + GROUP_PAUSE_S
                                    self.activity_var.set(
                                        f"Дорожка {ch + 1} готова. Пауза перед следующей "
                                        f"({self.group_done}/{self.group_total})…"
                                    )
                                else:
                                    total = self.group_total
                                    self.group_total = 0
                                    self.group_done = 0
                                    self.next_group_at = 0.0
                                    if self.calibration_test_active:
                                        self.calibration_test_active = False
                                        self.activity_var.set(f"Калибровочный тест завершён ({total})")
                                        if self.calibration_dialog:
                                            self.calibration_dialog.status_var.set(
                                                f"Проверка завершена: дорожек {total}"
                                            )
                                        self._log_event(f"CALIBRATION TEST DONE count={total}")
                                    else:
                                        self.activity_var.set(f"Готово: выданы все выбранные банки ({total})")
                                        self._log_event(f"GROUP DONE count={total}")
                            elif self.pending_channels:
                                self.activity_var.set(
                                    f"Дорожка {ch + 1} готова; ожидаю: "
                                    + ", ".join(str(item + 1) for item in sorted(self.pending_channels))
                                )
                            else:
                                self.activity_var.set(f"Готово: дорожка {ch + 1} выдала банку")
                            self._refresh_cards()
                    safe_detach = SAFE_DETACH_RE.match(line)
                    if safe_detach:
                        ch = int(safe_detach.group(1))
                        if ch in self.pending_channels:
                            self.pending_channels.discard(ch)
                            self.pending_tokens.pop(ch, None)
                            self.busy_deadlines.pop(ch, None)
                            self.group_queue.clear()
                            self.group_total = 0
                            self.group_done = 0
                            self.next_group_at = 0.0
                            self.calibration_test_active = False
                            self.activity_var.set(f"Автостоп дорожки {ch + 1}: проверь механизм")
                            self._log_event(f"SAFE DETACH lane={ch + 1}; group aborted")
                            self._refresh_cards()
                    error = ERROR_RE.match(line)
                    if error:
                        detail = (error.group(1) or "unknown").strip()
                        active = sorted(self.pending_channels)
                        for active_ch in active:
                            try:
                                self.link.send(f"STOP {active_ch}")
                                self.link.send(f"DETACH {active_ch}")
                            except Exception:
                                pass
                        self.pending_channels.clear()
                        self.pending_tokens.clear()
                        self.busy_deadlines.clear()
                        self.group_queue.clear()
                        self.group_total = 0
                        self.group_done = 0
                        self.next_group_at = 0.0
                        self.calibration_test_active = False
                        self.activity_var.set(f"ESP32 отклонила команду: {detail}")
                        self._log_event(f"ESP32 ERROR detail={detail}; operation aborted")
                        self._refresh_cards()
                if now - self.last_info > 0.35:
                    self.link.send("INFO ALL")
                    self.last_info = now
            except Exception as exc:
                detail = str(exc)
                if "PermissionError" in detail or "Отказано в доступе" in detail:
                    self.connection_var.set("COM-порт занят другой программой")
                    self.activity_var.set("Закрой другую программу, использующую ESP32, затем нажми «Подключить»")
                else:
                    self.connection_var.set(f"связь потеряна: {exc}")
                self.link.close()
                self.pending_channels.clear()
                self.pending_tokens.clear()
                self.busy_deadlines.clear()
                self.group_queue.clear()
                self.group_total = 0
                self.group_done = 0
                self.next_group_at = 0.0
                self.calibration_test_active = False
                self._log_event(f"CONNECTION LOST error={exc}")
                self._refresh_cards()

        expired = [ch for ch, deadline in self.busy_deadlines.items() if now > deadline]
        for ch in expired:
            try:
                self.link.send(f"STOP {ch}")
                self.link.send(f"DETACH {ch}")
            except Exception:
                pass
            self.pending_channels.discard(ch)
            self.pending_tokens.pop(ch, None)
            self.busy_deadlines.pop(ch, None)
            self.group_queue.clear()
            self.group_total = 0
            self.group_done = 0
            self.next_group_at = 0.0
            self.calibration_test_active = False
            self.activity_var.set(f"Дорожка {ch + 1}: цикл не подтвердился — безопасный STOP")
            self._log_event(f"TIMEOUT lane={ch + 1}; group aborted")
        if expired:
            self._refresh_cards()

        if (
            self.group_total > 0
            and not self.pending_channels
            and self.group_queue
            and now >= self.next_group_at
        ):
            self._start_next_group()
            self._refresh_cards()

        self.root.after(60, self._tick)

    def close(self) -> None:
        try:
            self._save_names()
            self.stop_all()
        finally:
            self.link.close()
            self.root.destroy()


class CalibrationDialog:
    def __init__(self, app: FullApp, channel: int) -> None:
        self.app = app
        self.channel = channel
        self.window = tk.Toplevel(app.root)
        self.window.title(f"Калибровка дорожки {channel + 1} — {PINS[channel]}")
        self.window.geometry("920x900")
        self.window.minsize(850, 820)
        self.window.configure(bg="#101820")
        self.window.transient(app.root)
        self.window.grab_set()
        self.current_us = app.positions[channel]
        self.jogging = False
        self.jog_channels: list[int] = []
        self.group_vars = [tk.BooleanVar(value=(ch == channel)) for ch in range(NUM_SERVOS)]
        self.angle_dragging = False
        self.angle_var = tk.IntVar(value=us_to_degrees(self.current_us))
        self.position_var = tk.StringVar(value=f"{self.angle_var.get()}°")
        self.status_var = tk.StringVar(value="Держи кнопку для движения; отпусти для остановки")
        self.saved_var = tk.StringVar()
        self.direction_var = tk.StringVar()
        self.rate_var = tk.IntVar(value=CAL_RATE)
        self._build()
        self._update_labels()
        self._prepare()
        self.window.protocol("WM_DELETE_WINDOW", self.close)

    def servo(self) -> dict:
        return self.app.cfg["servos"][self.channel]

    def send(self, command: str) -> None:
        self.app.link.send(command)

    def _build(self) -> None:
        top = tk.Frame(self.window, bg="#101820", padx=18, pady=15)
        top.pack(fill="x")
        tk.Label(top, text=f"ДОРОЖКА {self.channel + 1}", bg="#101820", fg="white", font=("Segoe UI", 23, "bold")).pack(side="left")
        tk.Label(top, text=PINS[self.channel], bg="#101820", fg="#69aefa", font=("Consolas", 15, "bold")).pack(side="right")

        group = tk.Frame(self.window, bg="#18232e", padx=14, pady=9)
        group.pack(fill="x", padx=18, pady=(0, 10))
        tk.Label(group, text="ДВИГАТЬ И КАЛИБРОВАТЬ ОДНОВРЕМЕННО:", bg="#18232e", fg="#dce6ee", font=("Segoe UI", 10, "bold")).pack(side="left", padx=(0, 12))
        for ch, variable in enumerate(self.group_vars):
            tk.Checkbutton(
                group,
                text=str(ch + 1),
                variable=variable,
                command=self._group_changed,
                bg="#18232e",
                fg="white",
                activebackground="#18232e",
                activeforeground="white",
                selectcolor="#1169c9",
                font=("Segoe UI", 11, "bold"),
            ).pack(side="left", padx=5)
        tk.Button(group, text="ВСЕ 5", command=self._select_all, bg="#526371", fg="white", relief="flat", padx=12, pady=4, font=("Segoe UI", 9, "bold")).pack(side="right")

        center = tk.Frame(self.window, bg="#18232e", padx=14, pady=12)
        center.pack(fill="x", padx=18)
        tk.Button(center, text="СБРОСИТЬ И ВЫСТАВИТЬ СЕРВУ НА 90°\nТОЛЬКО СО СНЯТОЙ КАЧАЛКОЙ", command=self.center_servo, bg="#d8d6d1", fg="#111", relief="flat", font=("Segoe UI", 12, "bold"), padx=12, pady=12).pack(side="left", fill="x", expand=True, padx=(0, 7))
        tk.Button(center, text="СНЯТЬ СИГНАЛ", command=self.release, bg="#9c2730", fg="white", relief="flat", font=("Segoe UI", 12, "bold"), padx=16, pady=20).pack(side="left", padx=(7, 0))

        pos = tk.Frame(self.window, bg="#18232e", padx=14, pady=12)
        pos.pack(fill="x", padx=18, pady=(10, 10))
        tk.Label(pos, text="ТЕКУЩЕЕ ПОЛОЖЕНИЕ", bg="#18232e", fg="#cbd6df", font=("Segoe UI", 11)).pack()
        tk.Label(pos, textvariable=self.position_var, bg="#18232e", fg="white", font=("Consolas", 31, "bold")).pack()
        tk.Label(pos, textvariable=self.status_var, bg="#18232e", fg="#a8b7c5", font=("Segoe UI", 10)).pack()

        angle_panel = tk.Frame(self.window, bg="#18232e", padx=14, pady=10)
        angle_panel.pack(fill="x", padx=18, pady=(0, 10))
        tk.Label(angle_panel, text="УГОЛ СЕРВЫ", bg="#18232e", fg="#cbd6df", font=("Segoe UI", 10, "bold")).pack(anchor="w")
        self.angle_scale = tk.Scale(
            angle_panel,
            from_=CAL_MIN_DEG,
            to=CAL_MAX_DEG,
            resolution=1,
            orient="horizontal",
            showvalue=True,
            variable=self.angle_var,
            command=self._preview_angle,
            bg="#18232e",
            fg="white",
            troughcolor="#344653",
            activebackground="#69aefa",
            highlightthickness=0,
            font=("Segoe UI", 10, "bold"),
        )
        self.angle_scale.pack(fill="x")
        self.angle_scale.bind("<ButtonPress-1>", self._angle_press)
        self.angle_scale.bind("<ButtonRelease-1>", self._angle_release)

        steps = tk.Frame(angle_panel, bg="#18232e")
        steps.pack(pady=(5, 0))
        for text, delta in (("−10°", -10), ("−5°", -5), ("−1°", -1), ("+1°", 1), ("+5°", 5), ("+10°", 10)):
            tk.Button(
                steps,
                text=text,
                command=lambda d=delta: self.step_angle(d),
                bg="#526371",
                fg="white",
                relief="flat",
                width=8,
                pady=5,
                font=("Segoe UI", 10, "bold"),
            ).pack(side="left", padx=3)

        presets = tk.Frame(angle_panel, bg="#18232e")
        presets.pack(fill="x", pady=(8, 0))
        tk.Button(
            presets,
            text="БЫСТРО К HOME 30°",
            command=lambda: self.move_to_angle(30),
            bg="#325f86",
            fg="white",
            relief="flat",
            pady=7,
            font=("Segoe UI", 10, "bold"),
        ).pack(side="left", fill="x", expand=True, padx=(0, 5))
        tk.Button(
            presets,
            text="БЫСТРО К ВЫДАЧЕ 110°",
            command=lambda: self.move_to_angle(110),
            bg="#325f86",
            fg="white",
            relief="flat",
            pady=7,
            font=("Segoe UI", 10, "bold"),
        ).pack(side="left", fill="x", expand=True, padx=(5, 0))

        moves = tk.Frame(self.window, bg="#101820")
        moves.pack(fill="x", padx=18)
        down = tk.Button(moves, text="▼ ДЛИННАЯ РУКА ВНИЗ\nHOME / ЗАКРЫТО", bg="#d8d6d1", fg="#111", relief="flat", font=("Segoe UI", 16, "bold"), pady=23)
        down.pack(side="left", fill="x", expand=True, padx=(0, 7))
        up = tk.Button(moves, text="▲ ДЛИННАЯ РУКА ВВЕРХ\nВЫДАЧА / ОТКРЫТО", bg="#d8d6d1", fg="#111", relief="flat", font=("Segoe UI", 16, "bold"), pady=23)
        up.pack(side="left", fill="x", expand=True, padx=(7, 0))
        down.bind("<ButtonPress-1>", lambda _e: self.start_jog("down"))
        down.bind("<ButtonRelease-1>", self.stop_jog)
        up.bind("<ButtonPress-1>", lambda _e: self.start_jog("up"))
        up.bind("<ButtonRelease-1>", self.stop_jog)

        options = tk.Frame(self.window, bg="#101820")
        options.pack(fill="x", padx=18, pady=10)
        tk.Button(options, text="↔ Поменять направления кнопок", command=self.reverse, bg="#526371", fg="white", relief="flat", font=("Segoe UI", 10, "bold"), pady=8).pack(side="left", fill="x", expand=True, padx=(0, 10))
        tk.Label(options, text="Скорость", bg="#101820", fg="#9fb0bf").pack(side="left")
        ttk.Scale(options, from_=30, to=300, variable=self.rate_var, orient="horizontal", length=180).pack(side="left", padx=8)
        tk.Label(options, textvariable=self.direction_var, bg="#101820", fg="#9fb0bf").pack(side="right")

        marks = tk.Frame(self.window, bg="#101820")
        marks.pack(fill="x", padx=18)
        tk.Button(marks, text="ЗАПОМНИТЬ КАК HOME", command=self.mark_home, bg="#325f86", fg="white", relief="flat", font=("Segoe UI", 13, "bold"), pady=14).pack(side="left", fill="x", expand=True, padx=(0, 7))
        tk.Button(marks, text="ЗАПОМНИТЬ КАК ВЫДАЧУ", command=self.mark_eject, bg="#325f86", fg="white", relief="flat", font=("Segoe UI", 13, "bold"), pady=14).pack(side="left", fill="x", expand=True, padx=(7, 0))

        tk.Label(self.window, textvariable=self.saved_var, bg="#18232e", fg="#e9f1f7", font=("Consolas", 12, "bold"), pady=12).pack(fill="x", padx=18, pady=(10, 10))

        bottom = tk.Frame(self.window, bg="#101820")
        bottom.pack(fill="x", padx=18)
        self.test_btn = tk.Button(bottom, text="ПРОВЕРИТЬ ЦИКЛ БЕЗ БАНКИ", command=self.test_cycle, bg="#8a6419", fg="white", relief="flat", font=("Segoe UI", 12, "bold"), pady=14)
        self.test_btn.pack(side="left", fill="x", expand=True, padx=(0, 7))
        self.save_btn = tk.Button(bottom, text="СОХРАНИТЬ И ЗАКРЫТЬ", command=self.save_and_close, bg="#168847", fg="white", relief="flat", font=("Segoe UI", 13, "bold"), pady=14)
        self.save_btn.pack(side="left", fill="x", expand=True, padx=(7, 0))
        self.window.bind("<Escape>", lambda _e: self.app.stop_all())

    def _prepare(self) -> None:
        ch = self.channel
        self.send(f"CH {ch}")
        self.send(f"LIM {ch} {CAL_MIN_US} {CAL_MAX_US}")
        self.send(f"RATE {ch} {CAL_RATE}")
        self.send(f"HOLD {ch} 0")
        self.send(f"POWERMAX {ch} 3000")
        self.send("INFO")

    def selected_channels(self) -> list[int]:
        channels = [ch for ch, variable in enumerate(self.group_vars) if variable.get()]
        if not channels:
            self.group_vars[self.channel].set(True)
            channels = [self.channel]
        return channels

    def _group_changed(self) -> None:
        channels = self.selected_channels()
        # Changing the group is a safety boundary: stop every output before the
        # next grouped command is allowed.
        self.app.stop_all()
        self.status_var.set("Выбраны дорожки: " + ", ".join(str(ch + 1) for ch in channels))
        self._update_labels()

    def _select_all(self) -> None:
        for variable in self.group_vars:
            variable.set(True)
        self._group_changed()

    def _update_labels(self) -> None:
        servo = self.servo()
        home = int(servo.get("closed_us", DEFAULT_US))
        eject = int(servo.get("eject_us", DEFAULT_US))
        self.saved_var.set(
            f"HOME: {us_to_degrees(home)}°" + (" ✓" if servo.get("home_fixed") else " (старт)")
            + f"     |     ВЫДАЧА: {us_to_degrees(eject)}°"
            + (" ✓" if servo.get("eject_fixed") else " (старт)")
        )
        self.direction_var.set("обычные направления" if int(servo.get("up_sign", 1)) > 0 else "направления поменяны")
        state = "normal" if all(calibrated(self.app.cfg["servos"][ch]) for ch in self.selected_channels()) else "disabled"
        self.test_btn.configure(state=state)
        self.save_btn.configure(state=state)

    def set_position(self, value: int) -> None:
        self.current_us = int(value)
        angle = us_to_degrees(self.current_us)
        self.position_var.set(f"{angle}°")
        if not self.angle_dragging:
            self.angle_var.set(angle)

    def _preview_angle(self, value: str) -> None:
        if self.angle_dragging:
            self.position_var.set(f"{int(float(value))}°")

    def _angle_press(self, _event=None) -> None:
        self.stop_jog()
        self.angle_dragging = True

    def _angle_release(self, _event=None) -> None:
        self.angle_dragging = False
        self.move_to_angle(int(self.angle_var.get()))

    def move_to_angle(self, angle: int) -> None:
        angle = max(CAL_MIN_DEG, min(CAL_MAX_DEG, int(angle)))
        target_us = max(CAL_MIN_US, min(CAL_MAX_US, degrees_to_us(angle)))
        channels = self.selected_channels()
        for ch in channels:
            self.send(f"LIM {ch} {CAL_MIN_US} {CAL_MAX_US}")
            self.send(f"RATE {ch} {max(100, min(800, int(self.rate_var.get()) * 3))}")
            self.send(f"POWERMAX {ch} 5000")
            self.send(f"HOLD {ch} 0")
            self.send(f"SET {ch} {target_us}")
            self.app.positions[ch] = target_us
        self.set_position(target_us)
        self.status_var.set(f"Дорожки {', '.join(str(ch + 1) for ch in channels)}: переход к {angle}°")

    def step_angle(self, delta: int) -> None:
        base = us_to_degrees(self.current_us)
        angle = max(CAL_MIN_DEG, min(CAL_MAX_DEG, base + int(delta)))
        self.angle_var.set(angle)
        self.move_to_angle(angle)

    def center_servo(self) -> None:
        center_us = degrees_to_us(90)
        channels = self.selected_channels()
        channel_text = ", ".join(str(ch + 1) for ch in channels)
        if not messagebox.askyesno(
            "Проверка безопасности",
            f"Выбраны дорожки: {channel_text}.\n\n"
            "Качалки и тяги сняты со ВСЕХ выбранных серв?\n\n"
            "Их старая калибровка будет удалена, валы одновременно повернутся на 90°.",
            parent=self.window,
        ):
            return
        self.app.stop_all()
        for ch in channels:
            servo = self.app.cfg["servos"][ch]
            servo.update({
                "closed_us": DEFAULT_HOME_US,
                "eject_us": DEFAULT_EJECT_US,
                "guard_min_us": 757,
                "guard_max_us": 1721,
                "home_fixed": False,
                "eject_fixed": False,
                "up_sign": 1,
            })
        save_config(self.app.cfg)
        for ch in channels:
            self.send(f"CLEARCAL {ch}")
            self.send(f"LIM {ch} {CAL_MIN_US} {CAL_MAX_US}")
            self.send(f"RATE {ch} 300")
            self.send(f"POWERMAX {ch} 15000")
            self.send(f"HOLD {ch} 0")
            self.send(f"SET {ch} {center_us}")
            self.app.positions[ch] = center_us
        self.set_position(center_us)
        self.status_var.set(f"Дорожки {channel_text} на 90°: установи качалки в механический HOME")
        self._update_labels()

    def release(self) -> None:
        self.stop_jog()
        if self.app.pending_channels or self.app.group_total > 0:
            self.app.stop_all()
        channels = self.selected_channels()
        for ch in channels:
            self.send(f"HOLD {ch} 0")
            self.send(f"STOP {ch}")
            self.send(f"DETACH {ch}")
        self.status_var.set("Сигнал снят с дорожек: " + ", ".join(str(ch + 1) for ch in channels))

    def start_jog(self, semantic: str) -> None:
        if self.jogging:
            return
        channels = self.selected_channels()
        for ch in channels:
            servo = self.app.cfg["servos"][ch]
            up_sign = 1 if int(servo.get("up_sign", 1)) > 0 else -1
            sign = up_sign if semantic == "up" else -up_sign
            self.send(f"LIM {ch} {CAL_MIN_US} {CAL_MAX_US}")
            self.send(f"RATE {ch} {max(30, min(300, int(self.rate_var.get())))}")
            self.send(f"POWERMAX {ch} 3000")
            self.send(f"HOLD {ch} 1")
            self.send(f"JOG {ch} {sign}")
        self.jogging = True
        self.jog_channels = channels
        self.status_var.set("Групповое движение… отпусти кнопку для остановки")

    def stop_jog(self, _event=None) -> None:
        if not self.jogging:
            return
        for ch in self.jog_channels:
            self.send(f"JOG {ch} 0")
            self.send(f"HOLD {ch} 0")
        self.send("INFO ALL")
        self.jogging = False
        self.jog_channels = []
        self.status_var.set("Остановлено")

    def reverse(self) -> None:
        self.stop_jog()
        for ch in self.selected_channels():
            servo = self.app.cfg["servos"][ch]
            servo["up_sign"] = -1 if int(servo.get("up_sign", 1)) > 0 else 1
        save_config(self.app.cfg)
        self._update_labels()

    def _persist_if_ready(self, ch: int) -> None:
        servo = self.app.cfg["servos"][ch]
        if not calibrated(servo):
            return
        home = int(servo["closed_us"])
        eject = int(servo["eject_us"])
        servo["guard_min_us"] = max(700, min(home, eject) - 60)
        servo["guard_max_us"] = min(2300, max(home, eject) + 60)
        save_config(self.app.cfg)
        self.send(f"CAL {ch} {home} {eject}")

    def mark_home(self) -> None:
        self.stop_jog()
        channels = self.selected_channels()
        for ch in channels:
            servo = self.app.cfg["servos"][ch]
            servo["closed_us"] = int(self.app.positions[ch])
            servo["home_fixed"] = True
        save_config(self.app.cfg)
        for ch in channels:
            self._persist_if_ready(ch)
        self._update_labels()
        self.status_var.set("HOME сохранён дорожкам: " + ", ".join(str(ch + 1) for ch in channels))

    def mark_eject(self) -> None:
        self.stop_jog()
        channels = self.selected_channels()
        for ch in channels:
            servo = self.app.cfg["servos"][ch]
            servo["eject_us"] = int(self.app.positions[ch])
            servo["eject_fixed"] = True
        save_config(self.app.cfg)
        for ch in channels:
            self._persist_if_ready(ch)
        self._update_labels()
        self.status_var.set("ВЫДАЧА сохранена дорожкам: " + ", ".join(str(ch + 1) for ch in channels))

    def test_cycle(self) -> None:
        channels = self.selected_channels()
        if not all(calibrated(self.app.cfg["servos"][ch]) for ch in channels):
            return
        if self.app.pending_channels or self.app.group_total > 0:
            return
        self.stop_jog()
        self.app.group_queue = list(channels)
        self.app.group_total = len(channels)
        self.app.group_done = 0
        self.app.next_group_at = 0.0
        self.app.calibration_test_active = True
        self.app._log_event("CALIBRATION TEST START lanes=" + ",".join(str(ch + 1) for ch in channels))
        self.app._start_next_group()
        self.status_var.set(
            "Безопасная последовательная проверка: "
            + ", ".join(str(ch + 1) for ch in channels)
        )

    def save_and_close(self) -> None:
        channels = self.selected_channels()
        if not all(calibrated(self.app.cfg["servos"][ch]) for ch in channels):
            return
        self.stop_jog()
        if self.app.pending_channels or self.app.group_total > 0:
            self.app.stop_all()
        for ch in channels:
            self._persist_if_ready(ch)
            servo = self.app.cfg["servos"][ch]
            self.send(f"RATE {ch} {TEST_RATE}")
            self.send(f"HOLD {ch} 0")
            self.send(f"POWERMAX {ch} 5000")
            self.send(f"SET {ch} {int(servo['closed_us'])}")
        self.window.grab_release()
        self.window.destroy()
        self.app.calibration_closed()

    def close(self) -> None:
        self.stop_jog()
        if self.app.pending_channels or self.app.group_total > 0:
            self.app.stop_all()
        for ch in self.selected_channels():
            servo = self.app.cfg["servos"][ch]
            if calibrated(servo):
                self.send(f"RATE {ch} {TEST_RATE}")
                self.send(f"HOLD {ch} 0")
                self.send(f"SET {ch} {int(servo['closed_us'])}")
            else:
                self.send(f"STOP {ch}")
                self.send(f"DETACH {ch}")
        self.window.grab_release()
        self.window.destroy()
        self.app.calibration_closed()


def main() -> None:
    if not acquire_single_instance():
        root = tk.Tk()
        root.withdraw()
        messagebox.showwarning(
            "Дозатор уже запущен",
            "Панель дозатора уже открыта. Используй существующее окно — вторая копия не запущена.",
        )
        root.destroy()
        return
    root = tk.Tk()
    try:
        FullApp(root)
        root.mainloop()
    finally:
        release_single_instance()


if __name__ == "__main__":
    main()
