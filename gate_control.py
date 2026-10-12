#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Окно с кнопками для управления двумя серво-шлагбаумами (Arduino Uno на COM6).
Кнопки шлют команды по USB-Serial: 1o/1c (G1 вверх/вниз), 2o/2c (G2), h (оба вниз).
Запуск:  python gate_control.py
Зависимость:  pip install pyserial
"""
import tkinter as tk
import serial
import serial.tools.list_ports

PORT = "COM6"
BAUD = 115200


def find_port():
    for p in serial.tools.list_ports.comports():
        d = (p.description or "").lower()
        if any(k in d for k in ("ch340", "usb-serial", "arduino", "uart", "cp210")):
            return p.device
    return PORT


class App:
    def __init__(self, root):
        self.root = root
        root.title("Дозатор — шлагбаумы")
        root.configure(bg="#171a21")
        root.geometry("360x430")

        self.port = find_port()
        self.ser = None
        try:
            self.ser = serial.Serial()
            self.ser.port = self.port
            self.ser.baudrate = BAUD
            self.ser.dtr = False     # не ресетить плату при подключении
            self.ser.rts = False
            self.ser.open()
        except Exception as e:
            self.ser = None
            self.err = str(e)

        tk.Label(root, text="Управление шлагбаумами", fg="#e6e9ef", bg="#171a21",
                 font=("Segoe UI", 15, "bold")).pack(pady=(14, 2))
        self.status = tk.Label(root, fg="#8a93a6", bg="#171a21", font=("Segoe UI", 9))
        self.status.pack()
        self._set_status()

        self._gate("G1 (главный)", "#ffb454", "1")
        self._gate("G2 (придержка)", "#4dd0a0", "2")

        tk.Button(root, text="⬇  Опустить оба", command=lambda: self.send("h"),
                  bg="#2b3340", fg="#e6e9ef", font=("Segoe UI", 12), relief="flat",
                  height=2).pack(fill="x", padx=16, pady=(14, 6))

        self.log = tk.Label(root, text="", fg="#8a93a6", bg="#171a21", font=("Consolas", 9))
        self.log.pack(pady=4)

    def _set_status(self):
        if self.ser and self.ser.is_open:
            self.status.config(text=f"подключено: {self.port}", fg="#4dd0a0")
        else:
            self.status.config(text=f"НЕТ связи ({self.port}). Закрой Serial Monitor!", fg="#ff6b6b")

    def _gate(self, name, color, idx):
        f = tk.Frame(self.root, bg="#12151c", highlightbackground="#262b36", highlightthickness=1)
        f.pack(fill="x", padx=16, pady=8)
        tk.Label(f, text=name, fg=color, bg="#12151c", font=("Segoe UI", 12, "bold")).pack(pady=(8, 4))
        row = tk.Frame(f, bg="#12151c"); row.pack(pady=(0, 10))
        tk.Button(row, text="▲ Поднять", width=12, height=2, relief="flat",
                  bg="#1e3a5f", fg="#cfe6ff", font=("Segoe UI", 12, "bold"),
                  command=lambda: self.send(idx + "o")).pack(side="left", padx=6)
        tk.Button(row, text="▼ Опустить", width=12, height=2, relief="flat",
                  bg="#3a2330", fg="#ffd6d6", font=("Segoe UI", 12, "bold"),
                  command=lambda: self.send(idx + "c")).pack(side="left", padx=6)

    def send(self, cmd):
        if not (self.ser and self.ser.is_open):
            self._set_status(); return
        try:
            self.ser.write((cmd + "\n").encode())
            self.log.config(text=f"→ {cmd}")
        except Exception as e:
            self.log.config(text=f"ошибка: {e}")
            self._set_status()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
