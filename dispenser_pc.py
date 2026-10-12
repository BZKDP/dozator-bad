#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Дозатор БАД — управление с ПК.
Выбираешь, сколько банок каждой позиции -> шлёт команду ORDER в ESP32 по USB.

Запуск GUI:   python dispenser_pc.py
Консоль:      python dispenser_pc.py --port COM5 --order 2 0 1
Калибровка:   python dispenser_pc.py --port COM5 --cal 0 1 1500

Зависимость:  pip install pyserial
"""
import sys, time, argparse

# позиции (порядок = индекс канала в прошивке)
SKUS = ["Берберин", "Мемон", "Магний B6"]

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None


def find_port():
    if not serial:
        return None
    ports = list(serial.tools.list_ports.comports())
    # ESP32 обычно CP210x / CH340 / USB-Serial
    for p in ports:
        d = (p.description or "").lower()
        if any(k in d for k in ("cp210", "ch340", "ch910", "silicon", "usb-serial", "uart")):
            return p.device
    return ports[0].device if ports else None


class Dispenser:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=5)
        time.sleep(2.0)            # ESP32 ребутится при открытии порта
        self.ser.reset_input_buffer()

    def _cmd(self, line, wait_done=True):
        self.ser.write((line + "\n").encode())
        out = []
        t0 = time.time()
        while time.time() - t0 < 60:
            ln = self.ser.readline().decode(errors="ignore").strip()
            if not ln:
                continue
            out.append(ln)
            if ln.startswith("ERR"):
                break
            if not wait_done and ln == "OK":
                break
            if wait_done and ln == "DONE":
                break
        return out

    def ping(self):
        return self._cmd("PING", wait_done=False)

    def order(self, counts):
        line = "ORDER " + " ".join(str(int(x)) for x in counts)
        return self._cmd(line)

    def dispense(self, channel, n):
        return self._cmd(f"D {channel} {n}")

    def cal(self, channel, gate, us):
        return self._cmd(f"CAL {channel} {gate} {us}", wait_done=False)

    def close(self):
        self.ser.close()


# ---------------- GUI ----------------
def run_gui(port):
    import tkinter as tk
    from tkinter import messagebox

    dev = None
    try:
        dev = Dispenser(port)
    except Exception as e:
        messagebox.showwarning("Порт", f"Не открыл {port}: {e}\nРаботаю в демо-режиме (без железа).")

    root = tk.Tk()
    root.title("Дозатор БАД")
    root.configure(bg="#171a21")
    root.geometry("360x360")

    tk.Label(root, text="Сборка заказа", fg="#e6e9ef", bg="#171a21",
             font=("Segoe UI", 15, "bold")).pack(pady=(14, 4))
    tk.Label(root, text=f"порт: {port or 'не найден'}", fg="#8a93a6",
             bg="#171a21", font=("Segoe UI", 9)).pack()

    vars_ = []
    for i, name in enumerate(SKUS):
        row = tk.Frame(root, bg="#12151c", highlightbackground="#262b36", highlightthickness=1)
        row.pack(fill="x", padx=16, pady=6, ipady=6)
        tk.Label(row, text=name, fg="#e6e9ef", bg="#12151c",
                 font=("Segoe UI", 12), width=12, anchor="w").pack(side="left", padx=10)
        v = tk.IntVar(value=0)
        vars_.append(v)
        tk.Button(row, text="–", width=2, command=lambda v=v: v.set(max(0, v.get() - 1)),
                  bg="#1d222c", fg="#fff", relief="flat").pack(side="left")
        tk.Label(row, textvariable=v, fg="#fff", bg="#0c0e13", width=4,
                 font=("Segoe UI", 12)).pack(side="left", padx=4)
        tk.Button(row, text="+", width=2, command=lambda v=v: v.set(v.get() + 1),
                  bg="#1d222c", fg="#fff", relief="flat").pack(side="left")

    status = tk.Label(root, text="готов", fg="#8a93a6", bg="#171a21", font=("Segoe UI", 9))
    status.pack(pady=4)

    def go():
        counts = [v.get() for v in vars_]
        if sum(counts) == 0:
            return
        status.config(text="выдача…")
        root.update()
        if dev:
            try:
                dev.order(counts)
                status.config(text="✓ выдано: " + ", ".join(
                    f"{n}×{SKUS[i]}" for i, n in enumerate(counts) if n))
            except Exception as e:
                status.config(text=f"ошибка: {e}")
        else:
            status.config(text="(демо) " + ", ".join(
                f"{n}×{SKUS[i]}" for i, n in enumerate(counts) if n))
        for v in vars_:
            v.set(0)

    tk.Button(root, text="Выдать заказ", command=go, bg="#4da3ff", fg="#06121f",
              font=("Segoe UI", 13, "bold"), relief="flat").pack(fill="x", padx=16, pady=10)

    root.mainloop()
    if dev:
        dev.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="COM-порт ESP32 (по умолчанию автопоиск)")
    ap.add_argument("--order", nargs="*", type=int, help="кол-во по каналам, напр. 2 0 1")
    ap.add_argument("--cal", nargs=3, type=int, metavar=("CH", "GATE", "US"),
                    help="калибровка: канал gate(0/1) микросекунды")
    args = ap.parse_args()

    if serial is None:
        print("Нет pyserial. Установи:  pip install pyserial")
        if not (args.order or args.cal):
            return

    port = args.port or find_port()

    if args.cal:
        dev = Dispenser(port)
        print(dev.cal(*args.cal)); dev.close(); return
    if args.order is not None:
        dev = Dispenser(port)
        print("PING:", dev.ping())
        print(dev.order(args.order))
        dev.close(); return

    run_gui(port)


if __name__ == "__main__":
    main()
