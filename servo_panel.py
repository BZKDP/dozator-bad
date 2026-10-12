# -*- coding: utf-8 -*-
"""Панель ручной калибровки сервы BX04 (ESP32 на COM6, скетч servo_bx04.ino).
Стрелки клавиатуры: ←/→ = ±1°, Shift+←/→ = ±5°."""
import tkinter as tk
from tkinter import ttk
import serial, threading, time
from serial.tools import list_ports

class App:
    def __init__(self, root):
        self.root = root
        root.title('BX04 серва — калибровка')
        root.geometry('520x560')
        root.configure(bg='#1b2330')
        self.ser = None
        self.angle = tk.IntVar(value=14)

        top = tk.Frame(root, bg='#1b2330'); top.pack(fill='x', padx=10, pady=8)
        self.port = ttk.Combobox(top, values=[p.device for p in list_ports.comports()], width=10)
        self.port.set('COM6'); self.port.pack(side='left')
        tk.Button(top, text='ПОДКЛЮЧИТЬ', command=self.connect, bg='#2f6ea5', fg='white',
                  font=('Segoe UI', 10, 'bold')).pack(side='left', padx=8)
        self.status = tk.Label(top, text='не подключено', bg='#1b2330', fg='#e05a5a')
        self.status.pack(side='left', padx=8)

        # большой текущий угол
        self.big = tk.Label(root, text='—', font=('Segoe UI', 44, 'bold'), bg='#1b2330', fg='#7fd1ff')
        self.big.pack(pady=4)

        # слайдер
        s = tk.Scale(root, from_=0, to=180, orient='horizontal', variable=self.angle,
                     length=480, bg='#1b2330', fg='white', troughcolor='#26313d',
                     highlightthickness=0, font=('Segoe UI', 11))
        s.pack(padx=10)
        s.bind('<ButtonRelease-1>', lambda e: self.go(self.angle.get()))

        # кнопки шагов
        row = tk.Frame(root, bg='#1b2330'); row.pack(pady=8)
        for txt, d in [('−10', -10), ('−5', -5), ('−1', -1), ('+1', 1), ('+5', 5), ('+10', 10)]:
            tk.Button(row, text=txt, width=6, font=('Segoe UI', 12, 'bold'),
                      command=lambda d=d: self.step(d), bg='#26313d', fg='white').pack(side='left', padx=3)

        # быстрые позиции
        row2 = tk.Frame(root, bg='#1b2330'); row2.pack(pady=4)
        for txt, cmd, col in [('0°', '0', '#444c57'), ('90°', '90', '#444c57'), ('180°', '180', '#444c57'),
                              ('HOME (h)', 'h', '#2f6ea5'), ('EJECT (e)', 'e', '#2f6ea5')]:
            tk.Button(row2, text=txt, width=9, command=lambda c=cmd: self.send(c),
                      bg=col, fg='white', font=('Segoe UI', 10)).pack(side='left', padx=3)

        # калибровка
        cal = tk.LabelFrame(root, text=' калибровка ', bg='#1b2330', fg='#8fd08f', font=('Segoe UI', 10))
        cal.pack(fill='x', padx=10, pady=8)
        r3 = tk.Frame(cal, bg='#1b2330'); r3.pack(pady=6)
        tk.Button(r3, text='ЭТО = HOME', command=lambda: self.send(f'H{self.angle.get()}'),
                  bg='#3a6b3a', fg='white', width=12, font=('Segoe UI', 10, 'bold')).pack(side='left', padx=4)
        tk.Button(r3, text='ЭТО = EJECT', command=lambda: self.send(f'E{self.angle.get()}'),
                  bg='#7a5a2a', fg='white', width=12, font=('Segoe UI', 10, 'bold')).pack(side='left', padx=4)
        tk.Button(r3, text='💾 СОХРАНИТЬ', command=lambda: self.send('w'),
                  bg='#2f6ea5', fg='white', width=12, font=('Segoe UI', 10, 'bold')).pack(side='left', padx=4)
        r4 = tk.Frame(cal, bg='#1b2330'); r4.pack(pady=4)
        tk.Button(r4, text='ЦИКЛ выдачи (t)', command=lambda: self.send('t'),
                  bg='#26313d', fg='white', width=14).pack(side='left', padx=4)
        tk.Button(r4, text='5 циклов (n)', command=lambda: self.send('n'),
                  bg='#26313d', fg='white', width=12).pack(side='left', padx=4)
        tk.Button(r4, text='Свип (s)', command=lambda: self.send('s'),
                  bg='#26313d', fg='white', width=10).pack(side='left', padx=4)
        tk.Button(r4, text='? статус', command=lambda: self.send('?'),
                  bg='#26313d', fg='white', width=9).pack(side='left', padx=4)

        # лог
        self.log = tk.Text(root, height=10, bg='#12161c', fg='#c7d2dd', font=('Consolas', 10))
        self.log.pack(fill='both', expand=True, padx=10, pady=6)
        self.put('Стрелки ←/→ = ±1°, Shift+стрелки = ±5°. Подключай и вперёд.')
        self.put('Ищи края: где серва упирается/жужжит — это край. Запомни оба края,')
        self.put('потом выставь HOME/EJECT внутри рабочей зоны и жми СОХРАНИТЬ.')

        root.bind('<Left>', lambda e: self.step(-5 if e.state & 1 else -1))
        root.bind('<Right>', lambda e: self.step(5 if e.state & 1 else 1))
        self.connect()

    def put(self, txt):
        t = txt.rstrip()
        if 'READY' in t and hasattr(self, '_seen_ready'):
            self.log.insert('end', t + '   ⚠ ПЛАТА РЕСЕТНУЛАСЬ — просадка питания сервы! Нужен внешний 5В\n', 'warn')
        else:
            self.log.insert('end', t + '\n')
            if 'READY' in t: self._seen_ready = True
        self.log.tag_config('warn', foreground='#ff7a6e')
        self.log.see('end')

    def connect(self):
        try:
            if self.ser: self.ser.close()
            self.ser = serial.Serial(self.port.get(), 115200, timeout=0.1, dsrdtr=False, rtscts=False)
            self.ser.dtr = False; self.ser.rts = False
            time.sleep(0.3); self.ser.reset_input_buffer()
            self.status.config(text='подключено ✓', fg='#8fd08f')
            threading.Thread(target=self.reader, daemon=True).start()
            self.send('?')
        except Exception as e:
            self.status.config(text=f'ошибка: {e}', fg='#e05a5a')

    def reader(self):
        buf = b''
        while self.ser and self.ser.is_open:
            try:
                b = self.ser.read(256)
                if b:
                    buf += b
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        t = line.decode('utf-8', 'replace').strip()
                        if t: self.root.after(0, self.put, '← ' + t)
                        if t.startswith('angle ='):
                            try:
                                a = int(t.split('=')[1])
                                self.root.after(0, lambda a=a: (self.angle.set(a), self.big.config(text=f'{a}°')))
                            except: pass
            except Exception:
                break
            time.sleep(0.02)

    def send(self, cmd):
        if not self.ser or not self.ser.is_open:
            self.put('!! не подключено'); return
        try:
            self.ser.write((cmd + '\n').encode()); self.ser.flush()
            self.put('→ ' + cmd)
        except Exception as e:
            self.put(f'!! обрыв порта: {e} — жми ПОДКЛЮЧИТЬ')

    def go(self, a):
        self.send(str(int(a)))

    def step(self, d):
        a = max(0, min(180, self.angle.get() + d))
        self.angle.set(a); self.go(a)

root = tk.Tk()
root.attributes('-topmost', True)          # поверх всех окон
root.after(3000, lambda: root.attributes('-topmost', False))
root.lift(); root.focus_force()
App(root)
root.mainloop()
