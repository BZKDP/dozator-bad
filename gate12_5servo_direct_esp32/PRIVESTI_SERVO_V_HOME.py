# -*- coding: utf-8 -*-
"""
Приведение серв MG996R в опорную позу HOME для замены без перекалибровки.
Читает реальную калибровку из gate12_direct5_app.json (те же HOME/EJECT мкс).
Логика: серва встаёт в HOME и ДЕРЖИТ → надеваешь качалку так, чтобы механизм
был в позе ХВАТ (передний носик вниз) → прикручиваешь → «отпустить».
После этого основное приложение работает без перекалибровки.

ВАЖНО: закрой основное приложение (gate12_direct5_full_app.py) — COM6 один хозяин.
"""
import json, os, threading, time
import tkinter as tk
from tkinter import ttk
import serial
from serial.tools import list_ports

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = json.load(open(os.path.join(HERE, 'gate12_direct5_app.json'), encoding='utf-8'))
PORT = CFG.get('port', 'COM6')
SERVOS = CFG['servos']
PINS = ['D13', 'D14', 'D27', 'D26', 'D25']   # дорожки 1..5 (servo 0..4)

class App:
    def __init__(self, root):
        self.root = root
        root.title('Приведение серв MG996R в HOME')
        root.geometry('640x560')
        root.configure(bg='#141821')
        self.ser = None

        top = tk.Frame(root, bg='#141821'); top.pack(fill='x', padx=12, pady=8)
        tk.Label(top, text=f'Порт {PORT}:', bg='#141821', fg='#dfe7ef',
                 font=('Segoe UI', 11)).pack(side='left')
        self.st = tk.Label(top, text='подключаюсь…', bg='#141821', fg='#e0b050',
                           font=('Segoe UI', 11, 'bold')); self.st.pack(side='left', padx=8)
        tk.Button(top, text='Переподключить', command=self.connect,
                  bg='#243244', fg='white').pack(side='right')

        tk.Label(root, text='Втыкаешь серву в её дорожку → «В HOME» → она держит позу →\n'
                            'надеваешь качалку в позе ХВАТ и прикручиваешь → «Отпустить».',
                 bg='#141821', fg='#9fb2c6', font=('Segoe UI', 10), justify='left'
                 ).pack(anchor='w', padx=12)

        body = tk.Frame(root, bg='#141821'); body.pack(fill='x', padx=12, pady=6)
        self.rowst = []
        for i, s in enumerate(SERVOS):
            r = tk.Frame(body, bg='#1b2330'); r.pack(fill='x', pady=3)
            tk.Label(r, text=f'{i+1}', bg='#2f6ea5', fg='white', width=3,
                     font=('Segoe UI', 14, 'bold')).pack(side='left', padx=(6, 8), pady=6)
            tk.Label(r, text=f"{s.get('name','Дорожка')}  ·  {PINS[i]}  ·  HOME {s['closed_us']}мкс",
                     bg='#1b2330', fg='#dfe7ef', width=26, anchor='w',
                     font=('Segoe UI', 10)).pack(side='left')
            tk.Button(r, text='▶ В HOME', width=9, bg='#2f8f4e', fg='white',
                      font=('Segoe UI', 10, 'bold'),
                      command=lambda ch=i: self.to_home(ch)).pack(side='left', padx=3)
            tk.Button(r, text='центр', width=6, bg='#3a4655', fg='white',
                      command=lambda ch=i: self.to_center(ch)).pack(side='left', padx=2)
            tk.Button(r, text='отпустить', width=9, bg='#7a4a2a', fg='white',
                      command=lambda ch=i: self.release(ch)).pack(side='left', padx=3)
            lb = tk.Label(r, text='—', bg='#1b2330', fg='#8fa4b8', width=10,
                          font=('Segoe UI', 9)); lb.pack(side='left', padx=4)
            self.rowst.append(lb)

        allf = tk.Frame(root, bg='#141821'); allf.pack(fill='x', padx=12, pady=8)
        tk.Button(allf, text='ВСЕ В HOME', bg='#2f8f4e', fg='white',
                  font=('Segoe UI', 11, 'bold'),
                  command=self.all_home).pack(side='left', expand=True, fill='x', padx=4)
        tk.Button(allf, text='ВСЁ ОТПУСТИТЬ', bg='#7a4a2a', fg='white',
                  font=('Segoe UI', 11, 'bold'),
                  command=self.all_release).pack(side='left', expand=True, fill='x', padx=4)

        self.log = tk.Text(root, height=11, bg='#0f1216', fg='#c7d2dd',
                           font=('Consolas', 9)); self.log.pack(fill='both', expand=True, padx=12, pady=6)
        self.put('Готово. Закрой основное приложение, если серва не двигается (порт занят).')
        root.protocol('WM_DELETE_WINDOW', self.on_close)
        self.connect()

    def put(self, t):
        self.log.insert('end', t.rstrip() + '\n'); self.log.see('end')

    def connect(self):
        try:
            if self.ser:
                try: self.ser.close()
                except: pass
            avail = [p.device for p in list_ports.comports()]
            port = PORT if PORT in avail else (avail[0] if avail else PORT)
            self.ser = serial.Serial(port, 115200, timeout=0.1, dsrdtr=False, rtscts=False)
            self.ser.dtr = False; self.ser.rts = False
            time.sleep(0.4); self.ser.reset_input_buffer()
            self.st.config(text=f'подключено {port} ✓', fg='#8fd08f')
            threading.Thread(target=self.reader, daemon=True).start()
            self.send('PING')
        except Exception as e:
            self.st.config(text=f'нет связи: {e}', fg='#e05a5a')
            self.put(f'!! {e}  — закрой основное приложение (оно держит {PORT})')

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
            self.put(f'!! обрыв: {e}')

    def _prep(self, ch):
        s = SERVOS[ch]
        self.send(f"LIM {ch} {s['guard_min_us']} {s['guard_max_us']}")   # расширить лимиты
        self.send(f"RATE {ch} {s.get('speed_us_s', 500)}")               # плавно
        self.send(f"HOLD {ch} 1")                                        # держать позу

    def to_home(self, ch):
        self._prep(ch)
        self.send(f"SET {ch} {SERVOS[ch]['closed_us']}")
        self.rowst[ch].config(text='держит HOME', fg='#8fd08f')

    def to_center(self, ch):
        self._prep(ch)
        self.send(f"SET {ch} 1500")
        self.rowst[ch].config(text='центр 1500', fg='#7fd1ff')

    def release(self, ch):
        self.send(f"HOLD {ch} 0")
        self.send(f"DETACH {ch}")
        self.rowst[ch].config(text='отпущено', fg='#8fa4b8')

    def all_home(self):
        for ch in range(len(SERVOS)): self.to_home(ch)

    def all_release(self):
        for ch in range(len(SERVOS)): self.release(ch)

    def on_close(self):
        try: self.all_release()
        except: pass
        time.sleep(0.2)
        try: self.ser.close()
        except: pass
        self.root.destroy()

root = tk.Tk()
root.attributes('-topmost', True)
root.after(2500, lambda: root.attributes('-topmost', False))
App(root)
root.mainloop()
