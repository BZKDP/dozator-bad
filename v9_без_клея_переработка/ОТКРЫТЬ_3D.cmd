@echo off
chcp 65001 >nul
set "PROJECT_ROOT=%~dp0.."
start "" powershell.exe -NoProfile -WindowStyle Hidden -Command "Start-Sleep -Milliseconds 900; Start-Process 'http://127.0.0.1:8765/v9_без_клея_переработка/ОТКРОЙ_3D.html'"
py -m http.server 8765 --bind 127.0.0.1 --directory "%PROJECT_ROOT%"
