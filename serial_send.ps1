# Отправка команды в ESP32 по serial без авто-сброса платы.
# Использование:  powershell -File serial_send.ps1 -Port COM6 -Cmd o
param(
  [string]$Port = "COM6",
  [int]$Baud = 115200,
  [string]$Cmd = "o",
  [int]$Wait = 800
)
$p = New-Object System.IO.Ports.SerialPort $Port, $Baud
$p.DtrEnable = $false      # не дёргать DTR/RTS -> ESP32 не ресетится при открытии/закрытии
$p.RtsEnable = $false
$p.ReadTimeout = 1200
$p.NewLine = "`n"
$p.Open()
Start-Sleep -Milliseconds 300
$p.DiscardInBuffer()
$p.WriteLine($Cmd)
Start-Sleep -Milliseconds $Wait
$resp = ""
try { while ($true) { $resp += ($p.ReadLine() + "`n") } } catch {}
$p.Close()
Write-Output ("SENT: " + $Cmd)
Write-Output ("RESP: " + $resp.Trim())
