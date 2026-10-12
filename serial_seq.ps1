# Отправка ПОСЛЕДОВАТЕЛЬНОСТИ команд в одном подключении (плата ресетится только раз при открытии).
# Токен "w<мс>" = пауза. Пример:
#   powershell -File serial_seq.ps1 -Port COM6 -Cmds 1o,w2000,1c,2o
param(
  [string]$Port = "COM6",
  [int]$Baud = 115200,
  [string]$Seq,            # команды через запятую, напр. "1o,w2000,1c,2o"
  [int]$BootWait = 1800,   # дать плате загрузиться после reset-on-open
  [int]$Gap = 350          # пауза после обычной команды
)
$Cmds = $Seq -split ','
$p = New-Object System.IO.Ports.SerialPort $Port, $Baud
$p.DtrEnable = $false
$p.RtsEnable = $false
$p.ReadTimeout = 300
$p.NewLine = "`n"
$p.Open()
Start-Sleep -Milliseconds $BootWait
$p.DiscardInBuffer()
foreach ($c in $Cmds) {
  if ($c -match '^w(\d+)$') {
    Start-Sleep -Milliseconds ([int]$Matches[1])
  } else {
    $p.WriteLine($c)
    Write-Output ("-> " + $c)
    Start-Sleep -Milliseconds $Gap
  }
}
Start-Sleep -Milliseconds 300
$resp = ""
try { while ($true) { $resp += ($p.ReadLine() + "`n") } } catch {}
$p.Close()
Write-Output ("RESP: " + $resp.Trim())
