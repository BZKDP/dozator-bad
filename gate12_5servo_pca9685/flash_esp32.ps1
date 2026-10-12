param(
    [string]$Port
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

if (-not (Get-Command arduino-cli -ErrorAction SilentlyContinue)) {
    throw 'arduino-cli не найден. Можно загрузить .ino через Arduino IDE.'
}

if ([string]::IsNullOrWhiteSpace($Port)) {
    arduino-cli board list
    $Port = Read-Host 'Введи COM-порт ESP32, например COM6'
}

if ($Port -notmatch '^COM\d+$') {
    throw "Некорректный COM-порт: $Port"
}

arduino-cli lib install 'Adafruit PWM Servo Driver Library'
arduino-cli compile --fqbn esp32:esp32:esp32 .\gate12_5servo_pca9685_safe
arduino-cli upload --fqbn esp32:esp32:esp32 --port $Port .\gate12_5servo_pca9685_safe

Write-Host "Готово. Прошивка загружена через $Port."
