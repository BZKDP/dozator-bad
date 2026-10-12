param(
    [string]$Port = 'COM6'
)

$ErrorActionPreference = 'Stop'
$projectDir = $PSScriptRoot
$sketchDir = Join-Path $projectDir 'gate12_direct5_safe'

$cli = Get-Command arduino-cli -ErrorAction Stop
$visiblePorts = & $cli.Source board list
if ($LASTEXITCODE -ne 0) {
    throw 'Не удалось получить список плат Arduino.'
}
if ($visiblePorts -notmatch [regex]::Escape($Port)) {
    throw "Порт $Port не найден. Подключи ESP32 по USB, но питание серв пока не включай."
}

Write-Host 'Компиляция безопасной прошивки v2...'
& $cli.Source compile --fqbn esp32:esp32:esp32 --warnings all --export-binaries $sketchDir
if ($LASTEXITCODE -ne 0) {
    throw 'Компиляция завершилась ошибкой; загрузка отменена.'
}

Write-Host "Загрузка в ESP32 через $Port..."
& $cli.Source upload --fqbn esp32:esp32:esp32 --port $Port $sketchDir
if ($LASTEXITCODE -ne 0) {
    throw 'Загрузка завершилась ошибкой. Питание серв не включать.'
}

Write-Host 'Прошивка загружена. Питание серв пока не включай: сначала проверь VERSION и INFO.'
