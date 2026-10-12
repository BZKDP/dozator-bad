$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

if (Get-Command py -ErrorAction SilentlyContinue) {
    & py .\gate12_5servo_app.py
} elseif (Get-Command python -ErrorAction SilentlyContinue) {
    & python .\gate12_5servo_app.py
} else {
    throw 'Python не найден. Установи Python 3 и повтори запуск.'
}

