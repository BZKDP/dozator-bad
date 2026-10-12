$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 .\gate12_direct5_full_app.py
} elseif (Get-Command python -ErrorAction SilentlyContinue) {
    & python .\gate12_direct5_full_app.py
} else {
    throw 'Python 3 не найден.'
}
