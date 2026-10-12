$ErrorActionPreference = 'Stop'

$modelDirectory = $PSScriptRoot
$projectDirectory = Split-Path -Parent $modelDirectory
$modelFolder = Split-Path -Leaf $modelDirectory
$htmlName = 'ОТКРОЙ_V10_ПРОФЕССИОНАЛЬНАЯ_3D.html'
$port = 8765

function Test-V10Page([int]$candidatePort) {
    $folderEscaped = [uri]::EscapeDataString($modelFolder)
    $fileEscaped = [uri]::EscapeDataString($htmlName)
    $candidateUrl = "http://127.0.0.1:$candidatePort/$folderEscaped/$fileEscaped"
    try {
        $response = Invoke-WebRequest -UseBasicParsing -Uri $candidateUrl -TimeoutSec 2
        return $response.StatusCode -eq 200
    }
    catch {
        return $false
    }
}

if (-not (Test-V10Page $port)) {
    $listener = Get-NetTCPConnection -LocalPort $port -State Listen -ErrorAction SilentlyContinue
    if ($listener) {
        $port = 8766
    }

    $python = Get-Command python.exe -ErrorAction SilentlyContinue
    if (-not $python) {
        $python = Get-Command py.exe -ErrorAction SilentlyContinue
    }
    if (-not $python) {
        throw 'Python не найден. Откройте HTML через уже запущенный локальный сервер.'
    }

    $arguments = @('-u', '-m', 'http.server', $port, '--bind', '127.0.0.1', '--directory', $projectDirectory)
    if ($python.Name -eq 'py.exe') {
        $arguments = @('-3') + $arguments
    }
    Start-Process -FilePath $python.Source -ArgumentList $arguments -WindowStyle Hidden

    $ready = $false
    for ($attempt = 0; $attempt -lt 20; $attempt++) {
        Start-Sleep -Milliseconds 200
        if (Test-V10Page $port) {
            $ready = $true
            break
        }
    }
    if (-not $ready) {
        throw "Локальный сервер V10 не запустился на порту $port."
    }
}

$folderEscaped = [uri]::EscapeDataString($modelFolder)
$fileEscaped = [uri]::EscapeDataString($htmlName)
$url = "http://127.0.0.1:$port/$folderEscaped/$fileEscaped"
$edgeCandidates = @(
    "${env:ProgramFiles(x86)}\Microsoft\Edge\Application\msedge.exe",
    "$env:ProgramFiles\Microsoft\Edge\Application\msedge.exe"
)
$edge = $edgeCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (-not $edge) {
    throw 'Microsoft Edge не найден.'
}

$viewerProfile = Join-Path $env:TEMP 'V10_Edge_Viewer_Profile'
Start-Process -FilePath $edge -ArgumentList @(
    "--user-data-dir=$viewerProfile",
    '--no-first-run',
    '--disable-features=msEdgeFirstRunExperience',
    '--start-maximized',
    '--new-window',
    $url
)
