$ErrorActionPreference = "Stop"

$projectRoot = $PSScriptRoot
$serverRoot = Split-Path -Parent $projectRoot
$port = 8765
$folderSegment = [Uri]::EscapeDataString((Split-Path -Leaf $projectRoot))
$fileSegment = [Uri]::EscapeDataString("ОТКРОЙ_V11_ТРИ_ВАРИАНТА.html")
$url = "http://127.0.0.1:$port/$folderSegment/$fileSegment"

function Test-V11Server {
    try {
        $response = Invoke-WebRequest -Uri $url -UseBasicParsing -TimeoutSec 1
        return $response.StatusCode -eq 200 -and $response.Content.Contains("V11 · три простых механизма")
    }
    catch {
        return $false
    }
}

if (-not (Test-V11Server)) {
    $python = (Get-Command python.exe -ErrorAction Stop).Source
    $serverArgs = @("-m", "http.server", "$port", "--bind", "127.0.0.1", "--directory", $serverRoot)
    Start-Process -FilePath $python -ArgumentList $serverArgs -WorkingDirectory $serverRoot -WindowStyle Hidden

    $ready = $false
    for ($attempt = 0; $attempt -lt 30; $attempt++) {
        Start-Sleep -Milliseconds 200
        if (Test-V11Server) {
            $ready = $true
            break
        }
    }
    if (-not $ready) {
        throw "Локальный сервер V11 не запустился на порту $port."
    }
}

$programFilesX86 = [Environment]::GetFolderPath("ProgramFilesX86")
$edgeCandidates = @(
    (Join-Path $programFilesX86 "Microsoft\Edge\Application\msedge.exe"),
    (Join-Path $env:ProgramFiles "Microsoft\Edge\Application\msedge.exe")
)
$edge = $edgeCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (-not $edge) {
    throw "Microsoft Edge не найден."
}

$profile = Join-Path ([IO.Path]::GetTempPath()) "edge-v11-dose-viewer"
$edgeArgs = @(
    "--user-data-dir=$profile",
    "--no-first-run",
    "--disable-features=msEdgeFirstRunExperience",
    "--start-maximized",
    "--new-window",
    $url
)
Start-Process -FilePath $edge -ArgumentList $edgeArgs
