param(
    [ValidateSet('build', 'flash', 'monitor', 'menuconfig', 'reconfigure', 'size')]
    [string]$Action = 'build',
    [string]$Port,
    [string]$BuildDir = 'build_windows',
    [string]$ToolsPath = $env:IDF_TOOLS_PATH
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot

# Some IDE terminals omit this variable; Python uses it to detect Windows tools.
if (-not $env:PROCESSOR_ARCHITECTURE) {
    $env:PROCESSOR_ARCHITECTURE = switch ([System.Runtime.InteropServices.RuntimeInformation]::OSArchitecture) {
        'X64' { 'AMD64' }
        'Arm64' { 'ARM64' }
        'X86' { 'x86' }
        default { throw 'Unsupported Windows CPU architecture.' }
    }
}

# Reuse the EIM installation without changing machine-wide environment/policy.
if (-not $env:IDF_PYTHON_ENV_PATH -or -not $env:IDF_PATH) {
    if (-not $ToolsPath) { $ToolsPath = 'C:\Espressif\tools' }
    $configPath = Join-Path $ToolsPath 'eim_idf.json'
    if (-not (Test-Path -LiteralPath $configPath)) {
        throw 'Open an ESP-IDF PowerShell terminal first, or supply -ToolsPath for your EIM installation.'
    }
    $config = Get-Content -Raw -LiteralPath $configPath | ConvertFrom-Json
    $installs = @($config.idfInstalled | Where-Object { $_.status -eq 'finished' })
    $install = $installs | Where-Object { $_.id -eq $config.idfSelectedId } | Select-Object -First 1
    if (-not $install -and $installs.Count -eq 1) { $install = $installs[0] }
    if (-not $install) { throw 'Select an ESP-IDF installation in EIM, then retry.' }
    $exports = & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $install.activationScript -e
    if ($LASTEXITCODE -ne 0) { throw 'Failed to read the EIM environment.' }
    foreach ($line in $exports) {
        if ($line -match '^([^=]+)=(.*)$') {
            $name = $matches[1]
            $value = $matches[2]
            if ($name -eq 'PATH') { $env:PATH = $value + ';' + $env:PATH }
            elseif ($name -ne 'SYSTEM_PATH') { Set-Item -Path "Env:$name" -Value $value }
        }
    }
}

$idfPython = Join-Path $env:IDF_PYTHON_ENV_PATH 'Scripts\python.exe'
$idfEntry = Join-Path $env:IDF_PATH 'tools\idf.py'
if (-not (Test-Path -LiteralPath $idfPython) -or -not (Test-Path -LiteralPath $idfEntry)) {
    throw 'ESP-IDF Python environment or idf.py is missing. Repair the ESP-IDF installation.'
}
if ($Action -in @('flash', 'monitor') -and -not $Port) {
    throw 'Specify the ESP32 serial port, for example -Port COM5.'
}
$idfArgs = @('-C', $projectRoot, '-B', $BuildDir)
if ($Port) { $idfArgs += @('-p', $Port) }
$idfArgs += $Action
& $idfPython $idfEntry @idfArgs
exit $LASTEXITCODE
