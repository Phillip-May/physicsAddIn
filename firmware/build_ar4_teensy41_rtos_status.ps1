$ErrorActionPreference = "Stop"

$arduinoCli = Join-Path $env:LOCALAPPDATA "Programs\ArduinoCLI\arduino-cli.exe"
$sketchDir = Join-Path $PSScriptRoot "AR4_teensy41_rtos_status"
$repoRoot = Split-Path -Parent $PSScriptRoot
$commonDir = Join-Path $repoRoot "Common"
$commonInclude = "-I$($commonDir -replace '\\', '/')"
$teensy41CppFlags = "-std=gnu++17 -fno-exceptions -fpermissive -fno-rtti -fno-threadsafe-statics -felide-constructors -Wno-error=narrowing -Wno-psabi -Wno-maybe-uninitialized $commonInclude"

if (-not (Test-Path $arduinoCli)) {
  throw "arduino-cli.exe not found at $arduinoCli"
}

# Same reason as Invoke-VsCmd in scripts/common.ps1: with $ErrorActionPreference set to Stop,
# PowerShell turns any native process writing to stderr into a terminating error. arduino-cli puts
# the compiler's own output there, so a single gcc note - not even a warning - aborted the build
# while the compile itself had succeeded. The exit code is what says whether it worked.
$previous = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
try {
  & $arduinoCli compile `
    --fqbn teensy:avr:teensy41 `
    --build-property "build.flags.cpp=$teensy41CppFlags" `
    --export-binaries `
    $sketchDir 2>&1 | ForEach-Object { Write-Host $_ }
  $compileExit = $LASTEXITCODE
} finally {
  $ErrorActionPreference = $previous
}

if ($compileExit -ne 0) {
  throw "arduino-cli compile failed with exit code $compileExit"
}
