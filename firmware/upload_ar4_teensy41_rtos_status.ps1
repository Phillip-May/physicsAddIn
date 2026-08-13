param(
  [string]$Port = "",
  [string]$ComPort = "",
  [int]$TimeoutSeconds = 240,
  [switch]$CloseRobotSimulator
)

$ErrorActionPreference = "Stop"

. (Join-Path $PSScriptRoot "teensy_script_helpers.ps1")

$arduinoCli = Join-Path $env:LOCALAPPDATA "Programs\ArduinoCLI\arduino-cli.exe"
$sketchDir = Join-Path $PSScriptRoot "AR4_teensy41_rtos_status"
$buildScript = Join-Path $PSScriptRoot "build_ar4_teensy41_rtos_status.ps1"

if (-not (Test-Path $arduinoCli)) {
  throw "arduino-cli.exe not found at $arduinoCli"
}

Stop-Ar4ToolProcesses -IncludeRobotSimulator:$CloseRobotSimulator
Start-Sleep -Milliseconds 500

if (-not $Port -or -not $ComPort) {
  $detected = Get-TeensyPortInfo
  if (-not $Port) { $Port = $detected.UploadPort }
  if (-not $ComPort) { $ComPort = $detected.ComPort }
  Write-Host "Detected Teensy: $($detected.Raw)"
}

try {
  Wait-SerialPortAvailable -ComPort $ComPort -TimeoutSeconds 15
} catch {
  $owners = Get-CimInstance Win32_Process |
    Where-Object { $_.Name -match '^(arduino-cli|teensy|teensy_post_compile|teensy_reboot|teensy-discovery|RobotSimulator)\.exe$' } |
    Select-Object ProcessId, Name, CommandLine |
    Format-List |
    Out-String
  if ([string]::IsNullOrWhiteSpace($owners)) {
    $owners = "No Arduino/Teensy/RobotSimulator owner was found. Unplug/replug the Teensy, then rerun this script."
  }
  throw "$ComPort is not available before upload: $($_.Exception.Message)`nCandidate tool processes:`n$owners"
}

$compileResult = Invoke-ProcessWithTimeout `
  -FilePath "powershell" `
  -ArgumentList @(
    "-NoProfile",
    "-ExecutionPolicy",
    "Bypass",
    "-File",
    $buildScript
  ) `
  -TimeoutSeconds $TimeoutSeconds `
  -WorkingDirectory (Get-Location).Path

Write-Host $compileResult.StdOut
if ($compileResult.StdErr) {
  Write-Host $compileResult.StdErr
}

if ($compileResult.ExitCode -ne 0 -or $compileResult.Combined -match "Error during build|fatal error|unknown flag|unknown shorthand flag") {
  Stop-Ar4ToolProcesses
  throw "Teensy compile failed before upload."
}

$hexPath = Join-Path $sketchDir "build\teensy.avr.teensy41\AR4_teensy41_rtos_status.ino.hex"
if (-not (Test-Path $hexPath)) {
  throw "Compiled Teensy hex was not found at $hexPath"
}

$result = Invoke-ProcessWithTimeout `
  -FilePath $arduinoCli `
  -ArgumentList @(
    "upload",
    "--verbose",
    "-p",
    $Port,
    "--protocol",
    "teensy",
    "--fqbn",
    "teensy:avr:teensy41",
    "--input-file",
    $hexPath
  ) `
  -TimeoutSeconds $TimeoutSeconds `
  -WorkingDirectory (Get-Location).Path

Write-Host $result.StdOut
if ($result.StdErr) {
  Write-Host $result.StdErr
}

$failurePatterns = @(
  "Unable to open",
  "Teensy did not respond",
  "Please press the PROGRAM MODE BUTTON",
  "Failed uploading",
  "uploading error",
  "unknown flag",
  "unknown shorthand flag",
  "exit status 1"
)

foreach ($pattern in $failurePatterns) {
  if ($result.Combined -match [regex]::Escape($pattern)) {
    Stop-Ar4ToolProcesses
    throw "Teensy upload failed: saw '$pattern' in tool output."
  }
}

if ($result.ExitCode -ne 0) {
  Stop-Ar4ToolProcesses
  throw "Teensy upload failed with exit code $($result.ExitCode)."
}

Stop-Ar4ToolProcesses
Write-Host "Teensy upload command completed without known failure output: $hexPath"
