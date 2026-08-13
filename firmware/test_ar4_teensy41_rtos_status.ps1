param(
  [string]$ComPort = "",
  [int]$TimeoutMs = 8000,
  # The repo's one canonical simulator output; see BUILDING.md.
  [string]$SimulatorExe = (Join-Path (Split-Path -Parent $PSScriptRoot) "dist\RobotSimulator\release\RobotSimulator.exe"),
  [int]$ProcessTimeoutSeconds = 15
)

$ErrorActionPreference = "Stop"

. (Join-Path $PSScriptRoot "teensy_script_helpers.ps1")

if (-not (Test-Path $SimulatorExe)) {
  throw "RobotSimulator.exe not found at $SimulatorExe"
}

Stop-Ar4ToolProcesses
Start-Sleep -Seconds 3

if (-not $ComPort) {
  $detected = Get-TeensyPortInfo
  $ComPort = $detected.ComPort
  Write-Host "Detected Teensy: $($detected.Raw)"
}

$result = Invoke-ProcessWithTimeout `
  -FilePath $SimulatorExe `
  -ArgumentList @("--hardware-io-status", $ComPort, "$TimeoutMs") `
  -TimeoutSeconds $ProcessTimeoutSeconds `
  -WorkingDirectory (Split-Path -Parent $SimulatorExe)

Write-Host $result.StdOut
if ($result.StdErr) {
  Write-Host $result.StdErr
}

if ($result.Combined -notmatch "robot_status") {
  throw "Hardware IO status check did not return robot_status."
}

if ($result.ExitCode -ne 0) {
  throw "Hardware IO status check failed with exit code $($result.ExitCode)."
}
