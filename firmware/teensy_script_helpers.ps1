$ErrorActionPreference = "Stop"

function Get-Ar4ToolProcesses {
  param(
    [switch]$IncludeRobotSimulator
  )

  Get-CimInstance Win32_Process |
    Where-Object {
      $_.Name -match '^(arduino-cli|teensy|teensy_post_compile|teensy_reboot|teensy-discovery)\.exe$' -or
      ($_.Name -eq 'RobotSimulator.exe' -and ($IncludeRobotSimulator -or $_.CommandLine -match 'hardware-io-status|hardware-io-monitor'))
    }
}

function Stop-Ar4ToolProcesses {
  param(
    [switch]$IncludeRobotSimulator
  )

  $processes = Get-Ar4ToolProcesses -IncludeRobotSimulator:$IncludeRobotSimulator | Where-Object {
    $IncludeRobotSimulator -or
    $_.Name -ne 'RobotSimulator.exe' -or
    $_.CommandLine -match 'hardware-io-status|hardware-io-monitor'
  }

  foreach ($process in $processes) {
    try {
      & taskkill.exe /PID $process.ProcessId /T /F | Out-Null
    } catch {
      Write-Warning "Failed to stop $($process.Name) pid=$($process.ProcessId): $($_.Exception.Message)"
    }
  }
}

function Test-SerialPortAvailable {
  param(
    [Parameter(Mandatory = $true)]
    [string]$ComPort
  )

  $serial = [System.IO.Ports.SerialPort]::new($ComPort, 115200)
  try {
    $serial.Open()
    return $true
  } finally {
    if ($serial.IsOpen) {
      $serial.Close()
    }
  }
}

function Wait-SerialPortAvailable {
  param(
    [Parameter(Mandatory = $true)]
    [string]$ComPort,

    [int]$TimeoutSeconds = 15
  )

  $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
  $lastError = $null
  while ([DateTime]::UtcNow -lt $deadline) {
    try {
      [void](Test-SerialPortAvailable -ComPort $ComPort)
      return
    } catch {
      $lastError = $_.Exception.Message
      Start-Sleep -Milliseconds 500
    }
  }

  throw "$ComPort did not become available within $TimeoutSeconds seconds. Last error: $lastError"
}

function Get-TeensyPortInfo {
  $teensyPorts = Join-Path $env:LOCALAPPDATA "Arduino15\packages\teensy\tools\teensy-tools\1.62.0\teensy_ports.exe"
  if (-not (Test-Path $teensyPorts)) {
    throw "teensy_ports.exe not found at $teensyPorts"
  }

  $lines = & $teensyPorts -L
  foreach ($line in $lines) {
    if ($line -match '^(?<upload>\S+)\s+(?<com>COM\d+)\s+\(Teensy\s+4\.1\)') {
      return [pscustomobject]@{
        UploadPort = $Matches.upload
        ComPort = $Matches.com
        Raw = $line
      }
    }
  }

  throw "No Teensy 4.1 serial port found. teensy_ports output:`n$($lines -join [Environment]::NewLine)"
}

function Stop-ProcessTree {
  param(
    [Parameter(Mandatory = $true)]
    [int]$ProcessId
  )

  $children = Get-CimInstance Win32_Process |
    Where-Object { $_.ParentProcessId -eq $ProcessId }

  foreach ($child in $children) {
    Stop-ProcessTree -ProcessId $child.ProcessId
  }

  try {
    Stop-Process -Id $ProcessId -Force -ErrorAction Stop
  } catch {
  }
}

function Invoke-ProcessWithTimeout {
  param(
    [Parameter(Mandatory = $true)]
    [string]$FilePath,

    [Parameter(Mandatory = $true)]
    [string[]]$ArgumentList,

    [int]$TimeoutSeconds = 60,

    [string]$WorkingDirectory = (Get-Location).Path
  )

  $stdout = [System.IO.Path]::GetTempFileName()
  $stderr = [System.IO.Path]::GetTempFileName()
  $process = $null
  try {
    $process = Start-Process -FilePath $FilePath `
      -ArgumentList $ArgumentList `
      -WorkingDirectory $WorkingDirectory `
      -NoNewWindow `
      -PassThru `
      -RedirectStandardOutput $stdout `
      -RedirectStandardError $stderr

    if (-not $process.WaitForExit($TimeoutSeconds * 1000)) {
      & taskkill.exe /PID $process.Id /T /F | Out-Null
      Stop-Ar4ToolProcesses
      throw "Timed out after $TimeoutSeconds seconds: $FilePath $($ArgumentList -join ' ')"
    }
    $process.Refresh()

    $outText = (Get-Content -Raw -ErrorAction SilentlyContinue $stdout)
    $errText = (Get-Content -Raw -ErrorAction SilentlyContinue $stderr)
    $combined = ($outText + $errText)
    $exitCode = if ($null -eq $process.ExitCode) { 0 } else { $process.ExitCode }
    [pscustomobject]@{
      ExitCode = $exitCode
      StdOut = $outText
      StdErr = $errText
      Combined = $combined
    }
  } finally {
    Remove-Item -LiteralPath $stdout, $stderr -Force -ErrorAction SilentlyContinue
  }
}
