<#
.SYNOPSIS
    Runs the conveyor characterization traces and compares them with the committed goldens.

.DESCRIPTION
    These traces are the integration coverage for conveyor transport over complete station files.
    build_motion_smokes.ps1 complements them with direct conveyor-core and scenery checks. This is
    the net under any change to the station-level rules.

    Logical mode is goldened byte for byte, because it is pure arithmetic over the station file.
    PhysX mode is not - a solver's per-machine timing decides how a box tumbles - so the physical
    cases only assert the invariants the trace command checks itself, and the command's exit code
    is the whole result there.

    The fixture case is the important one. Neither shipped station is a controlled test: the
    showcase has no maxActiveSpawns, and both are large enough that a rule change reads as noise.
    tests/robot_simulator/fixtures/conveyor_rules.station.json exists to reach rule 12 - a
    disconnected downstream interface stops a product instead of deleting it - and to hold a
    spawn cap closed with the products that stop there.

.EXAMPLE
    .\scripts\check_conveyor_traces.ps1
    .\scripts\check_conveyor_traces.ps1 -Update      # after an intended behaviour change
#>
param(
    [string]$Exe,
    # Rewrite the goldens from this build. Only ever correct when the trace changed because the
    # rules were meant to change; a port is supposed to leave every byte alone.
    [switch]$Update
)

. (Join-Path $PSScriptRoot 'common.ps1')

if (-not $Exe) {
    $Exe = Join-Path $RepoRoot 'dist\RobotSimulator\release\RobotSimulator.exe'
}
Assert-File $Exe 'RobotSimulator has not been built. Run scripts\build_robot_simulator.ps1 first.'

$goldenDir = Join-Path $RepoRoot 'tests\robot_simulator\golden\conveyor'
New-Item -ItemType Directory -Force -Path $goldenDir | Out-Null

$fixture = Join-Path $RepoRoot 'tests\robot_simulator\fixtures\conveyor_rules.station.json'
$showcase = Join-Path $RepoRoot 'examples\stations\conveyor_showcase.station.json'
$tending = Join-Path $RepoRoot 'examples\stations\fairino_gudel_machine_tending.station.json'

# Golden = byte-compared. Durations are chosen so each case reaches the rule it exists for: the
# fixture must run long enough for three products to stop at the dead end and hold the cap closed,
# and the two shipped stations long enough to transfer and to delete.
$cases = @(
    @{ Name = 'fixture_logical';  Station = $fixture;  Seconds = '8';  Mode = 'logical'; Every = '1'; Golden = $true }
    @{ Name = 'showcase_logical'; Station = $showcase; Seconds = '12'; Mode = 'logical'; Every = '6'; Golden = $true }
    @{ Name = 'tending_logical';  Station = $tending;  Seconds = '20'; Mode = 'logical'; Every = '6'; Golden = $true }
    @{ Name = 'fixture_physx';    Station = $fixture;  Seconds = '8';  Mode = 'physx';   Every = '6'; Golden = $false }
    @{ Name = 'showcase_physx';   Station = $showcase; Seconds = '12'; Mode = 'physx';   Every = '6'; Golden = $false }
    @{ Name = 'tending_physx';    Station = $tending;  Seconds = '20'; Mode = 'physx';   Every = '6'; Golden = $false }
)

$scratch = Join-Path ([System.IO.Path]::GetTempPath()) ('conveyor-trace-{0}' -f ([Guid]::NewGuid().ToString('N')))
New-Item -ItemType Directory -Force -Path $scratch | Out-Null

$failed = @()
$updated = 0
try {
    foreach ($case in $cases) {
        Assert-File $case.Station 'Trace station is missing.'
        $actual = Join-Path $scratch ("{0}.trace" -f $case.Name)
        # cmd's redirection, not PowerShell's: `>` here transcodes to UTF-16 and the capture would
        # not be the bytes the program wrote.
        $line = '"' + $Exe + '" --dump-conveyor-trace "' + $case.Station + '" ' + $case.Seconds +
            ' ' + $case.Mode + ' ' + $case.Every + ' > "' + $actual + '" 2>&1'
        & $env:ComSpec /c $line | Out-Null
        $exit = $LASTEXITCODE

        if ($exit -ne 0) {
            $failed += ("{0}: trace command exited {1}" -f $case.Name, $exit)
            Get-Content -LiteralPath $actual -Tail 10 | ForEach-Object { Write-Host "    $_" }
            continue
        }

        if (-not $case.Golden) {
            Write-Host ("{0,-18} invariants held ({1} bytes, not goldened)" -f $case.Name, (Get-Item -LiteralPath $actual).Length)
            continue
        }

        $golden = Join-Path $goldenDir ("{0}.trace" -f $case.Name)
        if ($Update -or -not (Test-Path -LiteralPath $golden)) {
            Copy-Item -LiteralPath $actual -Destination $golden -Force
            $updated++
            Write-Host ("{0,-18} golden written ({1} bytes)" -f $case.Name, (Get-Item -LiteralPath $golden).Length)
            continue
        }

        $actualHash = (Get-FileHash -LiteralPath $actual -Algorithm SHA256).Hash
        $goldenHash = (Get-FileHash -LiteralPath $golden -Algorithm SHA256).Hash
        if ($actualHash -eq $goldenHash) {
            Write-Host ("{0,-18} matches golden ({1} bytes)" -f $case.Name, (Get-Item -LiteralPath $golden).Length)
            continue
        }

        $failed += ("{0}: trace differs from golden" -f $case.Name)
        Write-Host ("DIFFERS: {0}" -f $case.Name) -ForegroundColor Red
        $diff = Compare-Object (Get-Content -LiteralPath $golden) (Get-Content -LiteralPath $actual) |
            Select-Object -First 10
        foreach ($entry in $diff) { Write-Host ("    {0} {1}" -f $entry.SideIndicator, $entry.InputObject) }
        Copy-Item -LiteralPath $actual -Destination (Join-Path $goldenDir ("{0}.actual" -f $case.Name)) -Force
        Write-Host ("    full output: {0}" -f (Join-Path $goldenDir ("{0}.actual" -f $case.Name)))
    }
} finally {
    Remove-Item -LiteralPath $scratch -Recurse -Force -ErrorAction SilentlyContinue
}

Write-Host ""
if ($failed.Count -gt 0) {
    throw ("Conveyor trace failures:`n  " + ($failed -join "`n  "))
}
if ($updated -gt 0) {
    Write-Host ("Conveyor traces: {0} golden(s) written, {1} case(s) checked." -f $updated, $cases.Count)
} else {
    Write-Host ("PASS: all {0} conveyor trace case(s) held." -f $cases.Count)
}
