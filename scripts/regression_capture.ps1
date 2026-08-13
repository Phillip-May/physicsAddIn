<#
.SYNOPSIS
    Captures RobotSimulator's deterministic CLI output for before/after comparison.

.DESCRIPTION
    Golden-output gate: runs the CLI paths whose output depends only on their inputs,
    writes each one to its own file, and can diff two captures byte-for-byte to catch
    motion regressions.

    Deliberately excluded: --collision-pose, which prints wall-clock pose_ms / ik_ms /
    collision_ms and therefore cannot byte-match across runs.

.EXAMPLE
    # Before touching anything
    .\scripts\regression_capture.ps1 -Exe <path>\RobotSimulator.exe -OutDir baseline

    # After a phase
    .\scripts\regression_capture.ps1 -Exe <path>\RobotSimulator.exe -OutDir after
    .\scripts\regression_capture.ps1 -Compare baseline, after
#>
[CmdletBinding(DefaultParameterSetName = 'Capture')]
param(
    [Parameter(ParameterSetName = 'Capture', Mandatory = $true)]
    [string]$Exe,

    [Parameter(ParameterSetName = 'Capture', Mandatory = $true)]
    [string]$OutDir,

    [Parameter(ParameterSetName = 'Compare', Mandatory = $true)]
    [string[]]$Compare
)

. (Join-Path $PSScriptRoot 'common.ps1')

$libraryPackages = Join-Path $RepoRoot 'library\packages'
$fixtures = Join-Path $RepoRoot 'tests\robot_simulator\fixtures'
$package = Join-Path $libraryPackages 'ar4_6dof_robot.zip'
$smallProgram = Join-Path $fixtures 'test.robotprog.txt'
# Weaving cases carry the config, because the planner limits it overrides - the per-joint dynamic
# limit scaling in particular - change what is feasible, and capturing against the package defaults
# would compare the planner to settings nobody runs.
$mastering = Join-Path $fixtures 'ar4_mastering.json'
$weaveCorners = Join-Path $fixtures 'ar4_rect_weave_corners.robotprog.txt'
$weaveDemo = Join-Path $fixtures 'ar4_weave_demo.robotprog.txt'
$rectProgram = Join-Path $fixtures 'ar4_rect_regen_100.robotprog.txt'
# A long program exercises blending and the corner-speed capping the short one never reaches.
$largeProgram = $rectProgram

function Compare-Captures {
    param([string]$Left, [string]$Right)

    if (-not (Test-Path -LiteralPath $Left))  { throw "Capture folder not found: $Left" }
    if (-not (Test-Path -LiteralPath $Right)) { throw "Capture folder not found: $Right" }

    $leftFiles  = Get-ChildItem -LiteralPath $Left  -File | Sort-Object Name
    $rightFiles = Get-ChildItem -LiteralPath $Right -File | Sort-Object Name
    $names = ($leftFiles.Name + $rightFiles.Name) | Sort-Object -Unique

    $differing = 0
    foreach ($name in $names) {
        $l = Join-Path $Left $name
        $r = Join-Path $Right $name
        if (-not (Test-Path -LiteralPath $l)) { Write-Host "ONLY IN $Right : $name" -ForegroundColor Yellow; $differing++; continue }
        if (-not (Test-Path -LiteralPath $r)) { Write-Host "ONLY IN $Left  : $name" -ForegroundColor Yellow; $differing++; continue }

        # Byte comparison, not line based: trailing whitespace and newline changes matter.
        $lh = (Get-FileHash -LiteralPath $l -Algorithm SHA256).Hash
        $rh = (Get-FileHash -LiteralPath $r -Algorithm SHA256).Hash
        if ($lh -ne $rh) {
            $differing++
            Write-Host "DIFFERS: $name" -ForegroundColor Red
            $diff = Compare-Object (Get-Content -LiteralPath $l) (Get-Content -LiteralPath $r) |
                    Select-Object -First 6
            foreach ($d in $diff) { Write-Host ("    {0} {1}" -f $d.SideIndicator, $d.InputObject) }
        }
    }

    if ($differing -eq 0) {
        Write-Host "PASS: all $($names.Count) captures byte-identical" -ForegroundColor Green
        return 0
    }
    Write-Host "FAIL: $differing of $($names.Count) captures differ" -ForegroundColor Red
    return 1
}

if ($PSCmdlet.ParameterSetName -eq 'Compare') {
    if ($Compare.Count -ne 2) { throw 'Compare takes exactly two folders.' }
    exit (Compare-Captures -Left $Compare[0] -Right $Compare[1])
}

if (-not (Test-Path -LiteralPath $Exe)) { throw "Executable not found: $Exe" }
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$OutDir = (Resolve-Path -LiteralPath $OutDir).Path

# PowerShell's own > operator transcodes to UTF-16, so it would not capture the bytes
# the program actually wrote. cmd's redirection passes them through untouched.
function Invoke-Capture {
    param([string]$Target, [string[]]$Arguments)

    $quoted = ($Arguments | ForEach-Object { '"' + $_ + '"' }) -join ' '
    $line = '"' + $Exe + '" ' + $quoted + ' > "' + $Target + '" 2>&1'
    & $env:ComSpec /c $line | Out-Null
    $exitCode = $LASTEXITCODE
    Normalize-Capture -Path $Target
    if ($exitCode -ne 0) {
        throw "Capture command failed with exit code $exitCode. See $Target"
    }
}

# Several commands echo their own output paths, so the capture folder name would leak
# into the text and every capture would differ from every other. Replace the volatile
# paths with stable placeholders so only real behaviour changes show up as a diff.
function Normalize-Capture {
    param([string]$Path)

    if (-not (Test-Path -LiteralPath $Path)) { return }
    if ([System.IO.Path]::GetExtension($Path) -eq '.csv') { return }  # never contains paths

    $text = [System.IO.File]::ReadAllText($Path)
    foreach ($pair in @(@($OutDir, '<OUTDIR>'), @($RepoRoot, '<REPO>'), @($env:USERPROFILE, '<HOME>'))) {
        if ([string]::IsNullOrEmpty($pair[0])) { continue }
        $text = [System.Text.RegularExpressions.Regex]::Replace(
            $text, [System.Text.RegularExpressions.Regex]::Escape($pair[0]), $pair[1],
            [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)
    }
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($Path, $text, $utf8NoBom)
}

# Each entry writes one capture file. Stderr is folded in so error text is compared too.
$cases = @(
    @{ Name = 'simulate_small';        Args = @('--simulate-program', $package, $smallProgram) }
    @{ Name = 'simulate_small_blend';  Args = @('--simulate-program', $package, $smallProgram, '--blend-mm', '5', '--linear-mm-s', '250') }
    @{ Name = 'dump_axes';             Args = @('--dump-axes', $package) }
    @{ Name = 'ik_smoke';              Args = @('--ik-smoke', $package) }
    @{ Name = 'validate_package';      Args = @('--validate-package', $package) }
)

# Weaving, against the config's own limits. These capture worst_window_seam_mm_s and the durations,
# which between them are what a regression in the weave or the lookahead seam actually shows up as:
# a run that used to plan and now reports a reject code, or a seam that used to meet and now does not.
foreach ($weaveCase in @(
    @{ Name = 'simulate_weave_corners'; Program = $weaveCorners },
    @{ Name = 'simulate_weave_demo';    Program = $weaveDemo },
    @{ Name = 'simulate_rect_mastered'; Program = $rectProgram }
)) {
    if ((Test-Path -LiteralPath $weaveCase.Program) -and (Test-Path -LiteralPath $mastering)) {
        $cases += @{
            Name = $weaveCase.Name
            Args = @('--simulate-program', $package, $weaveCase.Program, '--mastering', $mastering)
        }
    } else {
        Write-Host ("NOTE: {0} missing - skipping {1}" -f $weaveCase.Program, $weaveCase.Name) -ForegroundColor Yellow
    }
}

if (Test-Path -LiteralPath $largeProgram) {
    $cases += @{ Name = 'simulate_large'; Args = @('--simulate-program', $package, $largeProgram, '--blend-mm', '0', '--linear-mm-s', '100') }
} else {
    Write-Host "NOTE: $largeProgram missing - skipping the long-program cases" -ForegroundColor Yellow
}

foreach ($case in $cases) {
    $target = Join-Path $OutDir ("{0}.txt" -f $case.Name)
    Invoke-Capture -Target $target -Arguments $case.Args
    Write-Host ("captured {0,-24} ({1} bytes)" -f $case.Name, (Get-Item -LiteralPath $target).Length)
}

# The per-sample CSV is the most sensitive artefact here: ten 'g',17 columns a row.
$csv = Join-Path $OutDir 'trajectory_small.csv'
Invoke-Capture -Target (Join-Path $OutDir 'simulate_small_dump.txt') `
    -Arguments @('--simulate-program', $package, $smallProgram, '--dump-trajectory', $csv)
if (Test-Path -LiteralPath $csv) {
    Write-Host ("captured {0,-24} ({1} bytes)" -f 'trajectory_small.csv', (Get-Item -LiteralPath $csv).Length)
}

if (Test-Path -LiteralPath $largeProgram) {
    $csvLarge = Join-Path $OutDir 'trajectory_large.csv'
    Invoke-Capture -Target (Join-Path $OutDir 'simulate_large_dump.txt') `
        -Arguments @('--simulate-program', $package, $largeProgram, '--blend-mm', '0', '--linear-mm-s', '100', '--dump-trajectory', $csvLarge)
    if (Test-Path -LiteralPath $csvLarge) {
        Write-Host ("captured {0,-24} ({1} bytes)" -f 'trajectory_large.csv', (Get-Item -LiteralPath $csvLarge).Length)
    }
}

$generated = Join-Path $OutDir 'generated_rect.robotprog.txt'
Invoke-Capture -Target (Join-Path $OutDir 'generate_rect.txt') `
    -Arguments @('--generate-rect-program', $package, $smallProgram, $generated, '--points', '24', '--passes', '2')
if (Test-Path -LiteralPath $generated) {
    Normalize-Capture -Path $generated
    Write-Host ("captured {0,-24} ({1} bytes)" -f 'generated_rect.robotprog.txt', (Get-Item -LiteralPath $generated).Length)
    # Feeding the generated program back through the planner catches a parser change
    # that happens to mirror a writer change and would otherwise cancel out.
    Invoke-Capture -Target (Join-Path $OutDir 'simulate_generated.txt') `
        -Arguments @('--simulate-program', $package, $generated)
    Write-Host ("captured {0,-24}" -f 'simulate_generated')
}

Write-Host ""
Write-Host "Capture written to $OutDir" -ForegroundColor Green
