Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$Script:RepoRoot = Split-Path -Parent $PSScriptRoot
$RepoRoot = $Script:RepoRoot

function Convert-QMakePath {
    param([Parameter(Mandatory = $true)][string]$Path)
    # Both qmake spellings, $$REPO_ROOT and $${REPO_ROOT}, appear in local-env.pri files.
    return ($Path -replace '\$\$\{?REPO_ROOT\}?', $Script:RepoRoot) -replace '/', '\'
}

function Get-DependencyConfig {
    $config = @{
        QtRoot = 'C:\Qt\5.15.2\msvc2019_64'
        OcctRoot = 'C:\OpenCASCADE-7.6.0-vc14-64\opencascade-7.6.0'
        PhysXRoot = 'C:\PhysX-110.1-omni-and-physx-5.9.0\physx'
        PhysXTag = '110.1-omni-and-physx-5.9.0'
        PhysXPlatform = 'win.x86_64.vc142.md'
        CoacdRoot = Join-Path $Script:RepoRoot 'external\CoACD'
        EmsdkRoot = Join-Path $env:USERPROFILE 'emsdk'
        # Where the RoboDK plugin is installed to be loaded. Read by scripts only; qmake never sees
        # it, because the build writes to its own directory and deployment is a separate step.
        RoboDkRoot = 'C:\RoboDK'
    }

    $localEnv = Join-Path $Script:RepoRoot 'build\local-env.pri'
    if (Test-Path $localEnv) {
        foreach ($line in Get-Content $localEnv) {
            if ($line -match '^\s*([A-Za-z0-9_]+)\s*=\s*(.+?)\s*$') {
                $key = $matches[1]
                $value = Convert-QMakePath $matches[2]
                switch ($key) {
                    'QT_ROOT' { $config.QtRoot = $value }
                    'OCCT_ROOT' { $config.OcctRoot = $value }
                    'PHYSX_ROOT' { $config.PhysXRoot = $value }
                    'PHYSX_PLATFORM' { $config.PhysXPlatform = $value }
                    'COACD_ROOT' { $config.CoacdRoot = $value }
                    'EMSDK_ROOT' { $config.EmsdkRoot = $value }
                    'ROBODK_ROOT' { $config.RoboDkRoot = $value }
                }
            }
        }
    }

    return $config
}

function Assert-File {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$Message
    )
    if (-not (Test-Path $Path -PathType Leaf)) {
        throw "$Message Missing file: $Path"
    }
}

function Assert-Directory {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][string]$Message
    )
    if (-not (Test-Path $Path -PathType Container)) {
        throw "$Message Missing directory: $Path"
    }
}

function Get-VcVars64Path {
    $vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (Test-Path $vswhere) {
        $installPath = & $vswhere -products * -version '[16.0,17.0)' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath | Select-Object -First 1
        if (-not $installPath) {
            $installPath = & $vswhere -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath | Select-Object -First 1
        }
        if ($installPath) {
            $candidate = Join-Path $installPath 'VC\Auxiliary\Build\vcvars64.bat'
            if (Test-Path $candidate) {
                return $candidate
            }
        }
    }

    $fallbacks = @(
        'C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat',
        'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
    )
    foreach ($candidate in $fallbacks) {
        if (Test-Path $candidate) {
            return $candidate
        }
    }

    throw 'Visual Studio C++ vcvars64.bat was not found. Install VS 2019 or VS 2022 with Desktop development with C++.'
}

function Invoke-VsCmd {
    param(
        [Parameter(Mandatory = $true)][string[]]$Commands,
        [string]$WorkingDirectory = $Script:RepoRoot
    )

    $vcvars = Get-VcVars64Path
    $bat = Join-Path ([System.IO.Path]::GetTempPath()) ('physicsAddIn-build-{0}.bat' -f ([Guid]::NewGuid().ToString('N')))
    $content = @(
        '@echo off',
        ('call "{0}"' -f $vcvars),
        'if errorlevel 1 exit /b %errorlevel%'
    ) + $Commands + @('exit /b %errorlevel%')

    Set-Content -Path $bat -Value $content -Encoding ASCII
    try {
        Push-Location $WorkingDirectory
        # vcvars64.bat may write non-fatal diagnostics to stderr; use its exit code as the result.
        $previous = $ErrorActionPreference
        $ErrorActionPreference = 'Continue'
        try {
            cmd.exe /d /s /c "call `"$bat`"" 2>&1 | ForEach-Object { Write-Host $_ }
        } finally {
            $ErrorActionPreference = $previous
        }
        if ($LASTEXITCODE -ne 0) {
            throw "Command failed with exit code $LASTEXITCODE"
        }
    } finally {
        Pop-Location
        Remove-Item $bat -Force -ErrorAction SilentlyContinue
    }
}

function Test-StandaloneDependencies {
    param([Parameter(Mandatory = $true)]$Config)

    Assert-File (Join-Path $Config.QtRoot 'bin\qmake.exe') 'Qt 5.15.2 MSVC2019 is not installed.'
    Assert-File (Join-Path $Config.OcctRoot 'inc\Standard.hxx') 'OpenCascade is not installed.'
    Assert-File (Join-Path $Config.PhysXRoot 'include\PxPhysicsAPI.h') 'PhysX is not installed.'
    Assert-File (Join-Path $Config.CoacdRoot 'public\coacd.h') 'CoACD is not installed.'
    Assert-File (Join-Path $Config.CoacdRoot 'buildMD\Release\coacd.lib') 'CoACD has not been built.'
    Assert-File (Join-Path $Config.PhysXRoot ('bin\{0}\release\PhysX_64.lib' -f $Config.PhysXPlatform)) 'PhysX release libraries have not been built.'
}

function Invoke-NativeCommand {
    # Windows PowerShell wraps native stderr lines in NativeCommandError records, and with
    # ErrorActionPreference Stop an ordinary warning aborts the run. Merged and streamed here;
    # the exit code is the only failure signal.
    param(
        [Parameter(Mandatory = $true)][string]$Tool,
        [string[]]$Arguments = @(),
        [Parameter(Mandatory = $true)][string]$What,
        # Silence the tool and return its exit code, for probes whose failure is an answer.
        [switch]$IgnoreExitCode
    )
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        if ($IgnoreExitCode) {
            & $Tool @Arguments *>$null
        } else {
            & $Tool @Arguments 2>&1 | ForEach-Object { Write-Host $_ }
        }
    } finally { $ErrorActionPreference = $previous }
    if ($IgnoreExitCode) { return $LASTEXITCODE }
    if ($LASTEXITCODE -ne 0) { throw "$What failed with exit code $LASTEXITCODE" }
}

function Assert-FreshOutput {
    param(
        [Parameter(Mandatory = $true)][string]$Path,
        [Parameter(Mandatory = $true)][datetime]$Since,
        [Parameter(Mandatory = $true)][string]$What
    )
    Assert-File $Path "$What build completed but the output was not found."
    $info = Get-Item $Path
    if ($info.LastWriteTime -lt $Since.AddSeconds(-2)) {
        throw "$What build found a stale output instead of a fresh one: $Path"
    }
    return $info
}

function Invoke-QMakeBuild {
    param(
        [Parameter(Mandatory = $true)]$Config,
        [Parameter(Mandatory = $true)][string]$ProFile,
        [Parameter(Mandatory = $true)][ValidateSet('Release', 'Debug')][string]$Configuration,
        [Parameter(Mandatory = $true)][string]$BuildDir,
        # Extra qmake command-line assignments, e.g. 'DESTDIR=C:\out'. Each is quoted whole.
        [string[]]$QMakeAssignments = @()
    )
    $configLower = $Configuration.ToLowerInvariant()
    $removeConfig = if ($Configuration -eq 'Release') { 'debug' } else { 'release' }
    $qtBin = Join-Path $Config.QtRoot 'bin'
    $qmake = Join-Path $qtBin 'qmake.exe'
    $occtBin = Join-Path $Config.OcctRoot 'win64\vc14\bin'
    $physxBin = Join-Path $Config.PhysXRoot ('bin\{0}\{1}' -f $Config.PhysXPlatform, $configLower)
    New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
    $extra = ''
    foreach ($assignment in $QMakeAssignments) { $extra += (' "{0}"' -f $assignment) }
    $commands = @(
        ('set "PATH={0};{1};{2};%PATH%"' -f $qtBin, $occtBin, $physxBin),
        ('cd /d "{0}"' -f $BuildDir),
        ('"{0}" "{1}" -spec win32-msvc "CONFIG+={2}" "CONFIG-={3}" "CONFIG-=debug_and_release"{4}' -f $qmake, $ProFile, $configLower, $removeConfig, $extra),
        'nmake /NOLOGO'
    )
    Invoke-VsCmd -Commands $commands
}

function Get-BuiltinPackages {
    # library/packages is the authoritative built-in catalogue for the desktop copier and the
    # WASM preload; a stray non-zip would ship to every consumer, so it is an error, not a skip.
    $dir = Join-Path $Script:RepoRoot 'library\packages'
    $entries = @(Get-ChildItem -LiteralPath $dir -File | Sort-Object Name)
    $nonPackages = @($entries | Where-Object { $_.Extension -ne '.zip' })
    if ($nonPackages.Count -gt 0) {
        throw "library/packages contains non-package files: $(($nonPackages.Name | Sort-Object) -join ', ')"
    }
    if ($entries.Count -eq 0) { throw "No built-in packages found in $dir" }
    return $entries
}

function Initialize-Emscripten {
    # em++ is a launcher that shells out to `python` found on PATH, so emsdk's own interpreter
    # goes first - a Windows box otherwise hits the Microsoft Store python stub and the compiler
    # reports itself as missing. Returns the resolved tool directories.
    param([Parameter(Mandatory = $true)][string]$EmsdkRoot)

    $emscriptenDir = Join-Path $EmsdkRoot 'upstream\emscripten'
    if (-not (Test-Path (Join-Path $emscriptenDir 'em++.py'))) {
        throw "Emscripten not found at $emscriptenDir. Install it with: python $EmsdkRoot\emsdk.py install latest; python $EmsdkRoot\emsdk.py activate latest"
    }
    $newestSubdir = {
        param($root)
        if (Test-Path $root) {
            (Get-ChildItem $root -Directory | Sort-Object Name -Descending | Select-Object -First 1).FullName
        } else { $null }
    }
    $pythonDir = & $newestSubdir (Join-Path $EmsdkRoot 'python')
    $nodeVersionDir = & $newestSubdir (Join-Path $EmsdkRoot 'node')
    $nodeDir = if ($nodeVersionDir) { Join-Path $nodeVersionDir 'bin' } else { $null }

    $env:EMSDK = $EmsdkRoot
    $env:EM_CONFIG = Join-Path $EmsdkRoot '.emscripten'
    if ($pythonDir) { $env:EMSDK_PYTHON = Join-Path $pythonDir 'python.exe' }
    $prefix = @($pythonDir, $emscriptenDir, (Join-Path $EmsdkRoot 'upstream\bin'), $nodeDir) |
        Where-Object { $_ }
    $env:PATH = ($prefix -join ';') + ';' + $env:PATH

    return @{
        EmscriptenDir = $emscriptenDir
        PythonDir = $pythonDir
        NodeDir = $nodeDir
        NodeExe = if ($nodeDir) { Join-Path $nodeDir 'node.exe' } else { $null }
    }
}

function Get-EmscriptenTool {
    # emsdk layouts differ: current ships em++.exe, older ones em++.bat or a bare script.
    param(
        [Parameter(Mandatory = $true)][string]$EmscriptenDir,
        [Parameter(Mandatory = $true)][string]$Name
    )
    foreach ($candidate in @("$Name.exe", "$Name.bat", $Name)) {
        $path = Join-Path $EmscriptenDir $candidate
        if (Test-Path $path -PathType Leaf) { return $path }
    }
    throw "Emscripten tool '$Name' not found under $EmscriptenDir."
}

# One list; the same six archives are verified, smoke-linked and app-linked.
$Script:PhysXWasmArchives = @(
    'libPhysXExtensions_static_32.a'
    'libPhysXCooking_static_32.a'
    'libPhysX_static_32.a'
    'libPhysXPvdSDK_static_32.a'
    'libPhysXCommon_static_32.a'
    'libPhysXFoundation_static_32.a'
)

function Get-PhysXWasmLibraries {
    param([Parameter(Mandatory = $true)][string]$BuildDir)
    $libRoot = Join-Path $BuildDir 'lib'
    $anchor = Get-ChildItem -Path $libRoot -Recurse -Filter 'libPhysX_static_32.a' -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if (-not $anchor) {
        throw "PhysX WASM archives not found under $libRoot. Run scripts\build_physx_wasm.ps1 first."
    }
    return $Script:PhysXWasmArchives | ForEach-Object {
        $archive = Join-Path $anchor.DirectoryName $_
        Assert-File $archive 'PhysX WASM archive is unavailable.'
        $archive
    }
}

function Copy-RuntimeDlls {
    param(
        [Parameter(Mandatory = $true)][string]$ExePath,
        [Parameter(Mandatory = $true)]$Config
    )

    $releaseDir = Split-Path -Parent $ExePath
    & (Join-Path $Config.QtRoot 'bin\windeployqt.exe') --release --compiler-runtime $ExePath
    if ($LASTEXITCODE -ne 0) {
        throw "windeployqt failed with exit code $LASTEXITCODE"
    }

    $occtBin = Join-Path $Config.OcctRoot 'win64\vc14\bin'
    if (Test-Path $occtBin) {
        Copy-Item (Join-Path $occtBin '*.dll') $releaseDir -Force
    }

    $physxBin = Join-Path $Config.PhysXRoot ('bin\{0}\release' -f $Config.PhysXPlatform)
    foreach ($dll in @('PhysX_64.dll', 'PhysXFoundation_64.dll', 'PhysXCooking_64.dll', 'PhysXCommon_64.dll')) {
        $path = Join-Path $physxBin $dll
        if (Test-Path $path) {
            Copy-Item $path $releaseDir -Force
        }
    }

    $vcomp = Join-Path $env:WINDIR 'System32\VCOMP140.DLL'
    if (Test-Path $vcomp) {
        Copy-Item $vcomp $releaseDir -Force
    }
}
