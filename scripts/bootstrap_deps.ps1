param(
    [string]$CacheZip,
    [string]$CacheDir,
    [switch]$BuildFromSource,
    [switch]$Force
)

. (Join-Path $PSScriptRoot 'common.ps1')

$config = Get-DependencyConfig

if (-not $CacheDir) {
    $CacheDir = Join-Path $RepoRoot 'deps-cache'
}
if (-not $CacheZip) {
    $CacheZip = Join-Path $CacheDir 'physicsAddIn-deps-msvc2019-x64-physx-5.9.0.zip'
}

function Restore-DependencyCache {
    param([Parameter(Mandatory = $true)][string]$ZipPath)

    if (-not (Test-Path $ZipPath)) {
        return $false
    }

    Write-Host "Restoring dependency cache: $ZipPath"
    $temp = Join-Path ([System.IO.Path]::GetTempPath()) ('physicsAddIn-deps-{0}' -f ([Guid]::NewGuid().ToString('N')))
    New-Item -ItemType Directory -Force -Path $temp | Out-Null
    try {
        Expand-Archive -Path $ZipPath -DestinationPath $temp -Force

        $occt = Join-Path $temp 'OCCT\opencascade-7.6.0'
        if ((Test-Path $occt) -and ($Force -or -not (Test-Path $config.OcctRoot))) {
            New-Item -ItemType Directory -Force -Path (Split-Path -Parent $config.OcctRoot) | Out-Null
            Copy-Item $occt $config.OcctRoot -Recurse -Force
        }

        $physx = Join-Path $temp 'PhysX\physx'
        if ((Test-Path $physx) -and ($Force -or -not (Test-Path $config.PhysXRoot))) {
            New-Item -ItemType Directory -Force -Path (Split-Path -Parent $config.PhysXRoot) | Out-Null
            Copy-Item $physx $config.PhysXRoot -Recurse -Force
        }

        $coacd = Join-Path $temp 'CoACD'
        if ((Test-Path $coacd) -and ($Force -or -not (Test-Path $config.CoacdRoot))) {
            New-Item -ItemType Directory -Force -Path (Split-Path -Parent $config.CoacdRoot) | Out-Null
            Copy-Item $coacd $config.CoacdRoot -Recurse -Force
        }
    } finally {
        Remove-Item $temp -Recurse -Force -ErrorAction SilentlyContinue
    }

    return $true
}

function Ensure-Qt {
    $qmake = Join-Path $config.QtRoot 'bin\qmake.exe'
    if (Test-Path $qmake) {
        return
    }

    Write-Host 'Installing Qt 5.15.2 MSVC2019 with aqtinstall.'
    python -m pip install --user --upgrade aqtinstall
    if ($LASTEXITCODE -ne 0) {
        throw 'Failed to install aqtinstall. Install Python/pip or install Qt manually.'
    }

    python -m aqt install-qt windows desktop 5.15.2 win64_msvc2019_64 -O C:\Qt
    if ($LASTEXITCODE -ne 0) {
        throw 'Failed to install Qt with aqtinstall.'
    }
}

function Ensure-CMake {
    $kitwareCMake = 'C:\Program Files\CMake\bin\cmake.exe'
    if (Test-Path $kitwareCMake) {
        return
    }

    Write-Host 'Installing CMake with winget.'
    winget install --id Kitware.CMake -e --source winget --accept-source-agreements --accept-package-agreements
    if ($LASTEXITCODE -ne 0) {
        throw 'Failed to install CMake. Install Kitware CMake manually and rerun.'
    }
    if (-not (Test-Path $kitwareCMake)) {
        throw "Kitware CMake was installed but was not found at $kitwareCMake."
    }
}

function Get-WindowsCMakePath {
    $kitwareCMake = 'C:\Program Files\CMake\bin\cmake.exe'
    if (-not (Test-Path $kitwareCMake -PathType Leaf)) {
        throw 'Kitware CMake is required for MSVC/NMake builds. Run Ensure-CMake first.'
    }
    return $kitwareCMake
}

function Ensure-CoACD {
    $coacdLib = Join-Path $config.CoacdRoot 'buildMD\Release\coacd.lib'
    if ((Test-Path $coacdLib) -and -not $Force) {
        return
    }
    if (-not $BuildFromSource) {
        throw 'CoACD is missing. Provide deps-cache/physicsAddIn-deps-msvc2019-x64-physx-5.9.0.zip or rerun with -BuildFromSource.'
    }

    if (-not (Test-Path $config.CoacdRoot)) {
        git clone https://github.com/SarahWeiii/CoACD.git $config.CoacdRoot
        if ($LASTEXITCODE -ne 0) { throw 'Failed to clone CoACD.' }
    }

    Push-Location $config.CoacdRoot
    try {
        git checkout b678aa0802996fa03e1ec0e68bd05acf8cd20cf9
        git submodule update --init --recursive
    } finally {
        Pop-Location
    }

    $cmake = Get-WindowsCMakePath
    $buildDir = Join-Path $config.CoacdRoot 'buildMD'
    New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

    $commands = @(
        ('cd /d "{0}"' -f $buildDir),
        ('"{0}" -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL -DOPENVDB_CORE_SHARED=OFF -DTBB_TEST=OFF -DWITH_3RD_PARTY_LIBS=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5 "{1}"' -f $cmake, $config.CoacdRoot),
        'nmake /NOLOGO coacd'
    )
    Invoke-VsCmd -Commands $commands
}

function Ensure-PhysX {
    $physxLib = Join-Path $config.PhysXRoot ('bin\{0}\release\PhysX_64.lib' -f $config.PhysXPlatform)
    if ((Test-Path $physxLib) -and -not $Force) {
        return
    }
    if (-not $BuildFromSource) {
        throw 'PhysX is missing. Provide deps-cache/physicsAddIn-deps-msvc2019-x64-physx-5.9.0.zip or rerun with -BuildFromSource.'
    }

    $physxRepo = Split-Path -Parent $config.PhysXRoot
    if (-not (Test-Path $physxRepo)) {
        git clone --depth 1 --branch $config.PhysXTag https://github.com/NVIDIA-Omniverse/PhysX.git $physxRepo
        if ($LASTEXITCODE -ne 0) { throw 'Failed to clone PhysX.' }
    }

    $physxBuild = Join-Path $config.PhysXRoot 'compiler\vc16win64-cpu-only'
    $generateProjects = Join-Path $config.PhysXRoot 'generate_projects.bat'
    $solution = Join-Path $physxBuild 'PhysXSDK.sln'
    $cmakeBin = Split-Path -Parent (Get-WindowsCMakePath)
    $commands = @(
        ('set "PATH={0};%PATH%"' -f $cmakeBin),
        ('call "{0}" vc16win64-cpu-only' -f $generateProjects),
        ('msbuild "{0}" /m /nologo /p:Configuration=release /p:Platform=x64' -f $solution)
    )
    Invoke-VsCmd -Commands $commands
}

function Ensure-OCCT {
    $occtHeader = Join-Path $config.OcctRoot 'inc\Standard.hxx'
    if ((Test-Path $occtHeader) -and -not $Force) {
        return
    }
    if (-not $BuildFromSource) {
        throw 'OpenCascade is missing. Provide deps-cache/physicsAddIn-deps-msvc2019-x64-physx-5.9.0.zip or install OCCT 7.6.0 at OCCT_ROOT.'
    }

    $depsRoot = 'C:\deps'
    $occtSource = Join-Path $depsRoot 'OCCT-7.6.0'
    $occtBuild = Join-Path $depsRoot 'OCCT-7.6.0-build'

    if (-not (Test-Path $occtSource)) {
        New-Item -ItemType Directory -Force -Path $depsRoot | Out-Null
        git clone --branch V7_6_0 https://github.com/Open-Cascade-SAS/OCCT.git $occtSource
        if ($LASTEXITCODE -ne 0) { throw 'Failed to clone OpenCascade.' }
    }

    $cmake = Get-WindowsCMakePath
    New-Item -ItemType Directory -Force -Path $occtBuild | Out-Null
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $config.OcctRoot) | Out-Null

    $commands = @(
        ('cd /d "{0}"' -f $occtBuild),
        ('"{0}" -G "Visual Studio 16 2019" -A x64 -DCMAKE_INSTALL_PREFIX="{1}" -DBUILD_LIBRARY_TYPE=Shared -DBUILD_MODULE_Draw=OFF -DUSE_VTK=OFF -DUSE_OPENGL=ON "{2}"' -f $cmake, $config.OcctRoot, $occtSource),
        ('"{0}" --build . --config Release --target INSTALL' -f $cmake)
    )
    Invoke-VsCmd -Commands $commands
}

Get-VcVars64Path | Out-Null
Ensure-Qt
Ensure-CMake

$restored = Restore-DependencyCache -ZipPath $CacheZip
if ($restored) {
    Write-Host 'Dependency cache restored.'
}

Ensure-OCCT
Ensure-PhysX
Ensure-CoACD

Test-StandaloneDependencies $config
Write-Host 'Dependency bootstrap complete.'
