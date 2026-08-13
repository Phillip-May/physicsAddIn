param(
    [string]$OutputZip,
    [switch]$Force
)

. (Join-Path $PSScriptRoot 'common.ps1')

$config = Get-DependencyConfig
Test-StandaloneDependencies $config

if (-not $OutputZip) {
    $cacheDir = Join-Path $RepoRoot 'deps-cache'
    New-Item -ItemType Directory -Force -Path $cacheDir | Out-Null
    $OutputZip = Join-Path $cacheDir 'physicsAddIn-deps-msvc2019-x64-physx-5.9.0.zip'
}

if ((Test-Path $OutputZip) -and -not $Force) {
    throw "Cache zip already exists: $OutputZip. Use -Force to replace it."
}

$stage = Join-Path ([System.IO.Path]::GetTempPath()) ('physicsAddIn-cache-stage-{0}' -f ([Guid]::NewGuid().ToString('N')))
New-Item -ItemType Directory -Force -Path $stage | Out-Null

try {
    $occtStage = Join-Path $stage 'OCCT\opencascade-7.6.0'
    $physxStage = Join-Path $stage 'PhysX\physx'
    $coacdStage = Join-Path $stage 'CoACD'

    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $occtStage) | Out-Null
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $physxStage) | Out-Null
    New-Item -ItemType Directory -Force -Path $coacdStage | Out-Null

    Write-Host 'Staging OpenCascade runtime/install tree.'
    Copy-Item $config.OcctRoot $occtStage -Recurse -Force

    Write-Host 'Staging PhysX headers, sources needed by public headers, and binaries.'
    foreach ($relative in @('include', 'source\foundation\include', 'source\physx\include', 'bin')) {
        $source = Join-Path $config.PhysXRoot $relative
        if (Test-Path $source) {
            Copy-Item $source (Join-Path $physxStage $relative) -Recurse -Force
        }
    }

    Write-Host 'Staging CoACD public headers and linked /MD release libraries.'
    Copy-Item (Join-Path $config.CoacdRoot 'public') (Join-Path $coacdStage 'public') -Recurse -Force
    $coacdLibs = @(
        'buildMD\Release\coacd.lib',
        'buildMD\_deps\boost-build\libs\random\Release\libboost_random-vc142-mt-x64-1_81.lib',
        'buildMD\_deps\zlib-build\Release\zlibstatic.lib',
        'buildMD\_deps\boost-build\libs\iostreams\Release\libboost_iostreams-vc142-mt-x64-1_81.lib',
        'buildMD\msvc_19.29_cxx20_64_md_release\tbb12.lib',
        'buildMD\_deps\openvdb-build\openvdb\openvdb\Release\libopenvdb.lib',
        'buildMD\_deps\spdlog-build\Release\spdlog.lib'
    )
    foreach ($relative in $coacdLibs) {
        $source = Join-Path $config.CoacdRoot $relative
        $destination = Join-Path $coacdStage $relative
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $destination) | Out-Null
        Copy-Item $source $destination -Force
    }

    $manifest = [ordered]@{
        name = 'physicsAddIn-deps-msvc2019-x64-physx-5.9.0'
        created_utc = (Get-Date).ToUniversalTime().ToString('o')
        visual_studio = 'VS2019/v142 ABI'
        qt = '5.15.2 msvc2019_64, install separately with scripts/bootstrap_deps.ps1'
        occt = 'V7_6_0'
        physx = '110.1-omni-and-physx-5.9.0, CPU /MD'
        coacd = 'b678aa0802996fa03e1ec0e68bd05acf8cd20cf9, /MD'
        layout = @{
            occt = 'OCCT/opencascade-7.6.0'
            physx = 'PhysX/physx'
            coacd = 'CoACD'
        }
    }
    $manifest | ConvertTo-Json -Depth 5 | Set-Content (Join-Path $stage 'manifest.json') -Encoding ASCII

    if (Test-Path $OutputZip) {
        Remove-Item $OutputZip -Force
    }
    Compress-Archive -Path (Join-Path $stage '*') -DestinationPath $OutputZip -CompressionLevel Optimal
    Write-Host "Dependency cache written: $OutputZip"
} finally {
    Remove-Item $stage -Recurse -Force -ErrorAction SilentlyContinue
}
