<#
.SYNOPSIS
    Packages a built WebAssembly RobotSimulator into a distributable zip.

.DESCRIPTION
    Collects the Emscripten output plus the entry page and licence notices into
    dist/RobotSimulator-wasm.zip.

    The archive is flat: index.html sits at the root, so unzipping and serving that
    folder just works. Compress-Archive -Path <dir> would nest everything one level
    inside a RobotSimulator-wasm/ folder, which means the served root has no
    index.html in it.

    This does not build. Run the wasm build first; -BuildDir points at its output.

.EXAMPLE
    .\scripts\package_wasm.ps1 -BuildDir C:\path\to\wasm-build
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$BuildDir,

    [string]$OutputZip
)

. (Join-Path $PSScriptRoot 'common.ps1')

if (-not $OutputZip) { $OutputZip = Join-Path $RepoRoot 'dist\RobotSimulator-wasm.zip' }

# Emscripten output, plus the entry page build_wasm.ps1 copies beside it.
$fromBuild = @('index.html', 'RobotSimulator.js', 'RobotSimulator.wasm', 'RobotSimulator.data')
# Shipped alongside the binary; both are the source of truth in the repo.
$fromRepo = @('RobotSimulator\web\README.txt', 'RobotSimulator\web\THIRD-PARTY.txt')

$missing = @()
foreach ($name in $fromBuild) {
    if (-not (Test-Path -LiteralPath (Join-Path $BuildDir $name))) { $missing += $name }
}
if ($missing.Count -gt 0) {
    throw "Build output incomplete in ${BuildDir}: $($missing -join ', '). Build the wasm target first."
}

$stage = Join-Path ([System.IO.Path]::GetTempPath()) ("robotsim-wasm-" + [System.Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Force -Path $stage | Out-Null
try {
    foreach ($name in $fromBuild) { Copy-Item (Join-Path $BuildDir $name) $stage }
    foreach ($rel in $fromRepo)   { Copy-Item (Join-Path $RepoRoot $rel) $stage }

    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $OutputZip) | Out-Null
    if (Test-Path -LiteralPath $OutputZip) { Remove-Item -LiteralPath $OutputZip -Force }

    # The \* is what keeps the archive flat - see the note above.
    Compress-Archive -Path (Join-Path $stage '*') -DestinationPath $OutputZip -CompressionLevel Optimal

    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $archive = [System.IO.Compression.ZipFile]::OpenRead($OutputZip)
    try {
        $entries = $archive.Entries | ForEach-Object { $_.FullName }
        # Cheap guard against the nesting regression this script exists to prevent.
        if ($entries -notcontains 'index.html') {
            throw "index.html is not at the archive root. Entries: $($entries -join ', ')"
        }
        Write-Host "Packaged $OutputZip ($([math]::Round((Get-Item -LiteralPath $OutputZip).Length / 1MB, 2)) MB)"
        foreach ($entry in $archive.Entries) {
            Write-Host ("  {0,-24} {1,12:N0}" -f $entry.FullName, $entry.Length)
        }
    } finally {
        $archive.Dispose()
    }
} finally {
    if (Test-Path -LiteralPath $stage) { Remove-Item -LiteralPath $stage -Recurse -Force }
}
