param(
    [Parameter(Mandatory=$true)]
    [string]$ConfigFile
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path

# Resolve config path (absolute or relative to script dir)
if ([System.IO.Path]::IsPathRooted($ConfigFile)) {
    $ConfigPath = $ConfigFile
} else {
    $ConfigPath = Join-Path $ScriptDir $ConfigFile
}

if (-not (Test-Path $ConfigPath)) {
    Write-Host "ERROR: Config file not found: $ConfigPath" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  VGGT-SLAM Multi-Camera Pipeline"       -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  Config : $ConfigPath"
Write-Host ""

$Script = Join-Path $ScriptDir "BURNPIPE_VGGT_SLAM.py"

micromamba run -n sfm3rV2 python $Script $ConfigPath

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "Pipeline completed successfully." -ForegroundColor Green
} else {
    Write-Host ""
    Write-Host "Pipeline FAILED with exit code $LASTEXITCODE" -ForegroundColor Red
    exit $LASTEXITCODE
}
