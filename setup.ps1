#Requires -Version 5.1
<#
.SYNOPSIS
    VGGT-SLAM setup script for Windows PowerShell.
.DESCRIPTION
    Installs Python dependencies and the bundled third-party packages
    (salad, vggt) from 3rdParty/.  Perception Encoder and SAM3 are
    optional and cloned only if requested.

    Run from the repository root:
        .\setup.ps1

    For detailed Windows build instructions (GTSAM, CUDA, etc.)
    see local_install.md.
#>

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'Continue'

# ── Helpers ──────────────────────────────────────────────────────────
function Write-Step {
    param([string]$Message)
    Write-Host "`n>> $Message" -ForegroundColor Cyan
}

function Test-Command {
    param([string]$Name)
    $null -ne (Get-Command $Name -ErrorAction SilentlyContinue)
}

# ── Pre-flight checks ───────────────────────────────────────────────
Write-Step "Checking prerequisites"

if (-not (Test-Command 'git'))  { throw "git is not on PATH. Install Git for Windows: https://git-scm.com/downloads/win" }
if (-not (Test-Command 'pip'))  {
    if (Test-Command 'pip3') {
        Set-Alias -Name pip -Value pip3 -Scope Script
    } else {
        throw "pip is not on PATH. Activate your conda/venv environment first."
    }
}

$repoRoot = $PSScriptRoot
if (-not $repoRoot) { $repoRoot = (Get-Location).Path }
if (-not (Test-Path (Join-Path $repoRoot 'requirements.txt'))) {
    throw "requirements.txt not found. Run this script from the VGGT-SLAM repo root."
}

$thirdParty = Join-Path $repoRoot '3rdParty'

# ── 1. Install Python dependencies ──────────────────────────────────
Write-Step "Installing base requirements (requirements.txt)"
pip install --no-input -r (Join-Path $repoRoot 'requirements.txt')
if ($LASTEXITCODE -ne 0) { throw "pip install requirements.txt failed (exit code $LASTEXITCODE)." }

# ── 2. Install bundled Salad ─────────────────────────────────────────
Write-Step "Installing Salad (bundled in 3rdParty/salad)"
$saladDir = Join-Path $thirdParty 'salad'
if (-not (Test-Path $saladDir)) {
    throw "3rdParty/salad/ not found — the repo may be incomplete. Re-clone or pull."
}
pip install --no-input -e $saladDir
if ($LASTEXITCODE -ne 0) { throw "pip install salad failed." }

# ── 3. Install bundled VGGT ──────────────────────────────────────────
Write-Step "Installing VGGT (bundled in 3rdParty/vggt)"
$vggtDir = Join-Path $thirdParty 'vggt'
if (-not (Test-Path $vggtDir)) {
    throw "3rdParty/vggt/ not found — the repo may be incomplete. Re-clone or pull."
}
pip install --no-input --no-deps -e $vggtDir
if ($LASTEXITCODE -ne 0) { throw "pip install VGGT failed." }

# ── 4. (Optional) Clone and install Perception Encoder ───────────────
Write-Step "Perception Encoder + SAM3 (optional, for --run_os)"
$peDir = Join-Path $thirdParty 'perception_models'
$sam3Dir = Join-Path $thirdParty 'sam3'

if (Test-Path $peDir) {
    Write-Host "   perception_models/ found, installing..." -ForegroundColor Yellow
    pip install --no-input -e $peDir
} else {
    Write-Host "   perception_models/ not found — skipping (only needed for --run_os)" -ForegroundColor DarkGray
}

if (Test-Path $sam3Dir) {
    Write-Host "   sam3/ found, installing..." -ForegroundColor Yellow
    pip install --no-input -e $sam3Dir
} else {
    Write-Host "   sam3/ not found — skipping (only needed for --run_os)" -ForegroundColor DarkGray
}

# ── 5. Install VGGT-SLAM itself ─────────────────────────────────────
Write-Step "Installing VGGT-SLAM (editable mode)"
pip install --no-input -e $repoRoot
if ($LASTEXITCODE -ne 0) { throw "pip install VGGT-SLAM failed." }

# ── Done ─────────────────────────────────────────────────────────────
Write-Host ""
Write-Host "============================================" -ForegroundColor Green
Write-Host "  Installation Complete!" -ForegroundColor Green
Write-Host "============================================" -ForegroundColor Green
Write-Host ""
Write-Host "Next steps:" -ForegroundColor White
Write-Host "  1. Build GTSAM from source:" -ForegroundColor Gray
Write-Host "       3rdParty\gtsam-develop\build_gtsam.bat" -ForegroundColor DarkGray
Write-Host "     (auto-detects VS Build Tools location)" -ForegroundColor DarkGray
Write-Host "  2. Download the SALAD checkpoint:" -ForegroundColor Gray
Write-Host '     Invoke-WebRequest -Uri "https://github.com/serizba/salad/releases/download/v1.0.0/dino_salad.ckpt" `' -ForegroundColor DarkGray
Write-Host '       -OutFile "$env:USERPROFILE\.cache\torch\hub\checkpoints\dino_salad.ckpt"' -ForegroundColor DarkGray
Write-Host "  3. Run: python main.py --image_folder office_loop --vis_map" -ForegroundColor Gray
Write-Host ""
