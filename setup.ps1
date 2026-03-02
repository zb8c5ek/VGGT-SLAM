#Requires -Version 5.1
<#
.SYNOPSIS
    VGGT-SLAM setup script for Windows PowerShell.
.DESCRIPTION
    PowerShell equivalent of setup.sh. Installs Python dependencies
    and clones/installs third-party packages (Salad, VGGT, Perception
    Encoder, SAM3).

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

# ── 1. Install Python dependencies ──────────────────────────────────
Write-Step "Installing base requirements (requirements.txt)"
pip install --no-input -r (Join-Path $repoRoot 'requirements.txt')
if ($LASTEXITCODE -ne 0) { throw "pip install requirements.txt failed (exit code $LASTEXITCODE)." }

# ── 2. Prepare 3rdParty directory ────────────────────────────────────
$thirdParty = Join-Path $repoRoot '3rdParty'
if (-not (Test-Path $thirdParty)) { New-Item -ItemType Directory -Path $thirdParty | Out-Null }

# ── 3. Clone and install Salad ───────────────────────────────────────
Write-Step "Cloning and installing Salad"
$saladDir = Join-Path $thirdParty 'salad'
if (-not (Test-Path $saladDir)) {
    git clone --progress https://github.com/Dominic101/salad.git $saladDir
    if ($LASTEXITCODE -ne 0) { throw "git clone salad failed." }
} else {
    Write-Host "   salad/ already exists, skipping clone." -ForegroundColor Yellow
}
pip install --no-input -e $saladDir
if ($LASTEXITCODE -ne 0) { throw "pip install salad failed." }

# ── 4. Clone and install VGGT ────────────────────────────────────────
Write-Step "Cloning and installing VGGT"
$vggtDir = Join-Path $thirdParty 'vggt'
if (-not (Test-Path $vggtDir)) {
    git clone --progress https://github.com/MIT-SPARK/VGGT_SPARK.git $vggtDir
    if ($LASTEXITCODE -ne 0) { throw "git clone VGGT failed." }
} else {
    Write-Host "   vggt/ already exists, skipping clone." -ForegroundColor Yellow
}
pip install --no-input -e $vggtDir
if ($LASTEXITCODE -ne 0) { throw "pip install VGGT failed." }

# ── 5. Clone and install Perception Encoder ──────────────────────────
Write-Step "Cloning and installing Perception Encoder"
$peDir = Join-Path $thirdParty 'perception_models'
if (-not (Test-Path $peDir)) {
    git clone --progress https://github.com/facebookresearch/perception_models.git $peDir
    if ($LASTEXITCODE -ne 0) { throw "git clone perception_models failed." }
} else {
    Write-Host "   perception_models/ already exists, skipping clone." -ForegroundColor Yellow
}
pip install --no-input -e $peDir
if ($LASTEXITCODE -ne 0) { throw "pip install perception_models failed." }

# ── 6. Clone and install SAM 3 ───────────────────────────────────────
Write-Step "Cloning and installing SAM 3"
$sam3Dir = Join-Path $thirdParty 'sam3'
if (-not (Test-Path $sam3Dir)) {
    git clone --progress https://github.com/facebookresearch/sam3.git $sam3Dir
    if ($LASTEXITCODE -ne 0) { throw "git clone sam3 failed." }
} else {
    Write-Host "   sam3/ already exists, skipping clone." -ForegroundColor Yellow
}
pip install --no-input -e $sam3Dir
if ($LASTEXITCODE -ne 0) { throw "pip install sam3 failed." }

# ── 7. Install VGGT-SLAM itself ─────────────────────────────────────
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
Write-Host "  1. Build GTSAM from source (see local_install.md, section 6)" -ForegroundColor Gray
Write-Host "  2. Download the SALAD checkpoint:" -ForegroundColor Gray
Write-Host '     Invoke-WebRequest -Uri "https://github.com/serizba/salad/releases/download/v1.0.0/dino_salad.ckpt" `' -ForegroundColor DarkGray
Write-Host '       -OutFile "$env:USERPROFILE\.cache\torch\hub\checkpoints\dino_salad.ckpt"' -ForegroundColor DarkGray
Write-Host "  3. Run: python main.py --image_folder office_loop --vis_map" -ForegroundColor Gray
Write-Host ""
