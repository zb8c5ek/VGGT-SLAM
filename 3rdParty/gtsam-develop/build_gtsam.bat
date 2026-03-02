@echo off
setlocal enabledelayedexpansion

:: ── Auto-detect Visual Studio Build Tools ──────────────────────────
:: Try vswhere first (works with VS 2017+), fall back to common paths
set "VCVARS="

:: Method 1: vswhere (most reliable)
for /f "tokens=*" %%i in ('"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul') do (
    set "VSINSTALL=%%i"
)
if defined VSINSTALL (
    if exist "!VSINSTALL!\VC\Auxiliary\Build\vcvars64.bat" (
        set "VCVARS=!VSINSTALL!\VC\Auxiliary\Build\vcvars64.bat"
    )
)

:: Method 2: Try common install locations
if not defined VCVARS (
    for %%Y in (2022 2019) do (
        for %%E in (BuildTools Community Professional Enterprise) do (
            if exist "C:\Program Files\Microsoft Visual Studio\%%Y\%%E\VC\Auxiliary\Build\vcvars64.bat" (
                set "VCVARS=C:\Program Files\Microsoft Visual Studio\%%Y\%%E\VC\Auxiliary\Build\vcvars64.bat"
                goto :found_vcvars
            )
            if exist "C:\Program Files (x86)\Microsoft Visual Studio\%%Y\%%E\VC\Auxiliary\Build\vcvars64.bat" (
                set "VCVARS=C:\Program Files (x86)\Microsoft Visual Studio\%%Y\%%E\VC\Auxiliary\Build\vcvars64.bat"
                goto :found_vcvars
            )
        )
    )
)
:found_vcvars

if not defined VCVARS (
    echo ERROR: Could not find Visual Studio Build Tools.
    echo Install VS 2022 Build Tools with "Desktop development with C++" workload.
    exit /b 1
)

echo Using: %VCVARS%
call "%VCVARS%" >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo ERROR: vcvars64.bat failed!
    exit /b 1
)

:: ── Set UTF-8 codepage ─────────────────────────────────────────────
chcp 65001 >nul 2>&1

:: ── Resolve paths relative to this script ──────────────────────────
set "SCRIPT_DIR=%~dp0"
set "BUILD_DIR=%SCRIPT_DIR%build"
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
cd /d "%BUILD_DIR%"

echo ============================================
echo   GTSAM Build Script
echo ============================================

echo.
echo [1/3] Checking Python environment...
where python
python --version
python -c "import pyparsing; print('pyparsing', pyparsing.__version__)"
if %ERRORLEVEL% neq 0 (
    echo FAILED: pyparsing not found!
    exit /b 1
)

echo.
echo [2/3] Running CMake configure...
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_PYTHON=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_USE_SYSTEM_PYBIND=ON -DGTSAM_WITH_TBB=OFF -DBoost_USE_STATIC_LIBS=OFF -DCMAKE_INSTALL_PREFIX="%SCRIPT_DIR%install"
if %ERRORLEVEL% neq 0 (
    echo FAILED: CMake configure failed!
    exit /b 2
)

echo.
echo [3/3] Building (this may take 5-10 minutes)...
cmake --build . --config Release -j8
if %ERRORLEVEL% neq 0 (
    echo ============================================
    echo   BUILD FAILED
    echo ============================================
    exit /b 3
)

echo.
echo ============================================
echo   BUILD SUCCEEDED
echo ============================================
echo.
echo Next: cd build\python ^&^& python setup.py install
