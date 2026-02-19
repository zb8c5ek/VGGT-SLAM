@echo on
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
set PATH=D:\MICROMAMBA\envs\sfm3r;D:\MICROMAMBA\envs\sfm3r\Scripts;D:\MICROMAMBA\envs\sfm3r\Library\bin;%PATH%
chcp 65001 >nul 2>&1
cd /d D:\VGGT-SLAM\3rdParty\gtsam-develop\build

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
cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_PYTHON=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_USE_SYSTEM_PYBIND=ON -DGTSAM_WITH_TBB=OFF -DBoost_USE_STATIC_LIBS=OFF -DCMAKE_INSTALL_PREFIX=D:\VGGT-SLAM\3rdParty\gtsam-develop\install
if %ERRORLEVEL% neq 0 (
    echo FAILED: CMake configure failed!
    exit /b 2
)

echo.
echo [3/3] Building (this may take 5-10 minutes)...
cmake --build . --config Release -j8 -- -v
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
