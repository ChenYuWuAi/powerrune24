@echo off
echo Deploying PowerRune Operator environment...

REM 0. Check Path variables
echo Checking Path variables...

python --version >nul 2>&1

REM 如果错误码为1，则说明没有安装Python或者Python没有配置到环境变量中
if errorlevel 1 (
    echo Python is not installed or not configured properly.
    exit /b
)

pip --version >nul 2>&1
if errorlevel 1 (
    echo Pip is not installed or not configured properly.
    exit /b
)

REM 1. Check Python environment
python --version
pip --version

REM 2. Install dependencies
echo Installing requirements...
pip install -r requirements.txt
if errorlevel 1 (
    echo Failed to install requirements, please check your network and try again.
    exit /b
)

REM 3. Installation complete
echo Install finished. PowerRune Operator environment is ready.