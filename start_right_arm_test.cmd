@echo off
cd /d "%~dp0"
if errorlevel 1 exit /b 1
python "%~dp0run_right_arm_5cycles.py"
pause
