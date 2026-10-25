@echo off
REM Runs the flight sim without building the exe (needs Python).
cd /d "%~dp0"
python splat\app\sim_launcher.py %*
pause
