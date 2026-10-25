@echo off
REM Starts the Twinflie Python server (HTTP :5000, UDP 20003/20004).
cd /d "%~dp0python server\classe refactoring"
call "%~dp0.venv\Scripts\activate.bat"
python main.py
pause
