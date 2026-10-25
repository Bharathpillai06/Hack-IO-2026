@echo off
REM Full baseline: 1 5 10 20 40 mock drones, 15 s each. Appends to bench\results.csv.
cd /d "%~dp0"
call .venv\Scripts\activate.bat
set LABEL=%1
if "%LABEL%"=="" set LABEL=baseline
python bench\run_bench.py --n 1 5 10 20 40 --secs 15 --label %LABEL%
pause
