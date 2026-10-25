@echo off
REM One-time setup for Twinflie on Windows (sim-bench branch).
REM Creates .venv with the server deps, downloads Godot 3.5.3 into tools\godot,
REM then runs a 10 s smoke benchmark with mock drones.
cd /d "%~dp0"

set PY=python
where py >nul 2>nul && set PY=py -3

if not exist .venv (
  echo [1/3] Creating Python virtual environment...
  %PY% -m venv .venv || goto :fail
)
call .venv\Scripts\activate.bat || goto :fail
python -m pip install --upgrade pip >nul
echo [1/3] Installing flask gevent pymitter cflib psutil...
python -m pip install flask gevent pymitter cflib psutil || goto :fail

if not exist tools\godot\Godot_v3.5.3-stable_win64.exe (
  echo [2/3] Downloading Godot 3.5.3, about 38 MB...
  if not exist tools\godot mkdir tools\godot
  powershell -NoProfile -Command "$ErrorActionPreference='Stop'; Invoke-WebRequest -Uri 'https://github.com/godotengine/godot/releases/download/3.5.3-stable/Godot_v3.5.3-stable_win64.exe.zip' -OutFile 'tools\godot\godot.zip'; Expand-Archive 'tools\godot\godot.zip' -DestinationPath 'tools\godot' -Force; Remove-Item 'tools\godot\godot.zip'" || goto :fail
) else (
  echo [2/3] Godot 3.5.3 already in tools\godot
)

echo [3/3] Smoke test: server + 1 and 3 mock drones...
python bench\run_bench.py --n 1 3 --secs 5 --label smoke-windows || goto :fail

echo.
echo Setup done. Use run_server.bat, then open_godot.bat.
pause
exit /b 0

:fail
echo.
echo Setup failed. Scroll up for the error.
pause
exit /b 1
