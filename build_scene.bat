@echo off
REM Build a flight-sim scene from video of any location.
REM Drag one or more videos onto this file, or run:  build_scene.bat clip1.mp4 clip2.mp4 --name my-field
REM First run installs the reconstruction tools into .venv-splat (about 1 GB, CPU only).
REM The scene appears in scenes\<name>\ and in the Scene menu of Twinfield.exe.
cd /d "%~dp0"
if "%~1"=="" (
  echo Drag one or more videos onto build_scene.bat, or run: build_scene.bat video.mp4 --name my-field
  pause
  exit /b 1
)
set PY=python

if not exist .venv-splat (
  echo Installing reconstruction tools, one time only...
  %PY% -m venv .venv-splat || goto :fail
  call .venv-splat\Scripts\activate.bat
  python -m pip install --upgrade pip >nul
  python -m pip install pycolmap torch numpy pillow scipy imageio-ffmpeg opencv-python-headless || goto :fail
) else (
  call .venv-splat\Scripts\activate.bat
)
python splat\build_scene.py %* || goto :fail
echo.
echo Done. Open Twinfield.exe or run_sim.bat and pick the scene.
pause
exit /b 0
:fail
echo.
echo Scene build failed. Scroll up for the error.
pause
exit /b 1
