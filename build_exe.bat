@echo off
REM Builds dist\Twinfield.exe: a standalone flight-sim viewer that runs offline.
REM Copies every scene in scenes\ next to the exe so the Scene menu can find them.
cd /d "%~dp0"
set PY=python
where py >nul 2>nul && set PY=py -3
if not exist .venv-app (
  %PY% -m venv .venv-app || goto :fail
)
call .venv-app\Scripts\activate.bat
python -m pip install --upgrade pip >nul
python -m pip install pyinstaller || goto :fail
copy /y splat\site\index.html splat\app\web\index.html >nul
pyinstaller --noconfirm --onefile --console --name Twinfield --add-data "splat\app\web;web" splat\app\sim_launcher.py || goto :fail
if exist scenes robocopy scenes dist\scenes /E /XD work /NFL /NDL /NJH /NJS >nul
echo.
echo Built dist\Twinfield.exe (scenes copied to dist\scenes)
pause
exit /b 0
:fail
echo.
echo Build failed. Scroll up for the error.
pause
exit /b 1
