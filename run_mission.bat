@echo off
REM Run a Python mission script against the simulator.
REM   run_mission.bat                         -> square mission, streamed to an open Twinfield window
REM   run_mission.bat splat\sim\examples\replay_real_flight.py
REM   run_mission.bat my_script.py --headless -> no window, just the report
REM Open Twinfield.exe (or run_sim.bat) first to watch it fly.
cd /d "%~dp0"
set SCRIPT=%~1
if "%SCRIPT%"=="" set SCRIPT=splat\sim\examples\square_mission.py
if /i "%~2"=="--headless" (
  python "%SCRIPT%"
) else (
  python "%SCRIPT%" --viz http://127.0.0.1:8800
)
pause
