@echo off
REM Opens the Godot client project in the Godot 3.5.3 editor.
REM Add a drone with URL mock:__0 (or mock:__0-circle) once run_server.bat is up.
cd /d "%~dp0"
start "" "tools\godot\Godot_v3.5.3-stable_win64.exe" --path "godot client" -e
