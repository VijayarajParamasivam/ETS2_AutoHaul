@echo off
rem Double-click: opens the AutoHaul launcher (pick a mode, Start, then press / in the game).
cd /d "%~dp0"
if not exist ".venv\Scripts\pythonw.exe" (
    echo No .venv here: set up AutoHaul first ^(see README.md^).
    pause
    exit /b 1
)
start "" ".venv\Scripts\pythonw.exe" "tools\launcher.py"
