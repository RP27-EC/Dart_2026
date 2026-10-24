@echo off
cd /d "%~dp0"
echo Starting Judge Simulatiion GUI...
uv run .\src\gui_app.py
pause
