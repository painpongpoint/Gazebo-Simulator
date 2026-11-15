@echo off
wsl.exe -d Ubuntu-24.04 --cd "%~dp0." -- python3 car_controls.py
if errorlevel 1 pause
