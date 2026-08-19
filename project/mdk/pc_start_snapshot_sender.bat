@echo off
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0pc_keyboard_snapshot_sender.ps1" -Port COM11
pause
