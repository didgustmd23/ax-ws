@echo off
REM Install or refresh desktop shortcuts; no ROS build is performed.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0install-shortcuts.ps1" %*
if errorlevel 1 pause
