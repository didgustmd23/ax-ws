@echo off
REM Open the browser remote only; Gazebo must already be running to move robots.
if not exist "%~dp0..\log\desktop_launcher\config.json" (
  powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0install-shortcuts.ps1"
  if errorlevel 1 exit /b 1
)
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0ax-launch.ps1" -Action Remote
exit /b %errorlevel%
