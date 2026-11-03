@echo off
REM Stop the owned AX processes and windows, then exit this CMD without leaving a prompt.
if not exist "%~dp0..\log\desktop_launcher\config.json" (
  powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0install-shortcuts.ps1"
  if errorlevel 1 exit /b 1
)
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0ax-launch.ps1" -Action Stop
exit /b %errorlevel%
