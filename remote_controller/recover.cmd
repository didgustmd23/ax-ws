@echo off
REM Ask for confirmation before restarting all WSL distributions and reopening AX.
if not exist "%~dp0..\log\desktop_launcher\config.json" (
  powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0install-shortcuts.ps1"
  if errorlevel 1 exit /b 1
)
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0recover.ps1"
exit /b %errorlevel%
