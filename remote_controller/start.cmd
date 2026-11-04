@echo off
REM Select a world, then start Ubuntu/Gazebo. Cancel leaves the session unchanged.
if not exist "%~dp0..\log\desktop_launcher\config.json" (
  powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0install-shortcuts.ps1"
  if errorlevel 1 exit /b 1
)
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0ax-launch.ps1" -Action ChooseWorld
exit /b %errorlevel%
