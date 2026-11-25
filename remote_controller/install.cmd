@echo off
setlocal
REM Prepare WSL and the AX workspace before creating desktop shortcuts.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" %*
set "AX_INSTALL_EXIT=%errorlevel%"
if not "%AX_INSTALL_EXIT%"=="0" pause
exit /b %AX_INSTALL_EXIT%
