# 화면 복구 전용: 사용자가 확인한 경우에만 전체 WSL을 재시작한다. 일반 Start/Stop과 구분한다.
param([switch]$CheckOnly)
$ErrorActionPreference = 'Stop'
$launcher = Join-Path $PSScriptRoot 'ax-launch.ps1'
try {
    if (-not (Test-Path -LiteralPath $launcher)) { throw 'AX launcher not found.' }
# 검사 모드는 UI나 WSL 종료 없이 복구 절차의 구성만 확인한다.
    if ($CheckOnly) {
        @{ok=$true;operation='Stop AX, restart WSL graphics, start AX';confirmation_required=$true;restart_performed=$false} | ConvertTo-Json -Compress
        return
    }
    Add-Type -AssemblyName System.Windows.Forms
    $message = 'WSL 그래픽 환경을 재시작하고 AX를 다시 실행할까요?' + "`r`n`r`n" +
        '모든 WSL 배포판과 Ubuntu 터미널이 종료됩니다. 먼저 진행 중인 작업을 저장하세요.' + "`r`n`r`n" +
        'Gazebo와 로봇 두 대를 다시 실행합니다. 리모컨은 별도 바로가기로 열어주세요.'
# WSL 종료는 다른 배포판/터미널에도 영향을 주므로 기본 선택을 No로 둔다.
    $answer = [Windows.Forms.MessageBox]::Show($message, 'AX 화면 복구',
        [Windows.Forms.MessageBoxButtons]::YesNo, [Windows.Forms.MessageBoxIcon]::Warning,
        [Windows.Forms.MessageBoxDefaultButton]::Button2)
    if ($answer -ne [Windows.Forms.DialogResult]::Yes) { return }
    # Allow normal AX cleanup, but do not wait indefinitely on a stuck startup.
# 고장난 시작 프로세스가 Stop을 막아도 복구가 무한 대기하지 않도록 15초만 기다린다.
    $cleanup = Start-Process -FilePath (Join-Path $PSHOME 'powershell.exe') -ArgumentList @(
        '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', ('"'+$launcher+'"'), '-Action', 'Stop'
    ) -WindowStyle Hidden -PassThru
    if (-not $cleanup.WaitForExit(15000)) {
        Stop-Process -Id $cleanup.Id -ErrorAction SilentlyContinue
    }
# 이 명령만 전체 WSL을 종료한다. 데이터 파일을 삭제하거나 재설치하지 않는다.
    & wsl.exe --shutdown
    if ($LASTEXITCODE -ne 0) { throw 'WSL shutdown failed.' }
# 재시작은 AX 기본 실행만 수행한다. 리모컨은 별도 바로가기로 연다.
    & $launcher -Action Start
} catch {
    Add-Type -AssemblyName System.Windows.Forms
    [Windows.Forms.MessageBox]::Show($_.Exception.Message, 'AX 화면 복구', 'OK', 'Error') | Out-Null
    exit 1
}
