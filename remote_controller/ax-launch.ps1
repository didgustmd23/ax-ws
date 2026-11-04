# Windows 진입점: Start는 Ubuntu/Gazebo만, Remote는 리모컨만, Stop은 AX 전용 창까지 종료한다.
param(
    [ValidateSet('Start', 'Remote', 'Stop', 'Status', 'ChooseWorld')][string]$Action = 'Start',
    [string]$Distro,
    [string]$Workspace
)
$ErrorActionPreference = 'Stop'
$repoDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
# 개인별 경로는 Git에서 제외되는 log 설정에 저장하며 저장소 위치는 스크립트에서 계산한다.
$configPath = Join-Path $repoDir 'log/desktop_launcher/config.json'
$remoteTitle = 'AX '+[char]0xB85C+[char]0xBD07+' '+[char]0xB9AC+[char]0xBAA8+[char]0xCEE8
# 창 제목이 정확히 일치하는 AX 앱만 WM_CLOSE로 닫는다. 일반 Edge 창이나 다른 터미널은 종료하지 않는다.
function Close-RemoteWindows {
    if (-not ('AxRemoteWindow' -as [type])) {
        Add-Type @'
using System;
using System.Text;
using System.Runtime.InteropServices;
public static class AxRemoteWindow {
    private delegate bool Callback(IntPtr window, IntPtr data);
    [DllImport("user32.dll")] private static extern bool EnumWindows(Callback callback, IntPtr data);
    [DllImport("user32.dll", CharSet=CharSet.Unicode)] private static extern int GetWindowText(IntPtr window, StringBuilder text, int size);
    [DllImport("user32.dll")] private static extern bool PostMessage(IntPtr window, uint message, IntPtr wParam, IntPtr lParam);
    public static int Close(string title) {
        int count = 0;
        EnumWindows((window, data) => {
            var text = new StringBuilder(1024);
            GetWindowText(window, text, text.Capacity);
            if (text.ToString() == title && PostMessage(window, 0x0010, IntPtr.Zero, IntPtr.Zero)) count++;
            return true;
        }, IntPtr.Zero);
        return count;
    }
}
'@
    }
    return [AxRemoteWindow]::Close($remoteTitle)
}
try {
    $config = if (Test-Path -LiteralPath $configPath) { Get-Content -LiteralPath $configPath -Raw | ConvertFrom-Json } else { $null }
    if (-not $Distro) { $Distro = if ($config) { $config.distro } else { 'Ubuntu-24.04' } }
    if (-not $Workspace) { $Workspace = if ($config) { $config.workspace } else { '~/ax-play-ws' } }
    $tool = if ($config) { $config.tool } else { 'robotiq' }
    $port = if ($config) { [int]$config.port } else { 8765 }
    $worldArguments = @()
    if ($Action -eq 'ChooseWorld') {
        Add-Type -AssemblyName System.Windows.Forms
        Add-Type -AssemblyName System.Drawing
        $form = New-Object Windows.Forms.Form
        $form.Text = 'AX 월드 선택'
        $form.ClientSize = New-Object Drawing.Size(450, 220)
        $form.StartPosition = 'CenterScreen'
        $form.FormBorderStyle = 'FixedDialog'
        $form.MaximizeBox = $false
        $form.MinimizeBox = $false
        $label = New-Object Windows.Forms.Label
        $label.Text = '실행할 월드를 선택하세요.'
        $label.SetBounds(20, 18, 410, 24)
        $list = New-Object Windows.Forms.ComboBox
        $list.DropDownStyle = 'DropDownList'
        $list.SetBounds(20, 50, 410, 30)
        [void]$list.Items.Add('기존 시험 월드 (30 x 30 m)')
        [void]$list.Items.Add('창고 월드 (20 x 15 m)')
        $list.SelectedIndex = 0
        $note = New-Object Windows.Forms.Label
        $note.Text = "다른 월드가 실행 중이면 AX 세션을 종료하고 전환합니다.`r`n같은 월드는 다시 생성하지 않습니다."
        $note.SetBounds(20, 95, 410, 45)
        $open = New-Object Windows.Forms.Button
        $open.Text = '열기'
        $open.SetBounds(220, 165, 100, 32)
        $open.DialogResult = [Windows.Forms.DialogResult]::OK
        $cancel = New-Object Windows.Forms.Button
        $cancel.Text = '취소'
        $cancel.SetBounds(330, 165, 100, 32)
        $cancel.DialogResult = [Windows.Forms.DialogResult]::Cancel
        $form.Controls.AddRange(@($label, $list, $note, $open, $cancel))
        $form.AcceptButton = $open
        $form.CancelButton = $cancel
        try {
            if ($form.ShowDialog() -ne [Windows.Forms.DialogResult]::OK) { return }
            $world = @('ax_test.world', 'planned/ax_warehouse.world')[$list.SelectedIndex]
        } finally { $form.Dispose() }
        $worldArguments = @('--world', $world, '--switch-world')
        $Action = 'Start'
    }
# WSL의 초기 작업 폴더도 저장소 루트로 맞춰 tools 상대경로가 사용자 이름에 의존하지 않게 한다.
    Push-Location -LiteralPath $repoDir
    try {
        $raw = & wsl.exe -d $Distro --exec python3 tools/desktop_session.py ($Action.ToLowerInvariant()) --workspace $Workspace --tool $tool --port $port @worldArguments
        $exitCode = $LASTEXITCODE
    } finally { Pop-Location }
# Python은 stdout 마지막 줄에 JSON을 출력한다. JSON의 ok와 프로세스 종료 코드를 모두 확인한다.
    $result = ($raw | Select-Object -Last 1) | ConvertFrom-Json
    if ($exitCode -ne 0 -or -not $result.ok) { throw $result.error }
# AX 터미널은 전용 창/고정 제목으로 열어 Stop에서 다른 사용자 탭과 구분한다.
    if ($Action -eq 'Start') {
        if ($result.world_switched) {
            [void](Close-RemoteWindows)
            [void][AxRemoteWindow]::Close('Ubuntu 24.04 - AX')
        }
        $shell = New-Object -ComObject WScript.Shell
        if (-not $shell.AppActivate('Ubuntu 24.04 - AX')) {
            $terminal = Get-Command wt.exe -ErrorAction SilentlyContinue
            $terminalArgs = @('wsl.exe', '-d', $Distro, '--exec', 'bash', '--rcfile', ('"'+$result.terminal_rcfile+'"'), '-i')
            if ($terminal) {
                Start-Process -FilePath $terminal.Source -ArgumentList (@('-w','new','new-tab','--title','"Ubuntu 24.04 - AX"','--suppressApplicationTitle') + $terminalArgs) -WindowStyle Normal
            } else {
                Start-Process -FilePath 'wsl.exe' -ArgumentList $terminalArgs[1..($terminalArgs.Count-1)] -WindowStyle Normal
            }
        }
# 이미 있는 리모컨은 새 창을 늘리지 않고 새로고침해 재시작된 서버 토큰을 받는다.
    } elseif ($Action -eq 'Remote') {
        $shell = New-Object -ComObject WScript.Shell
        $edge = Join-Path ${env:ProgramFiles(x86)} 'Microsoft/Edge/Application/msedge.exe'
        if ($shell.AppActivate($remoteTitle)) {
            $shell.SendKeys('{F5}')
        } elseif (Test-Path -LiteralPath $edge) {
            Start-Process -FilePath $edge -ArgumentList @("--app=$($result.url)", '--window-size=1000,900') -WindowStyle Normal
        } else { Start-Process $result.url }
# ROS 프로세스 종료가 성공한 뒤 Windows 리모컨/AX 터미널 창을 정리한다.
    } elseif ($Action -eq 'Stop') {
        $closedWindows = Close-RemoteWindows
        $result | Add-Member -NotePropertyName remote_windows_closed -NotePropertyValue $closedWindows
        $result | Add-Member -NotePropertyName ax_terminal_windows_closed -NotePropertyValue ([AxRemoteWindow]::Close('Ubuntu 24.04 - AX'))
    }
    $result | ConvertTo-Json -Compress
} catch {
    $message = $_.Exception.Message
    Add-Type -AssemblyName System.Windows.Forms
    [Windows.Forms.MessageBox]::Show($message, 'AX launcher', 'OK', 'Error') | Out-Null
    Write-Error $message
    exit 1
}
