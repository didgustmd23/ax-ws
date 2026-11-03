param(
    [ValidateSet('Start', 'Remote', 'Stop', 'Status')][string]$Action = 'Start',
    [string]$Distro,
    [string]$Workspace
)
$ErrorActionPreference = 'Stop'
$repoDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$configPath = Join-Path $repoDir 'log/desktop_launcher/config.json'
$remoteTitle = 'AX '+[char]0xB85C+[char]0xBD07+' '+[char]0xB9AC+[char]0xBAA8+[char]0xCEE8
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
    Push-Location -LiteralPath $repoDir
    try {
        $raw = & wsl.exe -d $Distro --exec python3 tools/desktop_session.py ($Action.ToLowerInvariant()) --workspace $Workspace --tool $tool --port $port
        $exitCode = $LASTEXITCODE
    } finally { Pop-Location }
    $result = ($raw | Select-Object -Last 1) | ConvertFrom-Json
    if ($exitCode -ne 0 -or -not $result.ok) { throw $result.error }
    if ($Action -eq 'Start') {
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
    } elseif ($Action -eq 'Remote') {
        $shell = New-Object -ComObject WScript.Shell
        $edge = Join-Path ${env:ProgramFiles(x86)} 'Microsoft/Edge/Application/msedge.exe'
        if ($shell.AppActivate($remoteTitle)) {
            $shell.SendKeys('{F5}')
        } elseif (Test-Path -LiteralPath $edge) {
            Start-Process -FilePath $edge -ArgumentList @("--app=$($result.url)", '--window-size=1000,900') -WindowStyle Normal
        } else { Start-Process $result.url }
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
