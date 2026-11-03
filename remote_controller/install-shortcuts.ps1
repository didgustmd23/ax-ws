param(
    [string]$Distro,
    [string]$Workspace,
    [ValidateSet('robotiq', 'epick')][string]$Tool = 'robotiq',
    [ValidateRange(1024, 65535)][int]$Port = 8765,
    [string]$ShortcutDirectory = [Environment]::GetFolderPath('Desktop')
)
$ErrorActionPreference = 'Stop'
$repoDir = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$configDir = Join-Path $repoDir 'log/desktop_launcher'
$previousPath = Join-Path $configDir 'config.json'
$previous = if (Test-Path -LiteralPath $previousPath) { Get-Content -LiteralPath $previousPath -Raw | ConvertFrom-Json } else { $null }
if (-not $Distro) {
    if ($previous) { $Distro = $previous.distro }
    else {
        $distros = @(& wsl.exe --list --quiet | ForEach-Object { ($_ -replace "`0", '').Trim() } | Where-Object { $_ })
        $Distro = $distros | Where-Object { $_ -eq 'Ubuntu-24.04' } | Select-Object -First 1
        if (-not $Distro) { $Distro = $distros | Where-Object { $_ -like '*Ubuntu*' } | Select-Object -First 1 }
        if (-not $Distro) { throw 'No Ubuntu WSL distribution found. Install Ubuntu 24.04 first.' }
    }
}
if (-not $Workspace) {
    if ($previous -and $previous.distro -eq $Distro) { $Workspace = $previous.workspace }
    else {
        $probe = "from pathlib import Path; print(next((str(p) for p in [Path.home()/'ax-play-ws', Path.home()/'ax-ws'] if (p/'src'/'ax_description').is_dir()), ''))"
        $Workspace = (& wsl.exe -d $Distro --exec python3 -c $probe | Select-Object -Last 1)
        if ($LASTEXITCODE -ne 0 -or -not $Workspace) {
            throw 'No AX Ubuntu workspace found. Prepare it first, or rerun install-shortcuts.ps1 -Workspace ~/YOUR_FOLDER.'
        }
    }
}
New-Item -ItemType Directory -Path $configDir -Force | Out-Null
@{distro=$Distro;workspace=$Workspace;tool=$Tool;port=$Port} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $configDir 'config.json') -Encoding UTF8
$desktopDir = [IO.Path]::GetFullPath($ShortcutDirectory)
New-Item -ItemType Directory -Path $desktopDir -Force | Out-Null
$shell = New-Object -ComObject WScript.Shell
$names = @{Start=('AX '+[char]0xC2E4+[char]0xD589); Stop=('AX '+[char]0xC885+[char]0xB8CC); Remote=('AX '+[char]0xB9AC+[char]0xBAA8+[char]0xCEE8)}
foreach ($action in @('Start', 'Remote', 'Stop')) {
    $shortcut = $shell.CreateShortcut((Join-Path $desktopDir ($names[$action]+'.lnk')))
    $shortcut.TargetPath = Join-Path $PSHOME 'powershell.exe'
    $shortcut.Arguments = '-NoProfile -ExecutionPolicy Bypass -File "'+(Join-Path $PSScriptRoot 'ax-launch.ps1')+'" -Action '+$action
    $shortcut.WorkingDirectory = $repoDir
    $shortcut.WindowStyle = 7
    $shortcut.Description = 'AX simulation '+$action
    $shortcut.Save()
    Write-Output ('Created: '+(Join-Path $desktopDir ($names[$action]+'.lnk')))
}
