# 설치기는 실제 설치/빌드를 수행하지 않고 현재 PC의 경로로 바탕화면 .lnk를 만든다.
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
# 명시한 인자 > 기존 로컬 설정 > 설치된 Ubuntu 자동 탐색 순서로 배포판을 선택한다.
if (-not $Distro) {
    if ($previous) { $Distro = $previous.distro }
    else {
        $distros = @(& wsl.exe --list --quiet | ForEach-Object { ($_ -replace "`0", '').Trim() } | Where-Object { $_ })
        $Distro = $distros | Where-Object { $_ -eq 'Ubuntu-24.04' } | Select-Object -First 1
        if (-not $Distro) { $Distro = $distros | Where-Object { $_ -like '*Ubuntu*' } | Select-Object -First 1 }
        if (-not $Distro) { throw 'No Ubuntu WSL distribution found. Install Ubuntu 24.04 first.' }
    }
}
# 작업 폴더 이름은 자동 탐색 후보일 뿐이다. 다른 경로는 -Workspace로 지정한다.
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
# 설정은 저장소 log/ 아래에만 기록한다. 다른 사람에게 넘길 Git 소스에는 개인 경로를 넣지 않는다.
New-Item -ItemType Directory -Path $configDir -Force | Out-Null
@{distro=$Distro;workspace=$Workspace;tool=$Tool;port=$Port} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $configDir 'config.json') -Encoding UTF8
$desktopDir = [IO.Path]::GetFullPath($ShortcutDirectory)
New-Item -ItemType Directory -Path $desktopDir -Force | Out-Null
$shell = New-Object -ComObject WScript.Shell
$names = @{Start=('AX '+[char]0xC2E4+[char]0xD589); Stop=('AX '+[char]0xC885+[char]0xB8CC); Remote=('AX '+[char]0xB9AC+[char]0xBAA8+[char]0xCEE8)}
$names.Recover = 'AX '+[char]0xD654+[char]0xBA74+' '+[char]0xBCF5+[char]0xAD6C
# 바로가기는 이 PC에서 생성하므로 실행 시 절대경로를 써도 저장소의 이식성과 모순되지 않는다.
foreach ($action in @('Start', 'Remote', 'Stop', 'Recover')) {
    $shortcut = $shell.CreateShortcut((Join-Path $desktopDir ($names[$action]+'.lnk')))
    $shortcut.TargetPath = Join-Path $PSHOME 'powershell.exe'
    $shortcut.Arguments = '-NoProfile -ExecutionPolicy Bypass -File "'+(Join-Path $PSScriptRoot 'ax-launch.ps1')+'" -Action '+$action
    if ($action -eq 'Recover') {
        $shortcut.Arguments = '-NoProfile -ExecutionPolicy Bypass -File "'+(Join-Path $PSScriptRoot 'recover.ps1')+'"'
    }
    $shortcut.WorkingDirectory = $repoDir
    $shortcut.WindowStyle = 7
    $shortcut.Description = 'AX simulation '+$action
    $shortcut.Save()
    Write-Output ('Created: '+(Join-Path $desktopDir ($names[$action]+'.lnk')))
}
