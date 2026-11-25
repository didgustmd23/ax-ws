param(
    [string]$Distro,
    [string]$Workspace,
    [ValidateSet('robotiq', 'epick')][string]$Tool,
    [ValidateRange(1024, 65535)][int]$Port,
    [string]$ShortcutDirectory
)

$ErrorActionPreference = 'Stop'
$shortcutOptions = @{} + $PSBoundParameters
$repo = Split-Path -Parent $PSScriptRoot
$configPath = Join-Path $repo 'log/desktop_launcher/config.json'
$config = if (Test-Path -LiteralPath $configPath) { Get-Content -LiteralPath $configPath -Raw | ConvertFrom-Json }
if (-not $Distro) { $Distro = if ($config.distro) { $config.distro } else { 'Ubuntu-24.04' } }
if (-not $Workspace -and $config.distro -eq $Distro) { $Workspace = $config.workspace }
if ($Workspace -and $Workspace -notmatch '^(~/|/)') { $Workspace = '~/' + $Workspace }
if ($config.tool -and -not $shortcutOptions.ContainsKey('Tool')) { $shortcutOptions.Tool = $config.tool }
if ($config.port -and -not $shortcutOptions.ContainsKey('Port')) { $shortcutOptions.Port = $config.port }

$distros = @(& wsl.exe --list --quiet | ForEach-Object { ($_ -replace "`0", '').Trim() })
if ($Distro -notin $distros) {
    Write-Host "Installing $Distro. Complete the Ubuntu user setup if prompted."
    & wsl.exe --install -d $Distro
    $distros = @(& wsl.exe --list --quiet | ForEach-Object { ($_ -replace "`0", '').Trim() })
    if ($Distro -notin $distros) {
        throw "Finish WSL setup, restart Windows if requested, then run install.cmd again. If Windows reports elevation is required, run install.cmd as administrator."
    }
}

Push-Location -LiteralPath $repo
try {
    $installArgs = @()
    if ($Workspace) { $installArgs += $Workspace }
    & wsl.exe -d $Distro --exec bash remote_controller/prepare-workspace.sh @installArgs
    if ($LASTEXITCODE -ne 0) { throw 'AX workspace setup failed. See the Ubuntu error above, then run install.cmd again.' }
}
finally {
    Pop-Location
}

$shortcutOptions.Distro = $Distro
if ($Workspace) { $shortcutOptions.Workspace = $Workspace }
& (Join-Path $PSScriptRoot 'install-shortcuts.ps1') @shortcutOptions
