$ErrorActionPreference = 'Stop'
$path = Join-Path $env:APPDATA 'Code\User\keybindings.json'
$entries = @()
if (Test-Path -LiteralPath $path) {
    $entries = @(Get-Content -LiteralPath $path -Raw | ConvertFrom-Json)
}
$when = 'resource =~ /[\\\/]project[\\\/]/i && !inDebugMode'
$entries = @($entries | Where-Object { -not ($_.command -eq 'workbench.action.tasks.runTask' -and ($_.args -eq 'Keil: Flash' -or $_.args -eq 'Keil: Debug') -and $_.when -eq $when) })
$entries += @{ key = 'f5'; command = 'workbench.action.tasks.runTask'; args = 'Keil: Flash'; when = $when }
$entries += @{ key = 'f7'; command = 'workbench.action.tasks.runTask'; args = 'Keil: Debug'; when = $when }
$entries | ConvertTo-Json -Depth 10 | Set-Content -LiteralPath $path -Encoding UTF8
