$ErrorActionPreference = 'Stop'
$path = Join-Path $env:APPDATA 'Code\User\profiles\-2b97e5e1\keybindings.json'
$entries = @()
if (Test-Path -LiteralPath $path) {
    $existing = ConvertFrom-Json -InputObject (Get-Content -LiteralPath $path -Raw)
    foreach ($entry in $existing) {
        # Repair an array wrapper produced by an older installer version.
        if ($null -eq $entry.command -and $null -ne $entry.value -and $null -ne $entry.Count) {
            foreach ($nested in $entry.value) { $entries += $nested }
        } else {
            $entries += $entry
        }
    }
}
$entries = @($entries | Where-Object {
    -not ($_.command -eq 'workbench.action.tasks.runTask' -and ($_.args -eq 'Keil: Flash' -or $_.args -eq 'Keil: Debug'))
})
$when = '!inDebugMode'
$entries += @{ key = 'f5'; command = 'workbench.action.tasks.runTask'; args = 'Keil: Flash'; when = $when }
$entries += @{ key = 'f7'; command = 'workbench.action.tasks.runTask'; args = 'Keil: Debug'; when = $when }
$json = ConvertTo-Json -InputObject @($entries) -Depth 10
Set-Content -LiteralPath $path -Value $json -Encoding UTF8
