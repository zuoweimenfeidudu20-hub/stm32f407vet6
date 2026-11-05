param([Parameter(Mandatory = $true)][ValidateSet('build', 'flash', 'debug')][string]$Action)
$ErrorActionPreference = 'Stop'
$uv = 'C:\Keil_v5\UV4\UV4.exe'
$project = Join-Path $PSScriptRoot '..\MDK-ARM\car_f4.uvprojx'
if (-not (Test-Path -LiteralPath $uv)) { throw "Keil not found: $uv. Edit .vscode/keil.ps1 to set its path." }
if ($Action -eq 'debug') {
    Start-Process -FilePath $uv -ArgumentList @('-d', ('"{0}"' -f $project), '-t', 'car_f4')
    exit 0
}
$log = Join-Path $PSScriptRoot ("keil-$Action.log")
if (Test-Path -LiteralPath $log) { Remove-Item -LiteralPath $log }
$option = if ($Action -eq 'build') { '-b' } else { '-f' }
$process = Start-Process -FilePath $uv -ArgumentList @($option, ('"{0}"' -f $project), '-t', 'car_f4', '-o', ('"{0}"' -f $log)) -Wait -PassThru
if (Test-Path -LiteralPath $log) { Get-Content -LiteralPath $log }
else { Write-Error "Keil did not produce a $Action log."; exit 1 }
if ($process.ExitCode -ge 2) { exit $process.ExitCode }
exit 0
