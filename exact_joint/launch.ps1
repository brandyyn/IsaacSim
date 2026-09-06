param([string]$RuntimeRoot = $env:ISAAC_SIM_DIR)
$ErrorActionPreference = 'Stop'
if (-not $RuntimeRoot) { throw 'Pass -RuntimeRoot or set ISAAC_SIM_DIR to the built release directory.' }
$jointRuntime = (Resolve-Path -LiteralPath $RuntimeRoot).Path
$jointProject = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$jointKit = Join-Path $jointRuntime 'kit\kit.exe'
$jointExperience = Join-Path $jointRuntime 'apps\isaacsim.exp.full.kit'
if (-not (Test-Path -LiteralPath $jointKit)) { throw "Missing Kit executable: $jointKit" }
if (-not (Test-Path -LiteralPath $jointExperience)) { throw "Missing experience: $jointExperience" }
$env:PANEL_CREASE_PROJECT_ROOT = $jointProject
$jointArguments = @(
    ('"' + $jointExperience + '"'), '--no-ros-env', '--enable', 'isaacsim.code_editor.python_server',
    '--/app/window/width=1600', '--/app/window/height=1000',
    '--exec', ('"' + (Join-Path $PSScriptRoot 'open_gui.py') + '"')
)
Start-Process -FilePath $jointKit -ArgumentList $jointArguments -WorkingDirectory $jointRuntime -WindowStyle Normal
