# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
# Export actual native commands and an editor workspace without changing .vscode.
[CmdletBinding()]
param(
  [Parameter(Mandatory=$true)][string[]]$BuildBase,
  [string]$Output = 'build/unified-clangd',
  [string]$Clangd = '',
  [string]$Python = ''
)
$ErrorActionPreference = 'Stop'
$project = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
function Resolve-Program([string]$Configured, [string]$Name) {
  if ($Configured) {
    if (Test-Path -LiteralPath $Configured -PathType Leaf) { return (Resolve-Path -LiteralPath $Configured).Path }
    return (Get-Command -Name $Configured -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
  }
  return (Get-Command -Name $Name -CommandType Application -ErrorAction Stop | Select-Object -First 1).Source
}
$clangdProgram = Resolve-Program $Clangd 'clangd.exe'
$pythonProgram = Resolve-Program $Python 'python.exe'
$databaseArgs = @('-B', (Join-Path $project 'tools/clangd/clangd_database.py'), '--workspace', $project, '--output', $Output)
foreach ($build in $BuildBase) { $databaseArgs += @('--build-base', $build) }
& $pythonProgram @databaseArgs
if ($LASTEXITCODE -ne 0) { throw 'clangd database export failed; editor workspace was not updated.' }

$outputPath = [IO.Path]::GetFullPath((Join-Path $project $Output))
$buildRoot = [IO.Path]::GetFullPath((Join-Path $project 'build'))
if (-not $outputPath.StartsWith($buildRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
  throw 'Editor workspace must be generated inside project/build.'
}
$database = Get-Content -LiteralPath (Join-Path $outputPath 'compile_commands.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$drivers = @($database | ForEach-Object { $_.arguments[0] } | Sort-Object -Unique)
if (-not $drivers.Count) { throw 'No compiler commands in the exported database.' }
foreach ($driver in $drivers) {
  if (-not [IO.Path]::IsPathRooted($driver) -or -not (Test-Path -LiteralPath $driver -PathType Leaf)) {
    throw "Expected the native compiler's absolute executable path: $driver"
  }
}
$arguments = @('--background-index', '-j=2',
  ('--query-driver=' + (($drivers | ForEach-Object { $_.Replace('\','/') }) -join ',')))
# 默认由根和包级 .clangd 选库；仅显式自定义输出时覆盖生成的整个工作区。
if (-not $outputPath.Equals((Join-Path $buildRoot 'unified-clangd'), [StringComparison]::OrdinalIgnoreCase)) {
  $arguments += '--compile-commands-dir=' + $outputPath.Replace('\','/')
}
# GCC的cc1plus可能位于另一个目录；只在此clangd子进程中优先使用其配套DLL。
$driverBins = @($drivers | ForEach-Object { (Split-Path -Parent $_).Replace('/','\') } | Sort-Object -Unique)
$launcher = Join-Path $outputPath 'clangd-launch.cmd'
$binaryForCmd = $clangdProgram.Replace('%','%%')
$pathForCmd = (($driverBins -join ';').Replace('%','%%'))
$launcherText = "@echo off`r`nsetlocal DisableDelayedExpansion`r`nset `"PATH=$pathForCmd;%PATH%`"`r`n`"$binaryForCmd`" %*`r`nexit /b %errorlevel%`r`n"
[IO.File]::WriteAllText($launcher, $launcherText, (New-Object Text.UTF8Encoding($false)))
$workspace = [ordered]@{
  folders = @([ordered]@{name='Jetson'; path=$project.Replace('\','/')})
  settings = [ordered]@{
    'clangd.path' = $launcher.Replace('\','/')
    'clangd.useScriptAsExecutable' = $true
    'clangd.checkUpdates' = $false
    'clangd.arguments' = $arguments
    'C_Cpp.intelliSenseEngine' = 'disabled'
  }
  extensions = [ordered]@{recommendations=@('llvm-vs-code-extensions.vscode-clangd')}
}
$target = Join-Path $outputPath 'Jetson.code-workspace'
$temporary = $target + '.tmp'
[IO.File]::WriteAllText($temporary, (($workspace | ConvertTo-Json -Depth 12) + "`n"), (New-Object Text.UTF8Encoding($false)))
Move-Item -LiteralPath $temporary -Destination $target -Force
Write-Output "Generated editor workspace: $target"
Write-Output "Open it with: code $target"
