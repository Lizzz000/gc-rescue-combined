# Windows 本机（MSYS2/MinGW g++ + OpenCV pkg-config）编译并运行安全区测试；不改生产代码，产物放在 TEMP。
# 用法：run.ps1 [all|matrix|sequence|replay] [测试参数...]，例如 .\run.ps1 matrix --report
# 退出码 77 表示跳过（回放无数据）。
param([string]$Which = "all", [Parameter(ValueFromRemainingArguments = $true)][string[]]$TestArgs)
$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
# 系统 PATH 里可能有另一份 as.exe。链接器必须用这份 g++ 旁边的 binutils。
$env:PATH = "$(Split-Path (Get-Command g++).Source);$env:PATH"
$package = (Resolve-Path (Join-Path $here "..\..")).Path
$hostDir = Join-Path $here "..\host"
$out = Join-Path $env:TEMP "gc_rescue_field_tests"
New-Item -ItemType Directory -Force -Path $out | Out-Null

& python "$here\field_config.py" "$out\field_config_values.inc"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$cflags = (& pkg-config --cflags opencv4) -split " " | Where-Object { $_ }
$libs = (& pkg-config --libs opencv4) -split " " | Where-Object { $_ }
$common = @("-std=c++17", "-O2", "-Wall", "-Wextra", "-finput-charset=UTF-8",
  "-DFIELD_DATA_DIR=`"$($here -replace '\\','/')/data`"", "-I$here", "-I$hostDir", "-I$package", "-I$out") + $cflags
$sources = @(Get-ChildItem "$package\field\*.cpp") + @(Get-ChildItem "$package\field\opencv\*.cpp") |
  ForEach-Object { $_.FullName }

$status = 0
foreach ($name in @("matrix", "sequence", "replay")) {
  if ($Which -ne "all" -and $Which -ne $name) { continue }
  $exe = Join-Path $out "test_field_$name.exe"
  & g++ @common "$here\test_field_$name.cpp" @sources @libs "-o$exe"
  if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
  $watch = [Diagnostics.Stopwatch]::StartNew()
  & $exe @TestArgs
  $code = $LASTEXITCODE
  $watch.Stop()
  Write-Host ("test_field_{0} exit={1} elapsed_ms={2}" -f $name, $code, $watch.ElapsedMilliseconds)
  if ($code -ne 0 -and $code -ne 77) { $status = 1 }
}
exit $status
