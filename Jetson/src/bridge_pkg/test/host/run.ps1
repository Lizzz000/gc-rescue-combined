# 本机 gcc 编译 bridge_pkg 纯核心并跑行为基线。不改生产代码，产物放在 TEMP。
$ErrorActionPreference = "Stop"
$PSNativeCommandUseErrorActionPreference = $false

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
# 系统 PATH 里可能有另一份 as.exe。链接器必须用这份 g++ 旁边的 binutils。
$env:PATH = "$(Split-Path (Get-Command g++).Source);$env:PATH"
$package = (Resolve-Path (Join-Path $here "..\..")).Path
# 头文件写成 <interface_pkg/...>，所以 -I 指到 src，而不是包目录本身。
$src = (Resolve-Path (Join-Path $package "..")).Path
$stubs = Join-Path $here "stubs"
$out = Join-Path $env:TEMP "gc_rescue_host\bridge_pkg"
$log = Join-Path $out "test.log"
New-Item -ItemType Directory -Force -Path $out | Out-Null

$watch = [Diagnostics.Stopwatch]::StartNew()
& g++ -std=c++17 -Wall -Wextra -finput-charset=UTF-8 `
  "-I$here" "-I$stubs" "-I$package" "-I$src" `
  "$here\test_main.cpp" `
  "$here\test_can_protocol.cpp" `
  "$here\test_tx_queue.cpp" `
  "$here\test_bridge_state.cpp" `
  "$package\state\bridge_state.cpp" `
  "-o$out\bridge_host_tests.exe" *> $log
$compile = $LASTEXITCODE
if ($compile -ne 0) {
  Get-Content -LiteralPath $log
  Write-Host "log: $log"
  exit $compile
}

& "$out\bridge_host_tests.exe" *>> $log
$tests = $LASTEXITCODE
$watch.Stop()
Get-Content -LiteralPath $log
Write-Host ("elapsed_ms {0}" -f $watch.ElapsedMilliseconds)
Write-Host "log: $log"
exit $tests
