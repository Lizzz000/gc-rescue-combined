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
  "$here\test_usb_protocol.cpp" `
  "$here\test_tx_queue.cpp" `
  "$here\test_bridge_state.cpp" `
  "$here\test_arrival_point.cpp" `
  "$here\test_search_request.cpp" `
  "$package\state\bridge_state.cpp" `
  "$package\transport\transport.cpp" `
  "-o$out\bridge_host_tests.exe" *> $log
$compile = $LASTEXITCODE
if ($compile -ne 0) {
  Get-Content -LiteralPath $log
  Write-Host "log: $log"
  exit $compile
}

& "$out\bridge_host_tests.exe" *>> $log
$tests = $LASTEXITCODE
if ($tests -eq 0) {
  $suites = @(
    @{Name='bridge_runtime'; Includes=@("$here/runtime_stubs", $stubs, $here, $package, $src);
      Sources=@("$here/test_runtime_main.cpp", "$here/test_bridge_runtime.cpp", "$package/node/bridge_io.cpp", "$package/node/bridge_session.cpp", "$package/state/bridge_state.cpp", "$package/transport/transport.cpp")},
    @{Name='cdc_write'; Includes=@("$here/posix_stubs", $stubs, $here, $package, $src);
      Sources=@("$here/test_cdc_write.cpp", "$package/transport/cdc/cdc.cpp")},
    @{Name='s288_timing'; Includes=@("$here/posix_stubs", $here, $package);
      Sources=@("$here/test_s288_timing.cpp", "$package/gripper/s288_servo.cpp")}
  )
  foreach ($suite in $suites) {
    $flags = @($suite.Includes | ForEach-Object { "-I$_" })
    $binary = Join-Path $out ($suite.Name + '.exe')
    & g++ -std=c++17 -Wall -Wextra -finput-charset=UTF-8 @flags @($suite.Sources) -o $binary *>> $log
    $tests = $LASTEXITCODE
    if ($tests -ne 0) { break }
    & $binary *>> $log
    $tests = $LASTEXITCODE
    if ($tests -ne 0) { break }
  }
}
$watch.Stop()
Get-Content -LiteralPath $log
Write-Host ("elapsed_ms {0}" -f $watch.ElapsedMilliseconds)
Write-Host "log: $log"
exit $tests
