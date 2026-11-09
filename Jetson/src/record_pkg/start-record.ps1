param(
    [string]$Board = 'jetson-usb',
    [string]$BoardBinary = '/home/nvidia/WorkSpace/GC_Rescue/install/record-network/lib/record_pkg/camera_record',
    [string]$OutputDir = 'D:\A1_Workspace\GC_Rescue\MP4',
    [string]$MediaBin = 'D:\Software\msys64\ucrt64\bin',
    [int]$Port = 8090,
    [int]$TunnelPort = 18091,
    [int]$CameraPort = 8091,
    [switch]$NoBrowser
)
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)
$repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
$binary = Join-Path $repo 'build\record-pc\record_console.exe'
if (-not (Test-Path -LiteralPath $binary)) { throw "先按 record_pkg README 构建电脑程序：$binary" }
foreach ($number in @($Port, $TunnelPort)) {
    $probe = [Net.Sockets.TcpClient]::new()
    try { $probe.Connect('127.0.0.1', $number); throw "本机端口$number已占用，请使用已经打开的控制台或先结束旧实例。" }
    catch [Net.Sockets.SocketException] { }
    finally { $probe.Dispose() }
}
$output = [IO.Path]::GetFullPath($OutputDir).TrimEnd('\')
$log = Join-Path $repo ('log\record-console\' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0,8))
New-Item -ItemType Directory -Path $log -Force | Out-Null
$env:Path = "$MediaBin;" + $env:Path
$ssh = $null
$pc = $null
$token = $null
$base = "http://127.0.0.1:$Port"
# SSH仅承载视频字节和相机进程；不在板端建立录像目录或临时视频。
$sshArgs = @('-n','-tt','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','ExitOnForwardFailure=yes',
    '-o','ConnectTimeout=5','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2',
    '-L',"127.0.0.1:${TunnelPort}:127.0.0.1:$CameraPort",
    $Board,"exec $BoardBinary --bind 127.0.0.1 --port $CameraPort")
function Start-OwnedSsh {
    # 每次启动覆盖同一对日志，重连不新增日志文件。
    Start-Process -FilePath "$env:WINDIR\System32\OpenSSH\ssh.exe" -ArgumentList $sshArgs -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput (Join-Path $log 'camera.log') -RedirectStandardError (Join-Path $log 'ssh.log')
}
try {
    $pcArgs = @('--output-dir',('"' + $output + '"'),'--port',$Port,'--camera-host','127.0.0.1','--camera-port',$TunnelPort)
    $pc = Start-Process -FilePath $binary -ArgumentList $pcArgs -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput (Join-Path $log 'console.log') -RedirectStandardError (Join-Path $log 'encoder.log')
    for ($i=0; $i -lt 100; $i++) {
        if ($pc.HasExited) { throw "启动失败，请看 $log" }
        try { $state = Invoke-RestMethod "$base/api/status" -TimeoutSec 1; $token = $state.token; break } catch { Start-Sleep -Milliseconds 100 }
    }
    if (-not $token) { throw '电脑控制台未能启动。' }
    # 先取得PC停止令牌；SSH启动抛错时finally也能关闭本次控制台。
    $ssh = Start-OwnedSsh
    Write-Host "录像控制台：$base"
    Write-Host "保存目录：$output"
    Write-Host '在网页开始/停止录像；关闭网页仍可继续录像。此窗口Ctrl+C会收尾并退出。'
    if (-not $NoBrowser) { Start-Process $base }
    while (-not $pc.HasExited) {
        if (-not $ssh -or $ssh.HasExited) {
            if ($ssh) {
                # 等旧进程的重定向输出结束，再复用日志文件。
                $ssh.WaitForExit()
                $ssh.Dispose()
                $ssh = $null
            }
            Write-Warning 'SSH已结束；保留电脑控制台，3秒后重连恢复预览，录像需手动重新开始。'
            Start-Sleep -Seconds 3
            if ($pc.HasExited) { break }
            try { $ssh = Start-OwnedSsh }
            catch { Write-Warning "SSH重启失败：$($_.Exception.Message)；将继续重试。" }
        }
        Start-Sleep -Milliseconds 500
    }
} finally {
    if ($pc -and -not $pc.HasExited -and $token) {
        try { Invoke-RestMethod "$base/api/shutdown" -Method Post -Headers @{'X-Record-Token'=$token} -TimeoutSec 3 | Out-Null } catch { Write-Warning $_ }
        # 不强杀编码器；慢磁盘需要等封装结束。
        $pc.WaitForExit()
    }
    if ($ssh -and -not $ssh.HasExited) { Stop-Process -InputObject $ssh }
}
