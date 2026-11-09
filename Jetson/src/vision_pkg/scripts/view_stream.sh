#!/usr/bin/env bash
# Copyright 2026 jetson_yolo integration
# SPDX-License-Identifier: Apache-2.0
#
# 观看端辅助脚本，在笔记本上打开播放器或拉取快照与计数。
#
# 用法：
#   ./view_stream.sh <jetson-ip> [port] [mode]
#
#   模式：browser（默认）| ffplay | mpv | vlc | snapshot | counters | h264
#
# 示例：
#   ./view_stream.sh 192.168.1.42                # 打印选项并尝试打开浏览器
#   ./view_stream.sh 192.168.1.42 8080 ffplay
#   ./view_stream.sh 192.168.1.42 8080 snapshot

# 本脚本只运行观看端工具，不通过 SSH 启动检测节点，也不负责停止板端业务。
set -euo pipefail

HOST="${1:-}"
PORT="${2:-8080}"
MODE="${3:-browser}"

if [[ -z "${HOST}" ]]; then
  cat >&2 <<'EOF'
usage: view_stream.sh <jetson-ip> [port] [mode]

  mode: browser (default) | ffplay | mpv | vlc | snapshot | counters | h264
EOF
  exit 2
fi

BASE="http://${HOST}:${PORT}"

cat <<EOF
Viewer page:  ${BASE}/
Raw stream:   ${BASE}/stream.mjpg

  browser    open ${BASE}/            (观看页；类别与安全区标记位于视频画面)
  ffplay     ffplay -fflags nobuffer -flags low_delay -framedrop ${BASE}/stream.mjpg
  mpv        mpv --profile=low-latency --untimed ${BASE}/stream.mjpg
  vlc        vlc ${BASE}/stream.mjpg
  snapshot   curl -fsS ${BASE}/snapshot.jpg -o frame.jpg && xdg-open frame.jpg
  counters   curl -fsS ${BASE}/healthz
  h264       H.264 over UDP. Software-encoded: Orin Nano has no NVENC.
             Port 5000 is hardcoded on both ends, so the [port] argument
             above does not apply to this mode.
EOF

# 播放器选项减少本地缓冲，但实际延迟仍取决于网络与解码。
# snapshot 留存单帧；counters 查看 /healthz 的连接与丢弃计数。
case "${MODE}" in
  browser)
    command -v xdg-open >/dev/null 2>&1 && xdg-open "${BASE}/" || true
    ;;
  ffplay)
    # 禁用额外缓冲并允许丢帧，观看端优先追上最新画面。
    exec ffplay -fflags nobuffer -flags low_delay -framedrop -window_title "stream_node" "${BASE}/stream.mjpg"
    ;;
  mpv)
    exec mpv --profile=low-latency --untimed --title="stream_node" "${BASE}/stream.mjpg"
    ;;
  vlc)
    exec vlc --network-caching=150 "${BASE}/stream.mjpg"
    ;;
  counters)
    # 只查询服务器计数，不创建持续图像连接。
    exec curl -fsS "${BASE}/healthz"
    ;;
  snapshot)
    out="frame_$(date +%Y%m%d_%H%M%S).jpg"
    curl -fsS "${BASE}/snapshot.jpg" -o "${out}"
    echo "wrote ${out}"
    ;;
  h264)
    # 本分支只有接收端；端口 5000 两端写死，[port] 参数不生效。
    # 发送端须另行运行下面的 GStreamer 示例，并确认平台插件可用。
    # 示例绕过 ROS 节点，输出原始相机画面，不含检测叠图；编码使用 x264enc，占用 CPU。
    #   gst-launch-1.0 v4l2src device=/dev/video0 io-mode=2 ! \
    #     image/jpeg,width=1280,height=720,framerate=30/1 ! \
    #     nvv4l2decoder mjpeg=1 ! nvvidconv ! \
    #     'video/x-raw,format=I420' ! \
    #     x264enc speed-preset=ultrafast tune=zerolatency key-int-max=30 \
    #            ref=1 bframes=0 bitrate=4000 ! h264parse ! \
    #     rtph264pay config-interval=1 pt=96 ! \
    #     udpsink host=<this-laptop-ip> port=5000
    #
    # 本机接收并解码；此支路不使用上面的 HTTP 地址或端口。
    exec gst-launch-1.0 -v udpsrc port=5000 \
      caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! \
      rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
    ;;
  *)
    echo "unknown mode '${MODE}'" >&2
    exit 2
    ;;
esac
