# 安全区实拍回放数据

每个片段一个目录：`data/<clip>/labels.yaml` + 逐帧图片。没有任何片段时 `test_field_replay` 返回 77（跳过）。
也可用环境变量 `FIELD_REPLAY_DIR` 指向别处的数据目录（大文件不要进仓库）。

## 抽帧

```bash
mkdir -p data/<clip> && ffmpeg -i in.mp4 -vf fps=10 -q:v 2 data/<clip>/%06d.jpg
```

## labels.yaml

格式见 `labels.template.yaml`（OpenCV FileStorage YAML，首行必须是 `%YAML:1.0`）。

| 字段 | 含义 |
| --- | --- |
| `reference_only` | 1 = 只报告不判（手机拍摄、屏幕录制等非车载相机原始帧） |
| `image_width` | 标注所用原图宽度（核对用）；标注坐标为原图像素，测试按实际图片尺寸换算到处理分辨率 |
| `frames[].file` | 图片文件名 |
| `frames[].present` | 画面里有完整场地（紫框 + 红色内部）为 1 |
| `frames[].judge` | 中线可判性：`judgeable` / `borderline` / `unjudgeable`；口径与合成矩阵相同（可见比例 ≥60% 且两端各 ≥50% 为可判） |
| `frames[].divider` | 可选，中线远端、近端中心 `[x_far, y_far, x_near, y_near]` |
| `frames[].corners` | 可选，内沿（地面高度内边线交点）TL,TR,BR,BL 共 8 个数；画外角点按延长线交点填 |
