# 安全区实拍回放数据

每个片段一个目录：`data/<clip>/labels.yaml` + 逐帧图片。没有任何片段时 `test_field_replay` 返回 77（跳过）。
也可用环境变量 `FIELD_REPLAY_DIR` 指向别处的数据目录（大文件不要进仓库）。

`site-20261123-divider.png` 是 2026 年 11 月 23 日停车后，生产 NVMM 相机链路的 1920 × 1080 无损帧经正式内框单应变换得到的 252 × 132 鸟瞰图。现场内框宽 630 mm。`test_field_split` 用它检查中央黑带，并在同一底图上构造去带、宽实心暗条和局部可见的反例。

`site-20261123-wall-divider.png` 取自当天第六次搬运初始对准的第五张拒识原图。用同帧地面单应矫正并向远墙上方延伸 90 像素，保留了“紫墙—黑带”和“地面—中隔阴影”两种不同材料的边界。测试分别去掉墙标、地面黑带，以及让 14/22 像素宽暗条与真实墙标对齐。

`site-20261123-oblique-divider.png` 取自同日双趟试跑的第五张初始对准拒识图，保留另一侧视角下的红地面阴影。两种姿态各自构造对准墙标的宽暗条，验证清晰和三档高斯失焦，以及只将原红地面变暗的情况。

`site-20261123-occluded-left.jpg` 取自同日放置绿色物块后复定位的原图 013，按处理分辨率缩小至 1280 × 720。绿色物块的右沿比两段可见左墙更长。`test_field_geometry` 检查左墙拟合得到的远角和近角，独立参考来自物块遮挡前后的两段可见紫红交界。

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
