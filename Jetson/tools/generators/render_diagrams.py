#!/usr/bin/env python3
# Copyright 2026 li
# SPDX-License-Identifier: Apache-2.0
"""由同一份流程定义生成 Mermaid、SVG 和离线 HTML，需要 Graphviz 的 dot。"""
import html
import json
import re
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
DIRECTORY = ROOT / 'docs' / 'reference' / 'diagrams'


def main() -> int:
    dot = shutil.which('dot')
    if dot is None:
        print('缺少 Graphviz dot；已提供的 SVG 和 HTML 仍可直接阅读。', file=sys.stderr)
        return 1
    graphs = json.loads((DIRECTORY / 'graphs.json').read_text(encoding='utf-8'))
    cards, sections = [], []
    colors = {'node': '#ffffff', 'external': '#eef2f5',
              'control': '#e3f2ed', 'buffer': '#e9effb', 'decision': '#fff3d6'}
    for graph in graphs:
        name, title, direction = graph['id'], graph['title'], graph['direction']
        q = lambda value: json.dumps(value, ensure_ascii=False)
        lines = ['digraph {', f'rankdir={direction};',
                 'graph [bgcolor="transparent", pad="0.3", nodesep="0.35", ranksep="0.5"];',
                 'node [fontname="Noto Sans CJK SC", fontsize=12, shape=box, style="rounded,filled", color="#bbc9d2", margin="0.16,0.12"];',
                 'edge [fontname="Noto Sans CJK SC", fontsize=10, color="#6b7a84", arrowsize=0.65];']
        mermaid = [f'flowchart {direction}']
        for ident, label, role in graph['nodes']:
            shape = 'diamond' if role == 'decision' else 'box'
            lines.append(f'{ident} [label={q(label)}, shape={shape}, fillcolor="{colors[role]}"];')
            escaped = html.escape(label).replace('\n','<br/>')
            brackets = ('{', '}') if role == 'decision' else ('[', ']')
            mermaid.append(f'  {ident}{brackets[0]}"{escaped}"{brackets[1]}')
        for edge in graph['edges']:
            source, target, label, *kind = edge
            is_control = kind == ['control']
            style = ', style=dashed, constraint=false' if is_control else ''
            lines.append(f'{source} -> {target} [label={q(label)}{style}];')
            arrow = '-.->' if is_control else '-->'
            annotation = '|"'+html.escape(label)+'"|' if label else ''
            mermaid.append(f'  {source} {arrow}{annotation} {target}')
        lines.append('}')
        dot_path, svg_path = DIRECTORY/f'{name}.dot', DIRECTORY/f'{name}.svg'
        dot_path.write_text('\n'.join(lines)+'\n', encoding='utf-8')
        subprocess.run([dot, '-Tsvg', str(dot_path), '-o', str(svg_path)], check=True)
        source = '\n'.join(mermaid)+'\n'
        (DIRECTORY/f'{name}.mmd').write_text(source, encoding='utf-8')
        sections.append(f'## {title}\n\n{graph["subtitle"]}\n\n```mermaid\n{source}```\n\n[SVG](diagrams/{name}.svg) · [Mermaid 源码](diagrams/{name}.mmd)\n')
        svg = svg_path.read_text(encoding='utf-8')
        svg = svg[svg.index('<svg '):]
        svg = re.sub(r'id="([^"]+)"', lambda match: f'id="{name}-{match[1]}"', svg)
        svg = svg.replace('<svg ', f'<svg role="img" aria-label="{html.escape(title)}" ', 1)
        cards.append(f'<section id="{name}"><header><h2>{html.escape(title)}</h2><p>{html.escape(graph["subtitle"])}</p></header>'
                     f'<div class="diagram">{svg}</div>'
                     f'<footer><a href="diagrams/{name}.svg">单独打开 SVG</a><a href="diagrams/{name}.mmd">Mermaid 源码</a></footer></section>')
    intro = '# 流程图\n\n[离线浏览版](flowcharts.html) 不依赖网络。以下图与 SVG 由同一份 [graphs.json](diagrams/graphs.json) 生成。修改流程后执行 `python3 tools/generators/render_diagrams.py`，需要系统已安装 Graphviz。\n\n'
    (ROOT/'docs/reference/flowcharts.md').write_text(intro+'\n'.join(sections)+'\n接口字段见 [ROS 接口](ros-interfaces.md) 和 [USB 协议](usb-protocol.md)。\n', encoding='utf-8')
    nav = ''.join(f'<a href="#{g["id"]}">{html.escape(g["title"])}</a>' for g in graphs)
    page = '''<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>GC Rescue | 流程图</title>
<style>
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;background:#f4f6f8;color:#21303b;font:16px/1.65 system-ui,"Noto Sans CJK SC",sans-serif}main{max-width:1200px;margin:auto;padding:40px 24px 70px}h1{font-size:32px;margin:4px 0 8px}h2{font-size:21px;margin:0 0 8px}p{margin:0;color:#526573}.eyebrow{font-size:13px;font-weight:650;letter-spacing:.14em;color:#526573}nav{display:flex;flex-wrap:wrap;gap:10px;margin:24px 0}a{color:#235871;text-decoration:none}a:hover{text-decoration:underline}nav a{background:white;border:1px solid #d7e0e6;border-radius:8px;padding:8px 14px}section{background:white;border:1px solid #dbe3e8;border-radius:12px;margin:24px 0;overflow:hidden;scroll-margin-top:20px}section header{padding:22px 26px 16px;border-bottom:1px solid #edf0f2}.diagram{padding:24px 20px;overflow:auto}.diagram svg{display:block;width:auto;max-width:100%;height:auto;margin:auto;min-width:650px}footer{display:flex;gap:24px;border-top:1px solid #edf0f2;padding:13px 26px;font-size:14px}.note{border-left:3px solid #86aab8;padding:10px 16px;background:#edf2f5;margin-top:20px}.links{margin-top:20px;font-size:14px}@media(max-width:700px){main{padding:24px 12px}.diagram{padding:12px}section header{padding:18px}h1{font-size:27px}}@media print{body{background:white}nav{display:none}main{max-width:none;padding:0}section{break-inside:avoid;page-break-after:always;border:0}.diagram svg{min-width:0;max-height:800px}}
</style>
<main><div class="eyebrow">GC RESCUE / SOURCE GUIDE</div><h1>从数据到任务</h1><p>8 个节点、2 条识别通路，以及分开的实际识别阶段与 STM32 运输握手。</p>
<nav>''' + nav + '''</nav><div class="note">图说明软件流程，不代表本次已完成板端或整车验收。下位机固件和路径规划不在此源码包内。</div>''' + ''.join(cards) + '''<p class="links"><a href="../../README.md">README</a> · <a href="../README.md">文档索引</a> · <a href="usb-protocol.md">USB 协议</a> · <a href="ros-interfaces.md">ROS 接口</a> · <a href="../validation/unified-validation.md">验收规范</a></p></main></html>'''
    (ROOT/'docs/reference/flowcharts.html').write_text(page+'\n', encoding='utf-8')
    print(f'{len(graphs)} diagrams written to {DIRECTORY}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
