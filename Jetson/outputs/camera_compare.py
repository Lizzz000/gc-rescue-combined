#!/usr/bin/env python3
"""Offline camera A/B summaries using the existing static field analyzer.

Example: python3 outputs/camera_compare.py --input outputs/field_trial_20261116/camera_ab_01 --output outputs/field_trial_20261116/camera_ab_01_comparison.json
Stdlib only. Reports measurements; does not score or select a winner.
"""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path


ANALYZER = Path(__file__).resolve().parent / 'field_trial_20261116' / 'analyze_static_field.py'
spec = importlib.util.spec_from_file_location('static_field_analyzer', ANALYZER)
analyzer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyzer)


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def brief_stats(value):
    return {key: value.get(key) for key in ('n', 'mean', 'p95', 'max')}


def point_drift(point):
    return {'median_xy_px': point.get('median_xy_px'),
            'distance_from_median_px': brief_stats(point.get('distance_from_median_px', {}))}


def corner_jumps(report):
    """Same dedup/order/adjacency rules as analyze(); no bridging invalid frames."""
    seen, rows, result = set(), [], {}
    for row in report.get('raw', {}).get('field', []):
        stamp = row.get('source_stamp_ns')
        key = (row.get('frame_id', ''), stamp)
        if type(stamp) is not int or stamp <= 0 or key in seen:
            continue
        seen.add(key)
        rows.append(row)
    for before, after in zip(rows, rows[1:]):
        frame = before.get('frame_id', '')
        if (frame != after.get('frame_id', '') or after['source_stamp_ns'] <= before['source_stamp_ns'] or
                any(row.get('valid') is not True or row.get('committed') is not True for row in (before, after))):
            continue
        first, second = analyzer.points(before, 'corners'), analyzer.points(after, 'corners')
        if first is None or second is None:
            continue
        values = [math.dist(a, b) for a, b in zip(first, second)]
        group = result.setdefault(frame, {'pairs': 0, 'per_corner_max_px': [0.0] * 4,
                                         'max_any_corner_px': 0.0, 'max_source_pair_ns': None})
        group['pairs'] += 1
        group['per_corner_max_px'] = [max(a, b) for a, b in zip(group['per_corner_max_px'], values)]
        if max(values) > group['max_any_corner_px'] or group['max_source_pair_ns'] is None:
            group['max_any_corner_px'] = max(values)
            group['max_source_pair_ns'] = [before['source_stamp_ns'], after['source_stamp_ns']]
    return result


def compact(report):
    analysis = analyzer.analyze(report)
    counts = analysis['counts']
    total = counts['unique_source_frames']
    suspect = counts['split_suspect']
    detect = analysis.get('source_detect') or {}
    result = {
        'probe_ok': analysis['source_probe_ok'], 'probe_errors': analysis['source_probe_errors'],
        'sample_duration_s': analysis['sample_duration_s'], 'counts': counts,
        'valid_fraction': counts['valid'] / total if total else None,
        'valid_and_committed_fraction': counts['valid_and_committed_fraction'],
        'unique_received_fps': counts['unique_received_fps'],
        'server_reported_fps': {name: detect.get(name) for name in ('capture_fps', 'field_fps')},
        'invalid_or_uncommitted_runs': analysis['invalid_or_uncommitted_runs'],
        'split_suspect': {'frames': suspect, 'fraction_of_unique_frames':
                          suspect / total if suspect is not None and total else None},
        'labels': analysis['labels'], 'geometry_by_frame_id': {},
    }
    jumps = corner_jumps(report)
    for frame_id, group in analysis['geometry_by_frame_id'].items():
        geometry = group['geometry']
        corners = geometry['corners']
        corner_rows = [point_drift(point) for point in corners['points']]
        item = {'accepted_frames': group['accepted_frames'], 'corner_order': ['TL', 'TR', 'BR', 'BL'],
                'corners': corner_rows, 'corner_adjacent_rms_px': brief_stats(corners['adjacent_frame_rms_px']),
                'corner_adjacent_max_jump': jumps.get(frame_id), 'region_centers': {},
                'geometry_issues': group['geometry_issues']}
        for name in ('left_region', 'right_region'):
            center = geometry[name].get('projective_center', {})
            points = center.get('points', [])
            item['region_centers'][name] = {
                'frames': center.get('frames', 0),
                'projective_center': point_drift(points[0]) if points else None,
                'adjacent_displacement_px': brief_stats(center.get('adjacent_frame_rms_px', {}))}
        result['geometry_by_frame_id'][frame_id] = item
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True, help='camera_trials.py output directory')
    parser.add_argument('--output', type=Path, required=True, help='new comparison JSON path')
    args = parser.parse_args()
    root, destination = args.input.resolve(), args.output.resolve()
    if not root.is_dir():
        parser.error('--input must be a camera A/B directory')
    if destination.exists():
        parser.error('--output already exists; choose a new path')
    summary_path = root / 'summary.json'
    trial_summary = read_json(summary_path) if summary_path.is_file() else {}
    names = trial_summary.get('selected') or sorted(path.parent.name for path in root.glob('*/geometry.json'))
    result = {'schema': 1, 'input': str(root), 'camera_trials_ok': trial_summary.get('ok'),
              'restore': trial_summary.get('restore'), 'trials': [], 'errors': [],
              'interpretation': [
                  'No ranking or automatic winner is assigned.',
                  'Geometry is original-image pixels, valid AND committed frames only, deduplicated by source stamp and frame_id.',
                  'Per-corner and center p95/max are Euclidean distances from each coordinate-wise median over that trial.',
                  'Adjacent max jump is the largest single corner displacement between consecutive observed usable frames; corner RMS is separate.',
                  'Invalid-run time is the observed first-to-last span, a lower bound; a singleton has zero span.',
                  'Stationary-scene stability and valid rate do not establish boundary accuracy without ground truth.',
                  'Trials run sequentially, so illumination changes and short sample duration can confound camera setting comparisons.'
              ]}
    for name in names:
        folder = root / name
        source = folder / 'geometry.json'
        item = {'trial': name, 'geometry_path': str(source)}
        try:
            payload = source.read_bytes()
            item['geometry_sha256'] = hashlib.sha256(payload).hexdigest()
            item.update(compact(json.loads(payload.decode('utf-8-sig'))))
            metadata = folder / 'trial.json'
            item['trial_metadata'] = read_json(metadata) if metadata.is_file() else None
            matched = folder / 'matched' / 'geometry.json'
            pair = read_json(matched) if matched.is_file() else {}
            item['matched_reference'] = {'png': str(folder / 'matched' / 'raw.png'),
                'png_exists': (folder / 'matched' / 'raw.png').is_file(),
                'source_stamp_ns': pair.get('source_stamp_ns'), 'same_source_stamp': pair.get('matched_source_stamp'),
                'valid': pair.get('field', {}).get('valid'), 'committed': pair.get('field', {}).get('committed')}
        except Exception as error:
            item['error'] = repr(error)
            result['errors'].append({'trial': name, 'error': repr(error)})
        result['trials'].append(item)
    if not names:
        result['errors'].append({'error': 'No trial geometry files found'})
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({'output': str(destination), 'trials': len(result['trials']), 'errors': result['errors']}, ensure_ascii=False))
    return int(bool(result['errors']))


if __name__ == '__main__':
    raise SystemExit(main())
