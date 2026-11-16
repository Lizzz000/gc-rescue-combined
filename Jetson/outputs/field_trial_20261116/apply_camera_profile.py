#!/usr/bin/env python3
"""Apply an explicit camera profile, verify readback, retain it only on success.

Uses the existing Camera/BASELINE control whitelist. No probe, resolution/FPS,
JPEG, vision threshold or motion changes. The supplied current-field profile
still needs green segmentation validation; its static-scene stability evidence
is not ground-truth boundary or world-position accuracy.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from camera_trials import BASELINE, Camera, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='new log directory')
    parser.add_argument('--device', default='/dev/video0')
    args = parser.parse_args()
    try:
        profile = json.loads(args.profile.read_text(encoding='utf-8-sig'))
        values = profile['values']
        if not isinstance(values, dict) or set(values) != set(BASELINE):
            raise ValueError('values must contain exactly the 17 BASELINE control names')
        if any(type(value) is not int for value in values.values()):
            raise ValueError('all camera control values must be integers')
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error('Invalid profile: ' + str(error))
    output = args.output.resolve()
    if output.exists():
        parser.error('--output already exists; choose a new directory')
    output.mkdir(parents=True)
    workspace = Path(__file__).resolve().parent.parents[1]
    camera = Camera(args.device, output, workspace)
    report = {'schema': 1, 'started_utc': datetime.now(timezone.utc).isoformat(),
              'profile_path': str(args.profile.resolve()), 'profile': profile,
              'device': args.device, 'errors': [], 'target_verified': False,
              'restore': None, 'ok': False}
    original, changed = None, False
    try:
        controls = camera.snapshot('original', output / 'controls_original.json')
        camera.resolve(controls)
        original = {name: controls[actual]['value'] for name, actual in camera.names.items()}
        report.update(original=original, control_names=camera.names)
        write_json(output / 'profile_requested.json', profile)
        changed = True  # A partial first write must also trigger restoration.
        camera.profile('apply_target', values)
        readback = camera.snapshot('target_readback', output / 'controls_target.json')
        report['readback_mismatches'] = camera.verify(values, readback)
        if report['readback_mismatches']:
            raise RuntimeError('Target camera control readback did not match the requested profile')
        report['target_verified'] = True
    except KeyboardInterrupt:
        report['errors'].append('interrupted')
    except Exception as error:
        report['errors'].append(repr(error))
    finally:
        if changed and original is not None and not report['target_verified']:
            restore = {'errors': camera.profile('restore_original', original, best_effort=True)}
            try:
                restored = camera.snapshot('restore_readback', output / 'controls_restored.json')
                restore['mismatches'] = camera.verify(original, restored)
            except Exception as error:
                restore['errors'].append(repr(error))
            restore['ok'] = not restore['errors'] and not restore.get('mismatches')
            report['restore'] = restore
        elif report['target_verified']:
            report['restore'] = {'attempted': False, 'reason': 'Verified target retained as requested'}
        else:
            report['restore'] = {'attempted': False, 'reason': 'No camera control write attempted'}
        report['ok'] = report['target_verified'] and not report['errors']
        write_json(output / 'summary.json', report)
    print(json.dumps({'ok': report['ok'], 'target_retained': report['target_verified'],
                      'profile': str(args.profile.resolve()), 'output': str(output),
                      'restore': report['restore'], 'errors': report['errors']}, ensure_ascii=False), flush=True)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
