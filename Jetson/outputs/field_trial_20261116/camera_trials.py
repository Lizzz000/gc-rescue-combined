#!/usr/bin/env python3
"""Serial USB camera control trials; run only on Jetson, with the vehicle stationary.

No resolution/FPS/JPEG, vision thresholds or motion controls are changed. Each
trial starts from BASELINE, changes one conceptual setting, then runs the existing
field probe (12 s + its 3 s warmup) and source-matched PNG capture. On every exit,
restore the originally read camera modes and values. Automatic controls may then
resume adjusting their inactive manual values. No trial is run on import.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import time


BASELINE = {
    'brightness': 150, 'contrast': 65, 'saturation': 76, 'hue': 200,
    'white_balance_automatic': 1, 'gamma': 128, 'gain': 48,
    'power_line_frequency': 1, 'white_balance_temperature': 4650,
    'sharpness': 2000, 'backlight': 0, 'auto_exposure': 1,
    'exposure_time_absolute': 150, 'dynamic_framerate': 0,
    'focus_auto': 1, 'focus_absolute': 403, 'zoom': 100,
}
ALIASES = {
    'white_balance_automatic': ('white_balance_automatic', 'white_balance_temperature_auto'),
    'backlight': ('backlight', 'backlight_compensation'),
    'auto_exposure': ('auto_exposure', 'exposure_auto'),
    'exposure_time_absolute': ('exposure_time_absolute', 'exposure_absolute'),
    'dynamic_framerate': ('dynamic_framerate', 'exposure_dynamic_framerate', 'exposure_auto_priority'),
    'focus_auto': ('focus_auto', 'focus_automatic_continuous'),
    'zoom': ('zoom', 'zoom_absolute'),
}
TRIALS = {
    'baseline': {},
    'focus_lock403': {'focus_auto': 0, 'focus_absolute': 403},
    'exposure100': {'exposure_time_absolute': 100},
    'exposure80': {'exposure_time_absolute': 80},
    'gain36': {'gain': 36},
    'brightness135': {'brightness': 135},
    'saturation90': {'saturation': 90},
    'whitebal_lock4650': {'white_balance_automatic': 0, 'white_balance_temperature': 4650},
}
MANUAL_MODES = {'white_balance_automatic': 0, 'focus_auto': 0, 'auto_exposure': 1}
DEPENDENT = {'white_balance_temperature': ('white_balance_automatic', 0),
             'focus_absolute': ('focus_auto', 0),
             'exposure_time_absolute': ('auto_exposure', 1)}


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def parse_controls(text):
    result = {}
    for line in text.splitlines():
        match = re.match(r'^\s*(\w+)\s+0x[0-9a-fA-F]+\s+\([^)]*\)\s*:\s*(.*)$', line)
        if not match:
            continue
        value = re.search(r'\bvalue=(-?\d+)', match[2])
        if value:
            result[match[1]] = {'value': int(value[1]), 'inactive': 'inactive' in match[2],
                                'read_only': 'read-only' in match[2], 'description': match[2]}
    return result


class Camera:
    def __init__(self, device, output, workspace):
        self.device, self.output, self.workspace = device, output, workspace
        self.names = {}
        self.commands = []

    def command(self, label, command, timeout=10.0):
        started = time.monotonic()
        record = {'label': label, 'command': command, 'started_utc': datetime.now(timezone.utc).isoformat()}
        try:
            result = subprocess.run(command, cwd=self.workspace, capture_output=True, text=True,
                                    errors='replace', timeout=timeout, check=False)
            record.update(returncode=result.returncode, stdout=result.stdout, stderr=result.stderr)
        except subprocess.TimeoutExpired as error:
            def text(value):
                return value.decode('utf-8', errors='replace') if isinstance(value, bytes) else value or ''
            record.update(returncode=None, timeout=True, stdout=text(error.stdout), stderr=text(error.stderr))
        except OSError as error:
            record.update(returncode=None, stdout='', stderr=repr(error))
        finally:
            record['elapsed_s'] = time.monotonic() - started
            self.commands.append(record)
            write_json(self.output / 'commands.json', self.commands)
        return record

    def snapshot(self, label, path):
        result = self.command(label, ['v4l2-ctl', '-d', self.device, '--list-ctrls'])
        controls = parse_controls(result['stdout'])
        record = {'command': result, 'controls': controls}
        required = [next((alias for alias in ALIASES.get(name, (name,)) if alias in controls), None)
                    for name in BASELINE]
        if result['returncode'] != 0 and all(required):
            # Some UVC drivers list all usable controls, then fail on unrelated
            # Privacy ext_ctrl. Confirm our complete control set independently.
            readback = self.command(label + ':targeted_readback',
                                   ['v4l2-ctl', '-d', self.device, '--get-ctrl=' + ','.join(required)])
            values = {m[1]: int(m[2]) for line in readback['stdout'].splitlines()
                      if (m := re.match(r'^\s*(\w+)\s*:\s*(-?\d+)\s*$', line))}
            record['targeted_readback_command'] = readback
            if readback['returncode'] == 0 and all(name in values for name in required):
                for name in required:
                    controls[name]['value'] = values[name]
                record['listing_error_bypassed_by_complete_targeted_readback'] = True
        write_json(path, record)
        if not all(required) or (result['returncode'] != 0 and
                                not record.get('listing_error_bypassed_by_complete_targeted_readback')):
            raise RuntimeError(label + ': failed to read all 17 required camera controls')
        return controls

    def resolve(self, controls):
        for name in BASELINE:
            actual = next((alias for alias in ALIASES.get(name, (name,)) if alias in controls), None)
            if actual is None or controls[actual]['read_only']:
                raise RuntimeError('Required writable control unavailable: ' + name)
            self.names[name] = actual

    def set_one(self, label, name, value):
        result = self.command(label + ':' + name,
                              ['v4l2-ctl', '-d', self.device, '--set-ctrl=' + self.names[name] + '=' + str(value)])
        if result['returncode'] != 0:
            raise RuntimeError(label + ': cannot set ' + name + '=' + str(value))

    def profile(self, label, values, best_effort=False):
        errors = []
        # Enable manual setters before touching values marked inactive in auto mode.
        phases = (MANUAL_MODES, {k: v for k, v in values.items() if k not in MANUAL_MODES},
                  {k: values[k] for k in MANUAL_MODES})
        manual_ready = set()
        for index, phase in enumerate(phases):
            for name, value in phase.items():
                if index == 1 and name in DEPENDENT and DEPENDENT[name][0] not in manual_ready:
                    errors.append('Skipped inactive manual control: ' + name)
                    continue
                try:
                    self.set_one(label, name, value)
                    if index == 0:
                        manual_ready.add(name)
                except Exception as error:
                    errors.append(str(error))
                    if not best_effort:
                        raise
        return errors

    def verify(self, target, controls):
        mismatches = {}
        for name, expected in target.items():
            actual = controls.get(self.names[name])
            # A resumed automatic controller is allowed to update its manual value.
            if name in DEPENDENT:
                mode, manual = DEPENDENT[name]
                if target[mode] != manual:
                    continue
            if actual is None or actual['value'] != expected:
                mismatches[name] = {'expected': expected, 'readback': actual}
        return mismatches


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--duration', type=float, default=12.0, help='probe sample seconds, plus its fixed 3 s warmup')
    parser.add_argument('--only', nargs='+', help='trial names, space/comma separated: ' + ', '.join(TRIALS))
    parser.add_argument('--output', type=Path, required=True, help='new, nonexistent output directory')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 600:
        parser.error('--duration must be in (0, 600] seconds')
    names = [name for item in args.only for name in item.split(',')] if args.only else list(TRIALS)
    if not names or len(names) != len(set(names)) or any(name not in TRIALS for name in names):
        parser.error('--only must contain unique known trial names: ' + ', '.join(TRIALS))
    folder = Path(__file__).resolve().parent
    workspace = folder.parents[1]
    probe, matched = folder / 'vision_geometry_probe.py', folder / 'capture_matched.py'
    if not probe.is_file() or not matched.is_file():
        parser.error('Existing vision_geometry_probe.py/capture_matched.py must be beside this script')
    output = args.output.resolve()
    if output.exists():
        parser.error('--output already exists; use a new directory to avoid overwriting evidence')
    output.mkdir(parents=True)
    camera = Camera(args.device, output, workspace)
    report = {'schema': 1, 'baseline': BASELINE, 'selected': names, 'duration_s': args.duration,
              'device': args.device, 'trials': [], 'errors': [], 'restore': None}
    original, changed = None, False
    try:
        original_controls = camera.snapshot('original', output / 'controls_original.json')
        camera.resolve(original_controls)
        original = {name: original_controls[actual]['value'] for name, actual in camera.names.items()}
        report.update(original=original, control_names=camera.names)
        for name in names:
            trial_dir = output / name
            trial_dir.mkdir()
            start_index = len(camera.commands)
            trial = {'name': name, 'changes_from_baseline': TRIALS[name], 'ok': False}
            report['trials'].append(trial)
            changed = True  # Set before the first write, so a partial failure still restores.
            camera.profile(name + ':baseline', BASELINE)
            before = camera.snapshot(name + ':before', trial_dir / 'controls_before.json')
            mismatch = camera.verify(BASELINE, before)
            if mismatch:
                raise RuntimeError('Baseline readback mismatch: ' + json.dumps(mismatch))
            target = {**BASELINE, **TRIALS[name]}
            for control, value in TRIALS[name].items():
                camera.set_one(name + ':trial', control, value)
            after = camera.snapshot(name + ':after', trial_dir / 'controls_after.json')
            mismatch = camera.verify(target, after)
            trial['readback_mismatches'] = mismatch
            if mismatch:
                raise RuntimeError('Trial readback mismatch: ' + json.dumps(mismatch))
            result = camera.command(name + ':probe', [sys.executable, str(probe), '--mode', 'field',
                                    '--duration', str(args.duration), '--output', str(trial_dir / 'geometry.json')],
                                    timeout=args.duration + 30.0)
            capture = camera.command(name + ':matched', [sys.executable, str(matched), '--name',
                                      str(trial_dir / 'matched')], timeout=20.0)
            trial.update(probe_returncode=result['returncode'], capture_returncode=capture['returncode'],
                         ok=result['returncode'] == 0 and capture['returncode'] == 0)
            camera.snapshot(name + ':end', trial_dir / 'controls_end.json')
            write_json(trial_dir / 'commands.json', camera.commands[start_index:])
            write_json(trial_dir / 'trial.json', trial)
            print(json.dumps({'trial': name, 'ok': trial['ok'], 'changes': TRIALS[name],
                              'probe_rc': result['returncode'], 'capture_rc': capture['returncode'],
                              'path': str(trial_dir)}, ensure_ascii=False), flush=True)
            if not trial['ok']:
                raise RuntimeError('Trial probe/capture failed: ' + name)
    except KeyboardInterrupt:
        report['errors'].append('interrupted')
    except Exception as error:
        report['errors'].append(repr(error))
    finally:
        if changed and original is not None:
            restore = {'errors': camera.profile('restore_original', original, best_effort=True)}
            try:
                controls = camera.snapshot('restore_readback', output / 'controls_restored.json')
                restore['mismatches'] = camera.verify(original, controls)
            except Exception as error:
                restore['errors'].append(repr(error))
            restore['ok'] = not restore['errors'] and not restore.get('mismatches')
            report['restore'] = restore
        else:
            report['restore'] = {'ok': True, 'note': 'No control write was attempted'}
        report['ok'] = not report['errors'] and report['restore']['ok']
        write_json(output / 'summary.json', report)
        print(json.dumps({'finished': True, 'ok': report['ok'], 'restore': report['restore'],
                          'errors': report['errors'], 'output': str(output)}, ensure_ascii=False), flush=True)
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
