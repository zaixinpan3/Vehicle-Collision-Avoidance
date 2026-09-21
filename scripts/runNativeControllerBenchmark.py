#!/usr/bin/env python3
"""Recapture current scenarios, regenerate C, rebuild, verify and time the ELF.

Only prepared active-target numerical programs are currently native. Neither
capture/MATLAB timings nor fixture I/O are full-pipeline C performance results.
Every invocation uses a new external directory; there is no reuse-build option.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import statistics
import subprocess
import uuid


SCOPE = 'Standalone C prepared-program numerical kernel; NOT full controller pipeline'


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def fingerprints(paths, root):
    return {str(path.relative_to(root)): digest(path) for path in sorted(paths)}


def source_state(root):
    """Hash executable source and the native solver dependencies actually used."""
    paths = set()
    for folder in ('controller', 'config', 'scripts'):
        paths.update(path for path in (root / folder).glob('*')
                     if path.is_file() and path.suffix in ('.m', '.py', '.c', '.cpp', '.h'))
    solver = root / 'solver' / 'clarabel'
    paths.update((solver / 'include').rglob('*.h'))
    paths.update((solver / 'matlab').glob('*.m'))
    paths.update((solver / 'matlab').glob('*.mexa64'))
    library = solver / 'rust_wrapper/target/release/libclarabel_c.a'
    if library.exists():
        paths.add(library)
    return fingerprints(paths, root)


def verify_unchanged(expected, actual):
    if expected != actual:
        changed = sorted(key for key in set(expected) | set(actual)
                         if expected.get(key) != actual.get(key))
        raise RuntimeError('Benchmark provenance changed: ' + ', '.join(changed))


def verify_artifacts(expected, root):
    for name, value in expected.items():
        path = root / name
        if not path.is_file() or digest(path) != value:
            raise RuntimeError('Benchmark artifact changed: ' + name)


def matlab_literal(value):
    return "'" + str(value).replace("'", "''") + "'"


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def run_matlab(executable, root, code, log, environment):
    prefix = 'cd(' + matlab_literal(root) + ");addpath('scripts');"
    with log.open('w') as stream:
        subprocess.run([executable, '-singleCompThread', '-batch', prefix + code],
                       cwd=root, env=environment, stdout=stream, stderr=subprocess.STDOUT, check=True)


def checked_rows(baseline, rows, repetitions):
    """Reject missing, duplicate or malformed measurements before aggregation."""
    expected = {(frame['file'], i) for frame in baseline['frames'] for i in range(repetitions)}
    actual = [(row['file'], row['iteration']) for row in rows]
    if len(actual) != len(expected) or set(actual) != expected:
        raise RuntimeError('Native replay is missing or duplicates frame measurements.')
    for row in rows:
        if (not math.isfinite(row['seconds']) or row['seconds'] < 0
                or len(row['metrics']) != 4
                or any(not math.isfinite(x) or x < 0 for x in row['metrics'][:3])
                or row['status'] not in (0, 1, 2, 3, 4)):
            raise RuntimeError('Invalid native timing or acceptance status.')


def summarize(baseline, rows):
    frames = {frame['file']: frame for frame in baseline['frames']}
    groups = {'all': rows,
              'fresh': [row for row in rows if not frames[row['file']]['inherited']],
              'inherited': [row for row in rows if frames[row['file']]['inherited']]}
    for row in rows:
        frame = frames[row['file']]
        key = f"{frame['scenario']}-{frame['curvature']:g}"
        groups.setdefault(key, []).append(row)
    summary = {'scope': SCOPE, 'fullPipelineMeasured': False, 'frameCount': len(frames), 'groups': {}}
    for key, values in groups.items():
        if not values:
            continue
        times = [row['seconds'] * 1000 for row in values]
        summary['groups'][key] = {'calls': len(values), 'medianMs': statistics.median(times),
                                 'maximumMs': max(times), 'over50Ms': sum(x > 50 for x in times),
                                 'rejections': sum(row['status'] == 0 for row in values)}
    worst = max(rows, key=lambda row: row['seconds'])
    frame = frames[worst['file']]
    summary['slowestNativeCall'] = {
        **{key: frame[key] for key in ('scenario', 'curvature', 'frame', 'horizonSteps', 'inherited')},
        'iteration': worst['iteration'], 'status': worst['status'],
        'totalMs': worst['seconds'] * 1000,
        'assemblyOrAdmissionMs': worst['metrics'][0] * 1000,
        'nativeSolverMs': worst['metrics'][1] * 1000,
        'verificationMs': worst['metrics'][2] * 1000,
        'unattributedMs': (worst['seconds'] - sum(worst['metrics'][:3])) * 1000}
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='New directory outside the repository (must not exist).')
    parser.add_argument('--matlab', default=shutil.which('matlab'))
    parser.add_argument('--sample-count', type=int, default=600)
    parser.add_argument('--curvatures', nargs='+', type=float, default=[0, .01])
    parser.add_argument('--scenarios', nargs='+', choices=['stationary', 'oncoming', 'crossing'],
                        default=['stationary', 'oncoming', 'crossing'])
    parser.add_argument('--repetitions', type=int, default=5)
    parser.add_argument('--warmups', type=int, default=2)
    args = parser.parse_args()
    if (not args.matlab or args.sample_count < 1 or not 1 <= args.repetitions <= 10000
            or not 0 <= args.warmups <= 10000 or not all(math.isfinite(x) for x in args.curvatures)):
        parser.error('MATLAB, positive sample count and finite bounded replay settings are required.')
    root = Path(__file__).resolve().parents[1]
    directory = (args.output or Path.home() / '.cache/collisionAvoidance' /
                 ('native-' + datetime.now().strftime('%Y%m%d-%H%M%S-') + uuid.uuid4().hex[:8])).resolve()
    if directory == root or root in directory.parents:
        parser.error('Generated artifacts must be outside the repository.')
    directory.mkdir(parents=True, exist_ok=False)
    environment = dict(os.environ)
    for name in ('LD_LIBRARY_PATH', 'LD_PRELOAD'):
        environment.pop(name, None)
    for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
        environment[name] = '1'
    before = source_state(root)
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
    manifest = {'scope': SCOPE, 'fullPipelineMeasured': False, 'status': 'building',
                'startedAt': datetime.now(timezone.utc).isoformat(), 'sourceCommit': commit,
                'sourceFilesSha256': before, 'root': str(root), 'output': str(directory),
                'host': platform.platform(), 'settings': {**vars(args), 'output': str(directory)},
                'timingExclusions': ['fixture I/O', 'input parsing', 'prediction and upstream formulation',
                                     'reference/terminal synthesis', 'carried-certificate transfer'],
                'commands': [], 'artifactsSha256': {}}
    manifest_path = directory / 'manifest.json'
    write_json(manifest_path, manifest)
    print('Fresh native benchmark: ' + str(directory), flush=True)
    capture, build = directory / 'capture', directory / 'native'
    curvatures = '[' + ','.join(format(x, '.17g') for x in args.curvatures) + ']'
    scenarios = '[' + ','.join('"' + x + '"' for x in args.scenarios) + ']'
    code = (
        f'captureStandaloneControllerFrames({matlab_literal(capture)},SampleCount={args.sample_count},'
        f'Curvatures={curvatures},Scenarios={scenarios});'
        f'index=jsondecode(fileread(fullfile({matlab_literal(capture)},\'capture.json\')));'
        'files={};for t=1:numel(index.trials),for f=1:numel(index.trials(t).frames),'
        'files{end+1}=index.trials(t).frames(f).file;end;end;assert(~isempty(files));'
        'data=load(files{1},\'p\',\'c\');'
        f'standaloneControllerBenchmark.build(data.p,data.c,{matlab_literal(build)});'
        f'prepareStandaloneControllerReplay({matlab_literal(capture)},{matlab_literal(build)},'
        'Repetitions=1,Warmups=0);'
        f'fid=fopen({matlab_literal(directory / "toolchain.txt")},\'w\');'
        'fprintf(fid,\'%s\\n%s\\n\',version,matlabroot);fclose(fid);')
    try:
        print('Capturing current scenarios, generating C and linking (build.log).', flush=True)
        manifest['commands'].append([args.matlab, '-singleCompThread', '-batch', code])
        run_matlab(args.matlab, root, code, directory / 'build.log', environment)
        verify_unchanged(before, source_state(root))
        baseline = json.loads((capture / 'matlab-replay.json').read_text())
        paths = {path for path in build.rglob('*') if path.is_file() and
                 path.suffix in ('.c', '.h', '.a', '.mat', '.mk', '.m')}
        paths.add(build / 'controller-replay')
        paths.add(capture / 'capture.json')
        paths.add(capture / 'matlab-replay.json')
        paths.update(Path(frame[key]) for frame in baseline['frames'] for key in ('file', 'matFile'))
        manifest['artifactsSha256'] = fingerprints(paths, directory)
        manifest['toolchain'] = (directory / 'toolchain.txt').read_text().splitlines()
        manifest['gcc'] = subprocess.check_output(['gcc', '--version'], env=environment, text=True).splitlines()[0]
        manifest['status'] = 'built'
        write_json(manifest_path, manifest)
        verify_artifacts(manifest['artifactsSha256'], directory)
        command = [str(build / 'controller-replay'), str(args.repetitions), str(args.warmups)]
        command.extend(frame['file'] for frame in baseline['frames'])
        manifest['commands'].append(command)
        print('Timing the standalone executable (native-replay.jsonl).', flush=True)
        trace = directory / 'native-replay.jsonl'
        with trace.open('w') as stream:
            subprocess.run(command, env=environment, stdout=stream, check=True)
        rows = [json.loads(line) for line in trace.read_text().splitlines()]
        checked_rows(baseline, rows, args.repetitions)
        verify_artifacts(manifest['artifactsSha256'], directory)
        print('Independently checking native decisions in MATLAB (validation.log).', flush=True)
        validation_code = f'validateStandaloneControllerReplay({matlab_literal(capture)},{matlab_literal(trace)});'
        manifest['commands'].append([args.matlab, '-singleCompThread', '-batch', validation_code])
        run_matlab(args.matlab, root, validation_code, directory / 'validation.log', environment)
        verify_unchanged(before, source_state(root))
        verify_artifacts(manifest['artifactsSha256'], directory)
        report = summarize(baseline, rows)
        validation = json.loads((capture / 'native-validation.json').read_text())
        report['validation'] = {key: value for key, value in validation.items() if key != 'frames'}
        report['captureOutcomes'] = [{key: value for key, value in trial.items() if key != 'frames'}
                                    for trial in json.loads((capture / 'capture.json').read_text())['trials']]
        report['manifest'] = str(manifest_path)
        write_json(directory / 'summary.json', report)
        for path in (trace, capture / 'native-validation.json', directory / 'summary.json'):
            manifest['artifactsSha256'][str(path.relative_to(directory))] = digest(path)
        manifest['status'] = 'validated'
        manifest['completedAt'] = datetime.now(timezone.utc).isoformat()
        write_json(manifest_path, manifest)
        print(json.dumps(report, indent=2), flush=True)
    except Exception as error:
        manifest['status'] = 'failed'
        manifest['error'] = str(error)
        write_json(manifest_path, manifest)
        raise


if __name__ == '__main__':
    main()
