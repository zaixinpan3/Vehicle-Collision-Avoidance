#!/usr/bin/env python3
"""Build bounded admission experiments outside the production source tree.

These copies are profiling hypotheses, not controller replacements. The runner
checks complete decisions and the assembled safety problem on saved fixtures.
"""
import argparse
import hashlib
import json
from pathlib import Path


def replace_once(source, old, new):
    assert source.count(old) == 1, old
    return source.replace(old, new)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.directory.resolve()
    if output == root or root in output.parents:
        parser.error('Prototype copies must remain outside the repository.')
    geometry = (root / 'controller/avoidanceSafetyGeometry.m').read_text()
    solver = (root / 'controller/solveHardCbfClf.m').read_text()
    if 'includeCollisionRows' in geometry:
        parser.error('These shortcuts are already implemented. Use benchmarkAdmissionOptimization; '
                     'reproduce historical prototypes from the pre-adoption ff2a901 checkout.')
    reduced_geometry = replace_once(geometry,
        '                stateRadius = tube.radius;\n',
        '''                % Fixture experiment: omit rows removed by jointProgram.
                assert(tube.duration==0 && size(tube.offset,2)==1);
                geometric=allGeometricRows(cellIndex);
                keep=geometric.source>encounterCount;
                geometric.state=geometric.state(keep,:);
                geometric.bound=geometric.bound(keep,:);
                geometric.source=geometric.source(keep);
                allGeometricRows(cellIndex)=geometric;
                stateRadius = tube.radius;
''')
    physical = '''        physicalExcess=max(program.physicalMatrix*trial-program.physicalBound);
        physical=physicalExcess<=0;
        information.referencePhysicalExcess(candidate)=physicalExcess;
'''
    reduced_solver = replace_once(solver, physical, '')
    reduced_solver = replace_once(reduced_solver, '        directions=angles;\n',
        physical + '''        % The existing ranking cannot select this dominated fit.
        if bestPhysical && ~physical,continue;end
        directions=angles;
''')
    variants = {
        'base': {},
        'early-row-filter': {'avoidanceSafetyGeometry.m': reduced_geometry},
        'dominated-candidate': {'solveHardCbfClf.m': reduced_solver},
        'combined': {'avoidanceSafetyGeometry.m': reduced_geometry,
                     'solveHardCbfClf.m': reduced_solver},
    }
    manifest = {'scope': 'Saved fresh-admission fixtures only; not deployed',
                'sourceSha256': {}, 'variants': {}}
    for name in ('avoidanceSafetyGeometry.m', 'solveHardCbfClf.m'):
        manifest['sourceSha256'][name] = hashlib.sha256(
            (root / 'controller' / name).read_bytes()).hexdigest()
    for variant, files in variants.items():
        folder = output / variant
        folder.mkdir(parents=True, exist_ok=False)
        manifest['variants'][variant] = {}
        for name, source in files.items():
            (folder / name).write_text(source)
            manifest['variants'][variant][name] = hashlib.sha256(source.encode()).hexdigest()
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
