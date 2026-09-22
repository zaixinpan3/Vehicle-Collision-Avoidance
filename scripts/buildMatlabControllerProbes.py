#!/usr/bin/env python3
"""Create external diagnostic source copies; never modify production MATLAB.

Inclusive nested timers support attribution, not deadline qualification. The
paired replay checks complete decision equality against uninstrumented results.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re


TARGETS = {
    'collisionAvoidanceController.m': ['collisionAvoidanceController', 'localControllerConfiguration',
                                     'localFiniteModel', 'localCommand'],
    'readPlanningInputs.m': ['readPlanningInputs'],
    'formulateAvoidanceProblem.m': ['formulateAvoidanceProblem', 'localShift'],
    'hardEncounterBarrier.m': ['prepare', 'predict', 'completionRows', 'carriedData', 'localTerminalSet'],
    'ltvBicycleModel.m': ['sampledCruise', 'finitePredict'],
    'laneGeometry.m': ['sweptCellFrames', 'normalRoadChart'],
    'targetPrediction.m': ['nominalFlow'],
    'avoidanceSafetyGeometry.m': ['build', 'supportNormals', 'jointProgram', 'certifyJoint', 'jointResidual'],
    'avoidanceStageQp.m': ['fixedDirections', 'build', 'localDomainCertificate'],
    'solveHardCbfClf.m': ['certify', 'inspect', 'prepareFluidReference', 'localPrepareFluidReference',
                        'vffmReference', 'localTimeDependentReference', 'fluidInitialize', 'localFluidInitialize', 'localFixedDirectionSearch',
                        'localFitFluidReference', 'localDefaultSolve', 'localReducedProgram'],
}


def wrap_block(text, begin, end, tag):
    """Time a straight-line block; unique source anchors prevent silent drift."""
    assert text.count(begin) == 1 and text.count(end) == 1, tag
    token = 'blockProfileToken' + re.sub(r'\W', '', tag)
    text = text.replace(begin, f"    {token}=matlabControllerProbe('start','{tag}');\n" + begin)
    return text.replace(end, f"    matlabControllerProbe('stop',{token});\n" + end)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.directory.resolve() / 'instrumented'
    if output == root or root in output.parents:
        parser.error('Instrumented copies must remain outside the repository.')
    output.mkdir(parents=True, exist_ok=False)
    manifest = []
    for file, names in TARGETS.items():
        source = (root / 'controller' / file).read_text()
        found = []
        lines = source.splitlines(keepends=True)
        result = []
        declaration = ''
        for line in lines:
            result.append(line)
            if line.lstrip().startswith('function ') or declaration:
                declaration += line.strip().replace('...', ' ')
                if line.rstrip().endswith('...'):
                    continue
                match = re.search(r'(\w+)\s*\(', declaration)
                assert match, declaration
                name = match.group(1)
                if name in names:
                    found.append(name)
                    tag = Path(file).stem + '.' + name
                    result.append(f"    profileToken=matlabControllerProbe('start','{tag}');\n")
                    result.append("    profileCleanup=onCleanup(@()matlabControllerProbe('stop',profileToken));\n")
                declaration = ''
        assert sorted(found) == sorted(names), (file, found, names)
        text = ''.join(result)
        if file == 'avoidanceSafetyGeometry.m':
            text = wrap_block(text, '            cfg = model.cfg;\n            groups = cell(numel(prediction.cells), 1);',
                              '            data = vertcat(cellData{:});', 'avoidanceSafetyGeometry.cellDataAssembly')
            text = wrap_block(text, '            data = vertcat(cellData{:});',
                              '            projectionData = cell(numel(groups),1);', 'avoidanceSafetyGeometry.cellRowsKernel')
            text = wrap_block(text, '            projectionData = cell(numel(groups),1);',
                              '            data = vertcat(projectionData{:});', 'avoidanceSafetyGeometry.projectionDataAssembly')
            text = wrap_block(text, '            data = vertcat(projectionData{:});',
                              '            for cellIndex = 1:numel(groups)\n                names = sourceLabels{cellIndex};',
                              'avoidanceSafetyGeometry.projectionKernel')
            text = wrap_block(text, '            for cellIndex = 1:numel(groups)\n                names = sourceLabels{cellIndex};',
                              '        end\n\n        function program = jointProgram', 'avoidanceSafetyGeometry.projectedDataAssembly')
            text = wrap_block(text, '                records=cell(0,1);angles=zeros(0,1);',
                              '                reserve=zeros(numel(records),1);', 'avoidanceSafetyGeometry.jointRecordAssembly')
            text = wrap_block(text, '                reserve=zeros(numel(records),1);',
                              "                certificate=struct('records',records,", 'avoidanceSafetyGeometry.jointReserveAssembly')
            text = wrap_block(text, '            % Remove only collision and exit slices, keeping all actuator,',
                              '            program.geometry=geometry;program.jointCertificate=certificate;',
                              'avoidanceSafetyGeometry.discardedRowRemoval')
        if file == 'formulateAvoidanceProblem.m':
            text = wrap_block(text, '    safetyBound = bound;\n',
                              '    % Only the five path/velocity errors enter performance;',
                              'formulateAvoidanceProblem.clfAssembly')
            text = wrap_block(text, '    objectiveMap=zeros(5*count,planCount);objectiveOffset=zeros(5*count,1);\n',
                              "    layout = struct('planIndex',1:planCount,",
                              'formulateAvoidanceProblem.objectiveAssembly')
        if file == 'collisionAvoidanceController.m':
            text = wrap_block(text, '    predictedInput = reshape(result.decision(program.layout.planIndex),2,[]);\n',
                              '    if metadata.runtimeSeconds>cfg.solver.frameDeadlineSeconds\n',
                              'collisionAvoidanceController.outputAssembly')
        if file == 'solveHardCbfClf.m':
            text = wrap_block(text, '        for index=1:numel(records)\n            item=records(index);state=states(:,item.stage+1);',
                              '        residual=avoidanceSafetyGeometry.jointResidual(program,trial,directions);',
                              'solveHardCbfClf.candidateNormals')
            text = wrap_block(text, '    maps=program.prediction.egoStateMatrix;count=program.layout.planCount;',
                              "    inverse=hessian\\[map.'*error,scaled.'];",
                              'solveHardCbfClf.fitAssembly')
            text = wrap_block(text, "    inverse=hessian\\[map.'*error,scaled.'];",
                              '    change=inverse(:,1:2)', 'solveHardCbfClf.fitFactorization')
            begin = '    [nativeDecision, output] = nativeSolver( ...'
            end = '        options);\n    output.objectiveValue'
            assert text.count(begin) == 1 and text.count(end) == 1
            text = text.replace(begin, '    nativeProfileTimer=tic;\n' + begin)
            text = text.replace(end, "        options);\n"
                                "    nativeProfileSeconds=toc(nativeProfileTimer);\n"
                                "    matlabControllerProbe('native',struct('seconds',nativeProfileSeconds, ...\n"
                                "        'solverSeconds',output.solveTime,'iterations',output.iterations,'status',output.status, ...\n"
                                "        'variables',numel(linear),'rows',numel(bound),'nonzeros',nnz(program.A), ...\n"
                                "        'cones',program.cones));\n    output.objectiveValue")
        (output / file).write_text(text)
        manifest.append({'file': file, 'sourceSha256': hashlib.sha256(source.encode()).hexdigest(),
                         'instrumentedSha256': hashlib.sha256(text.encode()).hexdigest(), 'methods': names})
    (output / 'matlabControllerProbe.m').write_text('''function value=matlabControllerProbe(action,item)
% Temporary diagnostic timers. Does not receive or alter control decisions.
    persistent events stack nativeCalls
    value=[];
    switch action
        case 'reset'
            events={};stack=[];nativeCalls={};
        case 'start'
            value=numel(events)+1;
            events{value}=struct('name',item,'timer',tic,'seconds',0,'parent',0);
            if ~isempty(stack),events{value}.parent=stack(end);end
            stack(end+1)=value;
        case 'stop'
            events{item}.seconds=toc(events{item}.timer);
            assert(stack(end)==item);stack(end)=[];
        case 'native'
            nativeCalls{end+1}=item;
        case 'get'
            assert(isempty(stack));
            value=struct('events',{events},'nativeCalls',{nativeCalls});
            for index=1:numel(events),value.events{index}=rmfield(value.events{index},'timer');end
    end
end
''')
    (args.directory / 'instrumentation-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
