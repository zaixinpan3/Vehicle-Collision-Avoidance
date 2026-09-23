function report=validateStandaloneControllerReplay(captureDirectory,nativeFile)
%validateStandaloneControllerReplay Compare every native result with MATLAB.
% Acceptance status, decision and angles must agree with the MATLAB replay.
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    baseline=jsondecode(fileread(fullfile(captureDirectory,'matlab-replay.json')));
    lines=readlines(nativeFile);lines=lines(strlength(lines)>0);
    replay=cell(numel(lines),1);
    for index=1:numel(lines),replay{index}=jsondecode(lines(index));end
    first=find(cellfun(@(item)item.iteration==0,replay));
    assert(numel(first)==numel(baseline.frames));
    report=struct('scope',baseline.scope,'frameCount',numel(first), ...
        'acceptanceDifferences',0,'maximumDecisionDifference',0,'maximumAngleDifference',0,'frames',{{}});
    for index=1:numel(first)
        actual=replay{first(index)};expected=baseline.frames(index);
        assert(strcmp(actual.file,expected.file));
        decisionDifference=NaN;angleDifference=NaN;
        accepted=actual.status>0;
        if accepted
            decision=actual.decision(:);angles=actual.angles(:);
            if expected.status>0
                decisionDifference=max(abs(decision-expected.decision));
                angleDifference=max(abs(atan2(sin(angles-expected.angles),cos(angles-expected.angles))));
                report.maximumDecisionDifference=max(report.maximumDecisionDifference,decisionDifference);
                report.maximumAngleDifference=max(report.maximumAngleDifference,angleDifference);
            end
        end
        if accepted~=(expected.status>0),report.acceptanceDifferences=report.acceptanceDifferences+1;end
        record=struct('scenario',expected.scenario,'curvature',expected.curvature,'frame',expected.frame, ...
            'accepted',accepted, ...
            'decisionDifference',decisionDifference,'angleDifference',angleDifference, ...
            'matlabStatus',expected.status,'nativeStatus',actual.status);
        report.frames{end+1}=record;
    end
    fid=fopen(fullfile(captureDirectory,'native-validation.json'),'w');assert(fid>=0);
    fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
    assert(report.acceptanceDifferences==0,'MATLAB and native acceptance differ.');
end
