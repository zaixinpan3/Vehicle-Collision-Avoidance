root='/home/zai/.cache/collisionAvoidance/two-stage-cause-20261001';addpath(root);
data=load(fullfile(root,'problems.mat'),'cases');cases=data.cases;results=struct([]);points=cell(size(cases));
for j=1:numel(cases)
    c=cases{j};p=c.capture.problem;
    [primary,z1]=runProbe(p,p.safetyObjective);second=[];z2=[];
    if primary.flag>0
        p.a=[p.a;p.safetyObjective.'];p.b=[p.b;sum(max(0,z1(p.slackIndices)))+1e-6];
        objective=zeros(size(p.lower));objective(p.clfIndex)=1;
        [second,z2]=runProbe(p,objective);
    end
    entry=struct('speed',c.speed,'scenario',c.scenario,'primary',primary,'secondary',second, ...
        'clfScale',p.clfScale,'radius',c.capture.model.terminal.radius, ...
        'anchorMismatch',c.capture.model.initialState-c.capture.anchor.states(:,1), ...
        'encounterExit',p.encounterExit,'initialLowerUpperConflict',max(p.lower-p.upper));
    if isempty(results),results=entry;else,results(end+1)=entry;end
    points{j}=struct('primary',z1,'secondary',z2,'secondProblem',p);
    fprintf('BASE %g %s primary=%d',c.speed,c.scenario,primary.flag);
    if ~isempty(second),fprintf(' secondary=%d rho=%.9g normalized=%.9g safety=%.9g residual=%.9g dual=%.9g', ...
        second.flag,second.rho,second.rhoNormalized,second.safety,second.rawResidual,second.dual);end
    fprintf('\n');
end
save(fullfile(root,'baseline.mat'),'results','points');
file=fopen(fullfile(root,'baseline.json'),'w');fprintf(file,'%s\n',jsonencode(results));fclose(file);
