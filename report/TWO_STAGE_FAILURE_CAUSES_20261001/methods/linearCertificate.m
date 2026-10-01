root='/home/zai/.cache/collisionAvoidance/two-stage-cause-20261001';addpath(root);
data=load(fullfile(root,'problems.mat'),'cases');results=struct([]);certificates=cell(1,3);k=0;
options=optimoptions('linprog','Display','none','ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9);
for j=[4,6,13]
    c=data.cases{j};p=c.capture.problem;n=numel(p.lower);k=k+1;
    [~,~,flag]=linprog(zeros(n,1),p.a,p.b,p.equal,p.rhs,p.lower,p.upper,options);
    for mode=["original","zeroDynamicsDefect","zeroInitialMismatch","onlyDynamicsAndTrust"]
        q=p;groups=q.rowGroups;
        if mode=="zeroDynamicsDefect",q.rhs(7:end)=0;end
        if mode=="zeroInitialMismatch",q.rhs(1:6)=0;end
        if mode=="onlyDynamicsAndTrust"
            q.a=zeros(0,n);q.b=zeros(0,1);q.lower=-Inf(n,1);q.upper=Inf(n,1);
            radius=c.capture.model.cfg.nonlinear.trustRadius;
            q.lower(p.stateIndices(:))=-repmat(radius*[5;5;.5;5;3;1.5],size(p.stateIndices,2),1);
            q.lower(p.inputIndices(:))=-repmat(radius*[.15;.25],size(p.inputIndices,2),1);
            q.upper=-q.lower;
        end
        [~,~,trialFlag]=linprog(zeros(n,1),q.a,q.b,q.equal,q.rhs,q.lower,q.upper,options);
        fprintf('LINEAR %g %s %s flag=%d\n',c.speed,c.scenario,mode,trialFlag);
    end
    lower=find(isfinite(p.lower));upper=find(isfinite(p.upper));
    a=[p.a;sparse(1:numel(lower),lower,-1,numel(lower),n);sparse(1:numel(upper),upper,1,numel(upper),n)];
    b=[p.b;-p.lower(lower);p.upper(upper)];m=numel(b);ne=size(p.equal,1);
    costs=[b;p.rhs];eq=[a.',p.equal.';ones(1,m),zeros(1,ne)];rhs=[zeros(n,1);1];
    [dual,value,certificateFlag]=linprog(costs,[],[],eq,rhs,[zeros(m,1);-Inf(ne,1)],[],options);
    assert(certificateFlag==1 && value<0);
    indices=find(dual(1:m)>1e-7);terms=struct([]);
    for index=indices.'
        if index<=numel(p.b)
            group=p.rowGroups(index);stage=p.rowStages(index);variable=0;
        elseif index<=numel(p.b)+numel(lower)
            group="lowerBound";variable=lower(index-numel(p.b));stage=0;
        else
            group="upperBound";variable=upper(index-numel(p.b)-numel(lower));stage=0;
        end
        coordinate=0;anchorValue=NaN;
        if variable>0
            [coordinate,stage]=find(p.stateIndices==variable);
            if ~isempty(coordinate),group=group+"State";anchorValue=c.capture.anchor.states(coordinate,stage);
            else
                [coordinate,stage]=find(p.inputIndices==variable);
                if ~isempty(coordinate),group=group+"Input";anchorValue=c.capture.anchor.inputs(coordinate,stage);
                else,stage=0;coordinate=0;
                end
            end
        end
        term=struct('row',index,'group',group,'stage',stage,'coordinate',coordinate,'variable',variable, ...
            'multiplier',dual(index),'bound',b(index),'anchorValue',anchorValue);
        if isempty(terms),terms=term;else,terms(end+1)=term;end
    end
    [maximumDefect,ix]=max(abs(p.rhs(7:end)));[coord,node]=ind2sub([6,size(p.inputIndices,2)],ix);
    entry=struct('speed',c.speed,'scenario',c.scenario,'linearFeasibilityFlag',flag,'certificateValue',value, ...
        'certificateResidual',norm(eq*dual-rhs,inf),'support',terms, ...
        'maximumDynamicsDefect',maximumDefect,'defectCoordinate',coord,'defectStage',node);
    if isempty(results),results=entry;else,results(end+1)=entry;end
    certificates{k}=struct('a',a,'b',b,'dual',dual,'equal',p.equal,'rhs',p.rhs);
    fprintf('CERTIFICATE %g %s value=%.9g residual=%.3g support=%d maxDefect=%.6g coordinate=%d stage=%d\n', ...
        c.speed,c.scenario,value,entry.certificateResidual,numel(terms),maximumDefect,coord,node);
end
save(fullfile(root,'linear-certificates.mat'),'certificates','results');
file=fopen(fullfile(root,'linear-certificates.json'),'w');fprintf(file,'%s\n',jsonencode(results));fclose(file);
