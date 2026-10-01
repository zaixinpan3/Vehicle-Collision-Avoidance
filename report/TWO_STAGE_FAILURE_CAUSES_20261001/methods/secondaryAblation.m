root='/home/zai/.cache/collisionAvoidance/two-stage-cause-20261001';addpath(root);
data=load(fullfile(root,'problems.mat'),'cases');base=load(fullfile(root,'baseline.mat'),'points');results=struct([]);
for j=[7,3,11]
    c=data.cases{j};p=base.points{j}.secondProblem;
    for mode=["base","capTimesMillion","slackVariablesScaled","terminalConeUnitRadius", ...
            "bothConesRescaled","cap1e-4","zeroSlackEliminated","withoutEndpointCone"]
        trial=p;scale=ones(size(p.lower));
        if mode=="capTimesMillion",trial.a(end,:)=trial.a(end,:)*1e6;trial.b(end)=trial.b(end)*1e6;end
        if mode=="slackVariablesScaled"
            scale(p.slackIndices)=p.b(end);trial.a(end,:)=trial.a(end,:)/p.b(end);trial.b(end)=1;
        end
        if any(mode==["terminalConeUnitRadius","bothConesRescaled"])
            r=c.capture.model.terminal.radius;cone=p.cones(1);
            trial.cones(1)=secondordercone(cone.A/r,cone.b/r,cone.d/r,cone.gamma/r);
        end
        if mode=="bothConesRescaled"
            cone=p.cones(2);q=max([abs(nonzeros(cone.A));abs(cone.b);abs(cone.d);abs(cone.gamma)]);
            trial.cones(2)=secondordercone(cone.A/q,cone.b/q,cone.d/q,cone.gamma/q);
        end
        if mode=="cap1e-4",trial.b(end)=trial.b(end)+1e-4-1e-6;end
        if mode=="zeroSlackEliminated",trial.a(end,:)=[];trial.b(end)=[];trial.upper(p.slackIndices)=0;end
        if mode=="withoutEndpointCone",trial.cones=p.cones(2);end
        if mode=="slackVariablesScaled"
            d=spdiags(scale,0,numel(scale),numel(scale));trial.a=trial.a*d;trial.equal=trial.equal*d;
            trial.lower=trial.lower./scale;trial.upper=trial.upper./scale;
            for k=1:numel(trial.cones)
                cone=trial.cones(k);trial.cones(k)=secondordercone(cone.A*d,cone.b,d*cone.d,cone.gamma);
            end
        end
        objective=zeros(size(p.lower));objective(p.clfIndex)=1;
        [stats,w]=runProbe(trial,objective);stats.originalResidual=NaN;
        if ~isempty(w)
            z=w.*scale;stats.safety=sum(max(0,z(p.slackIndices)));
            residual=[0;p.a*z-p.b;abs(p.equal*z-p.rhs);p.lower-z;z-p.upper];
            for cone=p.cones,residual(end+1)=norm(cone.A*z-cone.b)-cone.d.'*z+cone.gamma;end
            stats.originalResidual=max(residual);
        end
        entry=struct('speed',c.speed,'scenario',c.scenario,'mode',mode,'stats',stats);
        if isempty(results),results=entry;else,results(end+1)=entry;end
        fprintf('SECONDARY %g %s %s flag=%d rho=%.9g safety=%.9g originalResidual=%.9g dual=%.9g\n', ...
            c.speed,c.scenario,mode,stats.flag,stats.rho,stats.safety,stats.originalResidual,stats.dual);
    end
end
file=fopen(fullfile(root,'secondary-ablation.json'),'w');fprintf(file,'%s\n',jsonencode(results));fclose(file);
