root='/home/zai/.cache/collisionAvoidance/two-stage-cause-20261001';addpath(root);
data=load(fullfile(root,'problems.mat'),'cases');base=load(fullfile(root,'baseline.mat'),'points','results');results=struct([]);
for j=find(arrayfun(@(s)s.primary.flag>0,base.results))
    c=data.cases{j};p=base.points{j}.secondProblem;objective=zeros(size(p.lower));objective(p.clfIndex)=1;
    for mode=["normalLinearSolver","capAndEndpointNormalized"]
        trial=p;
        if mode=="normalLinearSolver",[stats,z]=normalProbe(trial,objective);
        else
            trial.a(end,:)=p.a(end,:)/p.b(end);trial.b(end)=1;
            cone=p.cones(1);r=c.capture.model.terminal.radius;
            trial.cones(1)=secondordercone(cone.A/r,cone.b/r,cone.d/r,cone.gamma/r);
            [stats,z]=runProbe(trial,objective);
        end
        stats.originalResidual=NaN;
        if ~isempty(z)
            residual=[0;p.a*z-p.b;abs(p.equal*z-p.rhs);p.lower-z;z-p.upper];
            for cone=p.cones,residual(end+1)=norm(cone.A*z-cone.b)-cone.d.'*z+cone.gamma;end
            stats.originalResidual=max(residual);
        end
        entry=struct('speed',c.speed,'scenario',c.scenario,'mode',mode,'stats',stats);
        if isempty(results),results=entry;else,results(end+1)=entry;end
        fprintf('EQUIVALENT %g %s %s flag=%d rho=%.9g residual=%.9g\n',c.speed,c.scenario,mode,stats.flag,stats.rho,stats.originalResidual);
    end
end
file=fopen(fullfile(root,'equivalent-solves.json'),'w');fprintf(file,'%s\n',jsonencode(results));fclose(file);
