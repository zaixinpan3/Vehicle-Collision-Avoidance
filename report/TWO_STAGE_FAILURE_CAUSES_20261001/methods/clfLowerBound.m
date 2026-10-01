root='/home/zai/.cache/collisionAvoidance/two-stage-cause-20261001';addpath(root);
data=load(fullfile(root,'problems.mat'),'cases');base=load(fullfile(root,'baseline.mat'),'points','results');results=struct([]);
for j=find(arrayfun(@(s)s.primary.flag>0,base.results))
    c=data.cases{j};p=base.points{j}.secondProblem;z=base.points{j}.secondary;
    ix=p.stateIndices;iu=p.inputIndices;
    g=-p.clfMap(:,ix(:,2))*p.equal(7:12,iu(:,1));
    offset=p.clfOffset+p.clfMap(:,ix(:,2))*(p.rhs(7:12)-p.equal(7:12,ix(:,1))*p.rhs(1:6));
    h=g.'*g;f=g.'*offset;lo=p.lower(iu(:,1));hi=p.upper(iu(:,1));
    candidates=min(hi,max(lo,-h\f));
    for a=1:2
        b=3-a;
        for v=[lo(a),hi(a)]
            u=zeros(2,1);u(a)=v;u(b)=min(hi(b),max(lo(b),-(f(b)+h(b,a)*v)/h(b,b)));candidates(:,end+1)=u;
        end
    end
    [value,k]=min(sum((g*candidates+offset).^2,1));u=candidates(:,k);
    rho=max(0,p.clfScale*value-(1-c.capture.model.cfg.nonlinear.clfDecay)*p.initialClfValue);
    entry=struct('speed',c.speed,'scenario',c.scenario,'lowerBound',rho,'returnedSlack',base.results(j).secondary.rho, ...
        'optimalityGapUpperBound',base.results(j).secondary.rho-rho,'minimizingInputIncrement',u, ...
        'returnedInputIncrement',z(iu(:,1)),'quadraticCondition',cond(h));
    if isempty(results),results=entry;else,results(end+1)=entry;end
    fprintf('LOWERBOUND %g %s rho=%.10g returned=%.10g gap=%.6g\n',c.speed,c.scenario,rho,entry.returnedSlack,entry.optimalityGapUpperBound);
end
file=fopen(fullfile(root,'clf-lower-bound.json'),'w');fprintf(file,'%s\n',jsonencode(results));fclose(file);
