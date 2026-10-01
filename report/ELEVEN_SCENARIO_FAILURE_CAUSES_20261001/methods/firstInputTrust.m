root='/home/zai/.cache/collisionAvoidance/eleven-failure-causes-20261001';addpath(root);
source='/home/zai/.cache/collisionAvoidance/direct-two-stage-scenarios-20261001/source';cd(source);addpath('controller','config','scripts');
data=load(fullfile(root,'problems.mat'),'cases');base=load(fullfile(root,'baseline.mat'),'points');results=struct([]);
for j=[2,4,5,6,7,9,10,11]
    c=data.cases{j};p=c.capture.problem;m=c.capture.model;cfg=m.cfg;a=c.capture.anchor;
    for mode=["noFirstInputTrust","recenterFirstInputTrust"]
        q=p;z=base.points{j}.secondary;
        if mode=="noTerminal",q.cones=q.cones(2);end
        if any(mode==["noFutureTrust","noTrust"])
            ix=q.stateIndices;iu=q.inputIndices;lo=[max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor+1e-4);-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
            hi=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
            q.lower(ix(:))=-Inf;q.upper(ix(:))=Inf;q.lower(ix(4:6,2:end))=lo-a.states(4:6,2:end);q.upper(ix(4:6,2:end))=hi-a.states(4:6,2:end);
            il=[-cfg.model.frontWheelSteeringAngleMaximum;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];ih=[cfg.model.frontWheelSteeringAngleMaximum;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
            k=2;if mode=="noTrust",k=1;end
            q.lower(iu(:,k:end))=il-a.inputs(:,k:end);q.upper(iu(:,k:end))=ih-a.inputs(:,k:end);
        end
        if any(mode==["noFirstInputTrust","recenterFirstInputTrust"])
            il=[-cfg.model.frontWheelSteeringAngleMaximum;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];ih=[cfg.model.frontWheelSteeringAngleMaximum;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
            if mode=="recenterFirstInputTrust",il=max(il,m.previousInput-[.075;.125]);ih=min(ih,m.previousInput+[.075;.125]);end
            q.lower(q.inputIndices(:,1))=il-a.inputs(:,1);q.upper(q.inputIndices(:,1))=ih-a.inputs(:,1);
        end
        if mode=="freshFirstTangent"
            fresh=a;fresh.states(:,1)=m.initialState;fresh.states(:,2)=nonlinearBicycleModel.sample(m.initialState,a.inputs(:,1),cfg);q=reformulate(fresh,m);
        end
        if mode~="base"
            [s,z1]=runProbe(q,q.safetyObjective);z=[];
            if ~isempty(z1)
                q.a=[q.a;q.safetyObjective.'];q.b=[q.b;sum(max(0,z1(q.slackIndices)))+1e-6];o=zeros(size(q.lower));o(q.clfIndex)=1;[s,z]=runProbe(q,o);
            end
        else,s=struct('flag',NaN);end
        if isempty(z),fprintf('RECOVERY %g %s %s no-result\n',c.speed,c.scenario,mode);continue;end
        u=a.inputs(:,1)+z(q.inputIndices(:,1));[bound,umin,gradient]=localBound(q,a);
        actual=nonlinearBicycleModel.sample(m.initialState,u,cfg);e=nonlinearBicycleModel.error(actual,m.lane,m.nominalReference);actualV=norm(m.nominalReference.factor*e)^2;
        affineV=norm(q.clfMap*z+q.clfOffset)^2*q.clfScale;
        entry=struct('speed',c.speed,'scenario',c.scenario,'mode',mode,'flag',s.flag,'u',u,'independentMinInput',umin, ...
            'independentMinV',bound,'v0',q.initialClfValue,'predictedV',affineV,'actualV',actualV, ...
            'rho',q.clfScale*max(0,z(q.clfIndex)),'gradient',gradient,'initialError',nonlinearBicycleModel.error(m.initialState,m.lane,m.nominalReference), ...
            'anchorFirstInput',a.inputs(:,1),'firstStateMismatch',m.initialState-a.states(:,1));
        if isempty(results),results=entry;else,results(end+1)=entry;end
        fprintf('RECOVERY %g %s %s u=[%.5g %.5g] rho=%.5g best-gap=%.5g predDiff=%.5g realDiff=%.5g\n',c.speed,c.scenario,mode,u(1),u(2),entry.rho,affineV-bound,affineV-q.initialClfValue,actualV-q.initialClfValue);
    end
end
file=fopen(fullfile(root,'first-input-trust.json'),'w');fprintf(file,'%s\n',jsonencode(results));fclose(file);
function [value,u,gradient]=localBound(p,a)
    ix=p.stateIndices;iu=p.inputIndices;g=-p.clfMap(:,ix(:,2))*p.equal(7:12,iu(:,1));
    offset=p.clfOffset+p.clfMap(:,ix(:,2))*(p.rhs(7:12)-p.equal(7:12,ix(:,1))*p.rhs(1:6));
    h=g.'*g;f=g.'*offset;lo=p.lower(iu(:,1));hi=p.upper(iu(:,1));candidates=min(hi,max(lo,-h\f));
    for k=1:2
        l=3-k;for v=[lo(k),hi(k)],d=zeros(2,1);d(k)=v;d(l)=min(hi(l),max(lo(l),-(f(l)+h(l,k)*v)/h(l,l)));candidates(:,end+1)=d;end
    end
    [v,k]=min(sum((g*candidates+offset).^2,1));value=v*p.clfScale;u=a.inputs(:,1)+candidates(:,k);gradient=2*p.clfScale*f;
end
