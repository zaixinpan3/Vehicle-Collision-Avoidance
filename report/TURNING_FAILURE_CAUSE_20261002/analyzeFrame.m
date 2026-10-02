source='/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/assessment/source';
output='/home/zai/.cache/collisionAvoidance/turning-root-cause-20261002';
cd(source);addpath('controller','config','scripts');
d=load('/home/zai/.cache/collisionAvoidance/collision-diagnosis-20261002/primary-18.mat');
m=d.model;a=d.anchor;p=d.problem;cfg=m.cfg;count=size(a.inputs,2);nv=numel(p.lower);
shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
% Reconstruct each start/midpoint safety row, retaining their original indices.
records=struct([]);cursor=0;
assert(isinf(cfg.model.brakingRatioRateMaximum));
for j=1:count
    [mid,am,bm]=nonlinearBicycleModel.hold(a.states(:,j),a.inputs(:,j),cfg);
    for half=0:1
        if j<=p.encounterExit
            if half==0,x=a.states(:,j);map=sparse(6,nv);map(:,p.stateIndices(:,j))=eye(6);
            else,x=mid;map=sparse(6,nv);map(:,p.stateIndices(:,j))=am;map(:,p.inputIndices(:,j))=bm;end
            time=.85+(j-1+half*.5)*.05;q=predictiveSafetyGeometry.targetFlow(m.targetEpoch,time);
            dual=predictiveSafetyGeometry.dualLinearization(x(1:3),shape,q(1:3),q(8:11),[0;1]);
            rows=cursor+(1:4);cursor=cursor+4;
            rec=struct('rows',rows,'x',x,'q',q,'normal',dual.normal,'map',map,'time',time,'j',j);
            if isempty(records),records=rec;else,records(end+1)=rec;end
            check=-[dual.jacobian,zeros(4,3)]*map;if j<=16,check(:,p.slackIndices(j))=-1;end
            assert(norm(check-p.a(rows,:),inf)<1e-8);assert(norm(p.b(rows)-(dual.value-.1),inf)<1e-8);
        end
    end
    cursor=cursor+6;
end
options=optimoptions('coneprog','Display','none','MaxIterations',400, ...
    'ConstraintTolerance',1e-8,'OptimalityTolerance',1e-7);
experiments=struct([]);
variants=[0,1,.1,0; -20,1,.1,0;-10,1,.1,0;-5,1,.1,0;5,1,.1,0;10,1,.1,0;20,1,.1,0; ...
    0,1,0,0;0,1.25,.1,0;0,1.5,.1,0;0,2,.1,0;0,1,.1,1];
for k=1:size(variants,1)
    angle=variants(k,1);scale=variants(k,2);buffer=variants(k,3);removeState=variants(k,4);
    pp=p;lo=p.lower;hi=p.upper;iu=p.inputIndices;
    lo(iu(1,:))=max(lo(iu(1,:)),-.075*scale);hi(iu(1,:))=min(hi(iu(1,:)),.075*scale);
    lo(iu(2,:))=max(lo(iu(2,:)),-.125*scale);hi(iu(2,:))=min(hi(iu(2,:)),.125*scale);
    if removeState
        lo(p.stateIndices(1:3,:))=-Inf;hi(p.stateIndices(1:3,:))=Inf;
        low=[max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor+1e-4);-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
        high=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
        lo(p.stateIndices(4:6,:))=low-a.states(4:6,:);hi(p.stateIndices(4:6,:))=high-a.states(4:6,:);
    end
    for rec=records
        n=rec.normal;
        if rec.time>=1.4 && rec.time<=1.9,n=[cosd(angle),-sind(angle);sind(angle),cosd(angle)]*n;end
        [values,jac]=fixedRows(rec.x,rec.q,n,shape);
        rows=-[jac,zeros(4,3)]*rec.map;if rec.j<=16,rows(:,p.slackIndices(rec.j))=-1;end
        pp.a(rec.rows,:)=rows;pp.b(rec.rows)=values-buffer;
    end
    pc=p.clfIndex-1;e=p.cones(1);cone=secondordercone(e.A(:,1:pc),e.b,e.d(1:pc),e.gamma);
    [point,val,flag]=coneprog(p.safetyObjective(1:pc),cone,pp.a(:,1:pc),pp.b,p.equal(:,1:pc),p.rhs,lo(1:pc),hi(1:pc),options);
    entry=struct('angleDegrees',angle,'inputScale',scale,'buffer',buffer,'removeStateTrust',removeState,'flag',flag,'primary',val,'minimumNonlinearGap',NaN,'gapTime',NaN);
    if ~isempty(point)
        controls=a.inputs+reshape(point(p.inputIndices),size(a.inputs));
        [entry.minimumNonlinearGap,entry.gapTime,states]=evaluate(controls,m,shape);
        save(fullfile(output,sprintf('variant-%02d.mat',k)),'point','controls','states','entry');
    end
    if isempty(experiments),experiments=entry;else,experiments(end+1)=entry;end
    fprintf('VARIANT angle=%g scale=%g buffer=%g removeState=%d flag=%d primary=%g nonlinearGap=%g at=%g\n',angle,scale,buffer,removeState,flag,val,entry.minimumNonlinearGap,entry.gapTime);
end
fid=fopen(fullfile(output,'variants.json'),'w');fprintf(fid,'%s\n',jsonencode(experiments));fclose(fid);
% Finite-horizon physical counterexamples; this is not a new online policy.
trials=struct([]);tail=nonlinearBicycleModel.nominalTail(cfg,0);
for ratio=[-.5,-.65,-.8]
    for duration=[.4,.6,.8,1]
        x=m.initialState;last=m.previousInput;controls=zeros(2,240);states=zeros(6,241);states(:,1)=x;
        for j=1:240
            if (j-1)*.05<duration,u=[0;ratio];
            else,u=nonlinearBicycleModel.nominalFeedback(x,last,m.lane,m.nominalReference,cfg,tail);end
            controls(:,j)=u;x=nonlinearBicycleModel.sample(x,u,cfg);states(:,j+1)=x;last=u;
        end
        [gap,time,~]=evaluate(controls,m,shape);
        entry=struct('ratio',ratio,'duration',duration,'minimumGap',gap,'gapTime',time,'endState',x, ...
            'minimumSpeed',min(states(4,:)),'maximumLateralSpeed',max(abs(states(5,:))),'maximumYawRate',max(abs(states(6,:))));
        if isempty(trials),trials=entry;else,trials(end+1)=entry;end
        save(fullfile(output,sprintf('trial-%02d.mat',numel(trials))),'entry','controls','states');
        fprintf('TRIAL b=%g duration=%g gap=%g at=%g\n',ratio,duration,gap,time);
    end
end
fid=fopen(fullfile(output,'physical-trials.json'),'w');fprintf(fid,'%s\n',jsonencode(trials));fclose(fid);
function [values,jac]=fixedRows(x,q,n,shape)
    r=[cos(x(3)),-sin(x(3));sin(x(3)),cos(x(3))];rt=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];
    signs=[-1,1,1,-1;-1,-1,1,1];body=shape(3:4)+shape(1:2).*signs;
    ego=x(1:2)+r*body;target=q(1:2)+rt*(q(10:11)+q(8:9).*signs);
    values=(n.'*ego-max(n.'*target)).';jac=[repmat(n.',4,1),(n.'*r*[0,-1;1,0]*body).'];
end
function [gap,time,states]=evaluate(controls,m,shape)
    cfg=m.cfg;x=m.initialState;gap=Inf;time=NaN;states=zeros(6,size(controls,2)+1);states(:,1)=x;
    for j=1:size(controls,2)
        for sub=0:9
            absolute=.85+(j-1+sub/10)*.05;q=predictiveSafetyGeometry.targetFlow(m.targetEpoch,absolute);
            [value,~]=predictiveSafetyGeometry.rectangle(x(1:3),shape,q(1:3),q(8:11));
            if value<gap,gap=value;time=absolute;end
            x=nonlinearBicycleModel.sample(x,controls(:,j),cfg,[],.005);
        end
        states(:,j+1)=x;
    end
end
