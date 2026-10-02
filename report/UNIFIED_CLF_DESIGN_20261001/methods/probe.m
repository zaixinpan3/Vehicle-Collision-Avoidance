% Offline design diagnostic on recorded crossing states; no controller edit.
source='/home/zai/.cache/collisionAvoidance/recovery-clf-validation-20261001/source';
output='/home/zai/.cache/collisionAvoidance/unified-clf-design-20261001';
cd(source);addpath('controller','config','scripts');
records=jsondecode(fileread(fullfile(output,'states.json')));rows=struct([]);
for i=1:numel(records)
    record=records(i);cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',record.speed));
    lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',0,'length',200));
    reference=nonlinearBicycleModel.cruise(cfg,0);terminal=nonlinearBicycleModel.recoveryTerminal(cfg,0);
    x=record.state;previous=record.previousInput;issued=record.issuedInput;
    timer=tic;[v,~,steps,converged]=nonlinearBicycleModel.recoveryValue(x,previous,lane,reference,terminal,cfg);evaluation=toc(timer);
    e=nonlinearBicycleModel.error(x,lane,reference);stage=sum((terminal.stageFactor*e).^2);
    nominal=nonlinearBicycleModel.recoveryInput(x,previous,lane,reference,cfg,terminal);
    xn=nonlinearBicycleModel.sample(x,nominal,cfg);xi=nonlinearBicycleModel.sample(x,issued,cfg);
    [vn,~,~,nextConverged]=nonlinearBicycleModel.recoveryValue(xn,nominal,lane,reference,terminal,cfg);
    [vi,~,~,issuedConverged]=nonlinearBicycleModel.recoveryValue(xi,issued,lane,reference,terminal,cfg);
    row=struct('speed',record.speed,'time',record.time,'value',v,'stage',stage,'steps',steps, ...
        'converged',converged,'nextConverged',nextConverged,'issuedConverged',issuedConverged, ...
        'evaluationSeconds',evaluation,'nominalInput',nominal,'issuedInput',issued, ...
        'nominalValueChange',vn-v,'issuedValueChange',vi-v, ...
        'nominalBellmanResidual',vn-v+stage,'issuedMeetsDecrease',vi<=v-.5*stage+1e-6*max(1,v));
    if isempty(rows),rows=row;else,rows(end+1)=row;end
    fprintf('speed=%g t=%.2f V=%.5g l=%.5g K=%d converged=%d nominal dV=%.5g issued dV=%.5g eval=%.4f\n', ...
        record.speed,record.time,v,stage,steps,converged,vn-v,vi-v,evaluation);
end
% Examine the claimed exact identity inside the existing terminal quadratic.
core=struct([]);
for speed=[8,15]
    for curvature=[0,.005]
        cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
        lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
        reference=nonlinearBicycleModel.cruise(cfg,curvature);terminal=nonlinearBicycleModel.recoveryTerminal(cfg,curvature);
        for index=1:5
            for sign=[-1,1]
                e=zeros(5,1);e(index)=sign*sqrt(.0005/terminal.matrix(index,index));
                [p,heading]=laneGeometry.referencePose(0,e(1),lane.referenceCurve);
                x=[p;heading+reference.state(3)+e(2);reference.state(4:6)+e(3:5)];
                previous=reference.input;u=nonlinearBicycleModel.recoveryInput(x,previous,lane,reference,cfg,terminal);
                [v,~,steps]=nonlinearBicycleModel.recoveryValue(x,previous,lane,reference,terminal,cfg);
                xn=nonlinearBicycleModel.sample(x,u,cfg);
                vn=nonlinearBicycleModel.recoveryValue(xn,u,lane,reference,terminal,cfg);
                stage=sum((terminal.stageFactor*nonlinearBicycleModel.error(x,lane,reference)).^2);
                row=struct('speed',speed,'curvature',curvature,'axis',index,'sign',sign,'value',v,'steps',steps, ...
                    'stage',stage,'valueChange',vn-v,'bellmanResidual',vn-v+stage, ...
                    'meetsHalfDecrease',vn-v<=-.5*stage);
                if isempty(core),core=row;else,core(end+1)=row;end
            end
        end
    end
end
result=struct('recordedStates',rows,'terminalCore',core,'scope', ...
    'Offline target-independent CLF diagnostic; nominal feedback is not collision-screened or issued to the plant; no new closed-loop campaign.');
file=fopen(fullfile(output,'probe.json'),'w');fprintf(file,'%s\n',jsonencode(result));fclose(file);
fprintf('PROBE-COMPLETE\n');
