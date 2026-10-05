function rows = replayEstimatorValidity(campaign,speedList,nameList,observerOverride)
%replayEstimatorValidity Replay the NRMM adapter on recorded truth and audit its bounds.
% The adapter is deterministic for a given seed and truth trajectory, so this
% reproduces the estimate published in each recorded noisy campaign run; the
% controller is not called. The estimator configuration is the one the
% validation uses (estimatorConfigurationFromController with the scenario
% target contract), and every sensor sample is taken from the recorded
% plant trajectory: each hold is integrated again from its recorded state
% with its recorded input (ode45, 1e-11/1e-12), exactly as
% runNonlinearPredictiveSafetyValidation supplies its truth path.
% Each output row gives, per run: the first time the ego bound became
% invalid and why; frames whose true motion leaves the shared domain
% (sideslip cone, rear adhesion); frames with an available ego or target
% bound and those whose bound does not contain the truth; the largest
% difference between the replayed and the recorded estimate (a replay
% check); and median sizes of the published ego bound and target
% prediction set. observerOverride (optional) is merged into the
% estimator's observer configuration.
    arguments
        campaign (1,1) string
        speedList (1,:) double = [8 15]
        nameList (1,:) string = ["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"]
        observerOverride struct = struct()
    end
rows=struct([]);
R=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(R,'scripts'),fullfile(R,'config'),fullfile(R,'controller'),fullfile(R,'estimator'));
for speed=speedList
for name=nameList
  r=jsondecode(fileread(fullfile(campaign,sprintf('speed%d-%s.json',speed,name)))).results;tr=r.trace;
  if isempty(tr),fprintf('VALID %2d %-19s no frames\n',speed,name);continue;end
  cfgC=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed,'controller',struct('horizonSteps',8+8*(speed==15))));
  [~,q0,~,cfgC]=collisionThreatScenario(name,cfgC);
  c=estimatorControllerIntegrationConfig(); c.randomSeed=20261003;
  c.sensor.radar.rangeMaximum=cfgC.collision.encounterRangeMeters;
  c=estimatorConfigurationFromController(c,cfgC,collisionThreatContract(WindowSeconds=8));
  c.observer=localMerge(c.observer,observerOverride);
  h=cfgC.controller.sampleTime;sensorPeriod=c.observer.runtime.samplePeriod;
  pathTimes=(1:round(h/sensorPeriod))*sensorPeriod;
  tire=modifiedFialaTire.parameters(cfgC);k=3*tire.longitudinalForceScale(2)/tire.corneringStiffness(2);
  mk=@(x,t,u)struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6),'longitudinalVelocity',x(4),'stateTime',t,'heldActuatorInput',u);
  tgt=@(t,~)localTarget(predictiveSafetyGeometry.predictTarget(q0,t));
  ctx=nrmmEstimatorControllerAdapter("initialize",c,mk(tr(1).state(:),0,[0;0]),tgt);
  first=NaN;reason="";fr=NaN;counts=zeros(1,6);replayDifference=0;
  egoSize=NaN(numel(tr),3);setSize=NaN(numel(tr),5);truthPath=[];
  for f=1:numel(tr)
    x=tr(f).state(:);u=[0;0];if f>1,u=tr(f-1).input(:);end
    [ctx,egoOut,~,~,audit]=nrmmEstimatorControllerAdapter("sample",ctx,tr(f).time,mk(x,tr(f).time,u),tgt(tr(f).time),truthPath);
    published=[egoOut.egoPositionInertial(:);egoOut.egoYaw;egoOut.egoBodyVelocity(:);egoOut.egoYawRate];
    difference=published-tr(f).estimatedState(:);difference(3)=atan2(sin(difference(3)),cos(difference(3)));
    replayDifference=max(replayDifference,max(abs(difference)));
    t=audit.truthEnclosure;targetHere=~isempty(ctx.currentOutput.targetEstimate);
    adhesion=hypot((x(5)-cfgC.vehicle.lr*x(6))/k,x(4)*u(2))/x(4);
    outside=abs(atan2(x(5),x(4)))>cfgC.model.sideslipMaximum || adhesion>1;
    counts=counts+[outside,t.egoBoundAvailable,t.egoBoundAvailable && ~t.egoContained, ...
        targetHere && t.targetBoundAvailable,targetHere && t.targetBoundAvailable && ~t.checkedTargetComponentsContained, ...
        targetHere && t.targetBoundAvailable && any(t.targetBoundSlack(1:2)<-1e-9)];
    if t.egoBoundAvailable,egoSize(f,:)=egoOut.controllerStateErrorBound([1,3,5]).';end
    if targetHere && isfield(egoOut.targetEstimate,'predictionErrorSet')
      p=egoOut.targetEstimate.predictionErrorSet;
      if isstruct(p) && p.available
        setSize(f,:)=[p.positionRadius,p.courseRadius,diff(p.speedInterval)/2, ...
            diff(p.accelerationInterval)/2,diff(p.curvatureInterval)/2];
      end
    end
    b=ctx.runtime.positionErrorBound;
    if ~b.egoValid && isnan(first),first=tr(f).time;reason=string(b.reason);fr=f;end
    % The hold that this frame issued, integrated as the validation does.
    solution=ode45(@(~,state)nonlinearBicycleModel.derivative(state,tr(f).input(:),cfgC), ...
        [0,h],x,odeset('RelTol',1e-11,'AbsTol',1e-12));
    states=deval(solution,pathTimes);accelerations=zeros(2,numel(pathTimes));
    for j=1:numel(pathTimes)
      d=nonlinearBicycleModel.derivative(states(:,j),tr(f).input(:),cfgC);
      accelerations(:,j)=[d(4)-states(6,j)*states(5,j);d(5)+states(6,j)*states(4,j)];
    end
    truthPath=struct('times',tr(f).time+pathTimes,'states',states,'accelerations',accelerations,'input',tr(f).input(:));
  end
  row=struct('speed',speed,'scenario',name,'frames',numel(tr),'invalidFrom',first,'reason',reason, ...
      'domainExitFrames',counts(1),'egoAvailableFrames',counts(2),'egoUncontainedFrames',counts(3), ...
      'targetAvailableFrames',counts(4),'targetUncontainedFrames',counts(5),'targetPositionUncontainedFrames',counts(6), ...
      'replayDifference',replayDifference, ...
      'egoPositionBoundMedian',median(egoSize(:,1),'omitnan'),'egoYawBoundMedian',median(egoSize(:,2),'omitnan'), ...
      'egoLateralVelocityBoundMedian',median(egoSize(:,3),'omitnan'), ...
      'setPositionRadiusMedian',median(setSize(:,1),'omitnan'),'setCourseRadiusMedian',median(setSize(:,2),'omitnan'), ...
      'setSpeedHalfWidthMedian',median(setSize(:,3),'omitnan'),'setAccelerationHalfWidthMedian',median(setSize(:,4),'omitnan'), ...
      'setCurvatureHalfWidthMedian',median(setSize(:,5),'omitnan'));
  if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
  fprintf(['AUDIT %2d %-19s replay difference %.2g; outside shared domain %d/%d; ego bound available %d, not containing truth %d; ' ...
      'target bound available %d, not containing truth %d (position %d)\n'],speed,name,replayDifference,counts(1),numel(tr), ...
      counts(2),counts(3),counts(4),counts(5),counts(6));
  if isnan(first)
    fprintf('VALID %2d %-19s ego bound valid in all %d frames\n',speed,name,numel(tr));
  else
    fprintf('VALID %2d %-19s invalid from t=%.2f (frame %d of %d): %s\n',speed,name,first,fr,numel(tr),reason);
  end
end
end
end
function target=localTarget(q)
    direction=[cos(q(3)+q(6));sin(q(3)+q(6))];velocity=q(4)*direction;
    yawRate=q(4)*sin(q(6))/q(7);
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',velocity, ...
        'targetYawInertial',q(3),'targetYawRate',yawRate,'targetSideslip',q(6), ...
        'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7), ...
        'targetAccelerationInertial',q(5)*direction+yawRate*[-velocity(2);velocity(1)], ...
        'targetLength',2*q(8),'targetWidth',2*q(9),'targetRectangleOffset',q(10:11));
end

function base=localMerge(base,override)
    for field=string(fieldnames(override)).'
        if isstruct(override.(field)) && isfield(base,field) && isstruct(base.(field))
            base.(field)=localMerge(base.(field),override.(field));
        else
            base.(field)=override.(field);
        end
    end
end
