function rows = replayEstimatorValidity(campaign,speedList,nameList,observerOverride)
%replayEstimatorValidity Replay the NRMM adapter on recorded truth and report
% when the estimator's ego error bound becomes invalid, and why.
% The adapter is deterministic for a given seed and truth trajectory, so this
% reproduces the estimate published in each recorded noisy campaign run. The
% controller is not called. Output rows give the first invalid time, the
% bound's reason, the yaw rate and speed there, and the run's peak yaw rate.
% The adapter's truth audit is also counted: frames whose true motion
% violates a declared ego premise, frames with an available ego or target
% bound, and available bounds that do not contain the truth.
% observerOverride (optional) is merged into the estimator's observer
% configuration, e.g. struct('ego',struct('domain',struct('yawRateMaximum',5))).
    arguments
        campaign (1,1) string
        speedList (1,:) double = [8 15]
        nameList (1,:) string = ["headOn","acceleratingHeadOn","brakingLead","crossing","turningCrossing","curvedHeadOn","curvedCrossing"]
        observerOverride struct = struct()
    end
rows=struct('speed',{},'scenario',{},'frames',{},'invalidFrom',{},'reason',{},'yawRateThere',{},'speedThere',{},'maximumYawRate',{},'minimumSpeed',{}, ...
    'premiseViolationFrames',{},'egoAvailableFrames',{},'egoUncontainedFrames',{},'targetAvailableFrames',{},'targetUncontainedFrames',{});
R=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(R,'scripts'),fullfile(R,'config'),fullfile(R,'controller'),fullfile(R,'estimator'));
for speed=speedList
for name=nameList
  r=jsondecode(fileread(fullfile(campaign,sprintf('speed%d-%s.json',speed,name)))).results;tr=r.trace;
  if isempty(tr),fprintf('VALID %2d %-19s no frames\n',speed,name);continue;end
  cfgC=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed,'controller',struct('horizonSteps',8+8*(speed==15))));
  [~,q0,~,cfgC]=collisionThreatScenario(name,cfgC);
  c=estimatorControllerIntegrationConfig(); c.randomSeed=20261003; c.observer=localMerge(c.observer,observerOverride);
  c.sensor.radar.rangeMaximum=cfgC.collision.encounterRangeMeters; c.observer.ego.yaw.rearAxleDistance=cfgC.vehicle.lr;
  tire=modifiedFialaTire.parameters(cfgC);
  c.observer.ego.domain.yawAccelerationMaximum=[cfgC.vehicle.lf,cfgC.vehicle.lr]*tire.longitudinalForceScale/cfgC.vehicle.Iz;
  mk=@(x,t,u)struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6),'longitudinalVelocity',x(4),'stateTime',t,'heldActuatorInput',u);
  tgt=@(t,~)localTarget(predictiveSafetyGeometry.predictTarget(q0,t));
  ctx=nrmmEstimatorControllerAdapter("initialize",c,mk(tr(1).state(:),0,[0;0]),tgt);
  first=NaN;reason="";valid=0;maxYawRate=0;minSpeed=Inf;counts=zeros(1,5);
  for k=1:numel(tr)
    x=tr(k).state(:);u=[0;0];if k>1,u=tr(k-1).input(:);end
    [ctx,~,~,~,audit]=nrmmEstimatorControllerAdapter("sample",ctx,tr(k).time,mk(x,tr(k).time,u),tgt(tr(k).time));
    t=audit.truthEnclosure;targetHere=~isempty(ctx.currentOutput.targetEstimate);
    counts=counts+[~t.egoPremisesSatisfied,t.egoBoundAvailable,t.egoBoundAvailable && ~t.egoContained, ...
        targetHere && t.targetBoundAvailable,targetHere && t.targetBoundAvailable && ~t.checkedTargetComponentsContained];
    maxYawRate=max(maxYawRate,abs(x(6)));minSpeed=min(minSpeed,hypot(x(4),x(5)));
    b=ctx.runtime.positionErrorBound;
    if b.egoValid,valid=valid+1;elseif isnan(first),first=tr(k).time;reason=string(b.reason);fr=k;end
  end
  there=[NaN,NaN];if ~isnan(first),xs=tr(fr).state(:);there=[abs(xs(6)),hypot(xs(4),xs(5))];end
  rows(end+1)=struct('speed',speed,'scenario',name,'frames',numel(tr),'invalidFrom',first,'reason',reason, ...
      'yawRateThere',there(1),'speedThere',there(2),'maximumYawRate',maxYawRate,'minimumSpeed',minSpeed, ...
      'premiseViolationFrames',counts(1),'egoAvailableFrames',counts(2),'egoUncontainedFrames',counts(3), ...
      'targetAvailableFrames',counts(4),'targetUncontainedFrames',counts(5)); %#ok<AGROW>
  fprintf('AUDIT %2d %-19s premise violated in %d/%d frames; ego bound available %d, not containing truth %d; target bound available %d, not containing truth %d\n', ...
      speed,name,counts(1),numel(tr),counts(2),counts(3),counts(4),counts(5));
  if isnan(first)
    fprintf('VALID %2d %-19s ego bound valid in all %d frames; max |r| %.3f, min speed %.2f\n',speed,name,numel(tr),maxYawRate,minSpeed);
  else
    x=tr(fr).state(:);
    fprintf('VALID %2d %-19s invalid from t=%.2f (frame %d of %d): %s; at that frame |r|=%.3f speed=%.2f; run max |r| %.3f min speed %.2f\n',speed,name,first,fr,numel(tr),reason,abs(x(6)),hypot(x(4),x(5)),maxYawRate,minSpeed);
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
