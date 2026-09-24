for n=["estimatorStraightNoRoadFixed","estimatorCircularNoRoadFixed"]
  s=load(fullfile(getenv('OUT'),n+".mat")); r=s.result; fc=r.failureContext; te=fc.targetEstimate; cs=fc.controllerState;
  fprintf('\n### %s failure t=%.2f, attempts=%d, first visible attempt=%d\n',n,r.failure.time,numel(r.attempts.targetVisible),find(r.attempts.targetVisible,1));
  fprintf('range %.2f m; relPosErrBound %.3f; inertialRelPosErrBound %.3f; posErrBound [%s]; velErrBound [%s]; speedErrBound %.2f; courseErrBound %.3f; yawErrBound %.3f; yawRateErrBound %.4f; speedRateErrBound %.3f; curvatureInterval [%s]; init=%s\n', ...
    norm(te.relativePosition),te.relativePositionErrorBound,te.inertialRelativePositionErrorBound,num2str(te.targetPositionInertialErrorBound'),num2str(te.targetVelocityInertialErrorBound'),te.targetSpeedErrorBound,te.targetCourseErrorBound,te.targetYawErrorBound,te.targetYawRateErrorBound,te.targetSpeedRateErrorBound,num2str(te.targetCurvatureInterval'),te.velocityInitializationMethod);
  pm=te.predictionMotion; fprintf('contract %s jerk [%s] curvMax %.4f speedRateMax %.3f accMax %.3f\n',pm.kind,num2str(pm.jerkBound'),pm.curvatureMaximum,pm.speedRateMaximum,pm.scalarAccelerationMaximum);
  disp(fieldnames(cs)'); if isfield(cs,'stateErrorBound'), fprintf('ego bound [%s]\n',num2str(cs.stateErrorBound')); end
  tt=r.targetTruthAtControlSample{end}; if ~isempty(tt), disp(tt); end
end
