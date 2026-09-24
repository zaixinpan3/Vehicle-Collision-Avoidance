for n=["oncomingNoRoad","circularTargetNoRoad","estimatorStraightNoRoadFixed","estimatorCircularNoRoadFixed"]
  s=load(fullfile(getenv('OUT'),n+".mat")); r=s.result; a=r.attempts;
  fprintf('\n### %s  steps=%d completed=%d\n',n,numel(r.controlTime),r.metrics.completedControlSteps);
  md=a.metadata; if ~iscell(md), md=num2cell(md); end
  for k=1:numel(md)
    m=md{k}; if isempty(m), fprintf('k=%d empty metadata\n',k); continue; end
    if k==1, disp(fieldnames(m)'); if isfield(m,'search'), disp(fieldnames(m.search)'); end; end
    extra="";
    for nm=["nominalSource","readmittedAfterInconsistentObservation","targetReactionStrength","inheritedFeasibleFamily","usedFullPlanAdmission","targetErrorBound","runtimeSeconds","directionSeedSource","fullPlanStatus","previousPlanSeedStatus","reactionStrength","hardSolves","familyAttempts","usedCertifiedIncumbent","issuedAdmissionWitness"]
      v=[]; if isfield(m,nm), v=m.(nm); elseif isfield(m,'search') && isfield(m.search,nm), v=m.search.(nm); end
      if ~isempty(v), if isnumeric(v)||islogical(v), extra=extra+" "+nm+"="+mat2str(v,4); else, extra=extra+" "+nm+"="+string(v); end; end
    end
    fprintf('k=%3d t=%.2f vis=%d%s\n',k,a.time(k),a.targetVisible(k),extra);
  end
  fprintf('minimum SAT margin trace:\n'); if isfield(r.avoidance,'satMargin'), disp(r.avoidance.satMargin(:)'); end
  disp(r.avoidance); fc=r.failureContext; disp(fc); if isstruct(fc), fn=fieldnames(fc); for i=1:numel(fn), v=fc.(fn{i}); if isstruct(v), disp(fn{i}); disp(v); end; end; end
end
