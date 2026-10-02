cd('/home/zai/Downloads/ResearchProjects/collisionAvoidance');addpath('controller','config','scripts');
raw=jsondecode(fileread('/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/expanded/turningCrossing.json'));r=raw.results;cfg=r.configuration;delta=zeros(numel(r.trace),6);
for i=1:numel(r.trace),f=r.trace(i);pred=nonlinearBicycleModel.sample(f.state,f.input,cfg);delta(i,:)=(pred-f.nextState).';end
fprintf('ONE-HOLD errors %s\n',mat2str(max(abs(delta),[],1),9));
r=runNonlinearPredictiveSafetyValidation(Scenarios="turningCrossing",Frames=50,ControllerConfiguration=struct('referenceSpeed',15,'controller',struct('horizonSteps',16),'nonlinear',struct('integrationStep',.025)),RequireCollisionThreat=true,OutputFile='/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/expanded/mesh025.json');q=r.results;fprintf('MESH gap=%.9f frames=%d failure=%s\n',q.minimumReplayClearanceMeters,q.executedFrames,q.failure);
