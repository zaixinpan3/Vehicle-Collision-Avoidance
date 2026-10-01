root='/home/zai/.cache/collisionAvoidance/two-stage-cause-20261001';source='/home/zai/.cache/collisionAvoidance/two-stage-scenarios-20261001/source';
cd(source);addpath(root,'controller','config');data=load(fullfile(root,'problems.mat'),'cases');c=data.cases{13};p=c.capture.problem;
anchor=c.capture.anchor;model=c.capture.model;cfg=model.cfg;h=cfg.controller.sampleTime;results=struct([]);
for stage=[28,34]
    rows=find(p.rowGroups=="collisionStart" & p.rowStages==stage);
    fprintf('PLANE stage=%d absoluteTime=%.6f normal=[%.9g %.9g] rows=[%s]\n',stage, ...
        (model.sampleIndex+stage-1)*h,full(-p.a(rows(1),p.stateIndices(1,stage))), ...
        full(-p.a(rows(1),p.stateIndices(2,stage))),num2str(p.b(rows).',' %.9g'));
end
for side=[-1,1]
    trial=p;
    for stage=cfg.controller.horizonSteps+1:size(anchor.inputs,2)
        x=anchor.states(:,stage);u=anchor.inputs(:,stage);
        [middle,am,bm]=nonlinearBicycleModel.hold(x,u,cfg);
        for which=1:2
            if which==1,pose=x;time=(stage-1)*h;label="collisionStart";map=sparse(6,numel(p.lower));map(:,p.stateIndices(:,stage))=eye(6);
            else,pose=middle;time=(stage-.5)*h;label="collisionMid";map=sparse(6,numel(p.lower));map(:,p.stateIndices(:,stage))=am;map(:,p.inputIndices(:,stage))=bm;
            end
            rows=find(p.rowGroups==label & p.rowStages==stage);if isempty(rows),continue;end
            projection=laneGeometry.project(pose(1:2),model.lane);normal=side*[-sin(projection.heading);cos(projection.heading)];
            q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,model.sampleIndex*h+time);
            re=[cos(pose(3)),-sin(pose(3));sin(pose(3)),cos(pose(3))];rt=[cos(q(3)),-sin(q(3));sin(q(3)),cos(q(3))];
            body=cfg.vehicle.rectangleOffset+[cfg.vehicle.length/2;cfg.vehicle.width/2].*[-1,1,1,-1;-1,-1,1,1];
            ego=pose(1:2)+re*body;target=q(1:2)+rt*(q(10:11)+q(8:9).*[-1,1,1,-1;-1,-1,1,1]);
            g=(normal.'*ego-max(normal.'*target)).'-cfg.collision.safetyMarginMeters;
            jacobian=[repmat(normal.',4,1),(normal.'*re*[0,-1;1,0]*body).',zeros(4,3)];
            trial.a(rows,:)=-jacobian*map;trial.b(rows)=g;
        end
    end
    [stats,z]=runProbe(trial,p.safetyObjective);entry=struct('side',side,'stats',stats);
    if isempty(results),results=entry;else,results(end+1)=entry;end
    fprintf('LATERAL side=%d flag=%d slack=%.9g residual=%.9g\n',side,stats.flag,stats.safety,stats.rawResidual);
end
file=fopen(fullfile(root,'lateral-directions.json'),'w');fprintf(file,'%s\n',jsonencode(results));fclose(file);
