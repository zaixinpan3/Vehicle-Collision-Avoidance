root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
output='/home/zai/.cache/collisionAvoidance/li-consistency-review-20261002';
cd(root);addpath('controller','config','scripts');
r=runtests('tests/ordinaryDistanceDualTest.m');assertSuccess(r);
results=table(r);writetable(results,fullfile(output,'tests.csv'));
d=load('/home/zai/.cache/collisionAvoidance/collision-diagnosis-20261002/primary-18.mat');
m=d.model;a=d.anchor;cfg=m.cfg;shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
q=predictiveSafetyGeometry.targetFlow(m.targetEpoch,1.65);
rows=predictiveSafetyGeometry.dualLinearization(a.states(1:3,17),shape,q(1:3),q(8:11));
box=[2.5;2.5;.25];bestRows=rows.value-.1+abs(rows.jacobian)*box;
report=struct('reviewedCommit','a073453f293ff7f47bcef4b9e92e096a9e13bbc9','oldSeedPlanningTime',.85, ...
 'predictionTime',1.65,'lambda',rows.lambda,'normal',rows.normal,'normalNorm',norm(rows.normal), ...
 'dualDistance',rows.distance,'jacobian',rows.jacobian,'solverFlag',rows.exitFlag, ...
 'independentCoordinateBoxUpperBounds',bestRows,'passedTests',nnz([r.Passed]),'totalTests',numel(r));
file=fopen(fullfile(output,'review.json'),'w');fprintf(file,'%s\n',jsonencode(report));fclose(file);disp(report);
