source='/home/zai/.cache/collisionAvoidance/unified-clf-implementation-20261001/assessment/source';
output='/home/zai/.cache/collisionAvoidance/flow-visualization-20261002';
cd(source);addpath('controller','config','scripts');
data=load('/home/zai/.cache/collisionAvoidance/collision-diagnosis-20261002/primary-18.mat');
selected=load('/home/zai/.cache/collisionAvoidance/collision-diagnosis-20261002/frame-18.mat');
m=data.model;cfg=m.cfg;anchor=data.anchor;count=size(anchor.inputs,2);
shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
q=zeros(11,count+1);gaps=zeros(1,count+1);times=.85+(0:count)*.05;
for j=1:count+1
    q(:,j)=predictiveSafetyGeometry.targetFlow(m.targetEpoch,times(j));
    gaps(j)=predictiveSafetyGeometry.rectangle(anchor.states(1:3,j),shape,q(1:3,j),q(8:11,j));
end
export=struct('frameTime',.85,'sampleTime',.05,'prefixSteps',16,'times',times, ...
    'states',anchor.states,'inputs',anchor.inputs,'targetStates',q,'egoShape',shape, ...
    'nodeClearance',gaps,'targetEpoch',m.targetEpoch,'vehicle',cfg.vehicle, ...
    'safetyBuffer',cfg.collision.safetyMarginMeters,'search',selected.pred.metadata.search, ...
    'affineOptimizedStates',selected.pred.predictedState,'optimizedInputs',selected.pred.solution.inputs);
file=fopen(fullfile(output,'flow-frame.json'),'w');fprintf(file,'%s\n',jsonencode(export));fclose(file);
% Necessary row-wise feasibility bounds with dynamics and box constraints only.
% The selected saved problem has input scale two; reconstruct its scale-one box.
p=data.problem;low=p.lower;high=p.upper;iu=p.inputIndices;
low(iu(1,:))=max(low(iu(1,:)),-.075);high(iu(1,:))=min(high(iu(1,:)),.075);
low(iu(2,:))=max(low(iu(2,:)),-.125);high(iu(2,:))=min(high(iu(2,:)),.125);
rows=[];proof=struct();bestBound=Inf;options=optimoptions('linprog','Display','none');
% A violating single linear row would suffice to prove local infeasibility.
for index=17:22
    dual=predictiveSafetyGeometry.dualLinearization(anchor.states(1:3,index),shape,q(1:3,index),q(8:11,index),[0;1]);
    for corner=1:4
        objective=zeros(size(low));objective(p.stateIndices(1:3,index))=-dual.jacobian(corner,:).';
        [witness,value,flag]=linprog(objective,[],[],p.equal,p.rhs,low,high,options);
        if isempty(value),value=NaN;end
        maxMargin=dual.value(corner)-.1-value;
        if maxMargin<bestBound
            bestBound=maxMargin;
            matching=find(max(abs(p.a-objective.'),[],2)<1e-9 & abs(p.b-(dual.value(corner)-.1))<1e-9);
            assert(~isempty(matching),'The diagnosed row must exist in the saved formulation.');
            proof=struct('predictionStage',index,'absoluteTime',times(index),'corner',corner, ...
                'initialMargin',dual.value(corner)-.1,'maximumMargin',maxMargin, ...
                'solverFlag',flag,'matchingFormulationRows',matching,'equalityResidual',norm(p.equal*witness-p.rhs,inf), ...
                'boxResidual',max([low-witness;witness-high]),'interpretation', ...
                'Upper bound from affine dynamics and variable boxes only; other inequality rows and terminal cone omitted.');
        end
        rows=[rows;index,times(index),corner,dual.value(corner)-.1,maxMargin,flag];
    end
end
writematrix(rows,fullfile(output,'row-reachability.csv'));
% Test the complete linear part without the terminal cone, for scope attribution.
[~,~,linearFlag]=linprog(p.safetyObjective,p.a,p.b,p.equal,p.rhs,low,high,options);
fprintf('Scale-one linear constraints without endpoint cone: flag %d\n',linearFlag);
fprintf('Minimum one-row maximum attainable margin %.9f m\n',min(rows(:,5)));
file=fopen(fullfile(output,'infeasible-row.json'),'w');fprintf(file,'%s\n',jsonencode(proof));fclose(file);
