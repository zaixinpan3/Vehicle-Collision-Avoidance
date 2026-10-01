% Checks behind specific report statements.
W='/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001';
addpath(fullfile(W,'source','controller'),fullfile(W,'source','config'));
load(fullfile(W,'replay','crossing-8-full.mat'),'record');f=record.frame;late=[f.time]>=4;
d0=zeros(2,numel(f));a0=zeros(1,numel(f));for k=1:numel(f),d0(:,k)=f(k).input-f(k).anchorInputs(:,1);a0(k)=f(k).anchorInputs(1,1);end
fprintf('8 crossing t>=4: %d frames; steering +0.0750 edge %d; braking -0.1250 edge %d; anchor steering range [%.4f, %.4f]; heading error range [%.3f, %.3f] after 8 s\n', ...
    nnz(late),nnz(abs(d0(1,late)-0.075)<1e-6),nnz(abs(d0(2,late)+0.125)<1e-6),min(a0(late)),max(a0(late)), ...
    min(arrayfun(@(e)e.error0(2),f([f.time]>=8))),max(arrayfun(@(e)e.error0(2),f([f.time]>=8))));
files=dir(fullfile(W,'replay','*-full.mat'));
for i=1:numel(files)
    load(fullfile(files(i).folder,files(i).name),'record');f=record.frame;late=[f.time]>=10;
    d=zeros(1,numel(f));ey=d;ep=d;
    for k=1:numel(f),d(k)=f(k).input(1)-f(k).anchorInputs(1,1);ey(k)=f(k).error0(1);ep(k)=f(k).error0(2);end
    edge=abs(d)>0.0749&late;
    % toward the path: steering correction that reduces the heading component of the CLF gradient
    toward=sign(d)==-sign(148.7*ey+1564*ep);
    fprintf('%-26s edge frames %4d, of which turning in the CLF-descent direction of heading %4d\n',files(i).name,nnz(edge),nnz(edge&toward));
end
load(fullfile(W,'replay','curvedHeadOn-15-full.mat'),'record');f=record.frame;cfg=record.configuration;
shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
for k=1:3
    dm=Inf;st=NaN;
    for j=1:size(f(k).anchorStates,2)
        q=predictiveSafetyGeometry.targetFlow(record.targetEpoch,(f(k).sampleIndex+j-1)*cfg.controller.sampleTime);
        dd=predictiveSafetyGeometry.rectangle(f(k).anchorStates(1:3,j),shape,q(1:3),q(8:11));if dd<dm,dm=dd;st=j-1;end
    end
    fprintf('15 curvedHeadOn frame t=%.2f init=%s anchor min distance %.4f at stage %d; returned plan min %.4f at stage %d\n',f(k).time,f(k).initialization,dm,st,f(k).affineMargin,f(k).affineMarginStage);
end
for name=["trackingThirdStage-crossing-8","trackingThirdStage-turningCrossing-8","trackingThirdStage-curvedHeadOn-8","noStateTrust-crossing-8","noStateTrust-crossing-15","noStateTrust-turningCrossing-8"]
    file=fullfile(W,'cf',name+".mat");if ~isfile(file),continue;end
    load(file,'record');fprintf('%s: max controller seconds %.2f\n',name,max([record.frame.seconds]));
end
