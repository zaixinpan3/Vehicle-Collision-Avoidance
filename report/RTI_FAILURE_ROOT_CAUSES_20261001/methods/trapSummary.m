% Trust-box position of issued inputs over the post-encounter part of each full replay.
files=dir('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/replay/*-full.mat');
rows={};
for i=1:numel(files)
    load(fullfile(files(i).folder,files(i).name),'record');f=record.frame;n=numel(f);
    d0=zeros(2,n);d1=zeros(2,n);e0=zeros(5,n);exitStep=[f.encounterExit];time=[f.time];
    for k=1:n,d0(:,k)=f(k).input-f(k).anchorInputs(:,1);d1(:,k)=f(k).plan(:,2)-f(k).anchorInputs(:,2);e0(:,k)=f(k).error0;end
    post=exitStep==0;first=find(post,1);
    if isempty(first),first=n;end
    late=false(1,n);late(first:end)=true;
    upper=d0(1,:)>0.0749;lower=d0(1,:)<-0.0749;
    % Steering that would reduce heading toward the path: sign of -(P12*ey+P22*epsi) is not used;
    % report instead whether the edge reached is the one that turns toward the path (sign of -ey).
    towardPath=sign(d0(1,:))==-sign(e0(1,:));
    dV=[f.actualClfNextValue]-[f.clfInitialValue];
    fprintf('%-24s encounter ends t=%6.2f | post frames %4d: steer at upper edge %4d, lower edge %4d, interior %4d; edge turns toward path %4d; median(u1-a1)=%+.4f opposite sign to (u0-a0) in %4d; brake at edge %4d; actual dV>0 %4d | final ey=%8.2f epsi=%6.3f\n', ...
        erase(files(i).name,'-full.mat'),time(first),nnz(late),nnz(upper&late),nnz(lower&late),nnz(~upper&~lower&late), ...
        nnz(towardPath&(upper|lower)&late),median(d1(1,late)),nnz(sign(d1(1,:))==-sign(d0(1,:))&late),nnz(abs(d0(2,:))>0.1249&late),nnz(dV>0&late), ...
        e0(1,end),e0(2,end));
end
