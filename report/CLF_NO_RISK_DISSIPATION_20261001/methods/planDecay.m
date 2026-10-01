% Does the returned plan carry the CLF decrease past its first hold, and does the
% shifted plan certify the next frame's decrease? No-collision-risk frames only.
%   r1   : V(plan node 1)/V0, the constrained first-hold ratio (affine prediction)
%   later: share of the later plan holds with V(j+1) <= 0.99 V(j)
%   rEnd : V(plan end)/V0
%   a1   : V(anchor node 1)/V0, the first-hold ratio of the shifted previous plan
%          rolled out from the measured state with zero correction
% rho = 0 statistics use frames with V0 >= 1 only, so that ratios near the
% equilibrium do not divide by vanishing values.
% Also writes the no-risk trajectory of each case (start, closest and final errors).
D='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001';
source='/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'));
files=dir(fullfile(D,'replay','*-full.mat'));T=table();R=table();
tol=[.1;pi/180;.1;.05;.01];
for i=1:numel(files)
    load(fullfile(files(i).folder,files(i).name),'record');f=record.frame;n=numel(f);cfg=record.configuration;
    x=f(1).state;ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
    [~,lane]=readControllerInputs(ego,[],record.road,cfg);
    frame=predictiveSafetyGeometry.roadFrame(lane,[]);reference=nonlinearBicycleModel.cruise(cfg,frame(4));
    V=@(s)norm(reference.factor*nonlinearBicycleModel.error(s,lane,reference))^2;
    free=find([f.encounterExit]==0);m=numel(free);
    check=0;for k=free(1:max(1,floor(m/50)):end),check=max(check,abs(V(f(k).state)-f(k).clfInitialValue)/max(1,f(k).clfInitialValue));end
    r1=NaN(1,m);later=NaN(1,m);rEnd=NaN(1,m);a1=NaN(1,m);regime=strings(1,m);
    for j=1:m
        k=free(j);S=f(k).affineStates;v=zeros(1,size(S,2));for c=1:size(S,2),v(c)=V(S(:,c));end
        V0=f(k).clfInitialValue;r1(j)=v(2)/V0;later(j)=mean(v(3:end)<=0.99*v(2:end-1));rEnd(j)=v(end)/V0;
        a1(j)=V(f(k).anchorStates(:,2))/V0;
        d=f(k).plan(:,1)-f(k).anchorInputs(:,1);
        if f(k).clfSlack<=1e-6*max(1,V0),regime(j)="rhoZero";
        elseif abs(d(1))>0.0749,regime(j)="steeringEdge";
        elseif abs(d(2))>0.1249,regime(j)="brakingEdge";
        else,regime(j)="interior";
        end
    end
    trap=regime=="steeringEdge" | regime=="brakingEdge";zero=regime=="rhoZero" & [f(free).clfInitialValue]>=1;
    actual=[f(free).actualClfNextValue]./[f(free).clfInitialValue];
    row=table(string(record.scenario),record.speed,m,nnz(trap),nnz(zero),check,localMedian(actual(trap)), ...
        localMedian(later(trap)),localMedian(rEnd(trap)),localShare(rEnd(trap)<1),localMedian(a1(trap)),localShare(a1(trap)<=0.99), ...
        localMedian(later(zero)),localMedian(rEnd(zero)),localMedian(a1(zero)),localShare(a1(zero)<=0.99), ...
        'VariableNames',["scenario","speed","freeFrames","trappedFrames","rhoZeroFramesV0AtLeast1","maxRelativeVCheck","trappedMedianActualRatio", ...
        "trappedMedianLaterDecreaseShare","trappedMedianEndRatio","trappedShareEndBelowV0","trappedMedianAnchorRatio","trappedShareAnchorMeetsDecay", ...
        "rhoZeroMedianLaterDecreaseShare","rhoZeroMedianEndRatio","rhoZeroMedianAnchorRatio","rhoZeroShareAnchorMeetsDecay"]);
    T=[T;row]; %#ok<AGROW>
    % No-risk trajectory: start, closest approach to the path, end; replayed recovery time.
    e=[f.error1];time=[f.time]+0.05;inside=all(abs(e)<=tol,1) & time>=8;start=NaN;recovered=NaN;
    for k=1:n
        if ~inside(k),start=NaN;elseif isnan(start),start=time(k);end
        if inside(k) && time(k)-start>=5-1e-10,recovered=time(k);break;end
    end
    % The no-risk start is the first free frame at or after 2 s, as for the idealized
    % policies (15-m/s curvedHeadOn is also free at 0 and 0.05 s, before its encounter).
    after=free([f(free).time]>=2);
    if ~isempty(after)
        e0=f(after(1)).error0;[~,c]=min(abs(e(1,after)));closest=e(:,after(c));last=find(time<=recovered,1,'last');if isempty(last),last=n;end
        R=[R;table(string(record.scenario),record.speed,f(after(1)).time,e0(1),e0(2),e0(3),closest(1),time(after(c)), ...
            e(1,last),e(2,last),e(3,last),recovered,'VariableNames',["scenario","speed","noRiskStart","startLateral", ...
            "startHeading","startSpeedError","closestLateral","closestTime","finalLateral","finalHeading","finalSpeedError","recoveredAt"])]; %#ok<AGROW>
    else
        R=[R;table(string(record.scenario),record.speed,NaN,NaN,NaN,NaN,NaN,NaN,e(1,end),e(2,end),e(3,end),recovered, ...
            'VariableNames',["scenario","speed","noRiskStart","startLateral","startHeading","startSpeedError","closestLateral", ...
            "closestTime","finalLateral","finalHeading","finalSpeedError","recoveredAt"])]; %#ok<AGROW>
    end
end
T=sortrows(T,["speed","scenario"]);R=sortrows(R,["speed","scenario"]);
writetable(T,fullfile(D,'plan-decay.csv'));writetable(R,fullfile(D,'no-risk-trajectories.csv'));
disp(T(:,1:7));disp(T(:,[1:2,8:12]));disp(T(:,[1:2,13:16]));disp(R);

function v=localMedian(x)
    v=NaN;if ~isempty(x),v=median(x);end
end
function v=localShare(x)
    v=NaN;if ~isempty(x),v=mean(x);end
end
