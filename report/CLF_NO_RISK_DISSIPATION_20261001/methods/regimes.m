% Classify every no-collision-risk frame (target beyond the encounter range at the
% current state) of each full replay, and measure how the CLF behaves there.
%   B  : CLF slack rho <= 1e-6*max(1,V0)  (the 1% one-step decrease is met)
%   A  : rho > 0 and the issued steering is on its trust-box edge
%   Ab : rho > 0, steering interior, braking on its trust-box edge
%   C  : rho > 0, both inputs interior
D='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001';
source='/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'));
files=dir(fullfile(D,'replay','*-full.mat'));T=table();
for i=1:numel(files)
    load(fullfile(files(i).folder,files(i).name),'record');f=record.frame;n=numel(f);cfg=record.configuration;
    trim=terminalContinuation.build(cfg,0).reference.input;
    exitStep=[f.encounterExit];rho=[f.clfSlack];V0=[f.clfInitialValue];V1=[f.actualClfNextValue];
    d0=zeros(2,n);sumFuture=zeros(1,n);steer=zeros(1,n);e0=zeros(5,n);
    for k=1:n
        d=f(k).plan-f(k).anchorInputs;d0(:,k)=d(:,1);sumFuture(k)=sum(d(1,2:end));steer(k)=f(k).input(1);e0(:,k)=f(k).error0;
    end
    free=exitStep==0;
    B=free & rho<=1e-6*max(1,V0);
    A=free & ~B & abs(d0(1,:))>0.0749;
    Ab=free & ~B & ~A & abs(d0(2,:))>0.1249;
    C=free & ~B & ~A & ~Ab;
    ratio=V1./V0;
    farB=B & V0>10;nearB=B & V0<=10;
    row=table(string(record.scenario),record.speed,nnz(free),nnz(B),nnz(A),nnz(Ab),nnz(C), ...
        median(ratio(farB)),median(ratio(nearB)),median(ratio(A)),median(abs(steer(A)-trim(1))),median(sumFuture(A)), ...
        median(abs(e0(1,A))),median(abs(e0(2,A))),localMax(V0(free)),localLast(V0(free)), ...
        'VariableNames',["scenario","speed","freeFrames","rhoZero","steeringEdge","brakingEdgeOnly","interior", ...
        "medianRatioRhoZeroV0Above10","medianRatioRhoZeroV0Below10","medianRatioSteeringEdge","medianSteeringMinusTrim", ...
        "medianFutureSteeringSum","medianAbsLateralAtEdge","medianAbsHeadingAtEdge","maxV0","finalV0"]);
    T=[T;row]; %#ok<AGROW>
end
T=sortrows(T,["speed","scenario"]);writetable(T,fullfile(D,'no-risk-regimes.csv'));
disp(T(:,1:9));disp(T(:,[1:2,10:16]));

function v=localMax(x)
    v=NaN;if ~isempty(x),v=max(x);end
end
function v=localLast(x)
    v=NaN;if ~isempty(x),v=x(end);end
end
