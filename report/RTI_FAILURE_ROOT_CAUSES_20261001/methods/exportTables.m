% Export the ablation results and trust-box statistics to CSV for the report bundle.
W='/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001';out=fullfile(W,'bundle');if ~isfolder(out),mkdir(out);end
files=dir(fullfile(W,'ablation','*.mat'));T=table();
for i=1:numel(files)
    S=load(fullfile(files(i).folder,files(i).name),'all','record');
    for r=S.all
        row=table(string(S.record.scenario),S.record.speed,r.time,string(r.variant),r.exactReproduction,string(r.initialization), ...
            string(mat2str(r.flags)),r.pcbfOptimum,r.clfSlack,r.u0(1),r.u0(2),r.u0MinusAnchor(1),r.u0MinusAnchor(2), ...
            r.u1MinusAnchor(1),r.u1MinusAnchor(2),r.V0,r.V1affine-r.V0,r.V1actual-r.V0,r.holdClearance,r.futureSteerSum,r.endHeadingShift, ...
            'VariableNames',["scenario","speed","time","variant","exactReproduction","initialization","flags","pcbfOptimum","clfSlack", ...
            "steering","brakingRatio","steeringMinusAnchor","brakingMinusAnchor","nextSteeringMinusAnchor","nextBrakingMinusAnchor", ...
            "V0","predictedDeltaV","actualDeltaV","holdClearance","futureSteeringCorrectionSum","endpointHeadingShift"]);
        T=[T;row]; %#ok<AGROW>
    end
end
writetable(T,fullfile(out,'single-call-ablations.csv'));
files=dir(fullfile(W,'replay','*-full.mat'));U=table();
for i=1:numel(files)
    load(fullfile(files(i).folder,files(i).name),'record');f=record.frame;n=numel(f);
    d0=zeros(2,n);d1=zeros(2,n);ds=zeros(1,n);e0=zeros(5,n);time=[f.time];exitStep=[f.encounterExit];
    for k=1:n,d=f(k).plan-f(k).anchorInputs;d0(:,k)=d(:,1);d1(:,k)=d(:,2);ds(k)=sum(d(1,2:end));e0(:,k)=f(k).error0;end
    late=time>=10;
    row=table(string(record.scenario),record.speed,nnz(late),nnz(d0(1,:)>0.0749&late),nnz(d0(1,:)<-0.0749&late), ...
        nnz(abs(d0(2,:))>0.1249&late),nnz(sign(d1(1,:))==-sign(d0(1,:))&late),median(ds(late)),nnz(exitStep~=0&late), ...
        median(e0(3,late)),nnz([f(late).actualClfNextValue]>[f(late).clfInitialValue]), ...
        'VariableNames',["scenario","speed","framesAfter10s","steeringAtUpperEdge","steeringAtLowerEdge","brakingAtEdge", ...
        "nextSteeringCounterCorrected","medianFutureSteeringCorrectionSum","framesInEncounter","medianSpeedError","framesActualVIncrease"]);
    U=[U;row]; %#ok<AGROW>
end
writetable(U,fullfile(out,'trust-box-statistics.csv'));
disp(T(:,["scenario","speed","time","variant","flags","clfSlack","steeringMinusAnchor","actualDeltaV","futureSteeringCorrectionSum"]));disp(U);
