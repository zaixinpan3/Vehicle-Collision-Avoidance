% Does the CLF decrease carry over from one frame to the next?
% Compares the baseline controller (one-step CLF only) with the diagnostic
% "trackingThirdStage" variant, whose third stage minimizes sum_j V(x_j) over
% every plan node (a predictive CLF used as a cost), subject to both achieved
% slack levels. Both runs use the same frozen source (commit 5f74977, 6-mm
% margin) and the same fixtures; see report/RTI_FAILURE_ROOT_CAUSES_20261001.
% Frames: t >= 2 s, V0 >= 1 and zero first-stage (PCBF) optimum, so that the CLF
% stages are not overridden by a relaxed safety stage. Scope "all" uses every such
% frame (the target may still be within range); scope "noRisk" keeps only frames
% whose encounter-exit index is 0 (target beyond range).
%   anchorMeets : V(anchor node 1) <= 0.99 V0, i.e. the shifted previous plan,
%                 rolled out from the measured state, already meets this frame's
%                 one-step condition with zero correction
%   candidateDown: J(anchor) < J(previous plan), J = sum of V over the plan nodes;
%                 the shifted plan itself carries a lower horizon value
%   optimumDown : J(returned plan) < J(previous returned plan)
%   closedLoopDown: actual V after the hold < V0
W='/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001';
addpath(fullfile(W,'source','controller'),fullfile(W,'source','config'));
cases={{8,"turningCrossing"},{8,"curvedHeadOn"},{8,"curvedCrossing"},{8,"crossing"},{15,"headOn"}, ...
    {15,"turningCrossing"},{15,"curvedHeadOn"},{15,"curvedCrossing"},{15,"crossing"},{15,"brakingLead"}};
T=table();tol=[.1;pi/180;.1;.05;.01];
for c=1:numel(cases)
    speed=cases{c}{1};name=cases{c}{2};
    for variant=["baseline","trackingThirdStage"]
        if variant=="baseline",file=fullfile(W,'replay',sprintf('%s-%d-full.mat',name,speed));
        else,file=fullfile(W,'cf',sprintf('%s-%s-%d.mat',variant,name,speed));end
        load(file,'record');f=record.frame;n=numel(f);cfg=record.configuration;
        x=f(1).state;ego=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'lateralVelocity',x(5),'yawRate',x(6));
        [~,lane]=readControllerInputs(ego,[],record.road,cfg);
        frame=predictiveSafetyGeometry.roadFrame(lane,[]);reference=nonlinearBicycleModel.cruise(cfg,frame(4));
        V=@(s)norm(reference.factor*nonlinearBicycleModel.error(s,lane,reference))^2;
        Jplan=NaN(1,n);Janchor=NaN(1,n);a1=NaN(1,n);
        primary=[f.primaryOptimum];primary(~isfinite(primary))=Inf;
        base=[f.time]>=2 & [f.clfInitialValue]>=1 & primary<=1e-6;
        for k=find(base | [false,base(1:end-1)])
            S=f(k).affineStates;A=f(k).anchorStates;v=zeros(1,size(S,2)-1);w=v;
            for j=2:size(S,2),v(j-1)=V(S(:,j));w(j-1)=V(A(:,j));end
            Jplan(k)=sum(v);Janchor(k)=sum(w);a1(k)=w(1)/f(k).clfInitialValue;
        end
        V0=[f.clfInitialValue];rho=[f.clfSlack];V1=[f.actualClfNextValue];
        d0=arrayfun(@(e)e.plan(1,1)-e.anchorInputs(1,1),f);
        e=[f.error1];time=[f.time]+0.05;inside=all(abs(e)<=tol,1) & time>=8;start=NaN;recovered=NaN;
        for m=1:n
            if ~inside(m),start=NaN;elseif isnan(start),start=time(m);end
            if inside(m) && time(m)-start>=5-1e-10,recovered=time(m);break;end
        end
        for scope=["all","noRisk"]
            use=base;if scope=="noRisk",use=use & [f.encounterExit]==0;end
            k=find(use);pair=k(k>1 & use(max(1,k-1)));
            T=[T;table(variant,scope,string(name),speed,numel(k),mean([f(k).encounterExit]~=0),mean(rho(k)<=1e-6*max(1,V0(k))), ...
                mean(rho(k)>1e-6*max(1,V0(k)) & abs(d0(k))>0.0749),mean(a1(k)<=0.99),median(a1(k)), ...
                mean(Janchor(pair)<Jplan(pair-1)),mean(Jplan(pair)<Jplan(pair-1)),median(Jplan(pair)./Jplan(pair-1)), ...
                mean(V1(k)<V0(k)),median(V1(k)./V0(k)),recovered,n, ...
                'VariableNames',["variant","scope","scenario","speed","frames","shareInEncounter","shareRhoZero","shareSteeringEdge", ...
                "shareAnchorMeets","medianAnchorRatio","shareCandidateDown","shareOptimumDown","medianOptimumRatio", ...
                "shareClosedLoopDown","medianClosedLoopRatio","recoveredAt","recordedFrames"])]; %#ok<AGROW>
            fprintf('%-18s %-6s %-16s %2d: frames %4d (in encounter %5.1f%%) rho0 %5.1f%% edge %5.1f%% anchorMeets %5.1f%% (a1 %.4f) candDown %5.1f%% optDown %5.1f%% (J ratio %.4f) V1<V0 %5.1f%% (V ratio %.4f) recovered %s\n', ...
                variant,scope,name,speed,T.frames(end),100*T.shareInEncounter(end),100*T.shareRhoZero(end),100*T.shareSteeringEdge(end), ...
                100*T.shareAnchorMeets(end),T.medianAnchorRatio(end),100*T.shareCandidateDown(end),100*T.shareOptimumDown(end), ...
                T.medianOptimumRatio(end),100*T.shareClosedLoopDown(end),T.medianClosedLoopRatio(end),num2str(recovered));
        end
    end
end
writetable(T,'/home/zai/.cache/collisionAvoidance/predictive-clf-20261001/carry-forward.csv');
