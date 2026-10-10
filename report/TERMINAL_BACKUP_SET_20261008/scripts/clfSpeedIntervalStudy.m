function clfSpeedIntervalStudy(outFile)
% One quadratic CLF for a whole speed interval: common S = inv(P) with a
% certificate gain Y_k per grid speed, the same LMIs as synthesizeClfMatrices
% (contraction rho, certification input bounds, region and shape), bisection
% on rho. Reports the certified time constant and the tube scales
% a_i = sqrt(S_ii) against the per-speed synthesis.
root='/home/zai/Downloads/ResearchProjects/collisionAvoidance';
addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'));
addpath(genpath(fullfile(root,'solver','YALMIP')));addpath(genpath(fullfile(root,'solver','sedumi')));
fid=fopen(outFile,'w');
intervals={[8 8],[15 15],[6 8],[4 8],[10 15],[7.5 15],[4 15]};
for curvature=[0 .005]
    for k=1:numel(intervals)
        lo=intervals{k}(1);hi=intervals{k}(2);speeds=lo:.5:hi;if numel(speeds)==1,speeds=lo;end
        try
            [tc,scale]=localInterval(speeds,curvature);
            fprintf(fid,'curvature %.3f speeds %4.1f..%4.1f (%2d points): certified time constant %5.2f s (required 4.00); a = [%s]\n', ...
                curvature,lo,hi,numel(speeds),tc,sprintf('%.2f ',scale));
        catch e
            fprintf(fid,'curvature %.3f speeds %4.1f..%4.1f: %s\n',curvature,lo,hi,e.message);
        end
    end
end
fclose(fid);
end
function [tc,scale]=localInterval(speeds,curvature)
    cfg0=collisionAvoidanceControllerConfig(struct('referenceSpeed',speeds(1)));
    scales=[cfg0.clf.lateralPositionErrorScale,cfg0.clf.headingErrorScale,cfg0.clf.speedErrorScale, ...
        cfg0.clf.lateralVelocityErrorScale,cfg0.clf.yawRateErrorScale];
    region=cfg0.clf.certificationRegionScale;spread=cfg0.clf.shapeRatio;
    bound=[cfg0.clf.certificationSteeringRadians;cfg0.clf.certificationBrakingRatio];
    h=cfg0.controller.sampleTime;
    A=cell(1,numel(speeds));B=A;
    for k=1:numel(speeds)
        cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speeds(k)));
        point=nonlinearBicycleModel.operatingPoint(cfg,curvature);A{k}=point.a;B{k}=point.b;
    end
    low=0;high=1;solution=[];
    for step=1:18
        rho=(low+high)/2;[feasible,s]=localFeasible(A,B,rho,scales,region,spread,bound);
        if feasible,high=rho;solution=s;else,low=rho;end
    end
    if isempty(solution),error('infeasible at every contraction');end
    tc=-2*h/log(high);scale=sqrt(diag(solution)).';
end
function [feasible,s]=localFeasible(A,B,rho,scales,region,spread,bound)
    n=size(A{1},1);m=size(B{1},2);d=diag(scales.^2);
    s=sdpvar(n,n);constraints=[s>=region^2*d, s<=region^2*spread*d];
    for k=1:numel(A)
        y=sdpvar(m,n,'full');closed=A{k}*s+B{k}*y;
        constraints=[constraints,[rho*s,closed.';closed,s]>=0]; %#ok<AGROW>
        for j=1:m,constraints=[constraints,[bound(j)^2,y(j,:);y(j,:).',s]>=0];end %#ok<AGROW>
    end
    diagnostics=optimize(constraints,0,sdpsettings('solver','sedumi','verbose',0));
    feasible=diagnostics.problem==0;s=value(s);
end
