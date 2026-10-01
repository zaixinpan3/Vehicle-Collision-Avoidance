% Best achievable one-step change of the controller's CLF over the actuator range.
% For straight-lane states [x=0, y=e_y, psi=e_psi, v=v_ref, v_y=0, r=0], every input
% on a grid is propagated one 50-ms hold with the controller's own nonlinear model
% (one RK4 step), and V is evaluated with the controller's error and CLF matrix.
% Output: minimum V1/V0 per state and whether the 1% decrease is attainable.
source='/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'));
out='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001';
steering=linspace(-0.7,0.7,141);braking=linspace(-0.999,0.999,41);
lateral=[-100,-60,-40,-30,-20,-15,-10,-7,-5,-3,-2,-1,-0.5,-0.2];
heading=linspace(-pi,pi,73);heading=heading(1:end-1); % -180 .. 175 degrees
rows=table();
for speed=[8,15]
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
    road=struct('centerline',[-1000,0;3000,0]);
    ego=struct('position',[0;0],'yaw',0,'speed',speed,'lateralVelocity',0,'yawRate',0);
    [~,lane]=readControllerInputs(ego,[],road,cfg);
    reference=nonlinearBicycleModel.cruise(cfg,0);decay=1-cfg.nonlinear.clfDecay;
    for ey=lateral
        for epsi=heading
            x=[0;ey;epsi;reference.state(4);0;0];
            e0=nonlinearBicycleModel.error(x,lane,reference);V0=norm(reference.factor*e0)^2;
            best=Inf;bestU=[NaN;NaN];
            for d=steering
                for b=braking
                    try
                        x1=nonlinearBicycleModel.sample(x,[d;b],cfg);
                    catch
                        continue;
                    end
                    V1=norm(reference.factor*nonlinearBicycleModel.error(x1,lane,reference))^2;
                    if V1<best,best=V1;bestU=[d;b];end
                end
            end
            rows=[rows;table(speed,ey,epsi,V0,best,best/V0,best<=decay*V0,best<V0,bestU(1),bestU(2), ...
                'VariableNames',["speed","lateralError","headingError","V0","minV1","minRatio","onePercentFeasible", ...
                "anyDecrease","argminSteering","argminBraking"])]; %#ok<AGROW>
        end
    end
end
writetable(rows,fullfile(out,'clf-feasibility-map.csv'));
for speed=[8,15]
    fprintf('v=%d m/s: one-step 1%% CLF decrease attainable\n',speed);
    for ey=lateral
        r=rows(rows.speed==speed & rows.lateralError==ey,:);
        ok=r.headingError(r.onePercentFeasible);dec=r.headingError(r.anyDecrease);
        fprintf('  e_y=%6.1f m: 1%% feasible for %2d of %d headings (%s); any decrease for %2d\n',ey,numel(ok),height(r), ...
            localRange(ok),numel(dec));
    end
end
function s=localRange(h)
    if isempty(h),s='none';return;end
    s=sprintf('%.0f..%.0f deg',rad2deg(min(h)),rad2deg(max(h)));
end
