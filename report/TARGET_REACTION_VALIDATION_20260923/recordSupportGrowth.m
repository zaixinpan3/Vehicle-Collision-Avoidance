function table_ = recordSupportGrowth(outputFile)
%recordSupportGrowth Collision-record support versus prediction time, first frame.
% Offline diagnostic, run from the repository root. For maneuvering-target
% cases it builds the first-frame program for: the jerk-only reachable set with
% the ego-only feedback tube; the declared acceleration maximum with the
% ego-only tube; and the declared maximum with the target-reactive tube at
% reaction strengths 30 and 100. It reports the largest support
% ||G' n||_1 of the collision records at 1, 2, 3, 4 and 4.8 s. The
% directions are the frame's nominal ones; no solve is involved.
    addpath("controller","config","scripts");
    egoBound = [0.076;0.076;0.048;0.089;0.497;0.0015];
    specs = {"stationary",0,2,3;"stationary",0.01,2,3;"crossing",0.01,1,2};
    times = [1,2,3,4,4.8];rows = {};
    for index = 1:size(specs,1)
        [scenario,curvature,jerk,maximum] = specs{index,:};
        for variant = ["jerkOnly-egoOnly","capped-egoOnly","capped-reactive30","capped-reactive100"]
            cap = maximum;if variant=="jerkOnly-egoOnly",cap = Inf;end
            model = localCapture(scenario,curvature,jerk,cap,egoBound);
            [program,~] = formulateAvoidanceProblem(model);
            if startsWith(variant,"capped-reactive")
                strength = str2double(extractAfter(variant,"reactive"));
                records = program.jointCertificate.records;angles = program.jointCertificate.angles;
                residual = avoidanceSafetyGeometry.jointResidual(program,program.feasibleWitness,angles) ...
                    -program.jointCertificate.upperBound;
                weights = exp(-max(0,-residual-model.cfg.feedbackPrediction.targetReaction.relevanceMeters));
                design = ltvBicycleModel.reactionGains(model,program.prediction,records,angles,weights,strength);
                reactive = model;reactive.reactionDesign = design;
                program = formulateAvoidanceProblem(reactive);
            end
            records = program.jointCertificate.records;angles = program.jointCertificate.angles;
            stage = [records.stage];support = zeros(size(stage));
            for r = 1:numel(records)
                normal = [cos(angles(r));sin(angles(r))];
                support(r) = sum(abs(records(r).generators.'*normal))+records(r).positionBall;
            end
            collision = ~[records.isExit];
            values = nan(1,numel(times));
            for t = 1:numel(times)
                node = round(times(t)/model.sampleTime);
                pick = collision & stage==node;
                if any(pick),values(t) = max(support(pick));end
            end
            reserve = max(program.prediction.feedbackInputSupport,[],2);
            rows(end+1,:) = {scenario,curvature,jerk,maximum,variant,program.prediction.stageCount*model.sampleTime, ...
                values(1),values(2),values(3),values(4),values(5),reserve(1),reserve(2)}; %#ok<AGROW>
            fprintf("%-10s k=%-4g J=%g %-20s horizon %.2f s | support at 1/2/3/4/4.8 s: %s m | reserve steer %.3f brake %.3f\n", ...
                scenario,curvature,jerk,variant,rows{end,6},sprintf("%6.2f ",values),reserve(1),reserve(2));
        end
    end
    table_ = cell2table(rows,"VariableNames",["scenario","curvature","jerk","accelerationMaximum","variant", ...
        "horizonSeconds","support1s","support2s","support3s","support4s","support4p8s","reserveSteer","reserveBrake"]);
    if nargin>0,writetable(table_,outputFile);end
end

function model = localCapture(scenario,curvature,jerk,maximum,egoBound)
% The harness's first-frame inputs, prepared as collisionAvoidanceController does.
    cfg = collisionAvoidanceControllerConfig(struct("referenceSpeed",8, ...
        "controller",struct("sampleTime",0.05,"horizonSteps",32,"minimumHorizonSteps",1), ...
        "model",struct("lateralDomainRadius",4), ...
        "solver",struct("frameDeadlineSeconds",Inf,"certificateSearchTimeLimit",60)));
    [cruiseState,~] = ltvBicycleModel.cruiseEquilibrium(curvature,cfg);
    target = struct("center",[15;0;0;0;0;0;0;0]);
    road = struct("centerline",[-100,0;2000,0]);
    x = [0;0;0;8;0;0];position = x(1:2);heading = 0;
    if curvature~=0
        curve = struct('origin',[0;0],'heading',0,'curvature',curvature, ...
            'length',min(8*1*0.05+100,1.9*pi/abs(curvature)));
        road = struct('referenceCurve',curve,'centerline',laneGeometry.referencePose(linspace(0,curve.length,201),0,curve).');
        [point,pathHeading] = laneGeometry.referencePose(15,0,curve);
        target.center = [point;zeros(4,1);pathHeading;0];
        if scenario=="oncoming"
            [point,pathHeading] = laneGeometry.referencePose(30,0,curve);
            tangent = [cos(pathHeading);sin(pathHeading)];
            target.center = [point+30*tangent;-8*tangent;zeros(2,1);pathHeading+pi;0];
        elseif scenario=="crossing"
            normal = [-sin(pathHeading);cos(pathHeading)];
            target.center = [point-7.5*normal;4*normal;zeros(2,1);pathHeading+pi/2;0];
        end
        x = cruiseState;[position,heading] = laneGeometry.fromFrenet(x,road);
    else
        if scenario=="oncoming",target.center = [60;0;-8;0;0;0;pi;0];end
        if scenario=="crossing",target.center = [15;-4;0;32;0;0;pi/2;0];end
    end
    ego = struct("position",position,"yaw",heading,"speed",x(4),"lateralVelocity",x(5),"yawRate",x(6), ...
        "stateTime",0,"controllerStateErrorBound",egoBound, ...
        "perception",struct('time',0,'range',16,'completeWithinRange',true));
    motion = struct("kind","finite-sensing-motion-v1","jerkBound",jerk*[1;1],"yawAccelerationBound",0);
    if isfinite(maximum),motion.scalarAccelerationMaximum = maximum;end
    bound = [.1;.1;.05;.05;.01;.01;.01;.01];
    observation = struct("trackId",1,"targetPositionInertial",target.center(1:2), ...
        "targetVelocityInertial",target.center(3:4),"targetAccelerationInertial",target.center(5:6), ...
        "targetHeadingInertial",target.center(7),"targetYawRate",target.center(8), ...
        "targetPositionInertialErrorBound",bound(1:2),"targetVelocityInertialErrorBound",bound(3:4), ...
        "targetAccelerationInertialErrorBound",bound(5:6),"targetYawErrorBound",bound(7), ...
        "targetYawRateErrorBound",bound(8),"predictionMotion",motion);
    [egoInput,lane,roadInput,measured] = readPlanningInputs(ego,observation,road,cfg);
    model = localPrepared(egoInput,lane,roadInput,measured,cfg);
end

function model = localPrepared(ego,lane,road,observation,cfg)
% The same preparation as collisionAvoidanceController before its solve.
    projection = laneGeometry.project(ego.position,lane,[]);
    heading = atan2(sin(ego.yaw-projection.heading),cos(ego.yaw-projection.heading));
    radius = stateUncertainty.toFrenet(ego.modelState,ego.stateErrorBound,lane);
    model = struct("cfg",cfg,"lane",lane,"road",road,"stateTime",ego.stateTime,"sampleTime",cfg.controller.sampleTime, ...
        "horizonSteps",cfg.controller.horizonSteps,"referenceSpeed",cfg.referenceSpeed, ...
        "initialEgoState",[projection.station;projection.lateralPosition;heading;ego.modelState(4:6)], ...
        "initialFrenetErrorBound",radius,"longitudinalAccelerationBias",ego.longitudinalAccelerationBias, ...
        "previousInput",zeros(2,1),"requiredMargin",0,"confirmation",[]);
    if laneGeometry.isVaryingReference(lane),model.referenceBank = ltvBicycleModel.referenceSchedule(model);end
    identity = struct('configuration',rmfield(cfg,'solver'),'lane',lane,'road',road, ...
        'accelerationBias',ego.longitudinalAccelerationBias);
    model = hardEncounterBarrier.prepare(model,ego,observation,[],identity);
    if isempty(model.cruiseCertificate),model.cruiseCertificate = ltvBicycleModel.sampledCruise(model);end
end
