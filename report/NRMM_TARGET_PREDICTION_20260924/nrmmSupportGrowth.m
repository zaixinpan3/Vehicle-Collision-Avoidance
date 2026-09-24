function table_ = nrmmSupportGrowth(outputFile)
%nrmmSupportGrowth Collision-record support versus prediction time, first frame.
% Offline diagnostic, run from the repository root. For NRMM targets it builds
% the first-frame program under: a Cartesian contract whose jerk bound covers
% the target's turning, hypot(kappaMax^2 V^3, 3 |A| kappaMax V), with the
% ego-only tube; the nrmm-motion-v1 contract with the ego-only tube; and the
% nrmm-motion-v1 contract with the target-reactive tube at strengths 30 and
% 100. It reports the largest support ||G' n||_1 + rho of the collision records
% at 1, 2, 3, 4 and 4.8 s, the exit-record support and the input reserves. The
% directions are the frame's nominal ones; no solve is involved.
    addpath("controller","config","scripts");
    egoBound = [0.076;0.076;0.048;0.089;0.497;0.0015];
    curvatureMaximum = 0.03;
    specs = {"crossing",0.01,0;"crossing",0.01,0.02;"lead",0,0.02};
    scales = [1,3,10];times = [1,2,3,4,4.8];rows = {};
    for index = 1:size(specs,1)
        [scenario,roadCurvature,targetCurvature] = specs{index,:};
        for scale = scales
            for variant = ["cartesian-egoOnly","nrmm-egoOnly","nrmm-reactive30","nrmm-reactive100"]
                model = localCapture(scenario,roadCurvature,targetCurvature,scale,curvatureMaximum, ...
                    variant=="cartesian-egoOnly",egoBound);
                [program,~] = formulateAvoidanceProblem(model);
                if contains(variant,"reactive")
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
                    pick = collision & stage==round(times(t)/model.sampleTime);
                    if any(pick),values(t) = max(support(pick));end
                end
                [~,radius] = targetPrediction.finiteFlow(model.encounter,times);
                reserve = max(program.prediction.feedbackInputSupport,[],2);
                rows(end+1,:) = {scenario,roadCurvature,targetCurvature,scale,variant, ...
                    program.prediction.stageCount*model.sampleTime, ...
                    values(1),values(2),values(3),values(4),values(5), ...
                    max([support(~collision),NaN]),max(radius(1:2,end)),reserve(1),reserve(2)}; %#ok<AGROW>
                fprintf("%-8s k=%-4g kT=%-4g x%-2g %-18s | support 1/2/3/4/4.8 s: %s m | exit %6.2f" ...
                    +" | target box 4.8 s %6.2f | reserve %.3f/%.3f\n", ...
                    scenario,roadCurvature,targetCurvature,scale,variant,sprintf("%6.2f ",values),rows{end,12}, ...
                    rows{end,13},reserve(1),reserve(2));
            end
        end
    end
    table_ = cell2table(rows,"VariableNames",["scenario","roadCurvature","targetCurvature","uncertaintyScale", ...
        "variant","horizonSeconds","support1s","support2s","support3s","support4s","support4p8s","exitSupport", ...
        "targetBoxHalfWidth4p8s","reserveSteer","reserveBrake"]);
    if nargin>0,writetable(table_,outputFile);end
end

function model = localCapture(scenario,roadCurvature,targetCurvature,scale,curvatureMaximum,cartesian,egoBound)
% The harness's first-frame inputs with an NRMM target, prepared as
% collisionAvoidanceController does. "lead" is a 2 m/s target 15 m ahead.
    cfg = collisionAvoidanceControllerConfig(struct("referenceSpeed",8, ...
        "controller",struct("sampleTime",0.05,"horizonSteps",32,"minimumHorizonSteps",1), ...
        "model",struct("lateralDomainRadius",4), ...
        "solver",struct("frameDeadlineSeconds",Inf,"certificateSearchTimeLimit",60)));
    [cruiseState,~] = ltvBicycleModel.cruiseEquilibrium(roadCurvature,cfg);
    center = [15;0;2;0;0;0;0;0];
    road = struct("centerline",[-100,0;2000,0]);
    x = [0;0;0;8;0;0];position = x(1:2);heading = 0;
    if roadCurvature~=0
        curve = struct('origin',[0;0],'heading',0,'curvature',roadCurvature, ...
            'length',min(8*1*0.05+100,1.9*pi/abs(roadCurvature)));
        road = struct('referenceCurve',curve, ...
            'centerline',laneGeometry.referencePose(linspace(0,curve.length,201),0,curve).');
        [point,pathHeading] = laneGeometry.referencePose(15,0,curve);
        normal = [-sin(pathHeading);cos(pathHeading)];
        assert(scenario=="crossing");
        center = [point-7.5*normal;4*normal;zeros(2,1);pathHeading+pi/2;0];
        x = cruiseState;[position,heading] = laneGeometry.fromFrenet(x,road);
    end
    % NRMM-consistent state: a = kappa V^2 N, yaw rate kappa V.
    speed = norm(center(3:4));course = atan2(center(4),center(3));
    center(5:6) = targetCurvature*speed^2*[-sin(course);cos(course)];center(8) = targetCurvature*speed;
    ego = struct("position",position,"yaw",heading,"speed",x(4),"lateralVelocity",x(5),"yawRate",x(6), ...
        "stateTime",0,"controllerStateErrorBound",egoBound, ...
        "perception",struct('time',0,'range',16,'completeWithinRange',true));
    motion = struct("kind","nrmm-motion-v1","jerkBound",[0;0],"yawAccelerationBound",0, ...
        "curvatureMaximum",curvatureMaximum);
    if cartesian
        motion = struct("kind","finite-sensing-motion-v1", ...
            "jerkBound",curvatureMaximum^2*speed^3*[1;1],"yawAccelerationBound",0);
    end
    bound = scale*[.1;.1;.05;.05;.01;.01;.01;.01];
    observation = struct("trackId",1,"targetPositionInertial",center(1:2), ...
        "targetVelocityInertial",center(3:4),"targetAccelerationInertial",center(5:6), ...
        "targetHeadingInertial",center(7),"targetYawRate",center(8), ...
        "targetPositionInertialErrorBound",bound(1:2),"targetVelocityInertialErrorBound",bound(3:4), ...
        "targetAccelerationInertialErrorBound",bound(5:6),"targetYawErrorBound",bound(7), ...
        "targetYawRateErrorBound",bound(8),"predictionMotion",motion);
    [egoInput,lane,roadInput,measured] = readPlanningInputs(ego,observation,road,cfg);
    projection = laneGeometry.project(egoInput.position,lane,[]);
    yawError = atan2(sin(egoInput.yaw-projection.heading),cos(egoInput.yaw-projection.heading));
    radius = stateUncertainty.toFrenet(egoInput.modelState,egoInput.stateErrorBound,lane);
    model = struct("cfg",cfg,"lane",lane,"road",roadInput,"stateTime",egoInput.stateTime, ...
        "sampleTime",cfg.controller.sampleTime, ...
        "horizonSteps",cfg.controller.horizonSteps,"referenceSpeed",cfg.referenceSpeed, ...
        "initialEgoState",[projection.station;projection.lateralPosition;yawError;egoInput.modelState(4:6)], ...
        "initialFrenetErrorBound",radius,"longitudinalAccelerationBias",egoInput.longitudinalAccelerationBias, ...
        "previousInput",zeros(2,1),"requiredMargin",0,"confirmation",[]);
    if laneGeometry.isVaryingReference(lane),model.referenceBank = ltvBicycleModel.referenceSchedule(model);end
    identity = struct('configuration',rmfield(cfg,'solver'),'lane',lane,'road',roadInput, ...
        'accelerationBias',egoInput.longitudinalAccelerationBias);
    model = hardEncounterBarrier.prepare(model,egoInput,measured,[],identity);
    if isempty(model.cruiseCertificate),model.cruiseCertificate = ltvBicycleModel.sampledCruise(model);end
end
