classdef ltvBicycleModel
    %ltvBicycleModel Held-input bicycle dynamics and finite-horizon prediction.

    methods (Static)
        function derivative = fialaWorldDynamics(state,input,cfg)
        % Authoritative nonlinear Fiala flow in [px,py,psi,vx,vy,r].
        % Curvature zero makes the existing Frenet kernel Cartesian exactly.
        % This method does not add an affine-model or empirical plant residual.
            parameters = modifiedFialaTire.parameters(cfg);
            derivative = localNominalFlow(state,input,0,cfg,parameters,0);
        end

        function [states,stateJacobian,inputJacobian] = nominalRollout(model,inputs)
        % Nonlinear anchor prediction; safety uses complete uncertain held tubes.
            cfg = model.cfg;
            kernelCfg = struct("vehicle",struct("m",cfg.vehicle.m,"Iz",cfg.vehicle.Iz, ...
                "lf",cfg.vehicle.lf,"lr",cfg.vehicle.lr,"gravity",cfg.vehicle.gravity), ...
                "model",struct("scheduleSpeedFloor",cfg.model.scheduleSpeedFloor),"roadLoad",cfg.roadLoad);
            parameters = modifiedFialaTire.parameters(cfg);
            tire = struct("corneringStiffness",parameters.corneringStiffness, ...
                "longitudinalForceScale",parameters.longitudinalForceScale);
            station = model.lane.segmentStation(:);
            curvature = model.lane.segmentCurvature(:);
            if isfield(model.lane,"referenceCurve")
                station = 0;curvature = model.lane.referenceCurve.curvature;
            end
            if exist("bicycleNominalKernelMex","file")==3
                [states,stateJacobian,inputJacobian] = bicycleNominalKernelMex( ...
                    model.initialEgoState,inputs,model.sampleTime,station,curvature,kernelCfg,tire, ...
                    model.longitudinalAccelerationBias,nargout>1);
            else
                [states,stateJacobian,inputJacobian] = ltvBicycleModel.nominalKernel( ...
                    model.initialEgoState,inputs,model.sampleTime,station,curvature,kernelCfg,tire, ...
                    model.longitudinalAccelerationBias,nargout>1);
            end
        end

        function [states,stateJacobian,inputJacobian] = nominalKernel( ...
                initialState,inputs,sampleTime,segmentStation,segmentCurvature,cfg,parameters,bias,linearize)
        % Shared MATLAB/native RK4 implementation; all data remain runtime inputs.
            count = size(inputs,2);states = zeros(6,count+1);
            states(:,1) = initialState;
            stateJacobian = zeros(6,6,count);inputJacobian = zeros(6,2,count);
            subdivisions = max(1,ceil(sampleTime/0.01));
            h = sampleTime/subdivisions;
            coder.varsize('x',[6,17],[false,true]);
            coder.varsize('u',[2,17],[false,true]);
            for stage = 1:count
                x = states(:,stage);u = inputs(:,stage);
                segment = 1;
                for index = 2:numel(segmentStation)
                    if segmentStation(index)>x(1),break;end
                    segment = index;
                end
                curvature = segmentCurvature(segment);
                differenceStep = ones(8,1);
                if linearize
                    point = [states(:,stage);inputs(:,stage)];
                    differenceStep = eps^(1/3)*(1+abs(point));
                    differenceStep(8) = min(differenceStep(8),max(1e-10,(1-abs(u(2)))/4));
                    batch = point+[zeros(8,1),diag(differenceStep),-diag(differenceStep)];
                    x = batch(1:6,:);u = batch(7:8,:);
                end
                for substep = 1:subdivisions
                    k1 = localNominalFlow(x,u,curvature,cfg,parameters,bias);
                    k2 = localNominalFlow(x+h*k1/2,u,curvature,cfg,parameters,bias);
                    k3 = localNominalFlow(x+h*k2/2,u,curvature,cfg,parameters,bias);
                    k4 = localNominalFlow(x+h*k3,u,curvature,cfg,parameters,bias);
                    x = x+h*(k1+2*k2+2*k3+k4)/6;
                end
                states(:,stage+1) = x(:,1);
                if linearize
                    jacobian = (x(:,2:9)-x(:,10:17))./(2*differenceStep.');
                    jacobian(:,1) = [1;zeros(5,1)];
                    stateJacobian(:,:,stage) = jacobian(:,1:6);
                    inputJacobian(:,:,stage) = jacobian(:,7:8);
                end
            end
        end
        function [state, input] = cruiseEquilibrium(curvature, cfg, accelerationBias)
        % Solve the nonlinear bicycle's constant-speed path equilibrium.
        % Steering, lateral velocity, body heading and road-load force are
        % compatible with one model; no desired acceleration is introduced.
            if nargin < 3, accelerationBias = 0; end
            speed = cfg.referenceSpeed;
            if curvature==0
                % Exact straight equilibrium: lateral forces vanish and
                % longitudinal tire force balances the passive road load.
                state = [0;0;0;speed;0;0];
                ratio = (longitudinalRoadLoad(speed,cfg)/cfg.vehicle.m-accelerationBias) ...
                    /modifiedFialaTire.accelerationGain(cfg);
                input = [0;ratio];
                return;
            end
            % Necessary lateral-force feasibility: |vx*r| >= |kappa|*vx^2,
            % while the sum of axle force magnitudes cannot exceed mu*Fz.
            lateralCapacity = modifiedFialaTire.accelerationGain(cfg);
            if abs(curvature)*speed^2>lateralCapacity*(1+64*eps)
                error("collisionAvoidanceController:invalidCruiseOperatingPoint", ...
                    "The requested steady turn exceeds the tire lateral-force capacity.");
            end
            [a,b,c] = ltvBicycleModel.continuousMatrices(curvature,speed,cfg,0,accelerationBias);
            state = [0;0;0;speed;0;curvature*speed];
            rows = 4:6;
            solution = [a(rows,5),b(rows,:)]\(-a(rows,:)*state-c(rows));
            state(5) = solution(1);
            input = solution(2:3);
            if speed==0,return;end
            point = [state(5);input];
            for iteration = 1:20
                [state,input,residual,jacobian] = localCruiseResidual(point,speed,curvature,cfg,accelerationBias);
                if norm(residual,inf)<=1e-10,return;end
                step = -jacobian\residual;
                fraction = 1;
                accepted = false;
                for backtrack = 1:12
                    trial = point+fraction*step;
                    if all(isfinite(trial)) && abs(trial(3))<1-sqrt(eps)
                        [~,~,trialResidual] = localCruiseResidual(trial,speed,curvature,cfg,accelerationBias);
                        accepted = norm(trialResidual,inf)<norm(residual,inf);
                        if accepted,break;end
                    end
                    fraction = fraction/2;
                end
                if ~accepted,break;end
                point = trial;
            end
            error("collisionAvoidanceController:invalidCruiseOperatingPoint", ...
                "No nonlinear cruise trim was found at curvature %.6g and speed %.6g.",curvature,speed);
        end

        function [allA,allB,allC,allTires,allReserve] = linearizationKernel(states,inputs,curvatures,cfg,bias,rate,h)
        % Shared native/MATLAB stage Jacobians and Metzler disturbance flow.
            count = size(inputs,2);
            assert(count>=1);
            allA = zeros(6,6,count);allB = zeros(6,2,count);allC = zeros(6,count);
            allReserve = zeros(6,count);
            first = modifiedFialaTire.affineModel(states(:,1),inputs(:,1),cfg);
            allTires = repmat(first,count,1);
            for stage = 1:count
                [a,b,c,tire] = localOperatingPointMatrices(curvatures(stage),states(:,stage),inputs(:,stage),cfg,bias);
                allA(:,:,stage) = a;allB(:,:,stage) = b;allC(:,stage) = c;allTires(stage) = tire;
                allReserve(:,stage) = stateUncertainty.heldDisturbance(a,rate,h);
            end
        end

        function prediction = finitePredict(model, schedule)
        %finitePredict A finite held-input witness, without an appended rest tail.
            cfg = model.cfg;
            count = model.horizonSteps;
            planCount = 2*count;
            h = model.sampleTime;
            if isempty(schedule)
                speed = model.initialEgoState(4);
                station = model.initialEgoState(1)+(0:count)*h*speed;
                curvature = arrayfun(@(value) laneGeometry.curvature(value, model.lane), station);
                ratio = (longitudinalRoadLoad(speed, cfg)/cfg.vehicle.m ...
                    -model.longitudinalAccelerationBias)/modifiedFialaTire.accelerationGain(cfg);
                schedule = struct("speedProfile", repmat(speed, 1, count+1), ...
                    "station", station, "curvature", curvature, ...
                    "brakingRatio", repmat(min(max(ratio, -1+sqrt(eps)), 1-sqrt(eps)), 1, count));
            end
            reference = zeros(2,count);
            referenceStates = zeros(6,count);
            for stage = 1:count
                if stage>1 && schedule.speedProfile(stage)==schedule.speedProfile(stage-1) ...
                        && schedule.curvature(stage)==schedule.curvature(stage-1)
                    reference(:,stage) = reference(:,stage-1);
                    referenceStates(:,stage) = referenceStates(:,stage-1);
                    continue;
                end
                stageCfg = cfg;
                stageCfg.referenceSpeed = schedule.speedProfile(stage);
                [referenceStates(:,stage),reference(:,stage)] = ltvBicycleModel.cruiseEquilibrium( ...
                    schedule.curvature(stage),stageCfg,model.longitudinalAccelerationBias);
            end
            nonlinearAnchor = [];
            if string(cfg.model.linearizationPolicy)~="cruise"
                anchorInputs = reference;
                if isfield(model,"linearizationInputs") && ~isempty(model.linearizationInputs)
                    retained = min(count,size(model.linearizationInputs,2));
                    anchorInputs(:,1:retained) = model.linearizationInputs(:,1:retained);
                end
                nonlinearAnchor = ...
                    ltvBicycleModel.nominalRollout(model,anchorInputs);
                model.linearizationStates = nonlinearAnchor(:,1:end-1);
                model.linearizationInputs = anchorInputs;
            end
            prediction = struct("stageCount", count, "nodeCount", count+1, ...
                "planCount", planCount, "referencePlan", reference(:), ...
                "scheduleForStore", schedule, "scheduleSpeedProfile", schedule.speedProfile, ...
                "scheduleCurvature", schedule.curvature, "scheduleBrakingRatio", schedule.brakingRatio, ...
                "egoStateMatrix", zeros(6, planCount, count+1), ...
                "egoStateOffset", zeros(6, count+1), "egoStateErrorBound", zeros(6, count+1), ...
                "continuousA", zeros(6, 6, count), "continuousB", zeros(6, 2, count), ...
                "continuousC", zeros(6, count), "cells", []);
            prediction.stageMatrixA = zeros(6, 6, count);
            prediction.stageMatrixB = zeros(6, 2, count);
            prediction.stageAffine = zeros(6, count);
            prediction.executionReserve = zeros(6,count);
            prediction.domainErrorBound = zeros(6,count+1);
            prediction.domainErrorBound(:,1) = model.initialFrenetErrorBound;
            prediction.initialErrorBound = zeros(6,count+1);
            prediction.initialErrorBound(:,1) = model.initialFrenetErrorBound;
            initialGenerators = diag(model.initialFrenetErrorBound);
            % Preserve signed correlations across future linear maps.
            % Reboxing at every step turns a damped coupled model into a
            % growing comparison system and consumes the clearance reserve.
            domainGenerators = diag(model.initialFrenetErrorBound);
            prediction.modelErrorRateBound = zeros(6,count);
            map = zeros(6, planCount);
            offset = model.initialEgoState;
            radius = model.initialFrenetErrorBound;
            numericalRadius = zeros(6,1);
            prediction.egoStateOffset(:, 1) = offset;
            prediction.egoStateErrorBound(:, 1) = radius;
            baseRate = cfg.model.ltvModelErrorRateBound(:)+cfg.model.plantModelResidualRateBound(:);
            nativeStages = ~isempty(nonlinearAnchor) && exist("bicycleLinearizationKernelMex","file")==3;
            if nativeStages
                tireParameters = modifiedFialaTire.parameters(cfg);
                kernelCfg = struct("vehicle",struct("m",cfg.vehicle.m,"Iz",cfg.vehicle.Iz, ...
                    "lf",cfg.vehicle.lf,"lr",cfg.vehicle.lr,"gravity",cfg.vehicle.gravity), ...
                    "model",struct("scheduleSpeedFloor",cfg.model.scheduleSpeedFloor), ...
                    "roadLoad",cfg.roadLoad,"tire",struct("corneringStiffness",tireParameters.corneringStiffness, ...
                    "frictionCoefficient",tireParameters.frictionCoefficient));
                stageStates = model.linearizationStates;
                if string(cfg.model.linearizationPolicy)=="currentState"
                    stageStates = repmat(model.initialEgoState,1,count);
                end
                linearizedCount = count;
                [allA,allB,allC,allTires,allReserve] = bicycleLinearizationKernelMex( ...
                    stageStates(:,1:linearizedCount),model.linearizationInputs(:,1:linearizedCount),schedule.curvature(1:linearizedCount), ...
                    kernelCfg,model.longitudinalAccelerationBias,baseRate,h);
            end
            stateLimit = [model.lane.segmentStation(end)+model.lane.segmentLength(end); ...
                cfg.model.lateralDomainRadius; cfg.model.headingDomainRadius; ...
                cfg.model.speedMaximum; cfg.model.lateralVelocityMaximum; cfg.model.yawRateMaximum];
            inputLimit = repmat([cfg.model.frontWheelSteeringAngleMaximum; ...
                max(abs([cfg.actuation.brakingRatioMinimum, cfg.actuation.brakingRatioMaximum]))], count, 1);
            cells = cell(count, 1);
            tireModels = cell(count,1);
            priorOperatingPoint = [];
            for stage = 1:count
                changed = stage==1 || schedule.curvature(stage)~=schedule.curvature(stage-1) ...
                    || schedule.speedProfile(stage)~=schedule.speedProfile(stage-1) ...
                    || schedule.brakingRatio(stage)~=schedule.brakingRatio(stage-1);
                operatingPoint = [];
                if any(string(cfg.model.linearizationPolicy)==["trajectory","currentState"])
                    stateBar = referenceStates(:,stage);
                    stateBar(1) = schedule.station(stage);
                    inputBar = reference(:,stage);
                    if isfield(model,"linearizationStates") && stage<=size(model.linearizationStates,2)
                        stateBar = model.linearizationStates(:,stage);
                        inputBar = model.linearizationInputs(:,min(stage,size(model.linearizationInputs,2)));
                    end
                    if stage==1 || (string(cfg.model.linearizationPolicy)=="currentState")
                        stateBar = model.initialEgoState;
                        if isempty(nonlinearAnchor), inputBar = model.previousInput;end
                    end
                    operatingPoint = struct("state",stateBar,"input",inputBar);
                    comparison = [stateBar(2:6);inputBar;schedule.curvature(stage)];
                    changed = ~isequal(comparison,priorOperatingPoint);
                    priorOperatingPoint = comparison;
                    prediction.scheduleSpeedProfile(stage) = stateBar(4);
                    prediction.scheduleBrakingRatio(stage) = inputBar(2);
                end
                if changed
                    if nativeStages
                        a = allA(:,:,stage);b = allB(:,:,stage);c = allC(:,stage);tireModel = allTires(stage);
                        processReserve = allReserve(:,stage);
                    else
                        [a,b,c,tireModel] = ltvBicycleModel.continuousMatrices(schedule.curvature(stage), ...
                            schedule.speedProfile(stage),cfg,schedule.brakingRatio(stage),model.longitudinalAccelerationBias,operatingPoint);
                        processReserve = stateUncertainty.heldDisturbance(a,baseRate,h);
                    end
                    rate = baseRate;
                    exact = expm(h*[a,b,c;zeros(3,9)]);
                    executionReserve = abs(exact(1:6,1:6))*model.initialFrenetErrorBound+processReserve;
                end
                tireModels{stage} = tireModel;
                prediction.modelErrorRateBound(:,stage) = rate;
                prediction.executionReserve(:,stage) = executionReserve;
                initialGenerators = exact(1:6,1:6)*initialGenerators;
                prediction.initialErrorBound(:,stage+1) = sum(abs(initialGenerators),2);
                domainGenerators = [exact(1:6,1:6)*domainGenerators,diag(processReserve)];
                prediction.domainErrorBound(:,stage+1) = sum(abs(domainGenerators),2);
                prediction.continuousA(:, :, stage) = a;
                prediction.continuousB(:, :, stage) = b;
                prediction.continuousC(:, stage) = c;
                prediction.stageMatrixA(:, :, stage) = exact(1:6, 1:6);
                prediction.stageMatrixB(:, :, stage) = exact(1:6, 7:8);
                prediction.stageAffine(:, stage) = exact(1:6, 9);
                cellCount = max(cfg.encounter.minimumCells, ceil(2*norm(a, inf)*h));
                dt = h/cellCount;
                heldMap = zeros(6, planCount);
                heldMap(:, 2*stage-1:2*stage) = b;
                if exist("bicycleHeldIntervalKernelMex","file")==3
                    stageTubes = bicycleHeldIntervalKernelMex(a,heldMap,c,map,offset,radius, ...
                        rate,h,cfg.encounter.taylorOrder,stateLimit,inputLimit,numericalRadius,cellCount);
                else
                    stageTubes = stateUncertainty.heldInterval(a,heldMap,c,map,offset,radius, ...
                        rate,h,cfg.encounter.taylorOrder,stateLimit,inputLimit,numericalRadius,cellCount);
                end
                stageCells = cell(cellCount, 1);
                for cellIndex = 1:cellCount
                    tube = stageTubes(cellIndex);
                    tube.localInputMap = tube.localInputMap(:,2*stage-1:2*stage,:);
                    tube.stage = stage;
                    tube.start = (stage-1)*h+(cellIndex-1)*dt;
                    tube.duration = dt;
                    tube.time = tube.start+(0:cfg.encounter.taylorOrder+1)*dt/(cfg.encounter.taylorOrder+1);
                    stageCells{cellIndex} = tube;
                    map = tube.endMap;
                    offset = tube.endOffset;
                    radius = tube.endRadius;
                    numericalRadius = tube.endNumericalRadius;
                end
                cells{stage} = vertcat(stageCells{:});
                prediction.egoStateMatrix(:, :, stage+1) = map;
                prediction.egoStateOffset(:, stage+1) = offset;
                prediction.egoStateErrorBound(:, stage+1) = radius;
                prediction.domainErrorBound(:,stage+1) = radius;
                domainGenerators = diag(radius);
            end
            prediction.cells = vertcat(cells{:});
            prediction.tireModels = tireModels;
        end

        function [stateMatrix, inputMatrix, affineVector, continuousA] = ...
                stageMatrices(kappa, vBar, sampleTime, cfg, betaBar, accelerationBias)
        % ltvBicycleModel.stageMatrices Exact held-input flow of a scheduled affine bicycle.
        %
        % State [s; d; ePsi; vx; vy; r], input [deltaF; beta]. Modified Fiala
        % slip and beta derivatives are evaluated at the scheduled operating
        % point. Static loads and the tire-speed denominator are frozen. The
        % acceleration bias enters independently of the beta input column.
            if nargin < 5, betaBar = []; end
            if nargin < 6, accelerationBias = 0.0; end
            [continuousA, continuousB, continuousC] = ...
                ltvBicycleModel.continuousMatrices(kappa, vBar, cfg, betaBar, accelerationBias);
            heldTransition = expm(sampleTime*[continuousA, continuousB, continuousC; ...
                zeros(3, 9)]);
            stateMatrix = heldTransition(1:6, 1:6);
            inputMatrix = heldTransition(1:6, 7:8);
            affineVector = heldTransition(1:6, 9);
        end

        function [continuousA, continuousB, continuousC, tireModel] = continuousMatrices(kappa, vBar, cfg, betaBar, accelerationBias,operatingPoint)
        %continuousMatrices Continuous generator of the scheduled Frenet bicycle.
        % xDot = continuousA*x + continuousB*u + continuousC, including the
        % independent declared longitudinal acceleration bias in continuousC.

            arguments
                kappa (1,1) double {mustBeFinite}
                vBar (1,1) double {mustBeFinite, mustBeNonnegative}
                cfg (1,1) struct
                betaBar = []
                accelerationBias (1,1) double {mustBeReal, mustBeFinite} = 0.0
                operatingPoint = []
            end

            tireModel = [];
            if ~isempty(operatingPoint)
                [continuousA,continuousB,continuousC,tireModel] = ...
                    localOperatingPointMatrices(kappa,operatingPoint.state,operatingPoint.input,cfg,accelerationBias);
                return;
            end

            mass = cfg.vehicle.m;
            yawInertia = cfg.vehicle.Iz;
            lf = cfg.vehicle.lf;
            lr = cfg.vehicle.lr;
            inputGain = modifiedFialaTire.accelerationGain(cfg);
            if isempty(betaBar)
                betaBar = (longitudinalRoadLoad(vBar, cfg)/mass-accelerationBias)/inputGain;
                betaBar = min(max(betaBar, -1.0+sqrt(eps)), 1.0-sqrt(eps));
            end
            [tireSlope, ratioSlope, tireIntercept] = ...
                modifiedFialaTire.linearize(kappa, vBar, betaBar, cfg);
            corneringFront = -tireSlope(1);
            corneringRear = -tireSlope(2);
            % At zero schedule speed the rest state is invariant. Only the tire
            % denominator is regularized; kinematic transport uses vBar itself.
            tireSpeed = max(vBar, cfg.model.scheduleSpeedFloor);
            rBar = kappa*vBar;

            continuousA = zeros(6, 6);
            continuousB = zeros(6, 2);
            continuousC = zeros(6, 1);
            % Path kinematics linearized at (d = 0, ePsi = 0, vy = 0, vx = vBar):
            % every affine term of these rows vanishes at the schedule point.
            continuousA(1, 4) = 1.0;
            continuousA(1, 2) = kappa*vBar;
            continuousA(2, 3) = vBar;
            continuousA(2, 5) = 1.0;
            continuousA(3, 6) = 1.0;
            continuousA(3, 4) = -kappa;
            continuousA(3, 2) = -kappa^2*vBar;
            % Linearize passive road load at the scheduled speed, retaining
            % both its slope and affine intercept in the held-input flow.
            [roadForce, roadSlope] = longitudinalRoadLoad(vBar, cfg);
            continuousA(4, 4) = -roadSlope/mass;
            continuousA(4, 5) = rBar;
            continuousB(4, 2) = inputGain;
            continuousC(4) = (roadSlope*vBar-roadForce)/mass+accelerationBias;
            % Lateral channel at the frozen speed.
            yawStiffness = (lf*corneringFront-lr*corneringRear)/tireSpeed;
            lateralStiffness = (corneringFront+corneringRear)/tireSpeed;
            continuousA(5, 4) = -rBar;
            continuousA(5, 5) = -lateralStiffness/mass;
            continuousA(5, 6) = -(yawStiffness/mass+vBar);
            continuousB(5, 1) = corneringFront/mass;
            continuousB(5, 2) = sum(ratioSlope)/mass;
            continuousC(5) = rBar*vBar+sum(tireIntercept)/mass;
            continuousA(6, 5) = -yawStiffness/yawInertia;
            continuousA(6, 6) = -(lf^2*corneringFront ...
                + lr^2*corneringRear)/(tireSpeed*yawInertia);
            continuousB(6, 1) = lf*corneringFront/yawInertia;
            continuousB(6, 2) = (lf*ratioSlope(1)-lr*ratioSlope(2))/yawInertia;
            continuousC(6) = (lf*tireIntercept(1)-lr*tireIntercept(2))/yawInertia;
        end

    end
end

function [state,input,residual,jacobian] = localCruiseResidual(point,speed,curvature,cfg,bias)
% Exact Frenet station keeping reduces the trim solve to three force balances.
    lateralSpeed = point(1);
    pathSpeed = hypot(speed,lateralSpeed);
    state = [0;0;-atan2(lateralSpeed,speed);speed;lateralSpeed;curvature*pathSpeed];
    input = point(2:3);
    [a,b,c] = localOperatingPointMatrices(curvature,state,input,cfg,bias);
    flow = a*state+b*input+c;
    residual = flow(4:6);
    stateSlope = [0;0;-speed/pathSpeed^2;0;1;curvature*lateralSpeed/pathSpeed];
    jacobian = [a(4:6,:)*stateSlope,b(4:6,:)];
end

function [a,b,c,tire] = localOperatingPointMatrices(curvature,state,input,cfg,bias)
% Jacobian of nonlinear Frenet kinematics and axle-force balance.
    tire = modifiedFialaTire.affineModel(state,input,cfg);
    input = tire.operatingInput;
    parameters = modifiedFialaTire.parameters(cfg);
    mass = cfg.vehicle.m;
    vx = state(4);vy = state(5);yawRate = state(6);
    heading = state(3);
    denominator = 1-curvature*state(2);
    if denominator<=0
        error("collisionAvoidanceController:invalidReferenceCurve","The operating point is outside the regular Frenet chart.");
    end
    ct = cos(heading);st = sin(heading);
    stationRate = (vx*ct-vy*st)/denominator;
    a = zeros(6);b = zeros(6,2);flow = zeros(6,1);
    a(1,2) = curvature*stationRate/denominator;
    a(1,3:5) = [(-vx*st-vy*ct)/denominator,ct/denominator,-st/denominator];
    a(2,3:5) = [vx*ct-vy*st,st,ct];
    a(3,:) = -curvature*a(1,:);a(3,6) = 1;
    flow(1:3) = [stationRate;vx*st+vy*ct;yawRate-curvature*stationRate];
    delta = input(1);beta = input(2);
    rotation = [cos(delta),-sin(delta);sin(delta),cos(delta)];
    front = rotation*[parameters.longitudinalForceScale(1)*beta;tire.force(1)];
    frontState = rotation*[zeros(1,6);tire.state(1,:)];
    frontInput = rotation*[0,parameters.longitudinalForceScale(1);tire.input(1,:)];
    frontInput(:,1) = frontInput(:,1)+[-front(2);front(1)];
    rear = [parameters.longitudinalForceScale(2)*beta;tire.force(2)];
    rearState = [zeros(1,6);tire.state(2,:)];
    rearInput = [0,parameters.longitudinalForceScale(2);tire.input(2,:)];
    [roadForce,roadSlope] = longitudinalRoadLoad(vx,cfg);
    flow(4:6) = [(front(1)+rear(1)-roadForce)/mass+vy*yawRate+bias; ...
        (front(2)+rear(2))/mass-vx*yawRate; ...
        (cfg.vehicle.lf*front(2)-cfg.vehicle.lr*rear(2))/cfg.vehicle.Iz];
    a(4:5,:) = (frontState+rearState)/mass;
    b(4:5,:) = (frontInput+rearInput)/mass;
    a(4,4) = a(4,4)-roadSlope/mass;
    a(4,5:6) = a(4,5:6)+[yawRate,vy];
    a(5,[4,6]) = a(5,[4,6])-[yawRate,vx];
    a(6,:) = (cfg.vehicle.lf*frontState(2,:)-cfg.vehicle.lr*rearState(2,:))/cfg.vehicle.Iz;
    b(6,:) = (cfg.vehicle.lf*frontInput(2,:)-cfg.vehicle.lr*rearInput(2,:))/cfg.vehicle.Iz;
    c = flow-a*state-b*input;
end

function derivative = localNominalFlow(state,input,curvature,cfg,parameters,bias)
    vx = state(4,:);vy = state(5,:);r = state(6,:);delta = input(1,:);beta = input(2,:);
    speed = max(vx,cfg.model.scheduleSpeedFloor);
    slip = [atan2(vy+cfg.vehicle.lf*r,speed)-delta;atan2(vy-cfg.vehicle.lr*r,speed)];
    capacity = parameters.longitudinalForceScale*sqrt(max(0,1-beta.^2));
    tangent = tan(slip);cornering = repmat(parameters.corneringStiffness,1,size(state,2));
    fy = -capacity.*sign(slip);
    adhesion = capacity>0 & abs(tangent)<3*capacity./cornering;
    ratio = cornering(adhesion).*abs(tangent(adhesion))./(3*capacity(adhesion));
    fy(adhesion) = -cornering(adhesion).*tangent(adhesion).*(1-ratio+ratio.^2/3);
    fx = parameters.longitudinalForceScale*beta;
    frontX = fx(1,:).*cos(delta)-fy(1,:).*sin(delta);
    frontY = fx(1,:).*sin(delta)+fy(1,:).*cos(delta);
    stationRate = (vx.*cos(state(3,:))-vy.*sin(state(3,:)))./(1-curvature*state(2,:));
    derivative = [stationRate;vx.*sin(state(3,:))+vy.*cos(state(3,:));r-curvature*stationRate; ...
        (frontX+fx(2,:)-longitudinalRoadLoad(vx,cfg))/cfg.vehicle.m+vy.*r+bias; ...
        (frontY+fy(2,:))/cfg.vehicle.m-vx.*r; ...
        (cfg.vehicle.lf*frontY-cfg.vehicle.lr*fy(2,:))/cfg.vehicle.Iz];
end
