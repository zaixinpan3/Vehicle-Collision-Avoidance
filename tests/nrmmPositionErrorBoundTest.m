classdef nrmmPositionErrorBoundTest < matlab.unittest.TestCase
    % Deterministic containment, timing, uncertainty and numerical defects.
    properties
        Config
        Design
    end
    methods (TestClassSetup)
        function prepareDesign(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"estimator")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
            testCase.Config = nrmmTrackingConfig();
            testCase.Design = synthesizeNrmmObserverGains(testCase.Config);
        end
    end
    methods (Test)
        function changingForcingAndRadarModeRetainsTheExactComparisonFlow(testCase)
            for detected = [true,false,true]
                [runtime,frame] = localRuntime(localSmoothConfig(testCase.Config),testCase.Design,1,struct());
                frame.radarDetectionAvailable = detected;
                if ~detected,frame.radarRelativePosition(:) = NaN;end
                frame.yawRateMeasured = 0.03*double(detected);
                changed = onlineNrmmTrackingRuntime("step",runtime,frame);
                record = changed.positionErrorBound.lastComparison;
                count = numel(record.initial);
                transition = expm(record.step*[record.matrix,record.input;zeros(1,count+1)]);
                expected = max(0,transition(1:count,:)*[record.initial;1]);
                testCase.verifyEqual(record.final,expected,AbsTol=1e-9,RelTol=1e-12);
            end
        end

        function noisyTrajectoryIsContainedWithOnlyExistingDomainBounds(testCase)
            result = localScenario(testCase.Config,testCase.Design,"retained",zeros(0,2));
            testCase.verifyTrue(all(result.estimate.positionErrorBoundAvailable));
            testCase.verifyGreaterThanOrEqual(result.metrics.minimumPositionBoundSlack,-1e-10);
            testCase.verifyLessThan(result.metrics.meanPositionErrorBound,3);
        end

        function declaredIntersampleEnvelopesReduceThePositionRadius(testCase)
            coarse = localScenario(testCase.Config,testCase.Design,"retained",zeros(0,2));
            cfg = localSmoothConfig(testCase.Config);
            smooth = localScenario(cfg,testCase.Design,"retained",zeros(0,2));
            testCase.verifyGreaterThanOrEqual(smooth.metrics.minimumPositionBoundSlack,-1e-10);
            testCase.verifyLessThan(smooth.metrics.meanPositionErrorBound,coarse.metrics.meanPositionErrorBound);
            testCase.verifyEqual(smooth.estimate.targetState,coarse.estimate.targetState,AbsTol=0);
        end

        function dropoutPredictionRemainsFiniteAndContainsTruth(testCase)
            result = localScenario(localSmoothConfig(testCase.Config),testCase.Design,"retained",[2,3]);
            selected = result.truth.time >= 2 & result.truth.time <= 3;
            testCase.verifyTrue(all(result.estimate.positionErrorBoundAvailable));
            testCase.verifyGreaterThanOrEqual(result.metrics.minimumPositionBoundSlack,-1e-10);
            testCase.verifyGreaterThan(max(result.estimate.relativePositionErrorBound(selected)), ...
                result.estimate.relativePositionErrorBound(end));
        end

        function currentBoundUsesThePredictedStateTimestamp(testCase)
            [runtime,frame] = localRuntime(testCase.Config,testCase.Design,1,struct());
            [changed,output] = onlineNrmmTrackingRuntime("step",runtime,frame);
            testCase.verifyEqual(output.targetEstimates.positionErrorBound.time, ...
                frame.time+testCase.Config.runtime.samplePeriod,AbsTol=1e-14);
            testCase.verifyEqual(output.lastRadarTime,frame.time,AbsTol=0);
            testCase.verifyEqual(changed.positionErrorBound.time,output.stateTime,AbsTol=0);
            testCase.verifyTrue(output.targetEstimates.positionErrorBound.integrationErrorIncluded);
            testCase.verifyFalse(output.targetEstimates.positionErrorBound.futurePredictionIncluded);
            testCase.verifyFalse(output.targetEstimates.positionErrorBound.floatingPointVerified);
        end

        function aHeldGyroWithoutRateInformationUsesTheWholeDomain(testCase)
            [runtime,frame] = localRuntime(testCase.Config,testCase.Design,1,struct());
            frame.yawRateMeasured = 0.12;
            [changed,~] = onlineNrmmTrackingRuntime("step",runtime,frame);
            testCase.verifyEqual(changed.positionErrorBound.lastGyroscopeHoldError, ...
                testCase.Config.ego.domain.yawRateMaximum+abs(frame.yawRateMeasured),AbsTol=1e-14);
            testCase.verifyFalse(changed.positionErrorBound.usesAccelerationEnvelope);
        end

        function finiteInputRatesGiveAnExplicitHoldErrorEnvelope(testCase)
            cfg = localSmoothConfig(testCase.Config);
            [runtime,frame] = localRuntime(cfg,testCase.Design,1,struct());
            [changed,~] = onlineNrmmTrackingRuntime("step",runtime,frame);
            testCase.verifyEqual(changed.positionErrorBound.lastGyroscopeHoldError, ...
                cfg.measurement.gyroscope.noiseMaximum ...
                +cfg.ego.domain.yawAccelerationMaximum*cfg.runtime.samplePeriod,AbsTol=1e-14);
            testCase.verifyTrue(changed.positionErrorBound.usesAccelerationEnvelope);
        end

        function acceptedStepDefectsCoverTheEntireInterpolatingPath(testCase)
            excess = localDefectExcess(testCase.Config,testCase.Design,false);
            testCase.verifyLessThanOrEqual(max(excess),1e-8);
        end

        function yawBranchCrossingDoesNotEnterTheNumericalCore(testCase)
            [excess,orientationValid] = localDefectExcess(testCase.Config,testCase.Design,true);
            testCase.verifyLessThanOrEqual(max(excess),1e-8);
            testCase.verifyTrue(orientationValid);
        end

        function largeInitialErrorsRemainContainedThroughHighGainPeaking(testCase)
            [slack,available,orientationContained] = localLargeInitialErrors(testCase.Config,testCase.Design);
            testCase.verifyGreaterThanOrEqual(min(slack),-1e-10);
            testCase.verifyTrue(all(available));
            testCase.verifyTrue(orientationContained);
        end

        function oneStepPerSensorIntervalRetainsContainmentThroughPeaking(testCase)
            cfg = localSmoothConfig(testCase.Config);
            cfg.runtime.samplePeriod = 0.0125;
            cfg.runtime.integrationStepMaximum = cfg.runtime.samplePeriod;
            design = synthesizeNrmmObserverGains(cfg);
            [slack,available,orientationContained] = localLargeInitialErrors(cfg,design);
            testCase.verifyGreaterThanOrEqual(min(slack),-1e-10);
            testCase.verifyTrue(all(available));
            testCase.verifyTrue(orientationContained);
            excess = localDefectExcess(cfg,design,false,cfg.runtime.samplePeriod);
            testCase.verifyLessThanOrEqual(max(excess,[],"all"),1e-8);
        end

        function inconsistentRadarDisablesOnlyItsTrackBound(testCase)
            prior = struct("yaw",0.1,"bodyVelocity",1,"targetComponents",zeros(3,2));
            [runtime,frame] = localRuntime(testCase.Config,testCase.Design,2,prior);
            frame.radarRelativePosition(1,1) = 40;
            [~,output] = onlineNrmmTrackingRuntime("step",runtime,frame);
            testCase.verifyFalse(output.positionErrorBoundAvailable(1));
            testCase.verifyTrue(isinf(output.relativePositionErrorBound(1)));
            testCase.verifyTrue(output.positionErrorBoundAvailable(2));
        end

        function anInvalidEgoBoundCannotBeRepairedByResettingOneTarget(testCase)
            [runtime,frame] = localRuntime(testCase.Config,testCase.Design,1,struct());
            frame.vxGps = 40;
            [changed,~] = onlineNrmmTrackingRuntime("step",runtime,frame);
            changed = onlineNrmmTrackingRuntime("resetTarget",changed,1,changed.targetState);
            testCase.verifyFalse(changed.positionErrorBound.egoValid);
            testCase.verifyFalse(changed.positionErrorBound.valid);
        end

        function reacquisitionRestartsOnlyTheSelectedTargetEnclosure(testCase)
            [runtime,frame] = localRuntime(testCase.Config,testCase.Design,2,struct());
            [changed,~] = onlineNrmmTrackingRuntime("step",runtime,frame);
            preserved = changed.positionErrorBound;
            reset = onlineNrmmTrackingRuntime("resetTarget",changed,1,[40;3;12;2;0;0]);
            testCase.verifyEqual(reset.positionErrorBound.targetComponents(:,2), ...
                preserved.targetComponents(:,2),AbsTol=0);
            testCase.verifyEqual(reset.positionErrorBound.yaw,preserved.yaw,AbsTol=0);
            testCase.verifyTrue(isnan(reset.positionErrorBound.lastRadarTime(1)));
            testCase.verifyTrue(reset.positionErrorBound.valid(1));
        end

        function measurementUpdatesRejectStaleTimestamps(testCase)
            [runtime,~] = localRuntime(testCase.Config,testCase.Design,1,struct());
            input = localInput(1);
            state = localState(0);
            testCase.verifyError(@() nrmmPositionErrorBound("measure", ...
                runtime.positionErrorBound,input,state),"nrmmPositionErrorBound:measurementTimeMismatch");
        end

        function exactKnownStraightMotionKeepsANumericallySmallEnclosure(testCase)
            output = localExactCase(testCase.Config);
            testCase.verifyTrue(output.positionErrorBoundAvailable);
            testCase.verifyLessThan(output.relativePositionErrorBound,1e-6);
        end

        function adapterRespectsVectorNoiseRadiiAndPublishesTheOnlineBound(testCase)
            [noiseRatios,slack,available,stateSlack,publishedBounds] = localAdapterChecks();
            testCase.verifyLessThanOrEqual(max(noiseRatios,[],"all"),1+1e-10);
            testCase.verifyGreaterThanOrEqual(min(slack),-1e-10);
            testCase.verifyTrue(all(available));
            testCase.verifyGreaterThanOrEqual(min(stateSlack, [], "all"), -1e-10);
            testCase.verifyGreaterThan(max(publishedBounds(:, 1))-min(publishedBounds(:, 1)), 1e-4);
            testCase.verifyGreaterThan(max(publishedBounds(:, 7))-min(publishedBounds(:, 7)), 1e-4);
        end

        function aFreshMeasurementTightensThePublishedEgoBound(testCase)
            [runtime, frame] = localRuntime(testCase.Config, testCase.Design, 1, struct());
            [changed, predicted] = onlineNrmmTrackingRuntime("step", runtime, frame);
            frame.time = changed.currentTime;
            frame.xGps = 15*frame.time;
            fresh = onlineNrmmTrackingRuntime("output", changed, frame);
            testCase.verifyLessThan(fresh.egoPositionErrorBound, predicted.egoPositionErrorBound);
            testCase.verifyLessThan(fresh.egoYawRateErrorBound, predicted.egoYawRateErrorBound);
            testCase.verifyEqual(fresh.controllerErrorBound.time, fresh.stateTime, AbsTol=0.0);
            testCase.verifyEqual(fresh.controllerErrorBound.bounds, fresh.controllerStateErrorBound, AbsTol=0.0);
        end

        function invalidEgoCertificatesAreUnavailableForControl(testCase)
            [runtime, frame] = localRuntime(testCase.Config, testCase.Design, 1, struct());
            frame.vxGps = 40;
            output = onlineNrmmTrackingRuntime("output", runtime, frame);
            testCase.verifyFalse(output.controllerErrorBound.available);
            testCase.verifyTrue(all(isinf(output.controllerStateErrorBound)));
            testCase.verifyFalse(output.targetEstimates.controllerErrorBound.available);
        end

        function controllerBoundsRetainTheObserverStateTimestamp(testCase)
            [runtime, frame] = localRuntime(testCase.Config, testCase.Design, 1, struct());
            [~, output] = onlineNrmmTrackingRuntime("step", runtime, frame);
            testCase.verifyEqual(output.controllerErrorBound.time, output.stateTime, AbsTol=0.0);
            testCase.verifyEqual(output.targetEstimates.controllerErrorBound.time, output.stateTime, AbsTol=0.0);
            testCase.verifyFalse(output.controllerErrorBound.futurePredictionIncluded);
            testCase.verifyFalse(output.targetEstimates.controllerErrorBound.futurePredictionIncluded);
        end
    end
end

function [slack,available,orientationContained] = localLargeInitialErrors(cfg,design)
    options = struct("egoInitialPosition",zeros(2,1),"egoInitialYaw",3.1, ...
        "egoInitialBodyVelocity",[-15;0],"targetCount",1, ...
        "targetInitialState",[-30;-2;-12;4;10;-8]);
    runtime = onlineNrmmTrackingRuntime("initialize",cfg,options,design);
    [~,frame] = localRuntime(cfg,design,1,struct());
    frame = rmfield(frame,"radarTargetIdentifiers");
    slack = zeros(30,1);
    available = false(30,1);
    orientationContained = true;
    for index = 1:30
        frame.time = (index-1)*cfg.runtime.samplePeriod;
        frame.xGps = 15*frame.time;
        [runtime,output] = onlineNrmmTrackingRuntime("step",runtime,frame);
        slack(index) = output.relativePositionErrorBound ...
            -norm([30;2]-output.targetEstimates.relativePosition);
        available(index) = output.positionErrorBoundAvailable;
        orientationContained = orientationContained && output.orientationCertificateAvailable ...
            && abs(atan2(sin(output.egoYaw),cos(output.egoYaw))) <= output.egoYawErrorBound+1e-12;
    end
end

function output = localExactCase(cfg)
    cfg.ego.yaw.singleTrackYawRateMismatchMaximum = 0;
    cfg.measurement.gps.positionNoiseMaximum = 0;
    cfg.measurement.gps.velocityNoiseMaximum = 0;
    cfg.measurement.imu.noiseMaximum = 0;
    cfg.measurement.gyroscope.noiseMaximum = 0;
    cfg.measurement.radar.noiseMaximum = 0;
    cfg.ego.domain.accelerationNormMaximum = 0;
    cfg.ego.domain.bodyAccelerationRateMaximum = 0;
    cfg.ego.domain.yawAccelerationMaximum = 0;
    design = synthesizeNrmmObserverGains(cfg);
    prior = struct("yaw",0,"bodyVelocity",0,"targetComponents",zeros(3,1));
    [runtime,frame] = localRuntime(cfg,design,1,prior);
    [~,output] = onlineNrmmTrackingRuntime("step",runtime,frame);
end

function [ratios,slack,available,stateSlack,publishedBounds] = localAdapterChecks()
    cfg = estimatorControllerIntegrationConfig();
    cfg.randomSeed = 83;
    ego = struct("position",[0;0],"yawAngle",0,"longitudinalVelocity",15, ...
        "lateralVelocity",0,"yawRate",0);
    targetFunction = @(t,~) struct("targetPositionInertial",[25+10*t;2],"yawAngle",0, ...
        "speed",10,"yawRate",0);
    [context,~] = nrmmEstimatorControllerAdapter("initialize",cfg,ego,targetFunction);
    ratios = zeros(20,4);
    slack = zeros(20,1);
    available = false(20,1);
    stateSlack = zeros(20,14);
    publishedBounds = zeros(20,14);
    for index = 1:20
        time = (index-1)*0.1;
        ego.position = [15*time;0];
        truth = targetFunction(time,ego);
        [context,estimate,target,frame] = nrmmEstimatorControllerAdapter( ...
            "sample",context,time,ego,truth);
        relative = truth.targetPositionInertial-ego.position;
        ratios(index,:) = [norm([frame.xGps;frame.yGps]-ego.position)/cfg.sensor.gnss.positionNoiseMaximum, ...
            norm([frame.vxGps;frame.vyGps]-[15;0])/cfg.sensor.gnss.velocityNoiseMaximum, ...
            norm([frame.longitudinalAcceleration;frame.lateralAcceleration])/cfg.sensor.imu.accelerationNoiseMaximum, ...
            norm(frame.radarRelativePosition.'-relative)/cfg.sensor.radar.positionNoiseMaximum];
        slack(index) = target.relativePositionErrorBound-norm(relative-target.relativePosition);
        available(index) = target.positionErrorBound.available ...
            && target.positionErrorBound.time == estimate.stateTime ...
            && estimate.controllerErrorBound.available && target.controllerErrorBound.available;
        egoError = [abs(ego.position-estimate.egoPositionInertial); ...
            abs(atan2(sin(ego.yawAngle-estimate.egoYaw), cos(ego.yawAngle-estimate.egoYaw))); ...
            abs([15;0]-estimate.egoBodyVelocity); abs(estimate.egoYawRate)];
        targetError = [abs(truth.targetPositionInertial-target.targetPositionInertial); ...
            abs([10;0]-target.targetVelocityInertial); abs(target.targetAccelerationInertial); ...
            abs(atan2(sin(target.targetHeadingInertial), cos(target.targetHeadingInertial))); ...
            abs(target.targetYawRate)];
        publishedBounds(index,:) = [estimate.controllerErrorBound.bounds; target.controllerErrorBound.bounds].';
        stateSlack(index,:) = publishedBounds(index,:)-[egoError;targetError].';
    end
end

function cfg = localSmoothConfig(cfg)
    cfg.ego.domain.accelerationNormMaximum = 5;
    cfg.ego.domain.bodyAccelerationRateMaximum = 5;
    cfg.ego.domain.yawAccelerationMaximum = 0.1;
end

function result = localScenario(cfg,design,motion,dropouts)
    result = runOnlineNrmmComplexManeuverScenario("Plot",false,"Report",false, ...
        "Duration",5,"Seed",73,"NoiseModel","boundedUniform","Config",cfg, ...
        "TargetMotion",motion,"DropoutIntervals",dropouts,"DesignFunction",@(~) design);
end

function [runtime,frame] = localRuntime(cfg,design,count,prior)
    options = struct("egoInitialPosition",zeros(2,1),"egoInitialYaw",0, ...
        "egoInitialBodyVelocity",[15;0],"targetCount",count, ...
        "targetInitialState",repmat([30;2;15;0;0;0],1,count), ...
        "targetIdentifiers","track-"+string((1:count).'),"initialErrorBounds",prior);
    runtime = onlineNrmmTrackingRuntime("initialize",cfg,options,design);
    frame = struct("time",0,"xGps",0,"yGps",0,"vxGps",15,"vyGps",0, ...
        "longitudinalAcceleration",0,"lateralAcceleration",0,"yawRateMeasured",0, ...
        "radarRelativePosition",repmat([30,2],count,1), ...
        "radarDetectionAvailable",true(count,1),"radarTargetIdentifiers",options.targetIdentifiers);
end

function state = localState(yaw)
    state = struct("yaw",yaw,"bodyVelocity",[15;0], ...
        "targetState",[30;2;8;0.3;4;-2],"radarPredictor",[30.1;2.1]);
end

function input = localInput(time)
    input = struct("time",time,"gnssVelocity",[15;0], ...
        "bodyAcceleration",[0;0],"yawRate",0.12, ...
        "radarDetectionAvailable",true,"radarRelativePosition",[30.1,2.1]);
end

function derivative = localField(state,input,design)
    course = certifiedKinematicCourseCorrespondence(input.gnssVelocity,input.yawRate, ...
        design.yaw.courseModel.rearAxleDistance,design.sensors.velocityNoiseMaximum, ...
        design.sensors.gyroscopeNoiseMaximum, ...
        design.yaw.courseModel.singleTrackYawRateMismatchMaximum, ...
        design.yaw.courseModel.sideslipDomainMaximum);
    estimate = struct("yaw",state.yaw,"bodyVelocity",state.bodyVelocity, ...
        "position",zeros(2,1),"targetState",state.targetState);
    measurement = struct("yawRate",input.yawRate,"yawHeading",course.correspondence.heading, ...
        "gnssVelocity",input.gnssVelocity,"bodyAcceleration",input.bodyAcceleration, ...
        "positionReference",zeros(2,1),"radarReference",state.radarPredictor,"radarAvailable",true);
    derivative = nrmmObserverVectorField(estimate,measurement,design);
    derivative.radarPredictor = state.targetState(3:4)-state.bodyVelocity ...
        -input.yawRate*[0,-1;1,0]*state.radarPredictor;
end

function [excess,orientationValid] = localDefectExcess(cfg,design,crossing,step)
    if nargin < 4,step = 0.02;end
    before = localState(0.1);
    input = localInput(0);
    if crossing
        before.yaw = pi-0.02;
        input.yawRate = 0;
    end
    bound = nrmmPositionErrorBound("initialize",design,cfg,before,0,struct());
    bound = nrmmPositionErrorBound("measure",bound,input,before);
    after = before;
    after.yaw = before.yaw+0.04;
    after.bodyVelocity = before.bodyVelocity+[0.1;0.2];
    after.targetState = before.targetState+[0.05;-0.02;4;0.2;-0.4;0.7];
    after.radarPredictor = before.radarPredictor+[0.03;-0.01];
    first = localField(before,input,design);
    result = nrmmPositionErrorBound("advance",bound,input,before,after,first,step);
    defect = result.lastDefect;
    excess = zeros(5,51);
    names = ["bodyVelocity","targetState","radarPredictor"];
    for index = 1:51
        fraction = (index-1)/50;
        state = before;
        slope = before;
        for name = names
            state.(name) = before.(name)+fraction*(after.(name)-before.(name));
            slope.(name) = (after.(name)-before.(name))/step;
        end
        field = localField(state,input,design);
        residual = slope.targetState-field.targetState;
        excess(:,index) = [norm(slope.bodyVelocity-field.bodyVelocity)-defect.bodyVelocity; ...
            norm(residual(1:2))-defect.target(1);norm(residual(3:4))-defect.target(2); ...
            norm(residual(5:6))-defect.target(3); ...
            norm(slope.radarPredictor-field.radarPredictor)-defect.radarPredictor];
    end
    orientationValid = result.orientationSet.valid;
end
