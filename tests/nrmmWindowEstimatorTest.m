classdef nrmmWindowEstimatorTest < matlab.unittest.TestCase
% nrmmWindowEstimatorTest Geometry, nominal inference, and enclosure behavior.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"estimator")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"scripts")));
        end
    end

    methods (Test)
        function explicitTrackResetClearsInconsistentPriorInformation(testCase)
            runtime = localRuntime();
            frame = localFrame(0);
            frame.radarRelativePosition = [100,100];
            [runtime,~] = onlineNrmmTrackingRuntime("step",runtime,frame);
            reset = onlineNrmmTrackingRuntime("resetTarget",runtime,1,[20;2;12;0;0;0]);
            testCase.verifyTrue(all(reset.targetSets.lower <= reset.targetSets.upper));
            testCase.verifyFalse(any([reset.history.available]));
            testCase.verifyTrue(isnan(reset.lastRadarTime));
        end

        function nonlinearExactFlowMatchesOriginalUnsaturatedDynamics(testCase)
            [exact, numerical] = localFlowComparison();
            testCase.verifyEqual(exact, numerical, AbsTol=1.0e-9);
        end

        function straightFlowHasNoCurvatureSingularity(testCase)
            chart = [20;2;0;12;0.4;0];
            [next, physical] = nrmmExactFlow(chart, 0.5);
            testCase.verifyEqual(next, [26.05;2;0;12.2;0.4;0], AbsTol=1.0e-12);
            testCase.verifyEqual(physical, [26.05;2;12.2;0;0.4;0], AbsTol=1.0e-12);
        end

        function flowRejectsCrossingZeroSpeed(testCase)
            testCase.verifyError(@() nrmmExactFlow([0;0;0;1;-2;0],1), ...
                "nrmmExactFlow:nonpositiveSpeed");
        end

        function bodySetCoversBoundedMeasurementsAcrossTheDomain(testCase)
            [margin, feasibility] = localBodySweep();
            testCase.verifyGreaterThanOrEqual(margin, -1.0e-10);
            testCase.verifyTrue(feasibility);
        end

        function inadmissibleCentralInverseCanHaveAFeasibleRepresentative(testCase)
            cfg = nrmmTrackingConfig();
            cfg.ego.domain.speedMinimum = 0.5;
            cfg.ego.domain.speedMaximum = 2;
            cfg.ego.yaw.sideslipDomainMaximum = 1.4;
            cfg.measurement.gps.velocityNoiseMaximum = 0.1;
            cfg.ego.yaw.singleTrackYawRateMismatchMaximum = 0.3;
            information = nrmmBodyVelocitySet([1;0],1.01/cfg.ego.yaw.rearAxleDistance,cfg);
            testCase.verifyTrue(information.nonempty);
            testCase.verifyFalse(information.nominalInverseAdmissible);
            testCase.verifyGreaterThan(information.representative(1),0);
            testCase.verifyLessThanOrEqual(abs(atan2(information.representative(2), ...
                information.representative(1))),1.4+1.0e-12);
        end

        function bodySetReportsEmptyMeasurementInformation(testCase)
            information = nrmmBodyVelocitySet([0;0],0,nrmmTrackingConfig());
            testCase.verifyFalse(information.nonempty);
            testCase.verifyEmpty(information.yawArcs);
            testCase.verifyTrue(isinf(information.radius));
        end

        function yawIntersectionPreservesBothBranchesAtTheCut(testCase)
            first = nrmmCircularSet("arc",pi-0.02,0.1);
            second = nrmmCircularSet("arc",-pi+0.02,0.1);
            intersection = nrmmCircularSet("intersect",first,second);
            testCase.verifyEqual(intersection,[-pi,-pi+0.08;pi-0.08,pi],AbsTol=1.0e-12);
        end

        function disjointYawInformationStaysEmptyUnderPrediction(testCase)
            intersection = nrmmCircularSet("intersect",[-0.1,0.1],[1,1.1]);
            predicted = nrmmCircularSet("predict",intersection,0.2,0.1);
            testCase.verifyEmpty(predicted);
        end

        function straightConstantAccelerationIsIdentifiedFromPositions(testCase)
            cfg = nrmmTrackingConfig();
            time = [0;0.25;0.5;0.75];
            position = [20+12*time.'+0.2*time.'.^2;2+zeros(1,4)];
            fit = fitNrmmTrajectoryWindow(time,position,0.8, ...
                [30;2;12;0;0;0],nrmmWindowEstimatorDesign(cfg));
            testCase.verifyTrue(fit.available);
            testCase.verifyEqual(fit.parameters,[20;2;0;12;0.4;0],AbsTol=1.0e-6);
            testCase.verifyLessThan(fit.conditionNumber,1.0e5);
        end

        function curvedTrajectoryRecoversPhysicalAcceleration(testCase)
            [fit, expected] = localCurvedFit();
            testCase.verifyTrue(fit.available);
            testCase.verifyEqual(fit.state,expected,AbsTol=1.0e-5);
        end

        function coincidentPositionsDoNotInventCourseOrCurvature(testCase)
            cfg = nrmmTrackingConfig();
            prior = [20;2;12;0;0;0];
            fit = fitNrmmTrajectoryWindow([0;0.2;0.4],repmat([20;2],1,3), ...
                0.42,prior,nrmmWindowEstimatorDesign(cfg));
            testCase.verifyFalse(fit.available);
            testCase.verifyEqual(fit.status,"degeneratePositions");
            testCase.verifyEqual(fit.state,prior,AbsTol=0);
        end

        function relativeTrackingIsIndependentOfAbsoluteYaw(testCase)
            [first, second] = localYawIndependence();
            testCase.verifyEqual(first.targetStates,second.targetStates,AbsTol=1.0e-11);
            testCase.verifyEqual(first.egoBodyVelocity,second.egoBodyVelocity,AbsTol=1.0e-11);
            testCase.verifyGreaterThan(abs(first.egoYaw-second.egoYaw),0.5);
        end

        function dropoutPropagatesTheModelAndEnlargesUncertainty(testCase)
            [before, after, expected] = localDropout();
            testCase.verifyEqual(after.targetStates.',expected,AbsTol=1.0e-6);
            testCase.verifyGreaterThan(after.targetEstimates.errorBound.relativePosition, ...
                before.targetEstimates.errorBound.relativePosition);
            testCase.verifyEqual(after.lastRadarTime,before.lastRadarTime,AbsTol=0);
        end

        function impossibleRadarJumpInvalidatesTheOuterSet(testCase)
            runtime = localRuntime();
            [runtime,~] = onlineNrmmTrackingRuntime("step",runtime,localFrame(0));
            frame = localFrame(runtime.currentTime);
            frame.radarRelativePosition = [45,20];
            [~,output] = onlineNrmmTrackingRuntime("step",runtime,frame);
            testCase.verifyEqual(output.targetEstimates.errorBound.status,"inconsistent");
            testCase.verifyTrue(isinf(output.targetEstimates.errorBound.relativePosition));
        end

        function heldGyroNeedsMoreThanAnInstantaneousNoiseBound(testCase)
            runtime = localRuntime();
            [~,output] = onlineNrmmTrackingRuntime("step",runtime,localFrame(0));
            testCase.verifyGreaterThan(output.egoMotion.yawErrorMaximum, ...
                runtime.samplePeriod*runtime.observerDesign.configuration.measurement.gyroscope.noiseMaximum);
        end

        function boundedIncrementsAreUsedWithoutAbsoluteYaw(testCase)
            runtime = localRuntime();
            frame = localFrame(0);
            frame.egoMotion = struct("duration",0.02,"yawIncrement",0, ...
                "translation",[0.2;0],"yawErrorMaximum",1.0e-5,"translationErrorMaximum",0.001);
            [~,output] = onlineNrmmTrackingRuntime("step",runtime,frame);
            testCase.verifyEqual(output.egoMotion.yawErrorMaximum,1.0e-5,AbsTol=0);
            testCase.verifyEqual(output.egoMotion.source,"supplied-bounded-increments");
        end

        function boundedNoiseTrajectoryRemainsInsideAnalyticEnclosures(testCase)
            cfg = nrmmTrackingConfig();
            cfg.ego.intersample.accelerationMaximum = 3;
            cfg.ego.intersample.yawAccelerationMaximum = 0.05;
            result = runOnlineNrmmComplexManeuverScenario("Plot",false,"Report",false, ...
                "Duration",4,"NoiseModel","boundedUniform","Config",cfg);
            testCase.verifyEqual(result.metrics.radiusCoverageFraction,1,AbsTol=0);
            testCase.verifyLessThan(result.metrics.targetAccelerationRmse,1.0);
            testCase.verifyTrue(result.metrics.targetSpeedDomainValidFinalInterval);
            testCase.verifyFalse(result.estimate.finalOutput.targetEstimates.errorBound.machineVerified);
        end
    end
end

function [exact,numerical] = localFlowComparison()
    design = nrmmWindowEstimatorDesign(nrmmTrackingConfig());
    chart = [20;2;0.3;12;0.4;0.004];
    h = 0.7;
    omega = 0.12;
    body = [10;0.1];
    angle = omega*h;
    translation = h*sin(angle/2)/(angle/2) ...
        *[cos(angle/2),-sin(angle/2);sin(angle/2),cos(angle/2)]*body;
    [~,initial] = nrmmExactFlow(chart,0);
    [~,exact] = nrmmExactFlow(chart,h,angle,translation);
    ego = struct("bodyVelocity",body,"yawRate",omega);
    [~,states] = ode113(@(~,x) nrmmTargetTrackerDerivative(x,ego,design.target.domain), ...
        [0,h],initial,odeset("RelTol",1.0e-12,"AbsTol",1.0e-13));
    numerical = states(end,:).';
end

function [margin,feasible] = localBodySweep()
    cfg = nrmmTrackingConfig();
    margin = Inf;
    feasible = true;
    for speed = linspace(5,20,5)
        for beta = linspace(-0.02,0.02,5)
            body = speed*[cos(beta);sin(beta)];
            for yaw = [-pi+0.001,0,pi-0.001]
                rotation = [cos(yaw),-sin(yaw);sin(yaw),cos(yaw)];
                measured = rotation*body+cfg.measurement.gps.velocityNoiseMaximum*[0.6;0.8];
                gyro = body(2)/cfg.ego.yaw.rearAxleDistance+0.01 ...
                    +cfg.measurement.gyroscope.noiseMaximum;
                info = nrmmBodyVelocitySet(measured,gyro,cfg);
                margin = min([margin;body-info.lower;info.upper-body; ...
                    info.radius-norm(body-info.representative)]);
                feasible = feasible && info.nonempty && any(info.yawArcs(:,1) <= yaw & yaw <= info.yawArcs(:,2));
            end
        end
    end
end

function [fit,expected] = localCurvedFit()
    chart = [20;3;2.9;13;-0.2;-0.005];
    time = (0:0.1:0.8).';
    positions = zeros(2,numel(time));
    for index = 1:numel(time)
        state = nrmmExactFlow(chart,time(index));
        positions(:,index) = state(1:2);
    end
    [~,expected] = nrmmExactFlow(chart,0.82);
    fit = fitNrmmTrajectoryWindow(time,positions,0.82,expected, ...
        nrmmWindowEstimatorDesign(nrmmTrackingConfig()));
end

function runtime = localRuntime()
    cfg = nrmmTrackingConfig();
    options = struct("egoInitialPosition",[0;0],"egoInitialYaw",0, ...
        "egoInitialBodyVelocity",[10;0],"targetInitialState",[20;2;12;0;0;0]);
    runtime = onlineNrmmTrackingRuntime("initialize",cfg,options);
end

function frame = localFrame(time)
    frame = struct("time",time,"xGps",10*time,"yGps",0,"vxGps",10,"vyGps",0, ...
        "longitudinalAcceleration",0,"lateralAcceleration",0,"yawRateMeasured",0, ...
        "radarRelativePosition",[20+2*time,2],"radarDetectionAvailable",true);
end

function [first,second] = localYawIndependence()
    left = localRuntime();
    right = localRuntime();
    for index = 0:50
        frame = localFrame(index*0.02);
        [left,first] = onlineNrmmTrackingRuntime("step",left,frame);
        angle = 0.9;
        frame.vxGps = 10*cos(angle);
        frame.vyGps = 10*sin(angle);
        frame.xGps = 10*frame.time*cos(angle);
        frame.yGps = 10*frame.time*sin(angle);
        [right,second] = onlineNrmmTrackingRuntime("step",right,frame);
    end
end

function [before,after,expected] = localDropout()
    runtime = localRuntime();
    for index = 0:50
        [runtime,before] = onlineNrmmTrackingRuntime("step",runtime,localFrame(index*0.02));
    end
    start = runtime.currentTime;
    for index = 0:49
        frame = localFrame(start+index*0.02);
        frame.radarRelativePosition(:) = NaN;
        frame.radarDetectionAvailable = false;
        [runtime,after] = onlineNrmmTrackingRuntime("step",runtime,frame);
    end
    expected = [20+2*runtime.currentTime;2;12;0;0;0];
end
