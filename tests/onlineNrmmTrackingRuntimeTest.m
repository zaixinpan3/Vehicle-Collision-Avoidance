classdef onlineNrmmTrackingRuntimeTest < matlab.unittest.TestCase
% onlineNrmmTrackingRuntimeTest Tests for the cascaded measured-input observer.
% The retained cascade is direct kinematic body velocity and the
% covariant third-order NRMM target observer. Inertial position is an output
% reconstruction stage. Consistent truth has nonzero ego jerk and angular
% acceleration; neither input derivative is required by the vector field.

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            repoRoot = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repoRoot, "config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repoRoot, "estimator"), IncludingSubfolders=true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repoRoot, "controller")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(repoRoot, "scripts")));
        end
    end

    methods (TestMethodSetup)
        function resetControllerState(~)
            collisionAvoidanceController("resetNominalTrajectory");
        end
    end

    methods (Test)
        function stepAdvancesOutputTimeByOneSamplePeriod(testCase)
            runtime = localRuntime(1);
            samplePeriod = runtime.samplePeriod;

            [runtime, output] = onlineNrmmTrackingRuntime( ...
                "step", runtime, localCruiseFrame(0.0));

            testCase.verifyEqual(output.time, samplePeriod, ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(output.stateTime, output.time, ...
                AbsTol=0.0);
            testCase.verifyEqual(output.inputSampleTime, 0.0, AbsTol=0.0);
            testCase.verifyEqual(output.lastGnssTime, 0.0, AbsTol=0.0);
            testCase.verifyEqual(output.lastImuTime, 0.0, AbsTol=0.0);
            testCase.verifyEqual( ...
                output.lastGyroscopeTime, 0.0, AbsTol=0.0);
            testCase.verifyEqual(output.lastRadarTime, 0.0, AbsTol=0.0);
            testCase.verifyEqual(runtime.currentTime, samplePeriod, ...
                AbsTol=1.0e-12);
            testCase.verifyEqual( ...
                output.targetEstimates(1).estimateTime, samplePeriod, ...
                AbsTol=1.0e-12);

            [~, secondOutput] = onlineNrmmTrackingRuntime( ...
                "step", runtime, localCruiseFrame(samplePeriod));

            testCase.verifyEqual(secondOutput.time, 2.0*samplePeriod, ...
                AbsTol=1.0e-12);
            testCase.verifyEqual( ...
                secondOutput.inputSampleTime, samplePeriod, AbsTol=0.0);
        end

        function outputActionLeavesRuntimeStateUntouched(testCase)
            runtime = localRuntime(1);
            [runtime, stepOutput] = onlineNrmmTrackingRuntime( ...
                "step", runtime, localCruiseFrame(0.0));
            probeFrame = localCruiseFrame(runtime.samplePeriod);

            firstProbe = onlineNrmmTrackingRuntime( ...
                "output", runtime, probeFrame);
            secondProbe = onlineNrmmTrackingRuntime( ...
                "output", runtime, probeFrame);

            testCase.verifyEqual(firstProbe.time, runtime.currentTime, ...
                AbsTol=0.0);
            testCase.verifyEqual(firstProbe.stateTime, ...
                runtime.currentTime, AbsTol=0.0);
            testCase.verifyEqual(firstProbe.inputSampleTime, ...
                probeFrame.time, AbsTol=0.0);
            testCase.verifyEqual(firstProbe.egoPositionInertial, ...
                stepOutput.egoPositionInertial, AbsTol=0.0);
            testCase.verifyEqual(firstProbe.targetStates, ...
                stepOutput.targetStates, AbsTol=0.0);
            testCase.verifyEqual(secondProbe, firstProbe);

            [~, nextOutput] = onlineNrmmTrackingRuntime( ...
                "step", runtime, probeFrame);

            testCase.verifyEqual(nextOutput.time, ...
                probeFrame.time+runtime.samplePeriod, AbsTol=1.0e-12);
        end

        function framesMustLieOnTheSampleGrid(testCase)
            runtime = localRuntime(1);
            offGridFrame = localCruiseFrame(0.02);

            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "step", runtime, offGridFrame), ...
                "onlineNrmmTrackingRuntime:offSampleGrid");
            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "output", runtime, offGridFrame), ...
                "onlineNrmmTrackingRuntime:offSampleGrid");

            [runtime, ~] = onlineNrmmTrackingRuntime( ...
                "step", runtime, localCruiseFrame(0.0));

            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "step", runtime, localCruiseFrame(0.08)), ...
                "onlineNrmmTrackingRuntime:offSampleGrid");
        end

        function freshGyroKeepsItsSensorBoundAcrossTimestampRoundoff(testCase)
            runtime = localRuntime(1);
            [runtime, ~] = onlineNrmmTrackingRuntime("step", runtime, localCruiseFrame(0));
            frame = localCruiseFrame(runtime.currentTime);
            runtime.currentTime = runtime.currentTime+8*eps(max(1, runtime.currentTime));

            output = onlineNrmmTrackingRuntime("output", runtime, frame);

            testCase.verifyEqual(output.stateTime, frame.time, AbsTol=0);
            testCase.verifyEqual(output.controllerErrorBound.time, frame.time, AbsTol=0);
            testCase.verifyEqual(output.egoYawRateErrorBound, ...
                runtime.observerDesign.sensors.gyroscopeNoiseMaximum, AbsTol=0);
        end

        function anAdvancedStateStillChargesActualGyroAge(testCase)
            runtime = localRuntime(1);
            [~, output] = onlineNrmmTrackingRuntime("step", runtime, localCruiseFrame(0));
            testCase.verifyGreaterThan(output.stateTime-output.lastGyroscopeTime, 0);
            testCase.verifyGreaterThan(output.egoYawRateErrorBound, ...
                runtime.observerDesign.sensors.gyroscopeNoiseMaximum);
        end

        function initializationRequiresEveryInitialStateOption(testCase)
            cfg = nrmmTrackingConfig();
            design = synthesizeNrmmObserverGains(cfg);
            requiredOptionNames = ["egoInitialPosition", ...
                "egoInitialBodyVelocity", ...
                "targetInitialState"];

            for optionName = requiredOptionNames
                options = rmfield(localOptions(1), optionName);
                testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                    "initialize", cfg, options, design), ...
                    "onlineNrmmTrackingRuntime:missingInitialState", ...
                    "Missing " + optionName + " must be rejected.");
            end
        end

        function initializationRejectsWrongSizedOptions(testCase)
            cfg = nrmmTrackingConfig();
            design = synthesizeNrmmObserverGains(cfg);
            invalidCases = { ...
                "egoInitialPosition", [0.0; 0.0; 0.0]; ...
                "egoInitialYaw", [0.0; 0.0]; ...
                "egoInitialBodyVelocity", 10.0; ...
                "targetInitialState", zeros(5, 1)};

            for caseIdx = 1:size(invalidCases, 1)
                options = localOptions(1);
                options.(invalidCases{caseIdx, 1}) = ...
                    invalidCases{caseIdx, 2};
                testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                    "initialize", cfg, options, design), ...
                    "onlineNrmmTrackingRuntime:invalidInitialState", ...
                    "Wrong-sized " + invalidCases{caseIdx, 1} ...
                        + " must be rejected.");
            end

            twoTargetOptions = localOptions(2);
            twoTargetOptions.targetInitialState = ...
                [100.0; 2.0; -12.0; 0.0; 0.0; 0.0];
            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "initialize", cfg, twoTargetOptions, design), ...
                "onlineNrmmTrackingRuntime:invalidInitialState");

            fractionalCountOptions = localOptions(1);
            fractionalCountOptions.targetCount = 1.5;
            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "initialize", cfg, fractionalCountOptions, design), ...
                "onlineNrmmTrackingRuntime:invalidOption");
        end

        function multiTargetInitializationRequiresStableIdentifiers( ...
                testCase)
            cfg = nrmmTrackingConfig();
            design = synthesizeNrmmObserverGains(cfg);
            options = rmfield(localOptions(2), "targetIdentifiers");

            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "initialize", cfg, options, design), ...
                "onlineNrmmTrackingRuntime:missingTargetIdentifiers");
        end

        function initializationRejectsDuplicateTargetIdentifiers(testCase)
            cfg = nrmmTrackingConfig();
            design = synthesizeNrmmObserverGains(cfg);
            options = localOptions(2);
            options.targetIdentifiers = ["same"; "same"];

            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "initialize", cfg, options, design), ...
                "onlineNrmmTrackingRuntime:invalidTargetIdentifiers");
        end

        function continuousDesignIsReusableAtDifferentSamplePeriods(testCase)
            cfg = nrmmTrackingConfig();
            design = synthesizeNrmmObserverGains(rmfield(cfg, "runtime"));
            cfg.runtime.samplePeriod = 2.0*cfg.runtime.samplePeriod;
            runtime = onlineNrmmTrackingRuntime( ...
                "initialize", cfg, localOptions(1), design);
            [runtime, output] = onlineNrmmTrackingRuntime( ...
                "step", runtime, localCruiseFrame(0.0));

            testCase.verifyEqual(runtime.observerDesign, design, AbsTol=1.0e-12);
            testCase.verifyEqual(output.time, cfg.runtime.samplePeriod, AbsTol=1.0e-12);
            testCase.verifyLessThanOrEqual(runtime.integrationStep, ...
                cfg.runtime.integrationStepMaximum);
            testCase.verifyTrue(all(isfinite(runtime.targetState), "all"));
        end

        function integrationStepDoesNotChangeContinuousDesign(testCase)
            cfg = nrmmTrackingConfig();
            design = synthesizeNrmmObserverGains(rmfield(cfg, "runtime"));
            cfg.runtime.integrationStepMaximum = 0.002;
            runtime = onlineNrmmTrackingRuntime( ...
                "initialize", cfg, localOptions(1), design);

            testCase.verifyEqual(runtime.observerDesign, design, AbsTol=1.0e-12);
            testCase.verifyEqual(runtime.integrationStep, 0.002, AbsTol=1.0e-14);
            testCase.verifyEqual(runtime.integrationSubstepCount, 10);
        end

        function runtimeRejectsInvalidSamplePeriod(testCase)
            cfg = nrmmTrackingConfig();
            design = synthesizeNrmmObserverGains(rmfield(cfg, "runtime"));
            cfg.runtime.samplePeriod = 0.0;

            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "initialize", cfg, localOptions(1), design), ...
                "MATLAB:onlineNrmmTrackingRuntime:expectedPositive");
        end

        function runtimeRejectsInvalidIntegrationStep(testCase)
            cfg = nrmmTrackingConfig();
            design = synthesizeNrmmObserverGains(rmfield(cfg, "runtime"));
            cfg.runtime.integrationStepMaximum = Inf;

            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "initialize", cfg, localOptions(1), design), ...
                "MATLAB:onlineNrmmTrackingRuntime:expectedFinite");
        end

        function everyFrameRequiresSynchronizedRadar(testCase)
            runtime = localRuntime(1);
            frame = rmfield(localCruiseFrame(0.0), ...
                "radarRelativePosition");

            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "step", runtime, frame), ...
                "onlineNrmmTrackingRuntime:missingSynchronizedRadar");
        end

        function everyFrameRequiresGnssVelocity(testCase)
            runtime = localRuntime(1);
            frame = rmfield(localCruiseFrame(0.0), "vyGps");

            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "step", runtime, frame), ...
                "onlineNrmmTrackingRuntime:missingSynchronizedSignal");
        end

        function radarRowsMustMatchDetectionAvailability(testCase)
            availableWithNaN = localCruiseFrame(0.0);
            availableWithNaN.radarRelativePosition = [NaN, NaN];
            unavailableWithFinite = localCruiseFrame(0.0);
            unavailableWithFinite.radarDetectionAvailable = false;

            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "step", localRuntime(1), availableWithNaN), ...
                "onlineNrmmTrackingRuntime:invalidSynchronizedRadar");
            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "step", localRuntime(1), unavailableWithFinite), ...
                "onlineNrmmTrackingRuntime:invalidSynchronizedRadar");
        end

        function multiTargetFrameRequiresKnownUniqueIdentifiers(testCase)
            runtime = localRuntime(2);
            radar = [100.0, 2.0; 80.0, -2.0];
            missingIdentifiersFrame = localFrame(0.0, [0.0; 0.0], radar);
            duplicateIdentifiersFrame = localFrame( ...
                0.0, [0.0; 0.0], radar, ["same"; "same"]);
            unknownIdentifierFrame = localFrame(0.0, [0.0; 0.0], radar, ...
                ["test-target-1"; "not-initialized"]);

            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "step", runtime, missingIdentifiersFrame), ...
                "onlineNrmmTrackingRuntime:missingRadarTargetIdentifiers");
            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "step", runtime, duplicateIdentifiersFrame), ...
                "onlineNrmmTrackingRuntime:invalidRadarTargetIdentifiers");
            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "step", runtime, unknownIdentifierFrame), ...
                "onlineNrmmTrackingRuntime:unknownRadarTargetIdentifier");
        end

        function radarIdentifiersMakeRowOrderIrrelevant(testCase)
            radar = [100.0, 2.0; 80.0, -2.0];
            identifiers = ["test-target-1"; "test-target-2"];

            [~, canonicalOutput] = onlineNrmmTrackingRuntime( ...
                "step", localRuntime(2), ...
                localFrame(0.0, [0.0; 0.0], radar, identifiers));
            [~, permutedOutput] = onlineNrmmTrackingRuntime( ...
                "step", localRuntime(2), ...
                localFrame(0.0, [0.0; 0.0], ...
                    radar([2, 1], :), identifiers([2, 1])));

            testCase.verifyEqual(permutedOutput.targetStates, ...
                canonicalOutput.targetStates, AbsTol=1.0e-13);
            testCase.verifyEqual( ...
                string({canonicalOutput.targetEstimates.trackId}).', ...
                identifiers);

            slotOptions = rmfield(localOptions(1), "targetIdentifiers");
            cfg = nrmmTrackingConfig();
            slotRuntime = onlineNrmmTrackingRuntime("initialize", ...
                cfg, slotOptions, synthesizeNrmmObserverGains(cfg));
            [~, slotOutput] = onlineNrmmTrackingRuntime( ...
                "step", slotRuntime, localCruiseFrame(0.0));

            testCase.verifyFalse( ...
                isfield(slotOutput.targetEstimates, "trackId"));
            testCase.verifyEqual( ...
                slotOutput.targetEstimates(1).temporarySlotIdentifier, ...
                "target-slot-1");
        end

        function cascadedObserverConvergesOnAnalyticManeuver(testCase)
            cfg = nrmmTrackingConfig();
            design = synthesizeNrmmObserverGains(cfg);
            frameCount = round(10.0/cfg.runtime.samplePeriod);
            scenario = localAnalyticScenario(cfg, frameCount);
            options = struct( ...
                "initialTime", 0.0, ...
                "targetCount", 1, ...
                "egoInitialPosition", ...
                    scenario.egoPosition(:, 1)+[1.0; -1.0], ...
                "egoInitialYaw", scenario.egoYaw(1)+0.1, ...
                "egoInitialBodyVelocity", ...
                    scenario.egoBodyVelocity(:, 1)+[0.5; -0.2], ...
                "targetInitialState", [ ...
                    scenario.targetRho(:, 1)+[1.0; 0.5]; ...
                    scenario.targetQ(:, 1)+[2.0; -1.0]; ...
                    scenario.targetS(:, 1)]);
            runtime = onlineNrmmTrackingRuntime( ...
                "initialize", cfg, options, design);

            output = struct();
            for frameIdx = 1:frameCount
                [runtime, output] = onlineNrmmTrackingRuntime( ...
                    "step", runtime, scenario.frames(frameIdx));
            end

            finalIdx = frameCount+1;
            testCase.verifyLessThan(norm(output.egoPositionInertial ...
                - scenario.egoPosition(:, finalIdx)), 0.05, ...
                "The ego position estimate must settle below 5 cm.");
            testCase.verifyLessThan(abs(localWrapToPi( ...
                output.egoYaw-scenario.egoYaw(finalIdx))), 0.02, ...
                "The yaw error must stay within the sideslip bound.");
            testCase.verifyLessThan(norm(output.egoBodyVelocity ...
                - scenario.egoBodyVelocity(:, finalIdx)), 0.3, ...
                "The body-velocity error must settle below 0.3 m/s.");
            testCase.verifyEqual(output.egoYawRate, ...
                scenario.frames(frameCount).yawRateMeasured, ...
                "The published yaw rate is the measured gyroscope input.");

            estimate = output.targetEstimates(1);
            testCase.verifyLessThan(norm(estimate.relativePosition ...
                - scenario.targetRho(:, finalIdx)), 1.0e-3, ...
                "The continuous radar predictor must keep rho exact.");
            testCase.verifyLessThan(norm(estimate.targetVelocity ...
                - scenario.targetQ(:, finalIdx)), 0.3, ...
                "The reconstructed target velocity must converge.");
            testCase.verifyLessThan(abs(localWrapToPi( ...
                estimate.targetHeadingInertial ...
                - scenario.targetYaw(finalIdx))), 0.05, ...
                "The reconstructed target heading must approach psiC.");
        end

        function runtimeCarriesNoEgoJerkState(testCase)
            runtime = localRuntime(1);

            [runtime, output] = onlineNrmmTrackingRuntime( ...
                "step", runtime, localCruiseFrame(0.0));

            % The design records an ego jerk *bound* as a disturbance
            % magnitude; no runtime state or output estimates a jerk.
            runtimeFields = string(fieldnames(runtime));
            outputFields = string(fieldnames(output));
            testCase.verifyFalse(any(contains( ...
                lower(runtimeFields), "jerk")));
            testCase.verifyFalse(any(contains( ...
                lower(outputFields), "jerk")));
            testCase.verifyFalse(ismember("egoState", runtimeFields), ...
                "The runtime must not carry a lifted ego state vector.");
            testCase.verifyFalse(ismember("yawState", runtimeFields), ...
                "The runtime must not carry a lifted yaw chain.");
            testCase.verifyFalse(ismember("egoYawState", outputFields));
            testCase.verifySize(output.egoState, [6, 1]);
        end

        function radarDropoutAdvancesPurePrediction(testCase)
            runtime = localRuntime(1);
            [runtime, firstOutput] = onlineNrmmTrackingRuntime( ...
                "step", runtime, localCruiseFrame(0.0));
            priorTargetState = firstOutput.targetStates.';
            dropoutFrame = localFrame( ...
                runtime.samplePeriod, [0.5; 0.0], [NaN, NaN]);
            dropoutFrame.radarDetectionAvailable = false;

            [~, output] = onlineNrmmTrackingRuntime( ...
                "step", runtime, dropoutFrame);

            % The default cruise frames hold every ego observer state at
            % its exact equilibrium, so the dropout interval must equal
            % the pure RK4 flow of the target model with zero innovation.
            frozenEgo = struct( ...
                "bodyVelocity", [10.0; 0.0], "yawRate", 0.0);
            predictedTargetState = localTargetRk4( ...
                priorTargetState, frozenEgo, ...
                runtime.observerDesign.target.domain, ...
                runtime.integrationStep, runtime.integrationSubstepCount);
            testCase.verifyEqual(output.targetStates.', ...
                predictedTargetState, AbsTol=1.0e-9);
            % The floor is a closing rate times the sample period, so it
            % states the intent -- prediction is not frozen -- without
            % depending on the implementation-layer sample period.
            testCase.verifyGreaterThan( ...
                norm(predictedTargetState-priorTargetState), ...
                10.0*runtime.samplePeriod, ...
                "Pure prediction must still advance the target state.");
            testCase.verifyTrue( ...
                all(isfinite(output.targetStates), "all"));
            testCase.verifyFalse(output.radarDetectionAvailable);
            testCase.verifyEqual(output.stateTime, ...
                2.0*runtime.samplePeriod, AbsTol=1.0e-12);
            testCase.verifyEqual(output.lastRadarTime, 0.0, ...
                "The radar timestamp stays at the last detection.");
            testCase.verifyEqual( ...
                output.targetEstimates(1).lastRadarTime, 0.0, AbsTol=0.0);
            testCase.verifyEqual( ...
                output.targetEstimates(1).measurementTime, ...
                output.targetEstimates(1).lastRadarTime, AbsTol=0.0);
        end

        function speedDomainAuditReportsPeakingWithoutRejection(testCase)
            cfg = nrmmTrackingConfig();
            design = synthesizeNrmmObserverGains(cfg);
            options = localOptions(1);
            % Large target-velocity error: |q| starts far below the
            % certified minimum and the high-gain observer peaks.
            options.targetInitialState = ...
                [100.0; 2.0; 1.0; 0.0; 0.0; 0.0];
            runtime = onlineNrmmTrackingRuntime( ...
                "initialize", cfg, options, design);
            samplePeriod = cfg.runtime.samplePeriod;
            % Fixed simulated duration: the sample period is an
            % implementation setting and must not change what is tested.
            frameCount = round(2.5/samplePeriod);

            frameTime = 0.0;
            output = struct();
            for frameIdx = 1:frameCount
                [runtime, output] = onlineNrmmTrackingRuntime( ...
                    "step", runtime, localCruiseFrame(frameTime));
                frameTime = frameTime+samplePeriod;
            end

            speedMinimum = design.target.domain.speedMinimum;
            testCase.verifyFalse(output.targetCertifiedSpeedDomainValid, ...
                "The cumulative audit must remember the domain exit.");
            testCase.verifyLessThan( ...
                output.minimumReconstructedTargetSpeed, speedMinimum);
            testCase.verifyTrue( ...
                output.targetCertifiedSpeedDomainValidThisInterval, ...
                "The estimate must re-enter the certified speed domain.");
            testCase.verifyTrue( ...
                output.targetEstimates(1).certifiedSpeedDomainValid);
            testCase.verifyEqual( ...
                output.targetEstimates(1).targetVelocity, [-12.0; 0.0], ...
                "The target velocity must recover.", AbsTol=0.5);
        end

        function yawEstimateConvergesAcrossPiWrap(testCase)
            cfg = nrmmTrackingConfig();
            design = synthesizeNrmmObserverGains(cfg);
            trueHeading = pi-0.3;
            speed = 12.0;
            options = localHeadingOptions(-pi+0.3, speed);
            runtime = onlineNrmmTrackingRuntime( ...
                "initialize", cfg, options, design);
            initialError = abs(localWrapToPi( ...
                options.egoInitialYaw-trueHeading));
            samplePeriod = cfg.runtime.samplePeriod;
            % Fixed simulated duration; see above.
            frameCount = round(3.0/samplePeriod);

            frameTime = 0.0;
            output = struct();
            for frameIdx = 1:frameCount
                [runtime, output] = onlineNrmmTrackingRuntime("step", ...
                    runtime, localConstantHeadingFrame( ...
                        frameTime, trueHeading, speed));
                frameTime = frameTime+samplePeriod;
                testCase.verifyGreaterThanOrEqual(output.egoYaw, -pi);
                testCase.verifyLessThan(output.egoYaw, pi);
            end

            finalError = abs(localWrapToPi(output.egoYaw-trueHeading));
            testCase.verifyLessThan(finalError, 0.05, ...
                "The wrapped heading error must shrink across the cut.");
            testCase.verifyLessThan(finalError, initialError);
        end

        function yawConvergesTowardGnssVelocityCourse(testCase)
            cfg = nrmmTrackingConfig();
            design = synthesizeNrmmObserverGains(cfg);
            trueHeading = 0.4;
            speed = 15.0;
            options = localHeadingOptions(0.0, speed);
            runtime = onlineNrmmTrackingRuntime( ...
                "initialize", cfg, options, design);
            samplePeriod = cfg.runtime.samplePeriod;
            % Fixed simulated duration; see above.
            frameCount = round(3.0/samplePeriod);

            frameTime = 0.0;
            output = struct();
            for frameIdx = 1:frameCount
                [runtime, output] = onlineNrmmTrackingRuntime("step", ...
                    runtime, localConstantHeadingFrame( ...
                        frameTime, trueHeading, speed));
                frameTime = frameTime+samplePeriod;
            end

            testCase.verifyEqual(output.yawCoursePseudoHeading, ...
                trueHeading, ...
                "Zero yaw rate makes the kinematic correction zero.", ...
                AbsTol=1.0e-12);
            testCase.verifyLessThan(abs(localWrapToPi( ...
                output.egoYaw-trueHeading)), 0.02, ...
                "The yaw estimate must converge onto the GNSS course.");
            testCase.verifyLessThan(output.orientationSet.radius,0.02);
        end

        function inconsistentYawIntersectionPreservesTheBodyCore(testCase)
            cfg = nrmmTrackingConfig();
            runtime = onlineNrmmTrackingRuntime("initialize",cfg,localHeadingOptions(0,10));
            [runtime,~] = onlineNrmmTrackingRuntime("step",runtime,localConstantHeadingFrame(0,0,10));
            frame = localConstantHeadingFrame(runtime.currentTime,0.3,10);
            baselineFrame = localConstantHeadingFrame(runtime.currentTime,0,10);
            [changed,output] = onlineNrmmTrackingRuntime("step",runtime,frame);
            [baseline,~] = onlineNrmmTrackingRuntime("step",runtime,baselineFrame);
            testCase.verifyFalse(output.orientationCertificateAvailable);
            testCase.verifyEmpty(output.orientationSet.intervals);
            testCase.verifyTrue(output.positionErrorBoundAvailable);
            testCase.verifyFalse(output.controllerErrorBound.available);
            testCase.verifyTrue(isfinite(output.egoBodyVelocityErrorBound));
            testCase.verifyTrue(isfinite(output.egoPositionErrorBound));
            testCase.verifyEqual(changed.bodyVelocityEstimate,baseline.bodyVelocityEstimate,AbsTol=1e-12);
            testCase.verifyEqual(changed.targetState,baseline.targetState,AbsTol=1e-12);
        end

        function outputProbesUseTheirOwnCourseMeasurements(testCase)
            runtime = localRuntime(1);
            firstFrame = localConstantHeadingFrame(0.0, 0.25, 10.0);
            secondFrame = localConstantHeadingFrame(0.0, -0.2, 12.0);

            first = onlineNrmmTrackingRuntime("output", runtime, firstFrame);
            second = onlineNrmmTrackingRuntime("output", runtime, secondFrame);
            repeated = onlineNrmmTrackingRuntime("output", runtime, firstFrame);

            testCase.verifyEqual(first.yawCoursePseudoHeading, 0.25, AbsTol=1.0e-12);
            testCase.verifyEqual(second.yawCoursePseudoHeading, -0.2, AbsTol=1.0e-12);
            testCase.verifyEqual(repeated, first);
            testCase.verifyEqual(second.egoYaw,-0.2,AbsTol=1e-12);
            testCase.verifyEqual(second.egoBodyVelocity,runtime.bodyVelocityEstimate,AbsTol=0);
        end

        function uninformativeCourseLeavesTheObserverDefined(testCase)
            runtime = localRuntime(1);
            [runtime,~] = onlineNrmmTrackingRuntime("step",runtime,localCruiseFrame(0));
            frame = localCruiseFrame(runtime.currentTime);
            frame.vxGps = 0;
            frame.vyGps = 0;
            [changed,output] = onlineNrmmTrackingRuntime("step",runtime,frame);
            testCase.verifyTrue(all(isfinite(changed.bodyVelocityEstimate)));
            testCase.verifyTrue(all(isfinite(changed.targetState)));
            testCase.verifyFalse(output.yawCourseChannelValid);
            % Zero speed contradicts this test's positive-speed domain.
            testCase.verifyFalse(output.positionErrorBoundAvailable);
        end

        function offGridFramesAreRejectedBeforeTheirCourseIsUsed(testCase)
            runtime = localRuntime(1);
            frame = localCruiseFrame(runtime.samplePeriod);
            frame.vxGps = 0.0;
            frame.vyGps = 0.0;

            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "step", runtime, frame), ...
                "onlineNrmmTrackingRuntime:offSampleGrid");
            testCase.verifyError(@() onlineNrmmTrackingRuntime( ...
                "output", runtime, frame), ...
                "onlineNrmmTrackingRuntime:offSampleGrid");
        end

        function courseChannelPublishesCertifiedGeometry(testCase)
            runtime = localRuntime(1);

            [~, output] = onlineNrmmTrackingRuntime( ...
                "step", runtime, localCruiseFrame(0.0));

            course = certifiedKinematicCourseCorrespondence( ...
                [10.0; 0.0], 0.0, ...
                runtime.observerDesign.yaw.courseModel.rearAxleDistance, ...
                runtime.observerDesign.sensors.velocityNoiseMaximum, ...
                runtime.observerDesign.sensors.gyroscopeNoiseMaximum, ...
                runtime.observerDesign.yaw.courseModel ...
                    .singleTrackYawRateMismatchMaximum, ...
                runtime.observerDesign.yaw.courseModel ...
                    .sideslipDomainMaximum);
            testCase.verifyTrue(output.yawCourseChannelValid);
            testCase.verifyEqual(output.yawCourseCertifiedRadius, ...
                course.correspondence.radius, RelTol=1.0e-12);
            testCase.verifyEqual(output.yawCourseEstimatedSideslip, ...
                course.estimatedSideslip, AbsTol=0.0);
            testCase.verifyEqual(output.yawCourseSideslipErrorMaximum, ...
                course.sideslipErrorMaximum, RelTol=1.0e-12);
            testCase.verifyEqual( ...
                output.yawCourseSideslipSineErrorMaximum, ...
                output.yawCourseSideslipSpeedErrorContribution ...
                    + output.yawCourseSideslipYawRateAndModelContribution, ...
                RelTol=1.0e-12);
            testCase.verifyFalse(isfield(runtime.observerDesign.yaw, ...
                "courseGain"));
            testCase.verifyFalse(isfield(runtime.observerDesign.yaw, ...
                "accelerationGain"));
        end

        function coursePseudoHeadingRemovesKinematicSideslip(testCase)
            cfg = nrmmTrackingConfig();
            cfg.measurement.gps.velocityNoiseMaximum = 0.0;
            cfg.measurement.gyroscope.noiseMaximum = 0.0;
            cfg.ego.yaw.singleTrackYawRateMismatchMaximum = 0.0;
            design = synthesizeNrmmObserverGains(cfg);
            trueYaw = 0.35;
            trueSideslip = 0.04;
            speed = 12.0;
            rearAxleDistance = cfg.ego.yaw.rearAxleDistance;
            courseAngle = trueYaw+trueSideslip;
            bodyVelocity = speed ...
                * [cos(trueSideslip); sin(trueSideslip)];
            options = struct( ...
                "initialTime", 0.0, ...
                "targetCount", 1, ...
                "egoInitialPosition", [0.0; 0.0], ...
                "egoInitialYaw", trueYaw, ...
                "egoInitialBodyVelocity", bodyVelocity, ...
                "targetInitialState", ...
                    [50.0; 0.0; speed; 0.0; 0.0; 0.0]);
            runtime = onlineNrmmTrackingRuntime( ...
                "initialize", cfg, options, design);
            frame = struct( ...
                "time", 0.0, ...
                "xGps", 0.0, ...
                "yGps", 0.0, ...
                "vxGps", speed*cos(courseAngle), ...
                "vyGps", speed*sin(courseAngle), ...
                "longitudinalAcceleration", 0.0, ...
                "lateralAcceleration", 0.0, ...
                "yawRateMeasured", ...
                    speed*sin(trueSideslip)/rearAxleDistance, ...
                "radarRelativePosition", [50.0, 0.0]);

            output = onlineNrmmTrackingRuntime("output", runtime, frame);

            testCase.verifyTrue(output.yawCourseChannelValid);
            testCase.verifyEqual(output.yawCourseEstimatedSideslip, ...
                trueSideslip, AbsTol=1.0e-14);
            testCase.verifyEqual(output.yawCoursePseudoHeading, ...
                trueYaw, AbsTol=1.0e-14);
            testCase.verifyEqual(output.yawCourseCertifiedRadius, ...
                0.0, AbsTol=1.0e-14);
        end

        function planningInputsConsumeSynchronizedEstimatorOutput(testCase)
            output = localSynchronizedTurningOutput();
            controllerCfg = collisionAvoidanceControllerConfig();
            controllerCfg.referenceSpeed = 10.0;
            [ego, ~, ~, targets] = readPlanningInputs( ...
                output, [], localControllerRoad(output.egoPositionInertial), ...
                controllerCfg);

            testCase.verifyEqual(ego.position, ...
                output.egoPositionInertial, AbsTol=1.0e-12);
            testCase.verifyEqual(ego.inertialVelocity, ...
                output.egoState([2, 5]), AbsTol=1.0e-12);
            testCase.verifyEqual(ego.modelState(6), ...
                output.egoYawRate, AbsTol=0.0);
            testCase.verifyEqual(output.egoYawRate, 0.05, AbsTol=0.0);
            testCase.verifyEqual(targets(1).position, ...
                output.targetEstimates(1).targetPositionInertial, ...
                AbsTol=1.0e-12);
            testCase.verifyEqual(ego.stateErrorBound, ...
                output.controllerErrorBound.bounds, AbsTol=0.0);
            testCase.verifyEqual(targets(1).yaw, ...
                output.targetEstimates(1).targetHeadingInertial, AbsTol=1.0e-12);
        end

        function planningRejectsAnUnavailableOutOfDomainTargetBound(testCase)
            runtime = localRuntime(1);
            output = onlineNrmmTrackingRuntime("output", runtime, localCruiseFrame(0));
            cfg = collisionAvoidanceControllerConfig();
            testCase.verifyFalse(output.targetEstimates.controllerErrorBound.available);
            testCase.verifyError(@() readPlanningInputs(output, [], ...
                localControllerRoad(output.egoPositionInertial), cfg), ...
                "collisionAvoidanceController:unavailableEstimatorBound");
        end

        function multipleTargetsUseOneSynchronizedFrame(testCase)
            runtime = localRuntime(2);
            radar = [100.0, 2.0; 80.0, -2.0];
            identifiers = ["test-target-1"; "test-target-2"];

            [~, output] = onlineNrmmTrackingRuntime("step", runtime, ...
                localFrame(0.0, [0.0; 0.0], radar, identifiers));

            testCase.verifySize(output.targetStates, [2, 6]);
            testCase.verifyGreaterThan(norm( ...
                output.targetStates(1, :)-output.targetStates(2, :)), ...
                1.0);
        end
    end
end

function runtime = localRuntime(targetCount)
    cfg = nrmmTrackingConfig();
    runtime = onlineNrmmTrackingRuntime( ...
        "initialize", cfg, localOptions(targetCount), ...
        synthesizeNrmmObserverGains(cfg));
end

function options = localOptions(targetCount)
    targetState = zeros(6, targetCount);
    for targetIdx = 1:targetCount
        targetState(:, targetIdx) = [100.0-20.0*(targetIdx-1); ...
            2.0-4.0*(targetIdx-1); -12.0; 0.0; 0.0; 0.0];
    end
    options = struct( ...
        "initialTime", 0.0, ...
        "targetCount", targetCount, ...
        "egoInitialPosition", [0.0; 0.0], ...
        "egoInitialYaw", 0.0, ...
        "egoInitialBodyVelocity", [10.0; 0.0], ...
        "targetInitialState", targetState);
    options.targetIdentifiers = ...
        "test-target-" + string((1:targetCount).');
end

function output = localSynchronizedTurningOutput()
% Physically synchronized turning ego and straight target inside the
% declared 50 m range; no uncertainty radius is changed to admit this case.
    cfg = nrmmTrackingConfig();
    speed = 10;
    yawRate = 0.05;
    sideslip = asin(cfg.ego.yaw.rearAxleDistance*yawRate/speed);
    velocity = speed*[cos(sideslip); sin(sideslip)];
    options = localOptions(1);
    options.egoInitialBodyVelocity = velocity;
    options.targetInitialState = [30; 2; 8; 0; 0; 0];
    runtime = onlineNrmmTrackingRuntime("initialize", cfg, options);
    for index = 1:3
        time = runtime.currentTime;
        angle = yawRate*time;
        position = [sin(angle), cos(angle)-1; 1-cos(angle), sin(angle)]*velocity/yawRate;
        rotation = localRotation(angle);
        inertialVelocity = rotation*velocity;
        targetPosition = [30+8*time; 2];
        frame = localFrame(time, position, (rotation.'*(targetPosition-position)).');
        frame.vxGps = inertialVelocity(1);
        frame.vyGps = inertialVelocity(2);
        frame.longitudinalAcceleration = -yawRate*velocity(2);
        frame.lateralAcceleration = yawRate*velocity(1);
        frame.yawRateMeasured = yawRate;
        [runtime, output] = onlineNrmmTrackingRuntime("step", runtime, frame);
    end
end

function options = localHeadingOptions(initialYaw, speed)
% Heading-test options: a co-moving target keeps |q| in the domain.
    options = struct( ...
        "initialTime", 0.0, ...
        "targetCount", 1, ...
        "egoInitialPosition", [0.0; 0.0], ...
        "egoInitialYaw", initialYaw, ...
        "egoInitialBodyVelocity", [speed; 0.0], ...
        "targetInitialState", [50.0; 0.0; speed; 0.0; 0.0; 0.0]);
end

function frame = localFrame( ...
        time, gpsPosition, radarPosition, radarTargetIdentifiers)
    frame = struct( ...
        "time", time, ...
        "xGps", gpsPosition(1), ...
        "yGps", gpsPosition(2), ...
        "vxGps", 10.0, ...
        "vyGps", 0.0, ...
        "longitudinalAcceleration", 0.0, ...
        "lateralAcceleration", 0.0, ...
        "yawRateMeasured", 0.0, ...
        "radarRelativePosition", radarPosition);
    if nargin >= 4
        frame.radarTargetIdentifiers = radarTargetIdentifiers;
    end
end

function frame = localCruiseFrame(time)
% Frames consistent with the localOptions single-target equilibrium:
% the ego cruises at [10; 0] m/s with zero yaw while the oncoming
% target closes at 22 m/s along the same lane.
    frame = localFrame(time, [10.0*time; 0.0], [100.0-22.0*time, 2.0]);
end

function frame = localConstantHeadingFrame(time, heading, speed)
% Constant-course GNSS frames with a co-moving target at [50; 0].
    direction = [cos(heading); sin(heading)];
    frame = struct( ...
        "time", time, ...
        "xGps", speed*time*direction(1), ...
        "yGps", speed*time*direction(2), ...
        "vxGps", speed*direction(1), ...
        "vyGps", speed*direction(2), ...
        "longitudinalAcceleration", 0.0, ...
        "lateralAcceleration", 0.0, ...
        "yawRateMeasured", 0.0, ...
        "radarRelativePosition", [50.0, 0.0]);
end

function scenario = localAnalyticScenario(cfg, frameCount)
% localAnalyticScenario Consistent noise-free ego and Sharma target frames.
%
% The ego maneuver has nonzero jerk and nonzero yaw acceleration; the
% target follows the constant-scalar-acceleration, constant-sideslip
% Sharma kinematic single track (oncoming). Positions are recovered by
% composite Simpson quadrature of the closed-form velocities, so every
% frame is consistent with the continuous truth to near machine
% precision. Frame times accumulate by repeated addition of the sample
% period exactly as the runtime advances its own clock.

    samplePeriod = cfg.runtime.samplePeriod;
    rearAxleDistance = cfg.target.domain.rearAxleDistance;
    egoRearAxleDistance = cfg.ego.yaw.rearAxleDistance;
    targetSideslip = 0.01;
    targetScalarAcceleration = 0.3;
    targetSpeedInitial = 12.0;
    targetYawInitial = pi;
    targetPositionInitial = [150.0; 3.0];
    planarCross = [0.0, -1.0; 1.0, 0.0];
    curvatureRate = sin(targetSideslip)/rearAxleDistance;

    egoYawOf = @(t) 0.1*sin(0.2*t);
    egoYawRateOf = @(t) 0.02*cos(0.2*t);
    egoYawAccelerationOf = @(t) -0.004*sin(0.2*t);
    egoSpeedOf = @(t) 15.0+0.5*sin(0.3*t);
    egoSpeedRateOf = @(t) 0.15*cos(0.3*t);
    egoSideslipOf = @(t) asin(egoRearAxleDistance ...
        * egoYawRateOf(t)/egoSpeedOf(t));
    egoSideslipRateOf = @(t) egoRearAxleDistance ...
        * (egoYawAccelerationOf(t)*egoSpeedOf(t) ...
            - egoYawRateOf(t)*egoSpeedRateOf(t)) ...
        / (egoSpeedOf(t)^2 ...
            * sqrt(1.0-(egoRearAxleDistance ...
                * egoYawRateOf(t)/egoSpeedOf(t))^2));
    egoBodyVelocityOf = @(t) egoSpeedOf(t) ...
        * [cos(egoSideslipOf(t)); sin(egoSideslipOf(t))];
    egoBodyVelocityRateOf = @(t) egoSpeedRateOf(t) ...
            * [cos(egoSideslipOf(t)); sin(egoSideslipOf(t))] ...
        + egoSpeedOf(t)*egoSideslipRateOf(t) ...
            * [-sin(egoSideslipOf(t)); cos(egoSideslipOf(t))];
    egoInertialVelocityOf = ...
        @(t) localRotation(egoYawOf(t))*egoBodyVelocityOf(t);
    targetSpeedOf = @(t) targetSpeedInitial+targetScalarAcceleration*t;
    targetYawOf = @(t) targetYawInitial+curvatureRate ...
        * (targetSpeedInitial*t+0.5*targetScalarAcceleration*t^2);
    targetCourseOf = @(t) targetYawOf(t)+targetSideslip;
    targetVelocityOf = @(t) targetSpeedOf(t) ...
        * [cos(targetCourseOf(t)); sin(targetCourseOf(t))];

    sampleCount = frameCount+1;
    times = zeros(1, sampleCount);
    for sampleIdx = 2:sampleCount
        times(sampleIdx) = times(sampleIdx-1)+samplePeriod;
    end
    egoPosition = zeros(2, sampleCount);
    targetPosition = zeros(2, sampleCount);
    targetPosition(:, 1) = targetPositionInitial;
    for sampleIdx = 1:frameCount
        egoPosition(:, sampleIdx+1) = egoPosition(:, sampleIdx) ...
            + localSimpson(egoInertialVelocityOf, ...
                times(sampleIdx), samplePeriod);
        targetPosition(:, sampleIdx+1) = targetPosition(:, sampleIdx) ...
            + localSimpson(targetVelocityOf, ...
                times(sampleIdx), samplePeriod);
    end

    egoYaw = zeros(1, sampleCount);
    targetYaw = zeros(1, sampleCount);
    egoBodyVelocity = zeros(2, sampleCount);
    targetRho = zeros(2, sampleCount);
    targetQ = zeros(2, sampleCount);
    targetS = zeros(2, sampleCount);
    for sampleIdx = sampleCount:-1:1
        sampleTime = times(sampleIdx);
        yaw = egoYawOf(sampleTime);
        yawRate = egoYawRateOf(sampleTime);
        bodyVelocity = egoBodyVelocityOf(sampleTime);
        bodyAcceleration = egoBodyVelocityRateOf(sampleTime) ...
            + yawRate*planarCross*bodyVelocity;
        rotation = localRotation(yaw);
        courseDirection = [cos(targetCourseOf(sampleTime)); ...
            sin(targetCourseOf(sampleTime))];
        targetYawRate = targetSpeedOf(sampleTime)*curvatureRate;
        targetAcceleration = ...
            targetScalarAcceleration*courseDirection ...
            + targetSpeedOf(sampleTime)*targetYawRate ...
                * planarCross*courseDirection;

        egoYaw(sampleIdx) = yaw;
        targetYaw(sampleIdx) = localWrapToPi(targetYawOf(sampleTime));
        egoBodyVelocity(:, sampleIdx) = bodyVelocity;
        targetRho(:, sampleIdx) = rotation.' ...
            * (targetPosition(:, sampleIdx)-egoPosition(:, sampleIdx));
        targetQ(:, sampleIdx) = ...
            rotation.'*targetVelocityOf(sampleTime);
        targetS(:, sampleIdx) = rotation.'*targetAcceleration;
        if sampleIdx <= frameCount
            inertialVelocity = rotation*bodyVelocity;
            frames(sampleIdx) = struct( ...
                "time", sampleTime, ...
                "xGps", egoPosition(1, sampleIdx), ...
                "yGps", egoPosition(2, sampleIdx), ...
                "vxGps", inertialVelocity(1), ...
                "vyGps", inertialVelocity(2), ...
                "longitudinalAcceleration", bodyAcceleration(1), ...
                "lateralAcceleration", bodyAcceleration(2), ...
                "yawRateMeasured", yawRate, ...
                "radarRelativePosition", ...
                    targetRho(:, sampleIdx).');
        end
    end

    scenario = struct();
    scenario.times = times;
    scenario.frames = frames;
    scenario.egoPosition = egoPosition;
    scenario.egoYaw = egoYaw;
    scenario.egoBodyVelocity = egoBodyVelocity;
    scenario.targetRho = targetRho;
    scenario.targetQ = targetQ;
    scenario.targetS = targetS;
    scenario.targetYaw = targetYaw;
end

function integralValue = localSimpson(integrand, startTime, width)
% Composite Simpson quadrature with four panels over one frame interval.
    nodes = startTime+width*(0:4)/4.0;
    weights = (width/12.0)*[1.0, 4.0, 2.0, 4.0, 1.0];
    integralValue = zeros(2, 1);
    for nodeIdx = 1:numel(nodes)
        integralValue = integralValue ...
            + weights(nodeIdx)*integrand(nodes(nodeIdx));
    end
end

function state = localTargetRk4(state, ego, domain, stepSize, substepCount)
% Pure RK4 flow of the target model, matching the runtime substeps.
    for substepIdx = 1:substepCount
        first = nrmmTargetTrackerDerivative(state, ego, domain);
        second = nrmmTargetTrackerDerivative( ...
            state+0.5*stepSize*first, ego, domain);
        third = nrmmTargetTrackerDerivative( ...
            state+0.5*stepSize*second, ego, domain);
        fourth = nrmmTargetTrackerDerivative( ...
            state+stepSize*third, ego, domain);
        state = state+(stepSize/6.0) ...
            * (first+2.0*second+2.0*third+fourth);
    end
end

function road = localControllerRoad(anchorPosition)
% Perceived road input for the estimator-to-planner parsing contract.

    centerline = [-100.0, 0.0; 200.0, 0.0];
    perception = fitPerceivedRoadBoundaries( ...
        centerline, [anchorPosition(:); 0.0], ...
        PerceptionRange=30.0, ...
        RightOffset=0.001, LeftOffset=8.0, ...
        ShoulderWidth=2.60, ...
        RouteBranchId="through");
    road = perception.roadGeometry;
    road.boundaries = struct([]);
end

function rotation = localRotation(yaw)
    rotation = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
end

function value = localWrapToPi(value)
    value = mod(value+pi, 2.0*pi)-pi;
end
