classdef nrmmTruthEnclosureAuditTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            for folder = ["scripts","config"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,folder)));
            end
        end
    end
    methods (Test)
        function finiteEstimatesDoNotEstablishTruthContainment(testCase)
            [output,state,cfg] = localFixture();
            output.egoBodyVelocity(2) = 0.2;
            audit = auditNrmmTruthEnclosure(output,state,struct(),cfg);
            testCase.verifyTrue(audit.egoPremisesSatisfied);
            testCase.verifyTrue(audit.egoBoundAvailable);
            testCase.verifyFalse(audit.egoContained);
            testCase.verifyLessThan(audit.egoBoundSlack(5),0);
        end
        function dynamicRearSlipInvalidatesTheKinematicPremise(testCase)
            [output,state,cfg] = localFixture();
            cfg.ego.domain.yawRateMaximum = 1;
            cfg.ego.yaw.singleTrackYawRateMismatchMaximum = 0.1;
            state(5:6) = [-0.6;-0.7];
            output.egoBodyVelocity = state(4:5);
            output.egoYawRate = state(6);
            audit = auditNrmmTruthEnclosure(output,state,struct(),cfg);
            testCase.verifyTrue(audit.egoContained);
            testCase.verifyFalse(audit.egoPremisesSatisfied);
            testCase.verifyLessThan(audit.egoPremiseSlack(end),0);
        end
        function unavailableInfiniteBoundsAreNotCountedAsContained(testCase)
            [output,state,cfg] = localFixture();
            output.controllerErrorBound.available = false;
            output.controllerStateErrorBound(:) = Inf;
            audit = auditNrmmTruthEnclosure(output,state,struct(),cfg);
            testCase.verifyFalse(audit.egoContained);
            testCase.verifyTrue(all(isnan(audit.egoBoundSlack)));
        end
        function yawComparisonWrapsAtTheBranchCut(testCase)
            [output,state,cfg] = localFixture();
            state(3) = pi-0.01;
            output.egoYaw = -pi+0.01;
            audit = auditNrmmTruthEnclosure(output,state,struct(),cfg);
            testCase.verifyEqual(audit.egoError(3),0.02,AbsTol=1e-12);
            testCase.verifyTrue(audit.egoContained);
        end
    end
end

function [output,state,cfg] = localFixture()
    cfg = nrmmTrackingConfig();
    state = [0;0;0;10;0;0];
    output = struct("stateTime",0,"egoPositionInertial",state(1:2), ...
        "egoYaw",state(3),"egoBodyVelocity",state(4:5),"egoYawRate",state(6), ...
        "controllerErrorBound",struct("available",true), ...
        "controllerStateErrorBound",0.05*ones(6,1),"targetEstimates",struct([]));
end
