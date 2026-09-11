classdef fialaFeedbackSampleTest < matlab.unittest.TestCase
    properties (TestParameter)
        steering = {0,.04,-.04};
    end
    methods (TestClassSetup)
        function buildVerifiers(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
            output=testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            buildFialaIntervalVerifier(string(output.Folder));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(output.Folder));
        end
    end
    methods (Test)
        function completeSampleContainsHeldNonlinearTrajectories(testCase,steering)
            [certificate,violations]=localReplay(steering);
            testCase.verifyTrue(certificate.accepted);
            testCase.verifyEqual(certificate.verifiedThrough,.1);
            testCase.verifyGreaterThan(numel(certificate.cells),1);
            testCase.verifyLessThanOrEqual(max(violations),1e-9);
            testCase.verifyFalse(certificate.trajectoryCertified);
        end
        function subdivisionPreservesTheOriginalUncertainCommand(testCase)
            certificate=localCertificate();
            commands=cat(3,certificate.cells.endpoint);
            testCase.verifyEqual(commands(7:8,:,:),repmat(certificate.initialBox(7:8,:),1,1,numel(certificate.cells)));
            testCase.verifyTrue(localHeldGeneratorsPreserved(certificate));
        end
        function generatedCellsCoverTheWholePeriodWithoutGaps(testCase)
            certificate=localCertificate();
            testCase.verifyEqual(certificate.cells(1).start,0);
            testCase.verifyEqual([certificate.cells(1:end-1).end],[certificate.cells(2:end).start]);
            testCase.verifyEqual(certificate.cells(end).end,.1);
            testCase.verifyGreaterThan([certificate.cells.end]-[certificate.cells.start],0);
        end
        function coarseProposalIsRefinedWithoutResamplingFeedback(testCase)
            [state,input,gain,cfg]=localFixture(0);
            certificate=fialaCertificate.sample(state-1e-4,state+1e-4,[state;input],gain, ...
                1e-5*ones(6,1),input,cfg,maximumCellDuration=.1);
            testCase.verifyTrue(certificate.accepted);
            testCase.verifyGreaterThan(certificate.rejectedCells,0);
            testCase.verifyEqual(certificate.verifiedThrough,.1);
        end
        function exhaustedBudgetCannotCertifyAPartialSample(testCase)
            [state,input,gain,cfg]=localFixture(0);
            certificate=fialaCertificate.sample(state-1e-4,state+1e-4,[state;input],gain, ...
                1e-5*ones(6,1),input,cfg,maximumCells=1);
            testCase.verifyFalse(certificate.accepted);
            testCase.verifyEqual(string(certificate.reason),"cellBudget");
            testCase.verifyLessThan(certificate.verifiedThrough,.1);
        end
        function heldInputViolatingSlewIsRejectedBeforePropagation(testCase)
            [state,input,gain,cfg]=localFixture(0);cfg.model.frontWheelSteeringRateMaximum=.001;
            certificate=fialaCertificate.sample(state-1e-4,state+1e-4,[state;input],gain, ...
                1e-5*ones(6,1),input,cfg);
            testCase.verifyFalse(certificate.accepted);
            testCase.verifyEqual(string(certificate.reason),"executionBounds");
            testCase.verifyEmpty(certificate.cells);
        end
        function heldInputViolatingActuatorLimitIsRejected(testCase)
            [state,input,gain,cfg]=localFixture(0);cfg.model.frontWheelSteeringAngleMaximum=1e-5;
            certificate=fialaCertificate.sample(state-1e-4,state+1e-4,[state;input],gain, ...
                1e-5*ones(6,1),input,cfg);
            testCase.verifyFalse(certificate.accepted);
            testCase.verifyEqual(string(certificate.reason),"executionBounds");
        end
        function largerInletCompletesTheOriginalSample(testCase,steering)
            [state,input,gain,cfg]=localFixture(steering);
            certificate=fialaCertificate.sample(state-.01,state+.01,[state;input],gain, ...
                .001*ones(6,1),input,cfg);
            testCase.verifyTrue(certificate.accepted);
            testCase.verifyEqual(certificate.verifiedThrough,.1);
        end
        function correlatedSequenceContainsHeldNonlinearReplays(testCase,steering)
            [sequence,violation]=localSequenceReplay(steering);
            testCase.verifyTrue(sequence.accepted);
            testCase.verifyEqual(sequence.completedSamples,20);
            testCase.verifyLessThanOrEqual(violation,1e-8);
            testCase.verifyLessThanOrEqual(max(cellfun(@(c)size(c.generators,2),sequence.samples)),184);
            testCase.verifyFalse(sequence.trajectoryCertified);
        end
        function repeatedFeedbackKeepsALargerTubeBounded(testCase)
            [state,input,gain,cfg]=localFixture(0);
            sequence=fialaCertificate.sequence(state-.005,state+.005,state,repmat(input,1,20), ...
                gain,.0005*ones(6,1),input,cfg);
            testCase.verifyTrue(sequence.accepted);
            testCase.verifyLessThan(diff(sequence.samples{end}.endpoint(2,:)),.04);
        end
        function failedSampleCannotSeedAContinuation(testCase)
            [state,input,gain,cfg]=localFixture(0);inlet=localCertificate();inlet.accepted=false;
            testCase.verifyError(@()fialaCertificate.sample(state,state,[state;input],gain, ...
                zeros(6,1),input,cfg,inlet=inlet),'collisionAvoidanceController:invalidFeedbackInlet');
        end
        function changedModelCannotReuseACorrelatedInlet(testCase)
            [state,input,gain,cfg]=localFixture(0);inlet=localCertificate();cfg.vehicle.m=cfg.vehicle.m+1;
            testCase.verifyError(@()fialaCertificate.sample(state,state,[state;input],gain, ...
                zeros(6,1),input,cfg,inlet=inlet),'collisionAvoidanceController:changedFeedbackModel');
        end
        function expiredComputationBudgetCannotAcceptAPrefix(testCase)
            [state,input,gain,cfg]=localFixture(0);
            certificate=fialaCertificate.sample(state,state,[state;input],gain,zeros(6,1), ...
                input,cfg,maximumComputationTime=realmin);
            testCase.verifyFalse(certificate.accepted);
            testCase.verifyEqual(string(certificate.reason),"timeBudget");
            testCase.verifyEmpty(certificate.cells);
        end
    end
end

function preserved=localHeldGeneratorsPreserved(certificate)
    first=certificate.cells(1).generators(7:8,:);preserved=true;
    for index=1:numel(certificate.cells)
        current=certificate.cells(index).generators(7:8,:);
        preserved=preserved && isequal(current(:,1:size(first,2)),first) && ...
            all(current(:,size(first,2)+1:end)==0,'all');
    end
end

function [sequence,violation]=localSequenceReplay(steering)
    [state,input,gain,cfg]=localFixture(steering);
    inputs=repmat(input,1,20);inputs(1,4:end)=0;
    sequence=fialaCertificate.sequence(state-.001,state+.001,state,inputs,gain, ...
        .0001*ones(6,1),input,cfg);
    points=state+.001*cos((1:6).'*(1:32)*sqrt(2));previous=input;violation=0;
    for sample=1:numel(sequence.samples)
        certificate=sequence.samples{sample};
        noise=.0001*cos((1:6).'*(1:32)*sqrt(2)+sample);
        commands=certificate.feedbackNominal(7:8)+gain*(points-certificate.feedbackNominal(1:6)+noise);
        change=commands-previous;
        violation=max([violation;reshape(certificate.inputChange(:,1)-change,[],1); ...
            reshape(change-certificate.inputChange(:,2),[],1)]);
        for index=1:numel(certificate.cells)
            cell=certificate.cells(index);dt=(cell.end-cell.start)/10;
            for step=1:10
                violation=max([violation;reshape(cell.swept(1:6,1)-points,[],1); ...
                    reshape(points-cell.swept(1:6,2),[],1)]);
                first=ltvBicycleModel.fialaWorldDynamics(points,commands,cfg);
                second=ltvBicycleModel.fialaWorldDynamics(points+dt*first/2,commands,cfg);
                third=ltvBicycleModel.fialaWorldDynamics(points+dt*second/2,commands,cfg);
                fourth=ltvBicycleModel.fialaWorldDynamics(points+dt*third,commands,cfg);
                points=points+dt*(first+2*second+2*third+fourth)/6;
            end
            violation=max([violation;reshape(cell.endpoint(1:6,1)-points,[],1); ...
                reshape(points-cell.endpoint(1:6,2),[],1)]);
        end
        previous=commands;
    end
end

function [state,input,gain,cfg]=localFixture(steering)
    cfg=collisionAvoidanceControllerConfig(struct('controller',struct('sampleTime',.1)));
    state=[0;0;0;10;0;0];
    input=[steering;ltvBicycleModel.roadLoad(10,cfg)/cfg.vehicle.m/modifiedFialaTire.accelerationGain(cfg)];
    gain=[0,-.3,-1,0,-.03,-.03;-.2,0,0,-.3,0,0];
end

function certificate=localCertificate()
    [state,input,gain,cfg]=localFixture(0);
    certificate=fialaCertificate.sample(state-1e-4,state+1e-4,[state;input],gain, ...
        1e-5*ones(6,1),input,cfg);
end

function [certificate,violations]=localReplay(steering)
    [state,input,gain,cfg]=localFixture(steering);
    certificate=fialaCertificate.sample(state-1e-4,state+1e-4,[state;input],gain, ...
        1e-5*ones(6,1),input,cfg);
    phases=cos((1:12).'*(1:32)*sqrt(2));
    points=state+1e-4*phases(1:6,:);noise=1e-5*phases(7:12,:);
    commands=input+gain*(points-state+noise);violations=zeros(1,3);
    for index=1:numel(certificate.cells)
        cell=certificate.cells(index);dt=(cell.end-cell.start)/20;
        for step=1:20
            violations(1)=max(violations(1),max(cell.swept(1:6,1)-points,[],'all'));
            violations(2)=max(violations(2),max(points-cell.swept(1:6,2),[],'all'));
            first=ltvBicycleModel.fialaWorldDynamics(points,commands,cfg);
            second=ltvBicycleModel.fialaWorldDynamics(points+dt*first/2,commands,cfg);
            third=ltvBicycleModel.fialaWorldDynamics(points+dt*second/2,commands,cfg);
            fourth=ltvBicycleModel.fialaWorldDynamics(points+dt*third,commands,cfg);
            points=points+dt*(first+2*second+2*third+fourth)/6;
        end
        violations(3)=max([violations(3);reshape(cell.endpoint(1:6,1)-points,[],1);reshape(points-cell.endpoint(1:6,2),[],1)]);
    end
end
