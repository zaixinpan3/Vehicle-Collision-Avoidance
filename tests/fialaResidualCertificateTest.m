classdef fialaResidualCertificateTest < matlab.unittest.TestCase
    properties (TestParameter)
        tireCase = {"trim","adhesion","switch","saturation","negativeSteering"};
    end
    methods (TestClassSetup)
        function buildNativeVerifier(testCase)
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
        function wholeDomainEnclosureContainsIndependentFialaResiduals(testCase,tireCase)
            [certificate,samples]=localResidualSamples(tireCase);
            testCase.verifyGreaterThanOrEqual(min(samples-certificate.residual(:,1),[],'all'),-1e-9);
            testCase.verifyLessThanOrEqual(max(samples-certificate.residual(:,2),[],'all'),1e-9);
            testCase.verifyGreaterThan(max(certificate.residualRateBound),0);
            testCase.verifyFalse(certificate.trajectoryCertified);
        end
        function shrinkingDomainReducesTangentResidualQuadratically(testCase)
            [cfg,center,radius,a,b,c]=localFixture("trim");
            large=fialaCertificate.residual(center-radius,center+radius,a,b,c,cfg);
            small=fialaCertificate.residual(center-radius/2,center+radius/2,a,b,c,cfg);
            testCase.verifyLessThan(max(small.residualRateBound),.35*max(large.residualRateBound));
        end
        function exactYawKinematicRowHasZeroAffineRemainder(testCase)
            [cfg,center,radius,a,b,c]=localFixture("trim");
            certificate=fialaCertificate.residual(center-radius,center+radius,a,b,c,cfg);
            testCase.verifyEqual(certificate.residual(3,:),[0,0]);
        end
        function tireRatioEndpointIsRejected(testCase)
            [cfg,center,radius,a,b,c]=localFixture("trim");upper=center+radius;upper(8)=1;
            testCase.verifyError(@()fialaCertificate.residual(center-radius,upper,a,b,c,cfg), ...
                'collisionAvoidanceController:invalidFialaInterval');
        end
        function speedFloorIntersectionIsRejected(testCase)
            [cfg,center,radius,a,b,c]=localFixture("trim");lower=center-radius;lower(4)=cfg.model.scheduleSpeedFloor;
            testCase.verifyError(@()fialaCertificate.residual(lower,center+radius,a,b,c,cfg), ...
                'collisionAvoidanceController:invalidFialaInterval');
        end
        function actualSlipOutsideRegularChartIsRejected(testCase)
            [cfg,center,radius,a,b,c]=localFixture("trim");upper=center+radius;upper(7)=1.6;
            testCase.verifyError(@()fialaCertificate.residual(center-radius,upper,a,b,c,cfg), ...
                'collisionAvoidanceController:invalidFialaInterval');
        end
        function arithmeticOverflowCannotReturnACertificate(testCase)
            [cfg,center,radius,a,b,c]=localFixture("trim");upper=center+radius;upper(4)=realmax;
            testCase.verifyError(@()fialaCertificate.residual(center-radius,upper,a,b,c,cfg), ...
                'collisionAvoidanceController:invalidFialaInterval');
        end
        function heldFeedbackCellEstablishesItsOwnDomainContainment(testCase)
            certificate=localFeedbackCell(.005,false);
            testCase.verifyTrue(certificate.heldCell.accepted);
            testCase.verifyGreaterThan(certificate.heldCell.swept(:,1),certificate.lower(1:6));
            testCase.verifyLessThan(certificate.heldCell.swept(:,2),certificate.upper(1:6));
            testCase.verifyFalse(certificate.trajectoryCertified);
        end
        function oversizedHeldCellIsRejectedInsteadOfAssumingContainment(testCase)
            certificate=localFeedbackCell(.5,false);
            testCase.verifyFalse(certificate.heldCell.accepted);
        end
        function feedbackInputOutsideDeclaredDomainIsRejected(testCase)
            certificate=localFeedbackCell(.005,true);
            testCase.verifyFalse(certificate.heldCell.accepted);
        end
    end
end

function [cfg,center,radius,a,b,c]=localFixture(kind)
    cfg=collisionAvoidanceControllerConfig();state=[0;0;0;10;0;0];
    input=[0;ltvBicycleModel.roadLoad(10,cfg)/cfg.vehicle.m/modifiedFialaTire.accelerationGain(cfg)];
    switch kind
        case "adhesion",input(1)=.04;
        case "switch"
            p=modifiedFialaTire.parameters(cfg);
            input(1)=atan(3*p.longitudinalForceScale(1)*sqrt(1-input(2)^2)/p.corneringStiffness(1));
        case "saturation",input(1)=.4;
        case "negativeSteering",input(1)=-.12;
    end
    [a,b,c]=ltvBicycleModel.continuousMatrices(0,10,cfg,input(2),0,struct('state',state,'input',input));
    center=[state;input];radius=[.001;.001;.001;.03;.01;.01;.005;.003];
end

function [certificate,residuals]=localResidualSamples(kind)
    [cfg,center,radius,a,b,c]=localFixture(kind);
    certificate=fialaCertificate.residual(center-radius,center+radius,a,b,c,cfg);
    corners=2*double(dec2bin(0:255,8).'-'0')-1;
    interior=sin((1:8).'*(1:400)*sqrt(2));points=center+radius.*[corners,interior];
    residuals=zeros(6,size(points,2));
    for index=1:size(points,2)
        residuals(:,index)=ltvBicycleModel.fialaWorldDynamics(points(1:6,index),points(7:8,index),cfg) ...
            -a*points(1:6,index)-b*points(7:8,index)-c;
    end
end

function certificate=localFeedbackCell(duration,narrowInput)
    [cfg,center,~,a,b,c]=localFixture("trim");radius=[.1;.01;.01;.2;.02;.02;.02;.03];
    if narrowInput,radius(7:8)=1e-8;end
    feedback=struct('inletLower',center(1:6)-1e-4,'inletUpper',center(1:6)+1e-4, ...
        'nominal',center,'gain',[0,-.3,-1,0,-.03,-.03;-.2,0,0,-.3,0,0], ...
        'measurementRadius',1e-5*ones(6,1),'duration',duration);
    certificate=fialaCertificate.residual(center-radius,center+radius,a,b,c,cfg,feedback);
end
