classdef freePoseTerminalTest < matlab.unittest.TestCase
    % Symmetry reduction, terminal invariance and target geometry.
    properties (TestParameter)
        angle = {0,.7,1.9}
        motion = {[-20;12;.4;8;0;.08;1.6;2.4;.95;0;0], ...
            [-12;2;.1;4;-.2;0;1.6;2.4;.95;0;0]}
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function rigidPlacementPreservesMembershipAndControl(testCase,angle)
            [seed,cfg,y]=localCore();g=[14;-23;angle];transformed=localTransform(y,g);
            placed=terminalContinuation.fit(seed,transformed,11);
            original=terminalContinuation.fit(seed,y,11);
            first=terminalContinuation.membership(y,11,original);
            second=terminalContinuation.membership(transformed,11,placed);
            u=terminalContinuation.control(y,11,original);
            movedInput=terminalContinuation.control(transformed,11,placed);
            next=nonlinearBicycleModel.sample(y(1:6),u,cfg);
            movedNext=nonlinearBicycleModel.sample(transformed(1:6),movedInput,cfg);
            expected=localTransform([next;u],g);
            testCase.verifyEqual(first,second,AbsTol=1e-13);
            testCase.verifyEqual(u,movedInput,AbsTol=1e-12);
            testCase.verifyEqual(movedNext,expected(1:6),AbsTol=1e-12);
        end
        function fittedPoseMinimizesTheOriginalEllipsoidError(testCase)
            [seed,~,y]=localCore();fitted=terminalContinuation.fit(seed,y,11);
            minimum=terminalContinuation.membership(y,11,fitted);
            alternatives=localPerturbedPlacements(fitted,y);
            testCase.verifyLessThan(minimum,0);
            testCase.verifyGreaterThan(alternatives,repmat(minimum,size(alternatives)));
        end
        function fittedPoseDerivativeMatchesAugmentedStatePerturbations(testCase)
            [seed,~,y]=localCore();[~,analytic]=terminalContinuation.fit(seed,y,11);
            numeric=localFitDifferences(seed,y);
            testCase.verifyEqual(analytic,numeric,AbsTol=2e-8);
        end
        function separationDerivativeIncludesPoseAndInputMemory(testCase,motion)
            [seed,cfg,y]=localCore();y(1:3)=[40;20;.35];
            [fitted,jacobian]=terminalContinuation.fit(seed,y,11);
            [~,~,poseGradient]=terminalContinuation.separation(fitted,motion,[0;0;.2;0],cfg,11);
            numeric=localSeparationDifferences(seed,y,motion,cfg);
            testCase.verifyEqual(poseGradient*jacobian,numeric,AbsTol=3e-7);
        end
        function freePoseDoesNotAuthorizeDrivingIntoAStationaryTarget(testCase)
            [seed,cfg,~]=localCore();y=[0;0;0;seed.base(4:6);seed.reference.input];
            fitted=terminalContinuation.fit(seed,y,11);target=[12;0;0;0;0;0;1.6;2.4;.95;0;0];
            margin=terminalContinuation.separation(fitted,target,[0;0;0;0],cfg,11);
            testCase.verifyLessThan(margin,0);
        end
        function retainedCoreClosesTheSuffixWithFiniteBrakingSlew(testCase,angle)
            [seed,cfg,y]=localCore();y=localTransform(y,[12;-7;angle]);
            seed=terminalContinuation.fit(seed,y,11);
            [membership,slew,separation]=localContinueCore(seed,y,cfg);
            testCase.verifyLessThanOrEqual(membership,zeros(size(membership)));
            testCase.verifyLessThanOrEqual(slew(2,:),.5*cfg.controller.sampleTime+1e-12);
            testCase.verifyGreaterThan(separation,0);
        end
    end
end

function [seed,cfg,y]=localCore()
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8, ...
        'model',struct('brakingRatioRateMaximum',.5)));
    seed=terminalContinuation.build(cfg,0);
    direction=[1;-1;1;1;-1];
    intrinsic=seed.quotientFactor\(seed.radius*.3*direction/norm(direction));
    y=[2;-3;.2;[seed.base(4:6);seed.reference.input]+intrinsic];
end
function y=localTransform(y,g)
    rotation=[cos(g(3)),-sin(g(3));sin(g(3)),cos(g(3))];
    y(1:2)=rotation*y(1:2)+g(1:2);y(3)=y(3)+g(3);
end
function values=localPerturbedPlacements(seed,y)
    directions=[eye(3),-eye(3)];values=zeros(1,6);
    for j=1:6
        trial=seed;trial.epochState(1:3)=trial.epochState(1:3)+.001*directions(:,j);
        values(j)=terminalContinuation.membership(y,11,trial);
    end
end
function jacobian=localFitDifferences(seed,y)
    h=1e-6;directions=eye(8);jacobian=zeros(3,8);
    for j=1:8
        hi=terminalContinuation.fit(seed,y+h*directions(:,j),11);
        lo=terminalContinuation.fit(seed,y-h*directions(:,j),11);
        jacobian(:,j)=(hi.epochState(1:3)-lo.epochState(1:3))/(2*h);
    end
end
function jacobian=localSeparationDifferences(seed,y,q,cfg)
    h=1e-6;directions=eye(8);jacobian=zeros(1,8);frame=[0;0;.2;0];
    for j=1:8
        hi=terminalContinuation.fit(seed,y+h*directions(:,j),11);
        lo=terminalContinuation.fit(seed,y-h*directions(:,j),11);
        jacobian(j)=(terminalContinuation.separation(hi,q,frame,cfg,11) ...
            -terminalContinuation.separation(lo,q,frame,cfg,11))/(2*h);
    end
end
function [membership,slew,separation]=localContinueCore(seed,y,cfg)
    membership=zeros(1,80);slew=zeros(2,80);separation=Inf;
    reference=terminalContinuation.referenceAt(seed,11);direction=[cos(reference(3));sin(reference(3))];
    q=[reference(1:2)-100*direction;reference(3);8;0;.08;1.6;2.4;.95;0;0];
    for j=1:80
        u=terminalContinuation.control(y,10+j,seed);slew(:,j)=abs(u-y(7:8));
        y=[nonlinearBicycleModel.sample(y(1:6),u,cfg);u];
        membership(j)=terminalContinuation.membership(y,11+j,seed);
        target=predictiveSafetyGeometry.predictTarget(q,j*cfg.controller.sampleTime);
        separation=min(separation,terminalContinuation.separation(seed,target,[0;0;0;0],cfg,11+j));
    end
end
