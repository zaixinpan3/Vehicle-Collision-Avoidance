classdef nrmmVectorJacobianLmiTest < matlab.unittest.TestCase
    % Validate the full derivative and finite-error matrix certificate.
    properties
        Design
    end
    properties (TestParameter)
        radialRegion = struct('inverseFloor',1,'physicalSpeed',2,'speedSaturation',3)
        projected = struct('insideAccelerationBall',false,'outsideAccelerationBall',true)
        reflection = struct('rotation',1,'reflection',-1)
    end
    methods (TestClassSetup)
        function prepareDesign(testCase)
            root = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"config")));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,"estimator")));
            testCase.Design = synthesizeNrmmObserverGains(nrmmTrackingConfig());
        end
    end
    methods (Test)
        function analyticDerivativeMatchesTheRetainedVectorField(testCase,radialRegion,projected)
            domain = testCase.Design.target.domain;
            stream = RandStream("mt19937ar","Seed",171+radialRegion+10*projected);
            maximum = 0;
            for sample = 1:150
                [q,s] = localState(stream,domain,radialRegion,projected);
                jacobian = nrmmTargetJacobian("evaluate",q,s,domain);
                state = [q;s];
                difference = zeros(2,4);
                for index = 1:4
                    step = 1e-5*max(1,abs(state(index)));
                    direction = zeros(4,1); direction(index) = step;
                    difference(:,index) = (localPhi(state+direction,domain) ...
                        -localPhi(state-direction,domain))/(2*step);
                end
                maximum = max(maximum,norm(jacobian-difference,'fro'));
            end
            testCase.verifyLessThan(maximum,2e-7);
        end

        function rotationsAndReflectionsPreserveTheMatrixCertificate(testCase,reflection)
            target = testCase.Design.target;
            angle = 0.73;
            transform = [cos(angle),-sin(angle);sin(angle),cos(angle)]*diag([1,reflection]);
            q = [14;2]; s = [1;2.2];
            reference = nrmmTargetJacobian("evaluate",q,s,target.domain);
            rotated = nrmmTargetJacobian("evaluate",transform*q,transform*s,target.domain);
            testCase.verifyEqual(rotated,transform*reference*kron(eye(2),transform.'),AbsTol=1e-12);
            metric = kron(target.lyapunovMatrix,eye(2));
            frame = kron(eye(3),transform);
            testCase.verifyEqual(frame.'*metric*frame,metric,AbsTol=1e-12);
        end

        function signedJacobiansCertifyEverySaturationRegime(testCase)
            target = testCase.Design.target;
            stream = RandStream("mt19937ar","Seed",431);
            energy = kron(target.lyapunovMatrix,eye(2));
            worst = -Inf;
            for region = 1:3
                for accelerationProjected = [false,true]
                    for sample = 1:200
                        [q,s] = localState(stream,target.domain,region,accelerationProjected);
                        jacobian = nrmmTargetJacobian("evaluate",q,s,target.domain);
                        dynamics = target.bandwidth*kron(target.closedLoopMatrix,eye(2));
                        dynamics(5:6,3:6) = dynamics(5:6,3:6) ...
                            +[jacobian(:,1:2)/target.bandwidth,jacobian(:,3:4)];
                        % A common, arbitrarily large measured rotation cancels exactly.
                        dynamics = dynamics-100*randn(stream)*kron(eye(3),[0,-1;1,0]);
                        residual = dynamics.'*energy+energy*dynamics+2*target.lambda*energy;
                        worst = max(worst,max(eig((residual+residual.')/2)));
                    end
                end
            end
            testCase.verifyLessThan(worst,0);
        end

        function finiteErrorsAcrossSaturationSurfacesContract(testCase)
            target = testCase.Design.target;
            stream = RandStream("mt19937ar","Seed",719);
            energy = kron(target.lyapunovMatrix,eye(2));
            maximum = -Inf;
            for sample = 1:500
                [q,s] = localState(stream,target.domain,1+mod(sample,3),mod(sample,2)==0);
                [otherQ,otherS] = localState(stream,target.domain,1+mod(sample+1,3),mod(sample,2)==1);
                if mod(sample,5)==0, otherQ = -q; end % Segment crosses the origin.
                error = [10*randn(stream,2,1);(q-otherQ)/target.bandwidth; ...
                    (s-otherS)/target.bandwidth^2];
                derivative = target.bandwidth*kron(target.closedLoopMatrix,eye(2))*error;
                derivative(5:6) = derivative(5:6) ...
                    +(localPhi([q;s],target.domain)-localPhi([otherQ;otherS],target.domain)) ...
                        /target.bandwidth^2;
                excess = 2*error.'*energy*derivative+2*target.lambda*(error.'*energy*error);
                maximum = max(maximum,excess/(error.'*error));
            end
            testCase.verifyLessThan(maximum,0);
        end

        function stationaryVelocityHasZeroNonlinearJacobian(testCase)
            domain = testCase.Design.target.domain;
            domain.regularizationSpeedMinimum = domain.speedMinimum;
            domain.speedMinimum = 0;
            jacobian = nrmmTargetJacobian("evaluate",zeros(2,1),[20;-30],domain);
            testCase.verifyEqual(jacobian,zeros(2,4),AbsTol=0);
        end

        function enclosurePreservesSignedCoupledDerivativeDirections(testCase)
            target = testCase.Design.target;
            enclosure = nrmmTargetJacobian("enclosure",target.domain);
            stream = RandStream("mt19937ar","Seed",913);
            directions = randn(stream,8,100);
            vertices = reshape(enclosure.vertices,8,[]);
            support = max(directions.'*vertices,[],2);
            maximum = -Inf;
            for sample = 1:600
                [q,s] = localState(stream,target.domain,1+mod(sample,3),mod(sample,2)==0);
                frame = [q/norm(q),[0,-1;1,0]*q/norm(q)];
                alignedS = frame.'*s;
                if alignedS(2)<0, frame(:,2)=-frame(:,2); end
                jacobian = frame.'*nrmmTargetJacobian("evaluate",q,s,target.domain)*kron(eye(2),frame);
                maximum = max(maximum,max(directions.'*jacobian(:)-support));
            end
            testCase.verifyLessThanOrEqual(maximum,1e-10);
            testCase.verifyGreaterThan(enclosure.regionCellCounts,zeros(3,1));
            testCase.verifyFalse(enclosure.floatingPointVerified);
        end
    end
end

function [q,s] = localState(stream,domain,region,projected)
    radius = domain.speedMinimum*(0.08+0.84*rand(stream));
    if region == 2
        radius = domain.speedMinimum+(0.02+0.96*rand(stream)) ...
            *(domain.speedMaximum-domain.speedMinimum);
    elseif region == 3
        radius = domain.speedMaximum*(1.02+5*rand(stream));
    end
    angle = 2*pi*rand(stream);
    q = radius*[cos(angle);sin(angle)];
    acceleration = domain.accelerationNormBound*(0.05+0.9*rand(stream));
    if projected, acceleration = domain.accelerationNormBound*(1.05+5*rand(stream)); end
    angle = 2*pi*rand(stream);
    s = acceleration*[cos(angle);sin(angle)];
end

function value = localPhi(state,domain)
    derivative = nrmmTargetTrackerDerivative([zeros(2,1);state], ...
        struct("bodyVelocity",zeros(2,1),"yawRate",0),domain);
    value = derivative(5:6);
end
