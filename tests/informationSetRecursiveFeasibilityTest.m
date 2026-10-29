classdef informationSetRecursiveFeasibilityTest < matlab.unittest.TestCase
    % Exact scalar specializations retain estimation error in the shift.
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (Test)
        function aNonNestedMeasurementPreservesTheZeroSlackCandidateAfterIntersection(testCase)
            result=verifyInformationSetRecursiveFeasibility();
            settled=result(1);
            testCase.verifyFalse(settled.rawMeasurementNested);
            testCase.verifyTrue(settled.posteriorNested);
            testCase.verifyTrue(settled.shiftSetsIncluded);
            testCase.verifyGreaterThan(settled.initialPositionWidthMeters,0);
            testCase.verifyGreaterThan(settled.positionInnovationMeters,0);
            testCase.verifyEqual(settled.candidateCostMeters,0,AbsTol=1e-12);
            testCase.verifyGreaterThan(settled.candidateTerminalMarginMeters,0);
            testCase.verifyLessThan(settled.rawResetTerminalMarginMeters,0);
        end
        function theShiftedPositiveSlackBudgetLosesTheAppliedHoldDeficit(testCase)
            result=verifyInformationSetRecursiveFeasibility();
            relaxed=result(2);
            testCase.verifyTrue(relaxed.shiftSetsIncluded);
            testCase.verifyEqual(relaxed.oldCostMeters,.6,AbsTol=1e-12);
            testCase.verifyEqual(relaxed.firstDeficitMeters,.2,AbsTol=1e-12);
            testCase.verifyEqual(relaxed.candidateCostMeters,.4,AbsTol=1e-12);
            testCase.verifyEqual(relaxed.candidateCostMeters, ...
                relaxed.oldCostMeters-relaxed.firstDeficitMeters,AbsTol=1e-12);
            testCase.verifyGreaterThan(relaxed.candidateTerminalMarginMeters,0);
        end
        function aUniformNewRemainderCanExcludeAFeasibleShiftedWitness(testCase)
            % Exact scalar specialization: F(1+d)=(1+d)^2 <= 1.05.
            % The old witness d=0 is feasible. A sound uniform remainder
            % over |d|<=.5 nevertheless excludes it. Zero-update-compatible
            % rows retain it, and still imply the nonlinear constraint.
            rho=.5;limit=1.05;witness=0;
            testCase.verifyLessThanOrEqual((1+witness)^2,limit);
            testCase.verifyGreaterThan(1+2*witness+rho^2,limit);
            rows=[2+rho;2-rho];bounds=(limit-1)*ones(2,1);
            testCase.verifyLessThanOrEqual(rows*witness,bounds);
            for d=linspace(-rho,rho,101)
                if all(rows*d<=bounds),testCase.verifyLessThanOrEqual((1+d)^2,limit+1e-12);end
            end
        end
        function relinearizingShiftedNodesCanExcludeAnInvariantZeroSlackPlan(testCase)
            % Exact controlled system x+ = x^2+u, -0.01<=u<=0.
            % [0,1] is invariant with u=0. Every old tube below is safe.
            % The next exact measurement .5 belongs to the old first image.
            % Resetting only that node reproduces the online RTI structure.
            oldTube=[sqrt(.5),1;.5,1;.25,1];
            testCase.verifyEqual(oldTube(1:2,:).^2,oldTube(2:3,:),AbsTol=1e-15);
            measured=.5;oldSecondCenter=(.75)^2;
            testCase.verifyGreaterThanOrEqual(measured,oldTube(2,1));
            testCase.verifyLessThanOrEqual(measured,oldTube(2,2));
            nonlinearCandidate=[measured,measured^2,measured^4];
            testCase.verifyGreaterThanOrEqual(nonlinearCandidate,0);
            testCase.verifyLessThanOrEqual(nonlinearCandidate,1);
            % x1=.25+u0; x2=2*.5625*x1-.5625^2+u1.
            % Both control coefficients are positive and both inputs <=0.
            % Even the maximum affine x2 is negative, so the hard terminal
            % condition x2>=0 makes the new local problem infeasible.
            largestAffineEndpoint=2*oldSecondCenter*measured^2-oldSecondCenter^2;
            testCase.verifyEqual(largestAffineEndpoint,-9/256,AbsTol=0);
            testCase.verifyLessThan(largestAffineEndpoint,0);
            % Relaxing this last row would require positive slack, despite
            % the invariant old plan having zero slack.
            testCase.verifyEqual(max(0,-largestAffineEndpoint),9/256,AbsTol=0);
        end
        function retainedTubeIncrementsAllowZeroAndNonzeroSafeUpdates(testCase)
            % F(x,u,w)=x^2+u+w, shared w in [-.01,.01]. The complete
            % retained images are known exactly on these positive intervals.
            tube=[.6,.8;.45,.75;.2925,.6725];baseInput=.1;
            for k=1:2
                testCase.verifyEqual(tube(k,:).^2+baseInput+[-.01,.01], ...
                    tube(k+1,:),AbsTol=1e-15);
            end
            % On |d|+r<=.15, |2*x-A|<=M over the ENTIRE lifted domain.
            a=2*mean(tube(1:2,:),2);m=diff(tube(1:2,:),1,2)+.3;
            for inputs=[0,0;.02,-.01].'
                shift=zeros(3,1);radius=zeros(3,1);
                for k=1:2
                    testCase.verifyLessThanOrEqual(abs(shift(k))+radius(k),.15);
                    shift(k+1)=a(k)*shift(k)+inputs(k);
                    radius(k+1)=(abs(a(k))+m(k))*radius(k)+m(k)*abs(shift(k));
                end
                outer=tube+[shift-radius,shift+radius];
                if all(inputs==0)
                    testCase.verifyEqual(outer,tube,AbsTol=0);
                else
                    testCase.verifyGreaterThan(shift(2),0);
                end
                testCase.verifyGreaterThanOrEqual(outer(:,1),0);
                testCase.verifyLessThanOrEqual(outer(:,2),1);
                for x=linspace(tube(1,1),tube(1,2),11)
                    for w0=[-.01,0,.01]
                        for w1=[-.01,0,.01]
                            candidate=[x;x^2+baseInput+inputs(1)+w0;0];
                            candidate(3)=candidate(2)^2+baseInput+inputs(2)+w1;
                            testCase.verifyGreaterThanOrEqual(candidate,outer(:,1)-1e-14);
                            testCase.verifyLessThanOrEqual(candidate,outer(:,2)+1e-14);
                        end
                    end
                end
            end
        end
    end
end
