classdef nrmmConstantMotionFamilyTest < matlab.unittest.TestCase
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'estimator')));
        end
    end
    methods (Test)
        function coupledSpeedAndAccelerationAreNotReplacedByTheirHull(testCase)
            intervals=[6,-2,0,0;10,2,0,0];
            family=nrmmTargetHistory("constantMotion", 'condition',[],0,intervals,[],zeros(2,0),zeros(2,0));
            family=nrmmTargetHistory("constantMotion", 'condition',family,1,[4,-2,0,0;12,2,0,0],0,[8;0],[.02;.02]);
            testCase.verifyEqual(family.epochTime,0);
            testCase.verifyGreaterThan(size(family.lower,2),1);
            image=nrmmTargetHistory("constantMotion", 'image',family,1,2,0);
            hull=family;hull.lower=min(family.lower,[],2);hull.upper=max(family.upper,[],2);
            hullImage=nrmmTargetHistory("constantMotion", 'image',hull,1,2,0);
            width=max(image.upper(1,:))-min(image.lower(1,:));
            testCase.verifyLessThan(width,.75*(hullImage.upper(1)-hullImage.lower(1)));
            % Truth V0=8,A=0 remains, including its entire future displacement.
            testCase.verifyTrue(any(all(family.lower<=[8;0;0;0] & family.upper>=[8;0;0;0],1)));
        end
        function boundedNoiseCurvedHistoryRetainsTheSameTrueParameters(testCase)
            truth=[8;.4;.012;.7];family=[];
            for time=0:.2:1.4
                times=0:.05:time;positions=localPosition(truth,times);
                measured=positions+.025*[sin(13*times);cos(11*times)];
                intervals=[6,-1,-.03,.3;10+time,1,.03,1.3];
                centers=measured(:,end)-measured(:,1:end-1);
                family=nrmmTargetHistory("constantMotion", 'condition',family,time,intervals, ...
                    times(1:end-1),centers,.05*ones(size(centers)));
                contained=all(family.lower<=truth+1e-10 & family.upper>=truth-1e-10,1);
                testCase.verifyTrue(any(contained));
                image=nrmmTargetHistory("constantMotion", 'image',family,time,time+2,0);
                displacement=localPosition(truth,time+2)-localPosition(truth,time);
                testCase.verifyTrue(any(all(image.lower<=displacement & image.upper>=displacement,1)));
                testCase.verifyEqual(family.epochTime,0);
            end
        end
        function stoppingAndAcceleratingMembersRemainInsideTheImage(testCase)
            intervals=[0,-2,-.03,-.2;4,2,.03,.2];
            family=nrmmTargetHistory("constantMotion", 'condition',[],0,intervals,[],zeros(2,0),zeros(2,0));
            stream=RandStream('mt19937ar',Seed=13);
            for start=[0,.5,2]
                image=nrmmTargetHistory("constantMotion", 'image',family,start,start+3,0);
                for trial=1:100
                    p=intervals(1,:).'+diff(intervals,1,1).'.*rand(stream,4,1);
                    d=localPosition(p,start+3)-localPosition(p,start);
                    testCase.verifyLessThanOrEqual(image.lower,d+1e-10);
                    testCase.verifyGreaterThanOrEqual(image.upper,d-1e-10);
                end
            end
        end
        function unavailableBackwardStopHistoryDoesNotDeleteTheFamily(testCase)
            intervals=[0,-2,0,0;1,0,0,0];
            family=nrmmTargetHistory("constantMotion", 'condition',[],0,intervals,-1,[2;0],[.04;.04]);
            testCase.verifyTrue(family.available);
            testCase.verifyTrue(any(all(family.lower<=[0;-1;0;0] & family.upper>=[0;-1;0;0],1)));
        end
        function incompatibleHistoryIsNotAnEmptyZeroErrorCertificate(testCase)
            intervals=[7,0,0,0;9,0,0,0];
            family=nrmmTargetHistory("constantMotion", 'condition',[],0,intervals,[],zeros(2,0),zeros(2,0));
            testCase.verifyError(@()nrmmTargetHistory("constantMotion", 'condition',family,1,intervals,0,[20;0],[.01;.01]), ...
                'nrmmConstantMotionFamily:inconsistentHistory');
        end
    end
end

function p=localPosition(theta,times)
    duration=times;
    if theta(2)<0,duration=min(duration,-theta(1)/theta(2));end
    arc=theta(1)*duration+theta(2)*duration.^2/2;angle=theta(3)*arc/2;
    factor=ones(size(angle));active=angle~=0;factor(active)=sin(angle(active))./angle(active);
    p=(arc.*factor).*[cos(theta(4)+angle);sin(theta(4)+angle)];
end
