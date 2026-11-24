classdef arm64ExportTest < matlab.unittest.TestCase
    methods (Test)
        function theExportedUpdateMatchesTheOnlineObserverAcrossADropout(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            for folder=["config","estimator","controller","scripts/native"]
                testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,folder)));
            end
            cfg=collisionAvoidanceControllerConfig();
            [design,kernel,settings,bound]=onlineNrmmTrackingRuntime("targetDesign",cfg);
            p=struct('cfg',cfg,'design',design,'kernel',kernel,'settings',settings,'bound',bound);
            radius=.01;heading=.25;
            [track,~]=onlineNrmmTrackingRuntime("targetPoint",[],0,[20;1],heading,cfg,NaN,radius);
            exported=track;
            for k=1:16
                t=k*cfg.outputFeedback.sensorPeriod;
                pos=[20+9*t;1+.5*t];
                if ismember(k,[8,9]),pos=[];end
                [track,motion]=onlineNrmmTrackingRuntime("targetPoint",track,t,pos,heading,cfg,NaN,radius);
                [exported,other]=nrmmTargetSample(exported,t,pos,heading,NaN,radius,p);
                testCase.verifyEqual(other,motion,'AbsTol',1e-10);
                testCase.verifyEqual(exported.state,track.state,'AbsTol',1e-10);
                testCase.verifyEqual(exported.motionBounds,track.motionBounds,'AbsTol',1e-10);
            end
            testCase.verifyTrue(exported.initialized);
        end
    end
end
