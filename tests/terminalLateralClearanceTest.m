classdef terminalLateralClearanceTest < matlab.unittest.TestCase
% terminalLateralClearanceTest Declared lateral clearance in the terminal set.
    methods (TestClassSetup)
        function addPaths(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'scripts')));
        end
    end
    methods (TestMethodSetup)
        function resetController(~)
            collisionAvoidanceController("resetNominalTrajectory");
        end
    end
    methods (Test)
        function declaredClearanceShrinksTheStraightSetAndKeepsCornersInside(testCase)
            [free,cfg]=localTerminal(localRoad(0,zeros(2,0),false));
            collisionAvoidanceController("resetNominalTrajectory");
            road=localTerminal(localRoad(0,[4;5],true));
            testCase.verifyEmpty(free.lateralClearance.rows);
            testCase.verifySize(road.lateralClearance.rows,[4,5]);
            testCase.verifyEqual(road.lateralClearance.declared,[4;5]);
            testCase.verifyTrue(all(road.radius<=free.radius+1e-12));
            testCase.verifyLessThan(min(road.radius./free.radius),1-1e-6);
            corners=localCornerLateral(road,cfg,0);
            testCase.verifyLessThanOrEqual(max(corners.left),5+1e-9);
            testCase.verifyLessThanOrEqual(max(corners.right),4+1e-9);
            % The set is scaled to 95 percent of the first binding row, which is
            % on the tighter right side; sampled corners approach that edge.
            support=abs(road.lateralClearance.rows*road.modalBasis)*road.radius;
            room=road.lateralClearance.bound-road.reserve;
            testCase.verifyTrue(all(support<=room+1e-9));
            testCase.verifyGreaterThan(max(support./room),0.9);
            testCase.verifyGreaterThan(max(corners.right),3.5);
        end
        function aCurvedReferenceKeepsExactCornersInsideTheClearance(testCase)
            [terminal,cfg]=localTerminal(localRoad(0.01,[3.5;3.5],false));
            testCase.verifyGreaterThan(terminal.lateralClearance.curvatureAllowance,0);
            corners=localCornerLateral(terminal,cfg,0.01);
            testCase.verifyLessThanOrEqual(max(corners.left),3.5+1e-9);
            testCase.verifyLessThanOrEqual(max(corners.right),3.5+1e-9);
            testCase.verifyGreaterThan(max([corners.left,corners.right]),3.0);
        end
        function theErrorBoundTightensTheRows(testCase)
            geometry=localRoad(0,[4;5],true);geometry.lateralClearanceErrorBound=0.3;
            terminal=localTerminal(geometry);
            collisionAvoidanceController("resetNominalTrajectory");
            reference=localTerminal(localRoad(0,[4;5],true));
            testCase.verifyEqual(terminal.lateralClearance.bound,reference.lateralClearance.bound-0.3,AbsTol=1e-12);
        end
        function aRoadNarrowerThanTheFootprintIsRejected(testCase)
            cfg=collisionAvoidanceControllerConfig();
            testCase.verifyError(@() localTerminal(localRoad(0,[cfg.vehicle.width/2;cfg.vehicle.width/2],false)), ...
                'collisionAvoidanceController:insufficientLateralClearance');
        end
        function boundariesWithoutADeclaredClearanceAreRejected(testCase)
            testCase.verifyError(@() localTerminal(localRoad(0,zeros(2,0),true)), ...
                'collisionAvoidanceController:missingLateralClearance');
        end
        function aNonpositiveClearanceIsAnInputError(testCase)
            testCase.verifyError(@() localTerminal(localRoad(0,[0;5],false)), ...
                'collisionAvoidanceController:invalidRoadClearance');
        end
        function refittedBoundariesAreReadmittedAndAChangedClearanceIsNot(testCase)
            cfg=collisionAvoidanceControllerConfig();h=cfg.controller.sampleTime;
            road=localRoad(0,[5;5],true);
            [command,~,~,state]=collisionAvoidanceController(localEgo(0,[0;0;0;8;0;0],[]),[],road,cfg);
            next=state.predictedState(:,2);
            refit=road;refit.boundaries(1).origin=[1;0];refit.boundaries(1).parameterRange=[-101,1999];
            [~,~,problem]=collisionAvoidanceController(localEgo(h,next,command.actuatorInput),[],refit,cfg,state);
            testCase.verifyTrue(problem.metadata.readmittedAfterRoadRefit);
            testCase.verifyEqual(problem.metadata.terminalLateralClearance.declared,[5;5]);
            changed=road;changed.lateralClearance=[4.5;5];
            testCase.verifyError(@() collisionAvoidanceController(localEgo(h,next,command.actuatorInput),[],changed,cfg,state), ...
                'collisionAvoidanceController:changedExecutionContract');
        end
        function theBoundedStraightCruiseCompletesWithoutLeavingTheRoad(testCase)
            report=runExactStateRecursiveFeasibilityScenario(Scenario="cruise",SampleCount=12, ...
                DeadlineSeconds=30,UseRoadBoundaries=true,InitialTrackingError=[2.5;0.1;0;0;0]);
            testCase.verifyTrue(report.passed);
            testCase.verifyTrue(report.roadBoundariesEnabled);
            testCase.verifyGreaterThanOrEqual(report.minimumSampledRoadMargin,0);
            testCase.verifyEqual(report.executedHolds,12);
        end
    end
end

function geometry=localRoad(curvature,clearance,withBoundaries)
    geometry=struct("centerline",[-100,0;2000,0]);
    if curvature~=0
        curve=struct('origin',[0;0],'heading',0,'curvature',curvature,'length',300);
        geometry=struct('referenceCurve',curve,'centerline',laneGeometry.referencePose(linspace(0,300,201),0,curve).');
    end
    if withBoundaries
        boundary=struct("origin",zeros(2,1),"longitudinalDirection",[1;0],"lateralDirection",[0;1], ...
            "coefficients",[0;0;-5],"parameterRange",[-100;2000],"safeSideSign",1);
        boundaries=[boundary;boundary];boundaries(2).coefficients(3)=5;boundaries(2).safeSideSign=-1;
        geometry.boundaries=boundaries;
    end
    if ~isempty(clearance),geometry.lateralClearance=clearance;end
end

function ego=localEgo(time,x,heldInput)
    ego=struct("position",x(1:2),"yaw",x(3),"speed",x(4),"lateralVelocity",x(5),"yawRate",x(6), ...
        "stateTime",time,"controllerStateErrorBound",zeros(6,1), ...
        "perception",struct('time',time,'range',30,'completeWithinRange',true));
    if ~isempty(heldInput),ego.heldActuatorInput=heldInput;end
end

function [terminal,cfg]=localTerminal(geometry)
    cfg=collisionAvoidanceControllerConfig();
    x=[0;0;0;8;0;0];
    if isfield(geometry,'referenceCurve')
        x=ltvBicycleModel.cruiseEquilibrium(geometry.referenceCurve.curvature,cfg);
        [position,heading]=laneGeometry.fromFrenet(x,geometry);
        x=[position;heading;x(4:6)];
    end
    [~,~,problem]=collisionAvoidanceController(localEgo(0,x,[]),[],geometry,cfg);
    terminal=problem.program.terminal;
end

function corners=localCornerLateral(terminal,cfg,curvature)
% Exact corner Frenet lateral coordinates over boundary points of the modal set.
    basis=terminal.modalBasis;modal=terminal.modalMatrix;radius=terminal.radius;
    stream=RandStream('mt19937ar','Seed',20260924);
    directions=[real(basis),imag(basis),eye(5),randn(stream,5,400)];
    directions=directions(:,vecnorm(directions)>1e-12);
    scale=max(abs(modal*directions)./radius,[],1);
    errors=[directions./scale,-directions./scale];
    halfLength=cfg.vehicle.length/2;halfWidth=cfg.vehicle.width/2;
    signs=[1,1;1,-1;-1,1;-1,-1];
    left=zeros(1,size(errors,2));right=left;
    for k=1:size(errors,2)
        y=errors(1,k);psi=errors(2,k);lateral=zeros(4,1);
        for c=1:4
            u=signs(c,1)*halfLength*cos(psi)-signs(c,2)*halfWidth*sin(psi);
            v=y+signs(c,1)*halfLength*sin(psi)+signs(c,2)*halfWidth*cos(psi);
            if curvature==0
                lateral(c)=v;
            else
                lateral(c)=(1-sqrt((1-curvature*v)^2+(curvature*u)^2))/curvature;
            end
        end
        left(k)=max(lateral);right(k)=max(-lateral);
    end
    corners=struct('left',left,'right',right);
end
