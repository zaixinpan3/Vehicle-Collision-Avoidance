function carlaVehicleDynamicsDriver(b)
    b.NumDialogPrms=1;b.NumInputPorts=2;b.NumOutputPorts=1;b.SetPreCompInpPortInfoToDynamic;b.SetPreCompOutPortInfoToDynamic;
    b.InputPort(2).Dimensions=6;b.InputPort(2).DirectFeedthrough=true;
    b.InputPort(1).Dimensions=6;b.InputPort(1).DirectFeedthrough=true;b.OutputPort(1).Dimensions=12;b.SampleTimes=[.05 0];b.SimStateCompliance='DefaultSimState';b.RegBlockMethod('Outputs',@output);
end
function output(b)
    % Scenario-only truth feeds the synthetic sensor adapter. The controller
    % receives observer estimates and range-clipped road geometry.
    global pdcaVdbExperiment
    if isempty(pdcaVdbExperiment),b.OutputPort(1).Data=zeros(12,1);return;end
    e=pdcaVdbExperiment;t=b.CurrentTime;
    if isfield(e,'lastTime') && abs(t-e.lastTime)<1e-9,b.OutputPort(1).Data=e.command;return;end
    raw=b.InputPort(1).Data;x=raw.*[1;-1;-1;1;-1;-1];
    truth=struct('position',x(1:2),'yaw',x(3),'speed',x(4),'longitudinalVelocity',x(4),'lateralVelocity',x(5),'yawRate',x(6),'stateTime',t,'heldActuatorInput',e.held);
    writeline(e.bridge,jsonencode(struct('operation','step','time',t,'pose',b.InputPort(2).Data)));
    live=jsondecode(readline(e.bridge));q=live.target(:);target=localTarget(q);
    try
        if isempty(e.context)
            targetInitial=e.target;
            [e.context,~]=nrmmEstimatorControllerAdapter('initialize',e.estimator,truth,@(time,unused)localTarget(predictiveSafetyGeometry.predictTarget(targetInitial,time)));
        end
        [e.context,ego,estimate,~,audit]=nrmmEstimatorControllerAdapter('sample',e.context,t,truth,target);
        ego.heldActuatorInput=e.held;
        road=perceiveRoadBoundaries(e.road,x(1:2),30,t);
        e.lastEgo=ego;e.lastEstimate=estimate;e.lastRoad=road;
        [command,~,prediction,e.previous]=collisionAvoidanceController(ego,estimate,road,e.cfg,e.previous);
        u=command.actuatorInput;e.held=u;
        % Controller is FLU, the VDB template is SAE FRD.
        delta=-u(1);L=e.cfg.vehicle.wheelbase;track=1.922;
        angles=[atan2(L*sin(delta),L*cos(delta)+track/2*sin(delta));atan2(L*sin(delta),L*cos(delta)-track/2*sin(delta));0;0];
        force=u(2)*min(e.cfg.tire.frictionCoefficient)*e.cfg.vehicle.m*9.81;radius=e.radius;
        torque=[max(force,0)*radius/2;max(force,0)*radius/2;0;0];
        pressure=max(-force,0)*radius/4/(.2*pi*.05^2*.177*2/4);
        e.command=[angles;torque;repmat(pressure,4,1)];
        entry=struct('time',t,'carlaFrame',live.frame,'truth',x,'target',q,'input',u,'slack',prediction.metadata.predictiveBarrierValue,'enclosed',audit.truthEnclosure);
        e.trace=[e.trace;entry];
    catch exception
        e.failedTruth=x;e.failureTime=t;
        e.failure=getReport(exception,'extended','hyperlinks','off');
        set_param(bdroot(b.BlockHandle),'SimulationCommand','stop');
    end
    e.lastTime=t;pdcaVdbExperiment=e;b.OutputPort(1).Data=e.command;
end
function target=localTarget(q)
    v=q(4)*[cos(q(3)+q(6));sin(q(3)+q(6))];r=q(4)*sin(q(6))/q(7);
    target=struct('targetPositionInertial',q(1:2),'targetVelocityInertial',v,'targetYawInertial',q(3),'targetYawRate',r,'targetSideslip',q(6),'targetTangentialAcceleration',q(5),'targetRearAxleDistance',q(7),'targetAccelerationInertial',q(5)*[cos(q(3)+q(6));sin(q(3)+q(6))]+r*[-v(2);v(1)],'targetLength',2*q(8),'targetWidth',2*q(9),'targetRectangleOffset',q(10:11));
end
