function rows = ablateFrame(capture,cfg,frameRecord,variants)
%ablateFrame Rebuild one captured controller call and solve modified copies.
% The original problem is rebuilt with the copied local functions (rtiInternals)
% and must reproduce the issued input exactly. Each variant changes only the
% stated bounds or cones and repeats the same two lexicographic solves.
    if nargin<4
        variants=["original","noSteerTrustFirst","noInputTrustFirst","noStateTrust", ...
            "noInputTrust","noTrust","noEndpointCone","noStateTrustNoEndpoint"];
    end
    model0=capture.model;
    if isfield(model0,'linearization'),model0=rmfield(model0,'linearization');end
    if isstruct(capture.previousState),model0.terminal=capture.previousState.terminal;
    else,model0.terminal=terminalContinuation.build(cfg,0);
    end
    [anchor,source,failure]=rtiInternals('initialization',model0,capture.previousState);
    [reproduced,search]=rtiInternals('step',anchor,model0,source,tic);
    exact=~isempty(reproduced) && isequal(reproduced.inputs(:,1),frameRecord.input);
    [problem,~]=rtiInternals('formulate',anchor,model0);
    h=cfg.controller.sampleTime;x0=model0.initialState;
    ix=problem.stateIndices;iu=problem.inputIndices;
    [low,high]=localStateLimits(cfg);
    inputLower=[-Inf;max(-1+1e-8,cfg.actuation.brakingRatioMinimum)];
    inputUpper=[Inf;min(1-1e-8,cfg.actuation.brakingRatioMaximum)];
    rows=struct([]);
    for variant=variants
        p=problem;
        switch variant
            case "original"
            case "noSteerTrustFirst"
                p.lower(iu(1,1))=-Inf;p.upper(iu(1,1))=Inf;
            case "noInputTrustFirst"
                p=localFreeInputs(p,anchor,1,inputLower,inputUpper);
            case "noStateTrust"
                p=localFreeStates(p,anchor,low,high);
            case "noInputTrust"
                p=localFreeInputs(p,anchor,1:size(iu,2),inputLower,inputUpper);
            case "noTrust"
                p=localFreeInputs(localFreeStates(p,anchor,low,high),anchor,1:size(iu,2),inputLower,inputUpper);
            case "noEndpointCone"
                p.cones=p.cones(2);
            case "noStateTrustNoEndpoint"
                p=localFreeStates(p,anchor,low,high);p.cones=p.cones(2);
            case "noTailCollisionRows"
                p=localDropRows(p,ismember(p.rowLabels(:,1),[2,3]) & p.rowLabels(:,2)>cfg.controller.horizonSteps);
            case "noTerminalClearanceRows"
                p=localDropRows(p,ismember(p.rowLabels(:,1),[5,6]));
            case "noMidpointStateRows"
                p=localDropRows(p,p.rowLabels(:,1)==4);
            case "futureFixedAtAnchor"
                p.lower(iu(:,2:end))=0;p.upper(iu(:,2:end))=0;
            case "futureSteerUpperEdge"
                p.lower(iu(1,2:end))=p.upper(iu(1,2:end));
            case "futureSteerLowerEdge"
                p.upper(iu(1,2:end))=p.lower(iu(1,2:end));
            case "noTerminal"
                p=localDropRows(p,ismember(p.rowLabels(:,1),[5,6]));p.cones=p.cones(2);
            otherwise,error('ablateFrame:variant','Unknown variant %s.',variant);
        end
        [z,flags,optimum,rho]=localTwoStage(p,cfg);
        row=struct('variant',variant,'time',capture.time,'exactReproduction',exact,'initialization',source, ...
            'initializationFailure',failure,'flags',flags,'pcbfOptimum',optimum,'clfSlack',rho, ...
            'u0',NaN(2,1),'u1',NaN(2,1),'u0MinusAnchor',NaN(2,1),'u1MinusAnchor',NaN(2,1), ...
            'V0',problem.initialClfValue,'V1affine',NaN,'V1actual',NaN,'holdClearance',NaN, ...
            'stage1Slacks',NaN,'maxStateTrustUse',NaN,'futureSteerSum',NaN,'endHeadingShift',NaN);
        if ~isempty(z)
            u=anchor.inputs+reshape(z(iu),size(anchor.inputs));
            row.u0=u(:,1);row.u1=u(:,2);row.u0MinusAnchor=u(:,1)-anchor.inputs(:,1);row.u1MinusAnchor=u(:,2)-anchor.inputs(:,2);
            row.V1affine=norm(problem.clfMap*z+problem.clfOffset)^2*problem.clfScale;
            [row.V1actual,row.holdClearance]=localActual(x0,u(:,1),model0,cfg,h);
            row.stage1Slacks=sum(max(0,z(problem.slackIndices)));
            du=reshape(z(iu),2,[]);row.futureSteerSum=sum(du(1,2:end));dxs=reshape(z(ix),6,[]);row.endHeadingShift=dxs(3,end);
            dx=reshape(z(ix),6,[]);row.maxStateTrustUse=max(max(abs(dx(:,2:end))./(cfg.nonlinear.trustRadius*[5;5;.5;5;3;1.5]),[],2));
        end
        if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
    end
end

function p=localFreeInputs(p,anchor,stages,inputLower,inputUpper)
    iu=p.inputIndices;
    for k=stages
        p.lower(iu(:,k))=inputLower-anchor.inputs(:,k);p.upper(iu(:,k))=inputUpper-anchor.inputs(:,k);
    end
end

function p=localDropRows(p,drop)
    p.a=p.a(~drop,:);p.b=p.b(~drop);p.rowLabels=p.rowLabels(~drop,:);
end

function p=localFreeStates(p,anchor,low,high)
    ix=p.stateIndices;
    p.lower(ix(:,2:end))=-Inf;p.upper(ix(:,2:end))=Inf;
    p.lower(ix(4:6,2:end))=low-anchor.states(4:6,2:end);p.upper(ix(4:6,2:end))=high-anchor.states(4:6,2:end);
    % The initial state is fixed by equality; keep its bounds as built.
end

function [z,flags,optimum,rho]=localTwoStage(p,cfg)
    options=optimoptions('coneprog','Display','none','MaxIterations',cfg.solver.maxIterations, ...
        'ConstraintTolerance',cfg.solver.constraintTolerance,'OptimalityTolerance',cfg.solver.optimalityTolerance, ...
        'MaxTime',cfg.solver.timeLimitSeconds);
    [first,~,f1]=coneprog(p.safetyObjective,p.cones,p.a,p.b,p.equal,p.rhs,p.lower,p.upper,options);
    z=[];flags=f1;optimum=NaN;rho=NaN;
    if isempty(first) || any(~isfinite(first)),return;end
    optimum=sum(max(0,first(p.slackIndices)));
    a=[p.a;p.safetyObjective.'];b=[p.b;optimum+cfg.solver.lexicographicTieTolerance];
    objective=zeros(size(p.safetyObjective));objective(p.clfIndex)=1;
    [second,~,f2]=coneprog(objective,p.cones,a,b,p.equal,p.rhs,p.lower,p.upper,options);
    flags=[f1,f2];
    if isempty(second) || any(~isfinite(second)),return;end
    z=second;rho=p.clfScale*max(0,second(p.clfIndex));
end

function [value,clearance]=localActual(x0,u,model,cfg,h)
    value=NaN;clearance=NaN;
    try
        [~,s]=ode45(@(~,y)nonlinearBicycleModel.derivative(y,u,cfg),linspace(0,h,31),x0,odeset('RelTol',1e-11,'AbsTol',1e-12));
    catch
        return;
    end
    e=nonlinearBicycleModel.error(s(end,:).',model.lane,model.nominalReference);
    value=norm(model.nominalReference.factor*e)^2;
    if ~isempty(model.target)
        shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];clearance=Inf;
        for j=1:size(s,1)
            q=predictiveSafetyGeometry.targetFlow(model.targetEpoch,model.sampleIndex*h+(j-1)*h/30);
            clearance=min(clearance,predictiveSafetyGeometry.rectangle(s(j,1:3).',shape,q(1:3),q(8:11)));
        end
    end
end

function [low,high]=localStateLimits(cfg)
    low=[max(cfg.model.speedMinimum,cfg.model.scheduleSpeedFloor+1e-4);-cfg.model.lateralVelocityMaximum;-cfg.model.yawRateMaximum];
    high=[cfg.model.speedMaximum;cfg.model.lateralVelocityMaximum;cfg.model.yawRateMaximum];
end
