function report = runTerminalCbfProofAudit(options)
%runTerminalCbfProofAudit Terminal CBF and Huang-style affine MPC proof audit.
% This is a reproducible mathematical-model audit, not a vehicle controller
% simulation. The safety slack is auxiliary and never authorizes a command.
    arguments
        options.OutputDirectory (1,1) string = ""
    end
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(fullfile(root,"controller"),fullfile(root,"config"));
    cfg = collisionAvoidanceControllerConfig(struct("referenceSpeed",0.5, ...
        "model",struct("speedMaximum",1), ...
        "controller",struct("sampleTime",.1,"horizonSteps",4)));
    % Evaluate the equations in TERMINAL_CBF_PROOF.md as a research example.
    % There is no terminal-set admission or controller fallback API here.
    terminal = localAffineExample(cfg,0);
    poseRows = [eye(3);-eye(3)];
    poseLimits = [10;2;.4;10;2;.4];
    barrier = localHalfspaces(terminal,poseRows,poseLimits);
    cfgInputGain = modifiedFialaTire.accelerationGain(cfg);
    h = .1;
    inputColumn = [zeros(3,1);cfgInputGain;0;0];
    held = expm(h*[terminal.continuousA,inputColumn;zeros(1,7)]);
    transition = held(1:6,1:6);
    inputMap = held(1:6,7);
    stageRows = [poseRows,zeros(6,3)]./poseLimits;
    velocityRows = [eye(3);-eye(3)]./[terminal.velocityLimit;terminal.velocityLimit];
    stageRows = [stageRows;zeros(6,3),velocityRows];
    stageBounds = ones(size(stageRows,1),1);
    generators = zeros(6,2);generators(1,1) = .01;generators(4,2) = .001;
    recovery = localRecede([0;0;0;2.5;0;0],generators,transition,inputMap, ...
        stageRows,stageBounds,barrier,cfg,4,5);
    safe = localRecede([0;0;0;.8;0;0],generators,transition,inputMap, ...
        stageRows,stageBounds,barrier,cfg,4,5);

    % Persistent forcing invalidates the old finite excursion budget, even
    % though the velocity stays below its terminal envelope.
    rate = .001;
    duration = 60;
    damping = -terminal.continuousA(4,4);
    initialVelocity = .01;
    attenuation = exp(-damping*duration);
    forcedPosition = initialVelocity*(1-attenuation)/damping ...
        +rate/damping*(duration-(1-attenuation)/damping);
    forcedState = [forcedPosition;0;0; ...
        initialVelocity*attenuation+rate/damping*(1-attenuation);0;0];
    shortLimits = [.2;2;.4;.2;2;.4];
    shortBarrier = localHalfspaces(terminal,poseRows,shortLimits);
    initialForcedValue = localValue(shortBarrier,[0;0;0;initialVelocity;0;0]);
    finalForcedValue = localValue(shortBarrier,forcedState);

    % A nonzero heading causes lateral transport in the nonlinear model,
    % whereas the old zero-speed affine transport has dDot=vy.
    narrowLimits = [10;.05;.4;10;.05;.4];
    narrowBarrier = localHalfspaces(terminal,poseRows,narrowLimits);
    initial = [0;0;.2;.8;0;0];
    parameters = modifiedFialaTire.parameters(cfg);
    nonlinear = ltvBicycleModel.nominalKernel(initial,zeros(2,1),.5,0,0,cfg,parameters,0,false);
    affine = expm(.5*terminal.continuousA)*initial;
    initialNonlinearValue = localValue(narrowBarrier,initial);
    affineValue = localValue(narrowBarrier,affine);
    nonlinearValue = localValue(narrowBarrier,nonlinear(:,end));
    boundaryState = initial;boundaryState(2) = .05;
    boundaryValue = localValue(narrowBarrier,boundaryState);
    unavoidableOutwardRate = boundaryState(4)*sin(boundaryState(3));

    % Rest does not discharge a moving obstacle, even over a short encounter.
    targetTime = [0;3];
    targetPosition = [10-2*targetTime,zeros(2,1)];
    separation = rectangleSeparationMargin(zeros(2,2),zeros(2,1), ...
        targetPosition,pi*ones(2,1),cfg.vehicle.length,cfg.vehicle.width,5,2);
    report = struct("auditPassed",false,"configuration",cfg,"sampleTime",h, ...
        "predictionSteps",4,"recovery",recovery,"safe",safe, ...
        "persistentForcing",struct("rate",rate,"duration",duration, ...
            "initialBarrier",initialForcedValue,"finalBarrier",finalForcedValue, ...
            "finalState",forcedState,"budgetDerivative",rate/damping), ...
        "nonlinearTransport",struct("duration",.5,"initialState",initial, ...
            "initialBarrier",initialNonlinearValue,"affineBarrier",affineValue, ...
            "nonlinearBarrier",nonlinearValue,"affineState",affine,"nonlinearState",nonlinear(:,end), ...
            "boundaryState",boundaryState,"boundaryBarrier",boundaryValue, ...
            "unavoidableOutwardRate",unavoidableOutwardRate), ...
        "movingTarget",struct("time",targetTime,"sampledSatMargin",separation, ...
            "egoTerminalBarrier",localValue(barrier,zeros(6,1))), ...
        "affineChecks",localAffineChecks(), ...
        "terminal",terminal,"barrier",barrier, ...
        "fullControllerCbfEstablished",false, ...
        "scope","Declared affine terminal theorem, restricted auxiliary LP and counterexamples only; no online terminal integration or realtime qualification");
    affineChecks = report.affineChecks;
    report.auditPassed = all([affineChecks.minimumBarrierIncrement]>=-1e-12) ...
        && all([affineChecks.supportIdentityError]<1e-12) ...
        && recovery.maximumDecreaseViolation<1e-7 ...
        && recovery.maximumShiftViolation<1e-7 && recovery.values(1)>0 ...
        && recovery.values(end)<1e-7 && max(abs(safe.values))<1e-7 ...
        && safe.maximumShiftViolation<1e-7 && initialForcedValue>0 && finalForcedValue<0 ...
        && initialNonlinearValue>0 && affineValue>=initialNonlinearValue-1e-12 ...
        && nonlinearValue<0 && boundaryValue==0 && unavoidableOutwardRate>0 ...
        && separation(1)>0 && separation(end)<0;
    if strlength(options.OutputDirectory)>0
        if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
        save(fullfile(options.OutputDirectory,"terminal-cbf-audit.mat"),"report");
        fid = fopen(fullfile(options.OutputDirectory,"terminal-cbf-audit.json"),'w');
        cleanup = onCleanup(@() fclose(fid));
        fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
    end
    fprintf('Terminal CBF audit passed=%d; complete online controller proof=%d\n', ...
        report.auditPassed,report.fullControllerCbfEstablished);
end

function result = localRecede(center,generators,a,b,rows,bounds,barrier,cfg,horizon,steps)
    result = struct("values",zeros(steps,1),"headSlack",zeros(steps,1), ...
        "input",zeros(steps,1),"states",zeros(6,steps+1), ...
        "maximumShiftViolation",0,"maximumDecreaseViolation",0, ...
        "terminalMargins",zeros(steps,1));
    result.states(:,1) = center;
    priorValue = inf;priorSlack = 0;
    for index = 1:steps
        solve = localAuxiliaryLp(center,generators,a,b,rows,bounds,barrier,cfg,horizon);
        result.values(index) = solve.value;result.headSlack(index) = solve.slack(1);
        result.input(index) = solve.input(1);
        result.maximumDecreaseViolation = max(result.maximumDecreaseViolation,solve.value-priorValue+priorSlack);
        center = a*center+b*solve.input(1);generators = a*generators;
        shifted = [solve.input(2:end);0];
        witness = localWitness(center,generators,shifted,a,b,rows,bounds,barrier);
        result.maximumShiftViolation = max([result.maximumShiftViolation,witness.hardViolation, ...
            witness.cost-(solve.value-solve.slack(1))]);
        result.terminalMargins(index) = witness.terminalValue;
        result.states(:,index+1) = center;
        priorValue = solve.value;priorSlack = solve.slack(1);
    end
end

function solve = localAuxiliaryLp(center,generators,a,b,rows,bounds,barrier,cfg,count)
% Huang et al. Eq. (8), restricted to the exact longitudinal subsystem.
% Steering and lateral/yaw states are zero. Only this auxiliary problem may
% soften state constraints; its positive-value actions are not executable
% commands of collisionAvoidanceController.
    map = zeros(6,count);
    matrix = zeros(0,2*count);limit = zeros(0,1);
    for stage = 1:count
        slackRow = zeros(size(rows,1),count);slackRow(:,stage) = -1;
        matrix = [matrix;rows*map,slackRow;barrier.domainRow*map,zeros(1,count)]; %#ok<AGROW>
        limit = [limit;bounds-rows*center-sum(abs(rows*generators),2); ...
            -barrier.domainRow*center-sum(abs(barrier.domainRow*generators),2)]; %#ok<AGROW>
        map = a*map;map(:,stage) = map(:,stage)+b;
        center = a*center;generators = a*generators;
    end
    terminalRows = [barrier.matrix;barrier.domainRow];
    matrix = [matrix;terminalRows*map,zeros(size(terminalRows,1),count)];
    terminalBound = [barrier.bound;barrier.domainBound]-terminalRows*center ...
        -sum(abs(terminalRows*generators),2);
    limit = [limit;terminalBound];
    cost = [zeros(count,1);ones(count,1)];
    lower = [cfg.actuation.brakingRatioMinimum*ones(count,1);zeros(count,1)];
    upper = [zeros(count,1);inf(count,1)];
    options = optimoptions('linprog','Display','none');
    [decision,value,exitFlag] = linprog(cost,matrix,limit,[],[],lower,upper,options);
    if exitFlag<=0 || max([matrix*decision-limit;lower-decision;decision-upper])>1e-7
        error("runTerminalCbfProofAudit:invalidAuxiliarySolution","The auxiliary proof LP did not pass verification.");
    end
    solve = struct("value",value,"input",decision(1:count),"slack",decision(count+1:end));
end

function witness = localWitness(center,generators,input,a,b,rows,bounds,barrier)
    cost = 0;hardViolation = 0;
    for stage = 1:numel(input)
        cost = cost+max([0;rows*center+sum(abs(rows*generators),2)-bounds]);
        hardViolation = max(hardViolation,barrier.domainRow*center ...
            +sum(abs(barrier.domainRow*generators),2));
        center = a*center+b*input(stage);generators = a*generators;
    end
    [terminalValue,membership] = localValue(barrier,center,generators);
    hardViolation = max([hardViolation,-terminalValue,-membership.domainMargin]);
    witness = struct("cost",cost,"hardViolation",hardViolation,"terminalValue",terminalValue);
end

function example = localAffineExample(cfg,curvature)
% Fixed numerical example of Eq. (2), not an online certificate constructor.
    a = ltvBicycleModel.continuousMatrices(curvature,0,cfg,0,0);
    f = a(4:6,4:6);
    c = abs(f);c(1:4:end) = diag(f);
    direction = (-c)\ones(3,1);
    assert(max(real(eig(c)))<0 && all(direction>0));
    q = direction*cfg.model.speedMaximum/direction(1);
    example = struct("continuousA",a,"velocityComparison",c, ...
        "poseExcursionMatrix",abs(a(1:3,4:6))/(-c),"velocityLimit",q);
end

function barrier = localHalfspaces(example,poseRows,poseLimits)
% Exact finite sign expansion of Eq. (3) for this audit's centered boxes.
    signs = 2*double(dec2bin(0:7,3)-'0')-1;
    rows = repelem(poseRows,8,1);
    weights = repelem(abs(poseRows)*example.poseExcursionMatrix,8,1);
    scales = repelem(poseLimits,8);
    velocityRows = [eye(3);-eye(3)]./[example.velocityLimit;example.velocityLimit];
    barrier = struct("matrix",[[rows,weights.*repmat(signs,size(poseRows,1),1)]./scales; ...
        zeros(6,3),velocityRows],"bound",ones(8*numel(poseLimits)+6,1), ...
        "domainRow",[0,0,0,-1,0,0],"domainBound",0, ...
        "poseRows",poseRows,"poseLimits",poseLimits,"terminal",example);
end

function [value,membership] = localValue(barrier,center,generators)
% Eq. (7); signed generators retain affine correlations.
    if nargin<3,generators = zeros(6,0);end
    value = min(barrier.bound-barrier.matrix*center-sum(abs(barrier.matrix*generators),2));
    domainMargin = -barrier.domainRow*center-sum(abs(barrier.domainRow*generators),2);
    membership = struct("domainMargin",domainMargin,"inDomain",domainMargin>=0, ...
        "inTerminalSet",domainMargin>=0 && value>=0);
end

function checks = localAffineChecks()
    cfg = collisionAvoidanceControllerConfig();
    curvatures = [0,1/400,1/100,-1/100];
    rows = [eye(3);-eye(3)];limits = [100;2;.4;100;2;.4];
    times = [0,.001,.01,.1,1,5];
    entries = cell(size(curvatures));
    for index = 1:numel(curvatures)
        example = localAffineExample(cfg,curvatures(index));
        barrier = localHalfspaces(example,rows,limits);
        q = example.velocityLimit;
        center = [0;.1;.05;.1*q(1);.3*q(2);-.2*q(3)];
        generators = diag([.01;.02;.005;.03*q(1);.2*q(2);.15*q(3)]);
        values = zeros(size(times));
        for sample = 1:numel(times)
            flow = expm(times(sample)*example.continuousA);
            values(sample) = localValue(barrier,flow*center,flow*generators);
        end
        vertices = center+generators*(2*double(dec2bin(0:63,6).'-'0')-1);
        direct = min([(limits-rows*vertices(1:3,:) ...
            -abs(rows)*example.poseExcursionMatrix*abs(vertices(4:6,:)))./limits; ...
            1-max(abs(vertices(4:6,:))./q,[],1)],[],"all");
        [~,reverseMembership] = localValue(barrier,[0;0;0;-.01;0;0]);
        entries{index} = struct("curvature",curvatures(index),"time",times, ...
            "barrierValues",values,"minimumBarrierIncrement",min(diff(values)), ...
            "supportIdentityError",abs(direct-values(1)), ...
            "comparisonSpectralAbscissa",max(real(eig(example.velocityComparison))), ...
            "reverseSpeedInDomain",reverseMembership.inDomain);
    end
    checks = [entries{:}];
end
