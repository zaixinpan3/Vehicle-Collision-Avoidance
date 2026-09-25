classdef nrmmTruthFixture
    %nrmmTruthFixture Exact NRMM truth targets for controller tests.
    % A truth is a struct with fields p0 (2-by-1), speed, course, A (speed-rate),
    % kappa (curvature) and yaw0; its state at time t is an [p; v; a; yaw; yaw
    % rate] column computed by the harness's nrmmTargetTruth (scripts/). The
    % samplers keep A and kappa inside the encounter's parameter intervals when
    % it carries them (targetPrediction.nrmmParameters), otherwise inside the
    % contract's curvature maximum. These are declared-model fixtures, not
    % physical data.
    methods (Static)
        function truth = sampleBox(encounter,stream,extreme)
        % NRMM parameters whose state at t = 0 lies in the admitted box; extreme
        % draws put the position, velocity, yaw and yaw rate at box vertices
        % (uniform draws after 200 vertices without an NRMM-consistent
        % acceleration).
            x=encounter.center;r=encounter.radius;
            for attempt=1:50000
                vertex=extreme && attempt<=200;
                if vertex,u=sign(randn(stream,8,1));else,u=2*rand(stream,8,1)-1;end
                p0=x(1:2)+r(1:2).*u(1:2);v=x(3:4)+r(3:4).*u(3:4);speed=norm(v);
                if speed==0,continue;end
                kappa=(x(8)+r(8)*u(8))/speed;
                if ~localInside(kappa,localInterval(encounter,"curvature")),continue;end
                course=atan2(v(2),v(1));tangent=[cos(course);sin(course)];
                normal=kappa*speed^2*[-tangent(2);tangent(1)];
                % Speed-rates A with A*tangent + normal inside the acceleration box.
                low=-Inf;high=Inf;feasible=true;
                for component=1:2
                    lower=x(4+component)-r(4+component)-normal(component);
                    upper=x(4+component)+r(4+component)-normal(component);
                    if abs(tangent(component))<1e-12
                        feasible=feasible && lower<=0 && upper>=0;
                    else
                        bounds=sort([lower,upper]/tangent(component));
                        low=max(low,bounds(1));high=min(high,bounds(2));
                    end
                end
                rateInterval=localInterval(encounter,"speedRate");
                low=max(low,rateInterval(1));high=min(high,rateInterval(2));
                if ~feasible || low>high,continue;end
                if vertex,A=low+(high-low)*(randn(stream)>0);else,A=low+(high-low)*rand(stream);end
                truth=struct("p0",p0,"speed",speed,"course",course,"A",A,"kappa",kappa, ...
                    "yaw0",x(7)+r(7)*u(7));
                return;
            end
            error("nrmmTruthFixture:noSample","No NRMM state found in the box.");
        end

        function truth = sampleDisc(encounter,velocityRadius,accelerationRadius,stream)
        % NRMM parameters whose velocity and acceleration at t = 0 lie in discs
        % around the estimate (and so inside its box) and whose yaw rate lies in
        % the box.
            x=encounter.center;r=encounter.radius;
            for attempt=1:50000
                p0=x(1:2)+r(1:2).*(2*rand(stream,2,1)-1);
                direction=2*pi*rand(stream);
                v=x(3:4)+velocityRadius*sqrt(rand(stream))*[cos(direction);sin(direction)];
                speed=norm(v);
                if speed==0,continue;end
                kappa=(x(8)+r(8)*(2*rand(stream)-1))/speed;
                if ~localInside(kappa,localInterval(encounter,"curvature")),continue;end
                course=atan2(v(2),v(1));tangent=[cos(course);sin(course)];normal=[-tangent(2);tangent(1)];
                % Speed-rates A with |A*tangent + kappa*speed^2*normal - a| <= accelerationRadius.
                offset=dot(x(5:6),normal)-kappa*speed^2;
                if abs(offset)>accelerationRadius,continue;end
                halfWidth=sqrt(accelerationRadius^2-offset^2);
                A=dot(x(5:6),tangent)+halfWidth*(2*rand(stream)-1);
                if ~localInside(A,localInterval(encounter,"speedRate")),continue;end
                truth=struct("p0",p0,"speed",speed,"course",course,"A",A,"kappa",kappa, ...
                    "yaw0",x(7)+r(7)*(2*rand(stream)-1));
                return;
            end
            error("nrmmTruthFixture:noSample","No NRMM state found in the discs.");
        end

        function state = state(truth,t)
        % Exact NRMM state [p; v; a; yaw; yaw rate] at time t (nrmmTargetTruth).
            tangent=[cos(truth.course);sin(truth.course)];normal=[-tangent(2);tangent(1)];
            center=[truth.p0;truth.speed*tangent;truth.A*tangent+truth.kappa*truth.speed^2*normal; ...
                truth.yaw0;truth.kappa*truth.speed];
            state=nrmmTargetTruth(struct("center",center,"speedRate",truth.A,"curvature",truth.kappa),t);
        end

        function value = unit(stream,count,vertex)
        % Uniform draws in [-1, 1], or random signs at the vertices.
            if vertex
                value=sign(randn(stream,count,1));
            else
                value=2*rand(stream,count,1)-1;
            end
        end

        function excess = sampledExcess(program,model,plan,seed)
        % Simulate the declared plant under u_k = v_k + K_k (xhat - z) + L_k (shat - s0)
        % + N_k (ahat - a0) with exact NRMM targets from the admitted box and per-hold
        % estimator errors (random and box vertices). Returns the largest excess of
        % the sampled joint deviation over the certified tube, of the relative
        % record positions over their supports and of the input deviation and slew
        % over their reserves; nonpositive values are inside.
            prediction=program.prediction;encounter=model.encounter;
            count=prediction.stageCount;h=model.sampleTime;
            nominal=zeros(6,count+1);
            for node=0:count
                nominal(:,node+1)=prediction.egoStateOffset(:,node+1)+prediction.egoStateMatrix(:,:,node+1)*plan(:);
            end
            targetNominal=prediction.targetNominal;
            egoBound=prediction.estimatorBound;targetBound=prediction.targetEstimatorBound;
            records=program.jointCertificate.records;angles=program.jointCertificate.angles;
            stream=RandStream("mt19937ar",Seed=seed);
            excess=struct("joint",-Inf,"record",-Inf,"input",-Inf,"slew",-Inf);
            for trial=1:60
                vertex=mod(trial,2)==0;
                x=nominal(:,1)+model.initialFrenetErrorBound.*nrmmTruthFixture.unit(stream,6,vertex);
                truth=nrmmTruthFixture.sampleBox(encounter,stream,vertex);
                previous=zeros(2,1);
                for stage=1:count
                    deviation=zeros(2,1);
                    if stage>1
                        egoEstimate=x+egoBound.*nrmmTruthFixture.unit(stream,6,vertex);
                        s=nrmmTruthFixture.state(truth,(stage-1)*h);
                        targetDeviation=s(1:6)+targetBound.*nrmmTruthFixture.unit(stream,6,vertex) ...
                            -targetNominal(:,stage);
                        deviation=prediction.feedbackGainSequence(:,:,stage)*(egoEstimate-nominal(:,stage)) ...
                            +prediction.targetGainSequence(:,:,stage)*targetDeviation(1:4) ...
                            +prediction.targetAccelerationGainSequence(:,:,stage)*targetDeviation(5:6);
                    end
                    excess.input=max(excess.input,max(abs(deviation)-prediction.feedbackInputSupport(:,stage)));
                    excess.slew=max(excess.slew,max(abs(deviation-previous)-prediction.feedbackSlewSupport(:,stage)));
                    previous=deviation;
                    x=prediction.stageMatrixA(:,:,stage)*x ...
                        +prediction.stageMatrixB(:,:,stage)*(plan(:,stage)+deviation)+prediction.stageAffine(:,stage);
                    s=nrmmTruthFixture.state(truth,stage*h);
                    tube=prediction.cells(stage);
                    joint=[x-nominal(:,stage+1);s(1:4)-targetNominal(1:4,stage+1)];
                    generators=[tube.generators;tube.targetGenerators];
                    for direction=[eye(10),-eye(10),randn(stream,10,8)]
                        excess.joint=max(excess.joint,direction.'*joint-sum(abs(direction.'*generators)));
                    end
                    for index=find([records.stage]==stage)
                        normal=[cos(angles(index));sin(angles(index))];
                        relative=records(index).positionMap*joint(1:6)-joint(7:8);
                        excess.record=max(excess.record,normal.'*relative-sum(abs(records(index).generators.'*normal)));
                    end
                end
            end
        end
    end
end

function interval = localInterval(encounter,name)
% The encounter's parameter interval, or the contract's cap when it carries none.
    if isfield(encounter,"parameters")
        interval=encounter.parameters.(name);
    elseif name=="curvature"
        interval=encounter.contract.curvatureMaximum*[-1;1];
    else
        interval=[-Inf;Inf];
    end
end

function inside = localInside(value,interval)
    inside=value>=interval(1) && value<=interval(2);
end
