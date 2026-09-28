classdef nonlinearSafetyCertificate
    %nonlinearSafetyCertificate Exact footprints and validated nonlinear witnesses.
    methods (Static)
        function q = target(observation,raw,cfg)
            q=zeros(10,0);if isempty(observation),return;end
            headingKnown=false;
            for name=["targetYawInertial","targetHeadingInertial","targetYawRelative", ...
                    "targetYawAngle","targetYaw","yawAngle","yaw","heading","relativeYaw","relativeHeading"]
                headingKnown=headingKnown || (isfield(raw,name) && ~isempty(raw.(name)));
            end
            if ~headingKnown
                error('collisionAvoidanceController:missingBodyHeading', ...
                    'A rotating target rectangle requires an explicit body heading, separate from its velocity course.');
            end
            errors=[observation.positionErrorBound;observation.velocityErrorBound; ...
                observation.accelerationErrorBound;observation.yawErrorBound;observation.yawRateErrorBound];
            if any(errors~=0)
                error('collisionAvoidanceController:uncertainNonlinearTarget', ...
                    'This exact-model controller requires zero target estimation bounds.');
            end
            if ~isempty(observation.parameterErrorBounds)
                bounds=observation.parameterErrorBounds;
                if any([bounds.speed,bounds.course,bounds.speedRate]~=0) || diff(bounds.curvature)~=0
                    error('collisionAvoidanceController:uncertainNonlinearTarget', ...
                        'Nonzero or unspecified NRMM parameter bounds require a robust terminal proof.');
                end
            end
            speed=norm(observation.velocity);beta=0;
            if speed>0,beta=atan2(sin(atan2(observation.velocity(2),observation.velocity(1))-observation.yaw), ...
                    cos(atan2(observation.velocity(2),observation.velocity(1))-observation.yaw));end
            lr=cfg.nonlinear.targetRearLength;offset=zeros(2,1);
            if isfield(raw,'targetRearLength'),lr=raw.targetRearLength;end
            if isfield(raw,'targetRectangleOffset'),offset=raw.targetRectangleOffset;end
            validateattributes(lr,{'double'},{'scalar','real','finite','positive'});
            validateattributes(offset,{'double'},{'size',[2,1],'real','finite'});
            if isfield(raw,'targetSideslip')
                validateattributes(raw.targetSideslip,{'double'},{'scalar','real','finite'});
                if speed>0 && abs(atan2(sin(beta-raw.targetSideslip),cos(beta-raw.targetSideslip)))>1e-10
                    error('collisionAvoidanceController:inconsistentNrmm','Velocity course and supplied sideslip disagree.');
                end
                beta=raw.targetSideslip;
            end
            omega=speed*sin(beta)/lr;
            if abs(omega-observation.yawRate)>1e-10*max(1,abs(omega))
                error('collisionAvoidanceController:inconsistentNrmm', ...
                    'Literal constant-parameter NRMM requires yawRate = speed*sin(sideslip)/rearLength.');
            end
            expectedAcceleration=omega*[-observation.velocity(2);observation.velocity(1)];
            if norm(observation.acceleration-expectedAcceleration,inf)>1e-9*max(1,norm(expectedAcceleration,inf))
                error('collisionAvoidanceController:inconsistentNrmm', ...
                    'Constant-speed NRMM acceleration must equal yawRate*J*velocity.');
            end
            q=[observation.position;observation.yaw;speed;beta;omega;observation.length/2;observation.width/2;offset];
        end

        function next = targetFlow(q,time)
            if isempty(q),next=q;return;end
            a=q(6)*time/2;scale=1;
            if a~=0,scale=sin(a)/a;end
            next=q;next(1:2)=q(1:2)+q(4)*time*scale*[cos(q(3)+q(5)+a);sin(q(3)+q(5)+a)];
            next(3)=q(3)+q(6)*time;
        end

        function [distance,certificate] = rectangle(poseE,shapeE,poseT,shapeT)
            re=[cos(poseE(3)),-sin(poseE(3));sin(poseE(3)),cos(poseE(3))];
            rt=[cos(poseT(3)),-sin(poseT(3));sin(poseT(3)),cos(poseT(3))];
            signs=[-1,1,1,-1;-1,-1,1,1];
            e=poseE(1:2)+re*(shapeE(3:4)+shapeE(1:2).*signs);
            t=poseT(1:2)+rt*(shapeT(3:4)+shapeT(1:2).*signs);
            axes=[re,rt];separated=false;
            for j=1:4
                pe=axes(:,j).'*e;pt=axes(:,j).'*t;
                separated=separated || min(pe)>max(pt) || min(pt)>max(pe);
            end
            distance=0;normal=zeros(2,1);
            if separated
                distance=Inf;
                for j=1:4
                    for k=1:4
                        [d,p]=localPointSegment(e(:,j),t(:,k),t(:,mod(k,4)+1));
                        if d<distance,distance=d;normal=(e(:,j)-p)/d;end
                        [d,p]=localPointSegment(t(:,j),e(:,k),e(:,mod(k,4)+1));
                        if d<distance,distance=d;normal=(p-t(:,j))/d;end
                    end
                end
            end
            sE=-re.'*normal;sT=rt.'*normal;
            mu=[max(sE,0);max(-sE,0)];lambda=[max(sT,0);max(-sT,0)];
            hE=[shapeE(1:2)+shapeE(3:4);shapeE(1:2)-shapeE(3:4)];
            hT=[shapeT(1:2)+shapeT(3:4);shapeT(1:2)-shapeT(3:4)];
            certificate=struct('normal',normal,'mu',mu,'lambda',lambda, ...
                'value',normal.'*(poseE(1:2)-poseT(1:2))-hE.'*mu-hT.'*lambda, ...
                'normalConvention',"targetToEgo");
        end

        function frame = roadFrame(lane,road)
            if laneGeometry.isVaryingReference(lane)
                error('collisionAvoidanceController:uncertifiedReference', ...
                    'The built-in invariant backup requires a straight or constant-curvature reference.');
            end
            if ~isempty(road.boundaries)
                error('collisionAvoidanceController:uncertifiedRoadContinuation', ...
                    'Finite fitted boundaries require a separate invariant continuation proof; use a global lateralClearance corridor.');
            end
            if isfield(lane,'referenceCurve')
                curve=lane.referenceCurve;origin=curve.origin;heading=curve.heading;k=curve.curvature;
            else
                if any(abs(lane.segmentCurvature)>1e-12)
                    error('collisionAvoidanceController:uncertifiedReference','Polyline corners cannot define a smooth invariant backup.');
                end
                origin=lane.segmentStart(1,:).';heading=atan2(lane.tangent(1,2),lane.tangent(1,1));k=0;
            end
            clearance=road.lateralClearance;
            if isempty(clearance),clearance=[1e100;1e100];end
            clearance=clearance-road.lateralClearanceErrorBound;
            frame=[origin;heading;k;clearance];
        end

        function backup = backup(cfg,frame)
            persistent key saved
            synthesis=rmfield(cfg.nonlinear,{'maximumCertificateSeconds','maximumCertificateCells', ...
                'initialPlan','proposalFunction','maximumImprovementIterations','trustRadius'});
            current={cfg.vehicle,cfg.tire,cfg.roadLoad,cfg.model,cfg.actuation, ...
                cfg.referenceSpeed,cfg.controller.sampleTime,synthesis,cfg.clf,frame(4)};
            if isequaln(key,current),backup=saved;return;end
            reference=nonlinearBicycleModel.cruise(cfg,frame(4));radius=cfg.nonlinear.terminalRadius;
            gain=[zeros(2,1),reference.gain];backup=[];
            while radius>=cfg.nonlinear.minimumTerminalRadius
                widths=nonlinearSafetyMex('radii',reference.factor,radius);
                inlet=nonlinearSafetyMex('inlet',reference.state,widths);nominal=[reference.state;reference.input];
                try
                    sample=fialaCertificate.sample(inlet(:,1),inlet(:,2),nominal,gain, ...
                        zeros(6,1),reference.input,cfg,maximumCellDuration=cfg.nonlinear.certificateStep, ...
                        maximumComputationTime=cfg.nonlinear.maximumCertificateSeconds, ...
                        maximumCells=cfg.nonlinear.maximumCertificateCells,curvature=frame(4),inputRadius=1e-12*ones(2,1));
                    if sample.accepted
                        domains=cat(3,sample.cells.domain);
                        domain=[min(domains(:,1,:),[],3),max(domains(:,2,:),[],3)];
                        proof=nonlinearSafetyMex('terminalBound',nominal,reference.gain,reference.factor, ...
                            reference.continuousA,reference.continuousB,domain,cfg.controller.sampleTime, ...
                            fialaCertificate.parameters(cfg),frame(4),radius);
                        inputRadius=max(abs(sample.initialBox(7:8,:)-reference.input),[],2)+1e-12;
                        slew=[cfg.model.frontWheelSteeringRateMaximum;cfg.model.brakingRatioRateMaximum]*cfg.controller.sampleTime;
                        if proof(3)>cfg.nonlinear.clearanceReserve && all(2*inputRadius+1e-12<=slew) ...
                                && localPhysicalDomain(domain,cfg)
                            backup=struct('reference',reference,'radius',radius,'domain',domain, ...
                                'contractionUpper',proof(1),'remainderUpper',proof(2), ...
                                'invarianceMargin',proof(3),'inputRadius',inputRadius, ...
                                'proof',"directedFlowAndQuadraticRemainder");
                            break;
                        end
                    end
                catch exception
                    if ~startsWith(exception.identifier,'collisionAvoidanceController:'),rethrow(exception);end
                end
                radius=radius/2;
            end
            if isempty(backup)
                error('collisionAvoidanceController:noInvariantBackup','No invariant cruise ball was certified for these parameters.');
            end
            key=current;saved=backup;
        end

        function [accepted,details] = terminal(box,previous,q,time,backup,frame,shape,cfg)
            accepted=false;details=struct('normUpper',Inf,'tailMargin',-Inf,'inputMemoryValid',false);
            try
                errors=nonlinearSafetyMex('error',box,frame,backup.reference.state);
                normBounds=nonlinearSafetyMex('norm',errors,backup.reference.factor);
                margin=nonlinearSafetyMex('tail',box,backup.domain,shape,q,frame, ...
                    cfg.collision.safetyMarginMeters+cfg.nonlinear.clearanceReserve,time);
                memory=nonlinearSafetyMex('inputBoxContains',previous, ...
                    repmat(backup.reference.input,1,size(previous,2)),repmat(backup.inputRadius,1,size(previous,2)));
                details=struct('normUpper',normBounds(2),'tailMargin',margin,'inputMemoryValid',memory);
                accepted=normBounds(2)<=backup.radius && margin>=0 && memory;
            catch exception
                if ~startsWith(exception.identifier,'collisionAvoidanceController:'),rethrow(exception);end
            end
        end

        function certificate = plan(x,inputs,previous,q,lane,frame,backup,cfg,targetStep,policy)
            if nargin<9,targetStep=0;end
            if nargin<10,policy=[];end
            certificate=struct('accepted',false,'reason',"unverified",'inputs',inputs, ...
                'states',x,'samples',{{}},'minimumCollisionMargin',Inf,'minimumRoadMargin',Inf, ...
                'terminal',[],'clfSlack',0,'clfAvailable',false,'clfInitialValue',0,'clfNextValueUpper',0, ...
                'checkedStage',0,'checkedTime',0,'checkedBox',zeros(8,2),'feedbackGains',zeros(2,6,size(inputs,2)), ...
                'referenceStates',zeros(6,size(inputs,2)),'clfResidualUpper',0,'inputRadius',zeros(size(inputs)), ...
                'inherited',false,'terminalOnly',false);
            if isempty(inputs) || size(inputs,1)~=2 || any(~isfinite(inputs),'all'),return;end
            shape=[cfg.vehicle.length/2;cfg.vehicle.width/2;cfg.vehicle.rectangleOffset];
            inlet=[];box=[x,x];states=zeros(6,size(inputs,2)+1);states(:,1)=x;samples=cell(1,size(inputs,2));
            timer=tic;
            try
                if isempty(policy)
                    nominalStates=nonlinearBicycleModel.rollout(x,inputs,cfg);
                else
                    validateattributes(policy.referenceStates,{'double'},{'size',[6,size(inputs,2)],'finite','real'});
                    validateattributes(policy.feedbackGains,{'double'},{'size',[2,6,size(inputs,2)],'finite','real'});
                    validateattributes(policy.inputRadius,{'double'},{'size',size(inputs),'finite','real','nonnegative'});
                    if any(policy.feedbackGains(:,:,1),'all')
                        error('collisionAvoidanceController:invalidNonlinearPolicy','The first feedback gain must be zero.');
                    end
                    nominalStates=policy.referenceStates;certificate.inputRadius=policy.inputRadius;
                end
                certificate.referenceStates=nominalStates(:,1:size(inputs,2));
                initialError=localClfError(box,frame,backup.reference.state);
                for index=1:size(inputs,2)
                    certificate.checkedStage=index;
                    remaining=cfg.nonlinear.maximumCertificateSeconds-toc(timer);
                    if remaining<=0,certificate.reason="certificateTimeBudget";return;end
                    gain=zeros(2,6);
                    if ~isempty(policy)
                        gain=policy.feedbackGains(:,:,index);
                    elseif index>1 && cfg.feedbackPrediction.enabled
                        projection=laneGeometry.project(nominalStates(1:2,index),lane);
                        tangent=[cos(projection.heading);sin(projection.heading)];normal=[-tangent(2);tangent(1)];
                        jacobian=zeros(5,6);jacobian(1,1:2)=normal.';jacobian(2,3)=1;
                        jacobian(2,1:2)=-frame(4)*tangent.'/(1-frame(4)*projection.lateralPosition);
                        jacobian(3:5,4:6)=eye(3);gain=backup.reference.gain*jacobian;
                    end
                    certificate.feedbackGains(:,:,index)=gain;
                    sample=fialaCertificate.sample(box(:,1),box(:,2),[nominalStates(:,index);inputs(:,index)], ...
                        gain,zeros(6,1),previous,cfg,inlet=inlet, ...
                        inputRadius=certificate.inputRadius(:,index)+double(index>1)*1e-12, ...
                        maximumCellDuration=cfg.nonlinear.certificateStep, ...
                        maximumCells=cfg.nonlinear.maximumCertificateCells,maximumComputationTime=remaining);
                    if ~sample.accepted,certificate.reason="flow:"+string(sample.reason);return;end
                    for cellIndex=1:numel(sample.cells)
                        tubeCell=sample.cells(cellIndex);domain=tubeCell.swept;
                        certificate.checkedBox=domain;
                        if ~localPhysicalDomain(domain,cfg),certificate.reason="physicalDomain";return;end
                        time=[(targetStep+index-1)*cfg.controller.sampleTime+tubeCell.start; ...
                            (targetStep+index-1)*cfg.controller.sampleTime+tubeCell.end];
                        certificate.checkedTime=time(2);
                        normal=[1;0];
                        if ~isempty(q)
                            target=nonlinearSafetyCertificate.targetFlow(q,mean(time));
                            [~,geometry]=nonlinearSafetyCertificate.rectangle(mean(domain(1:3,:),2),shape,target(1:3),target(7:10));
                            normal=geometry.normal;if ~any(normal),certificate.reason="rectangleOverlap";return;end
                        end
                        certifiedTime=[targetStep+index-1;cfg.controller.sampleTime;tubeCell.start;tubeCell.end];
                        margins=nonlinearSafetyMex('geometry',domain(1:6,:),shape,q,certifiedTime,normal,frame,zeros(0,12), ...
                            cfg.collision.safetyMarginMeters+cfg.nonlinear.clearanceReserve);
                        certificate.minimumCollisionMargin=min(certificate.minimumCollisionMargin,margins(1));
                        certificate.minimumRoadMargin=min(certificate.minimumRoadMargin,margins(2));
                        if any(margins<0),certificate.reason="sweptGeometry";return;end
                    end
                    box=sample.endpoint(1:6,:);states(:,index+1)=sample.center(1:6);samples{index}=sample;
                    if index==1
                        nextError=localClfError(box,frame,backup.reference.state);
                        if ~isempty(initialError) && ~isempty(nextError)
                            clf=nonlinearSafetyMex('clf',initialError,nextError,backup.reference.factor,cfg.nonlinear.clfDecay);
                            certificate.clfInitialValue=clf(1);certificate.clfNextValueUpper=clf(2);certificate.clfSlack=clf(3);
                            certificate.clfResidualUpper=clf(4);
                            certificate.clfAvailable=true;
                        end
                    end
                    inlet=sample;previous=inputs(:,index);
                end
                [accepted,terminal]=nonlinearSafetyCertificate.terminal(box,sample.endpoint(7:8,:),q, ...
                    [targetStep+size(inputs,2);cfg.controller.sampleTime],backup,frame,shape,cfg);
                certificate.terminal=terminal;certificate.states=states;certificate.samples=samples;
                certificate.accepted=accepted;certificate.reason="terminal";
                if accepted,certificate.reason="certified";end
            catch exception
                if ~startsWith(exception.identifier,'collisionAvoidanceController:'),rethrow(exception);end
                certificate.reason=string(exception.identifier)+": "+string(exception.message);
            end
        end
    end
end

function errors=localClfError(box,frame,reference)
    errors=[];
    try
        errors=nonlinearSafetyMex('error',box,frame,reference);
    catch exception
        if ~strcmp(exception.identifier,'collisionAvoidanceController:invalidNonlinearCertificate'),rethrow(exception);end
        % An unavailable local CLF never invalidates an otherwise safe plan.
    end
end

function [distance,closest]=localPointSegment(p,a,b)
    d=b-a;t=min(1,max(0,d.'*(p-a)/(d.'*d)));closest=a+t*d;distance=norm(p-closest);
end

function valid=localPhysicalDomain(domain,cfg)
    valid=domain(4,1)>cfg.model.scheduleSpeedFloor && domain(4,1)>=cfg.model.speedMinimum ...
        && domain(4,2)<=cfg.model.speedMaximum ...
        && max(abs(domain(5,:)))<=cfg.model.lateralVelocityMaximum ...
        && max(abs(domain(6,:)))<=cfg.model.yawRateMaximum;
end
