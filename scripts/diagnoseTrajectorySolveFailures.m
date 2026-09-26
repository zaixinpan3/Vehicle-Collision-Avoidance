function summary = diagnoseTrajectorySolveFailures(snapshotDirectory, outputDirectory)
%diagnoseTrajectorySolveFailures Audit saved rejected affine planning frames.
% Rebuild previous/fluid fixed-direction cones, remove selected constraint
% groups for diagnosis only, and independently bound inconsistent groups with
% necessary LP relaxations. Compare previously admitted tire tangents and
% plans with the same nonlinear Fiala/bicycle model. No command is issued.
% Requires saved straight-replay.mat and sCurve-replay.mat snapshots from the
% trajectory-feedback validation, Optimization Toolbox and the native solver.
    arguments
        snapshotDirectory (1,1) string
        outputDirectory (1,1) string
    end
    root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'), ...
        fullfile(root,'solver','clarabel','matlab'));
    if ~isfolder(outputDirectory), mkdir(outputDirectory); end
    ablation = localAblation(snapshotDirectory,outputDirectory);
    collision = localCollisionGap(outputDirectory);
    phase = localPhaseGap(outputDirectory);
    rollout = localModelAudit(snapshotDirectory,outputDirectory);
    summary = struct('ablation',ablation,'collision',collision, ...
        'phase',phase,'rollout',rollout);
    save(fullfile(outputDirectory,'summary.mat'),'summary');
    file = fopen(fullfile(outputDirectory,'summary.json'),'w');
    cleanup = onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(summary,PrettyPrint=true));
end

function rows = localAblation(cache,outputDirectory)
    rows=struct([]);
    for name=["straight","sCurve"]
        data=load(fullfile(cache,name+'-replay.mat'),'snapshot');
        frame=data.snapshot.diagnosticFrame;p=frame.program;cfg=frame.model.cfg;
        [fluid,angles]=solveHardCbfClf.fluidInitialize(p,cfg);
        for seed=["previous","fluid"]
            if seed=="previous",point=p.feasibleWitness;theta=p.jointCertificate.angles;else,point=fluid;theta=angles;end
            c=avoidanceStageQp.fixedDirections(p,point,theta,cfg);
            pb=p;pb.anchorPlan=point(p.layout.planIndex);base=avoidanceStageQp.build(pb);
            labels=[p.physicalLabels;"slack"];
            [~,lateralDomain]=localPhaseRows(p);
            labels(lateralDomain)="referenceLateralDomain";
            group=repmat("auxiliary",size(c.b));
            eq=c.cones(1);lin=c.cones(2);
            group(1:eq)="dynamics";
            group(eq+(1:base.cones(2)))=labels(base.retainedRows(1:base.cones(2)));
            group(startsWith(group,"road:"))="road";
            group(c.separationRows(~[p.jointCertificate.records.isExit]))="collision";
            group(c.separationRows([p.jointCertificate.records.isExit]))="exit";
            cursor=eq+lin;
            for j=3:numel(c.cones)
                rr=cursor+(1:c.cones(j));cursor=cursor+c.cones(j);
                if j==3,group(rr)="clf";elseif j<=numel(p.cones),group(rr)="terminal";end
            end
            assert(all(ismember(group,["auxiliary","dynamics","road","actuator","slack", ...
                "collision","exit","clf","terminal","referencePhaseDomain","referenceLateralDomain"])));
            cases={"all",strings(0,1);"noRoad","road";"noCollision","collision"; ...
                "noExit","exit";"noTerminal","terminal";"noPhase","referencePhaseDomain"; ...
                "actuatorCollision",["road","exit","terminal","referencePhaseDomain","referenceLateralDomain"]; ...
                "actuatorTerminal",["road","collision","exit","referencePhaseDomain","referenceLateralDomain"]; ...
                "actuatorPhaseTerminal",["road","collision","exit","referenceLateralDomain"]};
            results=repmat(struct('caseName',"",'program',[],'decision',[],'output',[]),size(cases,1),1);
            for k=1:size(cases,1)
                keep=~ismember(group,cases{k,2});d=c;d.A=c.A(keep,:);d.b=c.b(keep);
                d.cones=[nnz(keep(1:eq));nnz(keep(eq+(1:lin)))];cursor=eq+lin;
                for j=3:numel(c.cones)
                    rr=cursor+(1:c.cones(j));cursor=cursor+c.cones(j);
                    if all(keep(rr)),d.cones(end+1,1)=c.cones(j);else,assert(~any(keep(rr)));end
                end
                center=zeros(numel(d.q),1);center(1:numel(d.anchorPlan))=d.anchorPlan;
                [x,o]=solveAvoidanceSocpMex(sparse(numel(d.q),numel(d.q)),zeros(size(d.q)), ...
                    sparse(d.A),d.b-d.A*center,d.cones,[1e-8,1e-7,400]);
                x=x+center;r=d.b-d.A*x;cur=d.cones(1)+d.cones(2);
                violation=max([0;abs(r(1:d.cones(1)));-r(d.cones(1)+1:cur)]);
                for j=3:numel(d.cones),v=r(cur+(1:d.cones(j)));cur=cur+d.cones(j);violation=max(violation,norm(v(2:end))-v(1));end
                row=struct('scenario',name,'seed',seed,'caseName',cases{k,1},'status',o.status,'violation',violation);
                rows=[rows;row]; %#ok<AGROW>
                results(k)=struct('caseName',cases{k,1},'program',d,'decision',x,'output',o);
            end
            save(fullfile(outputDirectory,name+'-'+seed+'.mat'),'p','c','base','group','results','frame','point','theta','-v7.3');
        end
    end
    writetable(struct2table(rows),fullfile(outputDirectory,'ablation.csv'));
end

function out = localCollisionGap(outputDirectory)
    out=struct([]);
    for seed=["previous","fluid"]
        data=load(fullfile(outputDirectory,'straight-'+seed+'.mat'));
        d=data.results(string({data.results.caseName})=="actuatorCollision").program;
        keep=~ismember(data.group,["road","exit","terminal"]);group=data.group(keep);
        center=zeros(numel(d.q),1);center(1:numel(d.anchorPlan))=d.anchorPlan;
        eq=d.cones(1);li=d.cones(2);cursor=eq+li;
        a=[d.A,-double(group=="collision")];b=d.b-d.A*center;
        cones=d.cones;clfRows=cursor+(1:cones(3));a(clfRows,:)=[];b(clfRows)=[];cones(3)=[];
        nn=size(a,2);a=[a(1:eq+li,:);sparse(1,nn,-1,1,nn);a(eq+li+1:end,:)];b=[b(1:eq+li);0;b(eq+li+1:end)];cones(2)=cones(2)+1;
        [xx,oo]=solveAvoidanceSocpMex(sparse(nn,nn),[zeros(nn-1,1);1],sparse(a),b,cones,[1e-9,1e-9,400]);
        assert(oo.status==1);
        row=struct('seed',seed,'status',oo.status,'minimumCollisionRelaxation',xx(end),'necessaryLowerBound',NaN,'lpExitFlag',NaN,'activeStages',"",'lpPrimalViolation',NaN,'lpDualResidual',NaN,'lpDualBound',NaN,'coneViolation',NaN);out=[out;row];
        % Necessary polyhedral cone relaxation gives an independent lower bound.
        li=cones(2);al=a(eq+(1:li),:);bl=b(eq+(1:li));cursor=eq+li;
        for j=3:numel(cones)
            rr=cursor+(1:cones(j));cursor=cursor+cones(j);
            for k=2:numel(rr),al=[al;a(rr(1),:)+a(rr(k),:);a(rr(1),:)-a(rr(k),:)];bl=[bl;b(rr(1))+b(rr(k));b(rr(1))-b(rr(k))];end
        end
        [xl,vl,fl,~,lambda]=linprog([zeros(nn-1,1);1],al,bl,a(1:eq,:),b(1:eq),[],[],optimoptions('linprog','Display','none','ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9));
        assert(fl==1 && vl>0);
        out(end).lpPrimalViolation=max([0;al*xl-bl;abs(a(1:eq,:)*xl-b(1:eq))]);
        out(end).lpDualResidual=norm([zeros(nn-1,1);1]+al'*lambda.ineqlin+a(1:eq,:)'*lambda.eqlin,inf);
        out(end).lpDualBound=-bl'*lambda.ineqlin-b(1:eq)'*lambda.eqlin;
        out(end).coneViolation=localConicViolation(a,b,cones,xx);
        assert(out(end).lpPrimalViolation<1e-7 && out(end).lpDualResidual<1e-7);
        assert(out(end).coneViolation<1e-6);
        out(end).necessaryLowerBound=vl;out(end).lpExitFlag=fl;
        ids=find(group(eq+(1:d.cones(2)))=="collision");active=lambda.ineqlin(ids)>1e-6;
        collisionRecords=data.p.jointCertificate.records(~[data.p.jointCertificate.records.isExit]);

        out(end).activeStages=string(mat2str([collisionRecords(active).stage]));
        save(fullfile(outputDirectory,'straight-'+seed+'-collision-gap.mat'),'xx','oo','xl','vl','fl','lambda','a','b','cones','al','bl');
    end
    writetable(struct2table(out),fullfile(outputDirectory,'collision-gap.csv'));
end

function result = localPhaseGap(outputDirectory)
    opts=optimoptions('linprog','Display','none','ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9);
    d=load(fullfile(outputDirectory,'sCurve-previous.mat'));p=d.p;n=p.layout.planCount;
    act=p.physicalLabels=="actuator";phase=localPhaseRows(p);
    selected=act|phase;a=p.A(selected,1:n);b=p.b(selected);expand=double(phase(selected));
    tc=p.terminalCone;rr=find(mod((1:size(tc.matrix,1))',3)~=1);r=repelem(tc.bound(1:3:end),2);
    aa=[a,-expand;tc.matrix(rr,:),zeros(numel(rr),1);-tc.matrix(rr,:),zeros(numel(rr),1)];bb=[b;tc.bound(rr)+r;-tc.bound(rr)+r];
    [xx,v,fl,~,lambda]=linprog([zeros(n,1);1],aa,bb,[],[],[-inf(n,1);0],[],opts);
    assert(fl==1 && v>0);
    soc=cell(size(tc.matrix,1)/3,1);
    for j=1:numel(soc),rr=3*j+[-1,0];soc{j}=secondordercone([tc.matrix(rr,:),zeros(2,1)],tc.bound(rr),zeros(n+1,1),-tc.bound(3*j-2));end
    [xSoc,vSoc,flSoc,outSoc]=coneprog([zeros(n,1);1],soc,[a,-expand],b,[],[],[-inf(n,1);0],[],optimoptions('coneprog','Display','none','ConstraintTolerance',1e-9));
    assert(flSoc==1 && vSoc>v);
    states=p.prediction.egoStateOffset+reshape(pagemtimes(p.prediction.egoStateMatrix,xSoc(1:n)),6,[]);
    tab=struct([]);cursor=0;
    for j=1:numel(p.geometry.local)
        node=p.geometry.local(j);r=cursor+(1:numel(node.bound));cursor=cursor+numel(r);
        phased=node.nodeLabels=="referencePhaseDomain";
        if ~any(phased),continue;end
        tab=[tab;struct('stage',node.stage,'time',d.frame.model.stateTime+node.stage*d.frame.model.sampleTime, ...
            'station',states(1,node.stage+1),'stationUpper',node.nodeLimits(find(phased,1)), ...
            'stationLower',-node.nodeLimits(find(phased,1)+1), ...
            'maxPhaseExcess',max(p.A(r(phased),1:n)*xSoc(1:n)-p.b(r(phased))))];
    end
    writetable(struct2table(tab),fullfile(outputDirectory,'sCurve-phase-expansion.csv'));
    residual=reshape(tc.bound-tc.matrix*xSoc(1:n),3,[]);
    result=struct('necessaryLowerBound',v,'minimumSubsetPhaseExpansion',vSoc, ...
        'lpExitFlag',fl,'coneprogExitFlag',flSoc, ...
        'lpPrimalViolation',max([0;aa*xx-bb]), ...
        'lpDualResidual',norm([zeros(n,1);1]+aa'*lambda.ineqlin-lambda.lower,inf), ...
        'lpDualBound',-bb'*lambda.ineqlin, ...
        'conePrimalViolation',max([0;[a,-expand]*xSoc-b;(vecnorm(residual(2:3,:),2,1)-residual(1,:))']));
    % The positive LP dual bound certifies the diagnostic conflict. The SOCP
    % estimate is reported with its unscaled residual, not used for admission.
    assert(result.lpPrimalViolation<1e-7 && result.lpDualResidual<1e-7 && result.conePrimalViolation<1e-5);
    save(fullfile(outputDirectory,'phase-quantification.mat'),'v','fl','xx','lambda','aa','bb','vSoc','flSoc','outSoc','xSoc','states');
end

function summary = localModelAudit(cache,outputDirectory)
    summary=struct([]);rows=struct([]);
    for name=["straight","sCurve"]
        data=load(fullfile(cache,name+'-replay.mat'));snap=data.snapshot;prior=snap.previousState;
        model=snap.diagnosticFrame.model;cfg=model.cfg;params=modifiedFialaTire.parameters(cfg);
        for k=1:size(prior.plan,2)
            tm=prior.prediction.tireModels{k};x=prior.predictedState(:,k);u=prior.plan(:,k);
            force=tm.state*x+tm.input*u+tm.constant;speed=max(x(4),cfg.model.scheduleSpeedFloor);
            alpha=atan2(x(5)+[cfg.vehicle.lf;-cfg.vehicle.lr]*x(6),speed)-[u(1);0];
            cap=params.longitudinalForceScale*sqrt(max(0,1-u(2)^2));actual=NaN(2,1);
            if all(abs(alpha)<pi/2),actual=modifiedFialaTire.evaluate(alpha,u(2),cfg);end
            for axle=1:2
                rows=[rows;struct('scenario',name,'planTime',prior.stateTime,'stage',k,'time',prior.stateTime+(k-1)*model.sampleTime, ...
                    'axle',axle,'anchorBeta',tm.operatingInput(2),'candidateBeta',u(2),'betaSlope',tm.input(axle,2),'anchorSteering',tm.operatingInput(1),'candidateSteering',u(1), ...
                    'anchorForce',tm.force(axle),'betaForceChange',tm.input(axle,2)*(u(2)-tm.operatingInput(2)), ...
                    'affineForce',force(axle),'nonlinearForce',actual(axle),'capacity',cap(axle),'slip',alpha(axle))];
            end
        end
        model.stateTime=prior.stateTime;model.initialEgoState=prior.predictedState(:,1);
        nonlinear=ltvBicycleModel.nominalRollout(model,prior.plan);
        err=prior.predictedState-nonlinear;
        tab=table((0:size(prior.plan,2))'*model.sampleTime+prior.stateTime, ...
            prior.predictedState(1,:)',nonlinear(1,:)',prior.predictedState(2,:)',nonlinear(2,:)', ...
            prior.predictedState(3,:)',nonlinear(3,:)',prior.predictedState(4,:)',nonlinear(4,:)', ...
            'VariableNames',{'time','affineStation','nonlinearStation','affineLateral','nonlinearLateral','affineYaw','nonlinearYaw','affineSpeed','nonlinearSpeed'});
        writetable(tab,fullfile(outputDirectory,name+'-previous-rollout.csv'));
        summary=[summary;struct('scenario',name,'priorTime',prior.stateTime,'maxStationError',max(abs(err(1,:))), ...
            'maxLateralError',max(abs(err(2,:))),'maxYawError',max(abs(err(3,:))), ...
            'affineFinalStation',prior.predictedState(1,end),'nonlinearFinalStation',nonlinear(1,end), ...
            'affineFinalLateral',prior.predictedState(2,end),'nonlinearFinalLateral',nonlinear(2,end), ...
            'affineFinalSpeed',prior.predictedState(4,end),'nonlinearFinalSpeed',nonlinear(4,end))];
    end
    writetable(struct2table(rows),fullfile(outputDirectory,'previous-tire-audit.csv'));
    writetable(struct2table(summary),fullfile(outputDirectory,'previous-rollout-summary.csv'));
end

function violation = localConicViolation(a,b,cones,x)
    r=b-a*x;eq=cones(1);cursor=eq+cones(2);
    violation=max([0;abs(r(1:eq));-r(eq+1:cursor)]);
    for j=3:numel(cones)
        rr=cursor+(1:cones(j));cursor=cursor+cones(j);
        violation=max(violation,norm(r(rr(2:end)))-r(rr(1)));
    end
end

function [station,lateral] = localPhaseRows(p)
    station=false(size(p.physicalLabels));lateral=station;cursor=0;
    for k=1:numel(p.geometry.local)
        node=p.geometry.local(k);rows=cursor+(1:numel(node.bound));cursor=cursor+numel(rows);
        phase=node.nodeLabels=="referencePhaseDomain";
        station(rows)=phase & node.nodeStateRows(:,1)~=0;
        lateral(rows)=phase & node.nodeStateRows(:,2)~=0;
    end
end
