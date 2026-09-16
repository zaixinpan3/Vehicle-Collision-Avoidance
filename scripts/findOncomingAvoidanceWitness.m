function result = findOncomingAvoidanceWitness(options)
%findOncomingAvoidanceWitness Offline nonlinear existence search after admission failure.
% This research optimization does not issue online commands or replace the controller.
    arguments
        options.DiagnosisFile (1,1) string
        options.OutputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
    d=load(options.DiagnosisFile);
    m=d.problem.model;cfg=m.cfg;cfg.solver.jointFunction=[];m.cfg=cfg;m.sampleTime=.02;
    % This deterministic existence experiment uses the diagnosed vehicle and
    % head-on encounter. Search reserves are stricter than its physical limits.
    assert(cfg.controller.sampleTime==.1 && cfg.referenceSpeed==8);
    assert(isequal([cfg.vehicle.length,cfg.vehicle.width,cfg.vehicle.lf,cfg.vehicle.lr], ...
        [4.8,1.9,1.4,1.65]));
    assert(isequal(d.target.targetVelocityInertial,[-8;0]) && ...
        isequal(d.target.targetAccelerationInertial,[0;0]) && ...
        d.target.targetHeadingInertial==pi && d.target.targetYawRate==0);
    m.initialEgoState=[d.ego.position;d.ego.yaw;d.ego.speed;d.ego.lateralVelocity;d.ego.yawRate];
    holdCount=40;n=2*holdCount;sub=5;nt=holdCount*sub+1;t=(0:nt-1)*m.sampleTime;target=d.target.targetPositionInertial+[-8;0]*t;
    initial=zeros(2,holdCount);initial(1,1:5)=.14;initial(1,6:9)=-.1;initial(1,19:23)=-.1;initial(1,24:28)=.12;
    initial(2,:)=d.captured.terminal.input(2);initial(2,1:8)=-.35;initial(2,9:17)=.32;
    last=[];xs=[];sens=[];cacheC=[];cacheJ=[];timer=tic;passes=cell(1,2);
    solverOptions=optimoptions('fmincon','Algorithm','interior-point', ...
        'HessianApproximation','lbfgs','Display','none','SpecifyObjectiveGradient',true, ...
        'SpecifyConstraintGradient',true,'MaxIterations',1200,'MaxFunctionEvaluations',2000, ...
        'ConstraintTolerance',1e-7,'OptimalityTolerance',1e-6,'OutputFcn',@progress);
    for pass=1:2
        budgets=[150,1200];solverOptions.MaxIterations=budgets(pass);timer=tic;
        [z,cost,flag,output]=fmincon(@objective,initial(:),[],[],[],[], ...
            repmat([-.68;-.98],holdCount,1),repmat([.68;.98],holdCount,1),@constraints,solverOptions);
        [passResidual,~]=constraints(z);
        passes{pass}=struct('seconds',toc(timer),'exitFlag',flag,'iterations',output.iterations, ...
            'maximumConstraint',max(passResidual));
        initial=reshape(z,2,[]);
    end
    roll(z);[c,~]=constraints(z);result=struct('input',reshape(z,2,[]),'state',xs,'time',t,'maximumConstraint',max(c),'cost',cost,'flag',flag,'output',output,'seconds',sum(cellfun(@(entry)entry.seconds,passes)),'searchPasses',[passes{:}],'initialState',m.initialEgoState,'configuration',cfg);
    save(fullfile(options.OutputDirectory,'nonlinear-witness.mat'),'result');disp(rmfield(result,{'state','input','time','configuration'}));
    function roll(z)
        if isequal(last,z),return;end
        u=repelem(reshape(z,2,[]),1,sub);[xs,ax,bu]=ltvBicycleModel.nominalRollout(m,u);sens=zeros(6,n,nt);
        for k=1:nt-1
            sens(:,:,k+1)=ax(:,:,k)*sens(:,:,k);ix=2*floor((k-1)/sub)+(1:2);sens(:,ix,k+1)=sens(:,ix,k+1)+bu(:,:,k);
        end
        last=z;cacheC=[];cacheJ=[];
    end
    function [value,gradient]=objective(z)
        roll(z);u=reshape(z,2,[]);err=xs(2:6,end)-[0;0;8;0;0];w=[1;4;1;1;1];du=diff(u,1,2);
        value=100*sum((w.*err).^2)+.02*sum(z.^2)+.2*sum(du.^2,'all');
        rate=zeros(2,holdCount);rate(:,1:end-1)=rate(:,1:end-1)-.4*du;rate(:,2:end)=rate(:,2:end)+.4*du;
        gradient=200*sens(2:6,:,end).'*(w.^2.*err)+.04*z+rate(:);
    end
    function [c,eq,jac,jeq]=constraints(z)
        roll(z);eq=[];jeq=[];
        if ~isempty(cacheC),c=cacheC;jac=cacheJ;return;end
        c=zeros(7*nt+4*(nt-1),1);jac=zeros(n,numel(c));cursor=0;u=reshape(z,2,[]);
        for k=1:nt
            x=xs(:,k);stateSensitivity=sens(:,:,k);co=cos(x(3));si=sin(x(3));dx=x(1)-target(1,k);dy=x(2)-target(2,k);aa=co*dx+si*dy;bb=-si*dx+co*dy;
            gaps=[abs(dx)-2.4-2.4*abs(co)-.95*abs(si);abs(dy)-.95-2.4*abs(si)-.95*abs(co);abs(aa)-2.4-2.4*abs(co)-.95*abs(si);abs(bb)-.95-2.4*abs(si)-.95*abs(co)];
            gg=[sign(dx),0,2.4*sign(co)*si-.95*sign(si)*co;0,sign(dy),-2.4*sign(si)*co+.95*sign(co)*si;sign(aa)*co,sign(aa)*si,sign(aa)*(-si*dx+co*dy)+2.4*sign(co)*si-.95*sign(si)*co;-sign(bb)*si,sign(bb)*co,sign(bb)*(-co*dx-si*dy)-2.4*sign(si)*co+.95*sign(co)*si];
            [gap,active]=max(gaps);select=cursor+(1:7);cursor=cursor+7;
            c(select)=[.30-gap;abs(x(2))-3.9;abs(x(3))-.395;-x(4);x(4)-18;abs(x(5))-12;abs(x(6))-5];
            local=zeros(7,6);local(1,1:3)=-gg(active,:);local(2,2)=sign(x(2));local(3,3)=sign(x(3));local(4,4)=-1;local(5,4)=1;local(6,5)=sign(x(5));local(7,6)=sign(x(6));jac(:,select)=(local*stateSensitivity).';
        end
        for k=1:nt-1
            step=floor((k-1)/sub)+1;
            for endpoint=[k,k+1]
                x=xs(:,endpoint);stateSensitivity=sens(:,:,endpoint);v=max(x(4),1);
                for axle=1:2
                    lever=[1.4,-1.65];vlat=x(5)+lever(axle)*x(6);alpha=atan2(vlat,v)-double(axle==1)*u(1,step);
                    local=zeros(1,6);den=v^2+vlat^2;local(4)=-vlat/den*double(x(4)>1);local(5)=v/den;local(6)=lever(axle)*v/den;
                    g=local*stateSensitivity;if axle==1,g(2*step-1)=g(2*step-1)-1;end
                    cursor=cursor+1;c(cursor)=abs(alpha)-.17;jac(:,cursor)=sign(alpha)*g.';
                end
            end
        end
        cacheC=c;cacheJ=jac;
    end
    function stop=progress(z,info,~)
        stop=toc(timer)>90;
        if mod(info.iteration,100)==0
            fprintf('iter=%d elapsed=%.1f violation=%.5g\n',info.iteration,toc(timer),info.constrviolation);
            input=reshape(z,2,[]);elapsed=toc(timer);iteration=info.iteration;violation=info.constrviolation;
            save(fullfile(options.OutputDirectory,'search-progress.mat'),'input','elapsed','iteration','violation');
        end
    end
end
