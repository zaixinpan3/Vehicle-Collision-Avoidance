function runCarlaVehicleDynamicsExperiment(outputFolder,mapFile)
%runCarlaVehicleDynamicsExperiment Run NRMM/PCBF against the installed VDB plant.
% Map input comes from exportCarlaVehicleDynamicsMap.py. MathWorks template assets
% and generated models stay in outputFolder. CARLA receives the resulting plant
% poses synchronously through serveCarlaVehicleDynamics.py on localhost:2605,
% with rendering disabled. Visualization is a separate replay operation.
% The ego uses VDB dynamics; the target currently follows scripted NRMM motion.
% Current nominal-reference interfaces support straight lines and circles;
% curved CARLA routes use a declared local constant-curvature approximation.
% Road-boundary polygons retain actual map geometry, including interior holes.
% Exploratory solver limit is 5 s, not a real-time qualification. Synthetic
% target positions are range-limited to 30 m with the adapter default 4 cm
% bounded noise. A fixed 5 m/s initialization prior is independent of truth;
% the acquired target state comes from measured relative-position history.
    arguments
        outputFolder (1,1) string
        mapFile (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'scripts'),fullfile(root,'estimator'));
    p=char(outputFolder);if ~isfolder(p),mkdir(p);end
    p=char(java.io.File(p).getCanonicalPath());
    for name=["PassVeh14DOF","VehicleConfig"]
        unzip(fullfile(matlabroot,'toolbox','vdynblks','vdynsolution',name+".zip"),fullfile(p,'vendor'));
    end
    addpath(genpath(fullfile(p,'vendor')));
    data=jsondecode(fileread(mapFile));
    load_system('PassVeh14DOF');m=get_param('PassVeh14DOF','ModelWorkspace');v=getVariable(m,'VEH');cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',8,'collision',struct('encounterRangeMeters',30),'solver',struct('timeLimitSeconds',5)));
    v.Mass=cfg.vehicle.m;v.SprungMass=v.Mass-260;v.YawMomentInertia=cfg.vehicle.Iz;v.FrontAxlePositionfromCG=cfg.vehicle.lf;v.RearAxlePositionfromCG=cfg.vehicle.lr;v.WheelBase=cfg.vehicle.wheelbase;v.InitialLongVel=8;v.InitialLatVel=0;assignin(m,'VEH',v);
    set_param('PassVeh14DOF/Suspension','LabelModeActiveChoice','6');set_param('PassVeh14DOF/Suspension/Kinematics and Compliance Independent Suspension/Independent K and C Suspension','IdealSuspEn','off');
    tire='PassVeh14DOF/Wheels and Tires/VDBS/Tires';set_param(tire,'LabelModeActiveChoice','0');
    for k=1:4
        suffix='';if k>1,suffix=num2str(k-1);end
        b=[tire '/MF Tires/Combined Slip Wheel 2DOF' suffix];radius=str2double(get_param(b,'UNLOADED_RADIUS'));set_param(b,'omegao',num2str(8/radius));
    end
    set_param('PassVeh14DOF','ModelReferenceMinAlgLoopOccurrences','on');save_system('PassVeh14DOF');
    mdl='CollisionVdb';if bdIsLoaded(mdl),close_system(mdl,0);end;new_system(mdl);mw=get_param(mdl,'ModelWorkspace');assignin(mw,'experimentConfig',cfg);
    add_block('built-in/Subsystem',[mdl '/Plant']);Simulink.BlockDiagram.copyContentsToSubsystem('PassVeh14DOF',[mdl '/Plant']);
    vars=whos(m);for j=1:numel(vars),assignin(mw,vars(j).name,getVariable(m,vars(j).name));end
    set_param(mdl,'DataDictionary',get_param('PassVeh14DOF','DataDictionary'));
    add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[mdl '/Controller'],'FunctionName','carlaVehicleDynamicsDriver','Parameters','experimentConfig');
    add_block('simulink/Signal Routing/Demux',[mdl '/Commands'],'Outputs','[4 4 4]');add_line(mdl,'Controller/1','Commands/1');for k=1:3,add_line(mdl,['Commands/' num2str(k)],['Plant/' num2str(k)]);end
    vals={'zeros(3,1)','zeros(4,1)','.85*ones(4,1)','repmat(eye(3),1,1,4)'};
    for k=1:4
        name=['Environment' num2str(k)];add_block('simulink/Sources/Constant',[mdl '/' name],'Value',vals{k});add_line(mdl,[name '/1'],['Plant/' num2str(k+3)]);
    end
    signals='InertFrm.Cg.Disp.X,InertFrm.Cg.Disp.Y,InertFrm.Cg.Ang.psi,BdyFrm.Cg.Vel.xdot,BdyFrm.Cg.Vel.ydot,BdyFrm.Cg.AngVel.r';
    add_block('simulink/Signal Routing/Bus Selector',[mdl '/Pose'],'OutputSignals',signals);add_line(mdl,'Plant/1','Pose/1');add_block('simulink/Signal Routing/Mux',[mdl '/State'],'Inputs','6');for k=1:6,add_line(mdl,['Pose/' num2str(k)],['State/' num2str(k)]);end;add_line(mdl,'State/1','Controller/1');
    add_block('simulink/Sinks/To Workspace',[mdl '/Log'],'VariableName','states','SaveFormat','Timeseries','SampleTime','.05');add_line(mdl,'State/1','Log/1');
    set_param(mdl,'Solver','ode23t','MaxStep','.002','RelTol','1e-5','AbsTol','1e-6','StopTime','12','ReturnWorkspaceOutputs','on');save_system(mdl,fullfile(p,[mdl '.slx']));
    mw=get_param(mdl,'ModelWorkspace');
    map=polyshape();for i=1:numel(data.roadRegions)
        reg=data.roadRegions(i);poly=polyshape(reg.outer(:,1),reg.outer(:,2));holes=reg.holes;if ~iscell(holes)&&~isempty(holes),holes={holes};end
        for j=1:numel(holes),h=holes{j};poly=subtract(poly,polyshape(h(:,1),h(:,2)));end
        map=union(map,poly);
    end
    tr=triangulation(map);polys=cell(size(tr.ConnectivityList,1),1);for i=1:numel(polys),polys{i}=tr.Points(tr.ConnectivityList(i,:),:);end
    if isempty(find_system('CollisionVdb','SearchDepth',1,'Name','Pose6Dof'))
        add_block('simulink/Signal Routing/Bus Selector','CollisionVdb/Pose6Dof','OutputSignals','InertFrm.Cg.Disp.X,InertFrm.Cg.Disp.Y,InertFrm.Cg.Disp.Z,InertFrm.Cg.Ang.phi,InertFrm.Cg.Ang.theta,InertFrm.Cg.Ang.psi');
        add_line('CollisionVdb','Plant/1','Pose6Dof/1');add_block('simulink/Signal Routing/Mux','CollisionVdb/PoseMux','Inputs','6');for k=1:6,add_line('CollisionVdb',['Pose6Dof/' num2str(k)],['PoseMux/' num2str(k)]);end
        add_block('simulink/Sinks/To Workspace','CollisionVdb/PoseLog','VariableName','pose6dof','SaveFormat','Timeseries','SampleTime','.05');add_line('CollisionVdb','PoseMux/1','PoseLog/1');
    end
    add_line(mdl,'PoseMux/1','Controller/2');
    bridge=tcpclient('127.0.0.1',2605,'Timeout',60);configureTerminator(bridge,'LF');
    names=fieldnames(data.routes);global pdcaVdbExperiment
    for scenario=1:numel(names)
        name=names{scenario};route=data.routes.(name);x0=[route(1,1:3).';8;0;0];yaw=x0(3);L=cfg.vehicle.lr;
        q=[route(13,1:2).';yaw+pi;4;0;0;L;2.4;.95;0;0];
        if strcmp(name,'intersection'),q=data.intersectionTarget(:);end
        if strcmp(name,'curve'),curv=atan2(sin(route(6,3)-route(1,3)),cos(route(6,3)-route(1,3)))/10;q(6)=asin(max(-1,min(1,-curv*L)));end
        ec=estimatorControllerIntegrationConfig();ec.sensor.radar.rangeMaximum=30;ec.initialization.targetSpeedPrior=5;contract=struct('speedMinimum',0,'speedMaximum',15,'scalarAccelerationMaximum',2,'sideslipMaximum',.5,'yawRateMaximum',1,'accelerationNormBound',15);ec=estimatorConfigurationFromController(ec,cfg,contract);
        kappa=0;if strcmp(name,'curve'),kappa=atan2(sin(route(11,3)-route(1,3)),cos(route(11,3)-route(1,3)))/20;end
        curve=struct('origin',route(1,1:2).','heading',route(1,3),'curvature',kappa,'length',500);
        road=struct('referenceCurve',curve,'drivablePolygons',{polys});
        writeline(bridge,jsonencode(struct('operation','begin','target',q)));reply=jsondecode(readline(bridge));assert(contains(reply.map,'Town10'));
        pdcaVdbExperiment=struct('bridge',bridge,'cfg',cfg,'road',road,'target',q,'estimator',ec,'radius',.3,'context',[],'previous',[],'held',[0;0],'command',zeros(12,1),'trace',struct([]),'failure',"");
        v=getVariable(mw,'VEH');v.InitialLongPosition=x0(1);v.InitialLatPosition=-x0(2);v.InitialYawAngle=-x0(3);assignin(mw,'VEH',v);set_param('CollisionVdb','StopTime','12');
        try,out=sim('CollisionVdb');catch ex,pdcaVdbExperiment.failure=getReport(ex,'extended','hyperlinks','off');out=[];end
        result=pdcaVdbExperiment;result=rmfield(result,'bridge');result.route=route;save(fullfile(p,[name '.mat']),'result','out','-v7.3');if ~isempty(out),view=struct('scenario',name,'failure',result.failure,'trace',result.trace,'time',out.pose6dof.Time,'pose',squeeze(out.pose6dof.Data),'targetInitial',q,'roadHeight',route(1,4));fid=fopen(fullfile(p,[name '.json']),'w');fprintf(fid,'%s',jsonencode(view));fclose(fid);end
        fprintf('SCENARIO %s frames=%d failure=%s\n',name,numel(result.trace),result.failure);
    end
    writeline(bridge,jsonencode(struct('operation','end')));readline(bridge);clear bridge;
    close_system(mdl,0);close_system('PassVeh14DOF',0);
end
