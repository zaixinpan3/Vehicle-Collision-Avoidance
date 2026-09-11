function result = evaluateFialaSampledFeedback(outputDirectory)
%evaluateFialaSampledFeedback Compare held feedback and open-loop Fiala errors.
% Numerical local tracking experiment, not a validated nonlinear tube bound.
    arguments
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    cfg=collisionAvoidanceControllerConfig();h=.1;count=60;speed=10;
    beta=ltvBicycleModel.roadLoad(speed,cfg)/cfg.vehicle.m/modifiedFialaTire.accelerationGain(cfg);
    nominal=[0;0;0;speed;0;0];input=[0;beta];
    point=struct('state',nominal,'input',input);
    [a,b]=ltvBicycleModel.continuousMatrices(0,speed,cfg,beta,0,point);
    [~,~,ad,bd]=stateUncertainty.sampledFeedbackTransition(a,b,zeros(2,6),h);
    [lqrGain,p]=dlqr(ad,bd,diag([1,4,4,1,.1,.1]),diag([20,20]));gain=-lqrGain;
    transition=stateUncertainty.sampledFeedbackTransition(a,b,gain,h);
    rho=sqrt(max(real(eig(transition.'*p*transition,p))));
    initialError=[.01;-.01;.001;.005;-.001;.001];
    histories=zeros(6,count+1,2);inputs=zeros(2,count,2);norms=zeros(count+1,2);
    for method=1:2
        state=nominal+initialError;histories(:,1,method)=state;norms(1,method)=sqrt(initialError.'*p*initialError);
        for stage=1:count
            center=nominal;center(1)=speed*(stage-1)*h;
            command=input;
            if method==2,command=command+gain*(state-center);end
            inputs(:,stage,method)=command;
            for substep=1:100
                dt=h/100;flow=@(x)ltvBicycleModel.fialaWorldDynamics(x,command,cfg);
                k1=flow(state);k2=flow(state+dt*k1/2);k3=flow(state+dt*k2/2);k4=flow(state+dt*k3);
                state=state+dt*(k1+2*k2+2*k3+k4)/6;
            end
            center(1)=speed*stage*h;error=state-center;
            histories(:,stage+1,method)=state;norms(stage+1,method)=sqrt(error.'*p*error);
        end
    end
    result=struct('scope',"exact-model assumption; numerical Fiala tracking only, no interval-certified tube", ...
        'sampleTime',h,'duration',count*h,'speed',speed,'initialError',initialError, ...
        'gain',gain,'fixedMetric',p,'metricEigenvalues',eig(p),'linearMetricContraction',rho, ...
        'spectralRadius',max(abs(eig(transition))),'initialMetricError',norms(1,1), ...
        'finalOpenLoopMetricError',norms(end,1),'finalFeedbackMetricError',norms(end,2), ...
        'maximumObservedFeedbackContraction',max(norms(2:end,2)./norms(1:end-1,2)), ...
        'maximumInputMagnitude',max(abs(inputs(:,:,2)),[],2), ...
        'maximumInputRate',max(abs(diff([input,inputs(:,:,2)],1,2)),[],2)/h, ...
        'minimumLongitudinalSpeed',min(histories(4,:,2)));
    assert(result.maximumInputMagnitude(1)<cfg.model.frontWheelSteeringAngleMaximum);
    assert(all(inputs(2,:,2)>cfg.actuation.brakingRatioMinimum & inputs(2,:,2)<cfg.actuation.brakingRatioMaximum));
    assert(rho<1 && result.finalFeedbackMetricError<result.initialMetricError);
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    save(fullfile(outputDirectory,'sampled-feedback.mat'),'result','histories','inputs','norms','cfg');
    file=fopen(fullfile(outputDirectory,'sampled-feedback.json'),'w');assert(file>=0);
    cleanup=onCleanup(@()fclose(file));fprintf(file,'%s\n',jsonencode(result,PrettyPrint=true));
    fprintf('Linear rho %.6f; initial %.6g, open-loop final %.6g, feedback final %.6g\n', ...
        rho,result.initialMetricError,result.finalOpenLoopMetricError,result.finalFeedbackMetricError);
end
