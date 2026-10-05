function summary = validateNominalClf(options)
%validateNominalClf Sample the analytic quadratic's local nonlinear decrease.
% Signed coordinate perturbations use the input minimizing the nonlinear
% successor value as a decrease witness. This diagnostic does not issue vehicle commands, certify an entire
% neighborhood, or establish a global CLF. Closed-loop recovery is tested by
% clfNominalRecoveryTest and the obstacle scenario campaign.
    arguments
        options.OutputDirectory (1,1) string = ""
        options.Speeds (1,:) double = [8,15]
        options.Curvatures (1,:) double = [0,.005]
        options.NormalizedAmplitudes (1,:) double {mustBePositive} = [1e-3,.01,.1]
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(root,'controller'),fullfile(root,'config'));
    rows=cell(0,9);
    for speed=options.Speeds
        cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
        scales=[cfg.clf.lateralPositionErrorScale;cfg.clf.headingErrorScale;cfg.clf.speedErrorScale; ...
            cfg.clf.lateralVelocityErrorScale;cfg.clf.yawRateErrorScale];
        for curvature=options.Curvatures
            reference=nonlinearBicycleModel.cruise(cfg,curvature);
            lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
            for amplitude=options.NormalizedAmplitudes
                for coordinate=1:5
                    for direction=[-1,1]
                        e=zeros(5,1);e(coordinate)=direction*amplitude*scales(coordinate);
                        x=[0;e(1);reference.state(3)+e(2);reference.state(4:6)+e(3:5)];
                        successor=@(u)nonlinearBicycleModel.nominalValue(nonlinearBicycleModel.sample(x,u,cfg),lane,reference);
                        input=fminsearch(successor,reference.input,optimset('TolX',1e-12,'TolFun',1e-16,'MaxFunEvals',4000,'MaxIter',2000));
                        value=nonlinearBicycleModel.nominalValue(x,lane,reference);
                        next=successor(input);
                        required=cfg.nominalClf.decreaseFraction*sum((e./scales).^2);
                        rows(end+1,:)={speed,curvature,amplitude,coordinate,direction,value,next,required,next-value+required}; %#ok<AGROW>
                    end
                end
            end
        end
    end
    samples=cell2table(rows,'VariableNames',{'speed','curvature','normalizedAmplitude', ...
        'coordinate','direction','initialValue','nextValue','requiredDecrease','decreaseResidual'});
    summary=struct('samples',height(samples),'passed',nnz(samples.decreaseResidual<=1e-12), ...
        'maximumDecreaseResidual',max(samples.decreaseResidual),'sampleTable',samples);
    if strlength(options.OutputDirectory)>0
        if ~isfolder(options.OutputDirectory),mkdir(options.OutputDirectory);end
        writetable(samples,fullfile(options.OutputDirectory,'quadratic-clf-local-decrease.csv'));
    end
    fprintf('Local quadratic CLF: %d/%d sampled decrease tests passed; maximum residual %.3g.\n', ...
        summary.passed,summary.samples,summary.maximumDecreaseResidual);
end
