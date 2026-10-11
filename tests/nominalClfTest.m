classdef nominalClfTest < matlab.unittest.TestCase
    % One analytic CLF, with a local (not global) nonlinear decrease scope.
    properties (TestParameter)
        referenceSpeed={8,15};
        curvature={0,.005};
        coordinate={1,2,3,4,5};
        direction={-1,1};
    end
    methods (TestClassSetup)
        function prepare(testCase)
            root=fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'controller')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root,'config')));
        end
    end
    methods (Test)
        function theClfMatrixIsReadFromThePrecomputedTable(testCase,referenceSpeed,curvature)
            % P was synthesized offline; the controller only reads it.
            [cfg,~,reference]=localSetup(referenceSpeed,curvature);
            root=fileparts(fileparts(mfilename('fullpath')));
            entries=jsondecode(fileread(fullfile(root,'config','clfMatrices.json')));
            entry=entries(arrayfun(@(e)string(e.key)==nonlinearBicycleModel.clfKey(cfg,curvature),entries));
            testCase.verifyNumElements(entry,1);
            testCase.verifyEqual(reference.matrix,(entry.matrix+entry.matrix.')/2,AbsTol=1e-12);
            % The terminal controller's gain and its certificate: the input bound
            % on V <= 1, the set inside the certification box with the recorded
            % fill, a certified contraction no slower than the requested one, a
            % hold factor of at least one, and the vertex inequalities verified.
            testCase.verifyEqual(reference.contraction,exp(-2*cfg.controller.sampleTime/cfg.clf.convergenceTimeConstantSeconds),AbsTol=1e-15);
            testCase.verifyEqual(reference.gain,reshape(entry.gain,2,5),AbsTol=1e-12);
            testCase.verifyEqual(reference.nominalA,reshape(entry.nominalA,5,5),AbsTol=1e-12);
            inputBound=sqrt(diag(reference.gain*(reference.matrix\reference.gain.')));
            testCase.verifyLessThanOrEqual(inputBound,[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio]*(1+1e-6));
            testCase.verifyEqual(entry.certificate.inputBox(:),[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio],RelTol=1e-12);
            box=[cfg.clf.certificationLateralMeters;cfg.clf.certificationHeadingRadians;cfg.clf.certificationSpeedMetersPerSecond; ...
                cfg.clf.certificationLateralVelocityMetersPerSecond;cfg.clf.certificationYawRateRadiansPerSecond];
            s=inv(reference.matrix);
            testCase.verifyLessThanOrEqual(sqrt(diag(s)),box*(1+1e-6));
            testCase.verifyGreaterThan(entry.regionFill,0);
            testCase.verifyGreaterThanOrEqual(min(eig(diag(1./box)*s*diag(1./box))),entry.regionFill-1e-6);
            testCase.verifyEqual(reference.certifiedLevel,1);
            testCase.verifyLessThanOrEqual(entry.certifiedContraction,reference.contraction*(1+1e-9));
            testCase.verifyGreaterThanOrEqual(entry.certificate.lmiMinimumEigenvalue,-1e-6);
            testCase.verifyGreaterThanOrEqual(reference.holdFactor,1);
        end
        function anOperatingPointWithoutAPrecomputedMatrixIsRejected(testCase)
            cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',9.375));
            testCase.verifyError(@()nonlinearBicycleModel.cruise(cfg,0),'collisionAvoidanceController:missingClfMatrix');
        end
        function theCertifiedGainContractsOnTheCertifiedLevelSet(testCase,referenceSpeed,curvature)
            % An independent sample of the boundary of the certified level set:
            % under u = u* + K e the nonlinear hold contracts by rho, the
            % remainder fits the budget without its margin, the hold factor
            % is not exceeded, and below the state-row level the hold meets the
            % problem's rows at its midpoint and endpoint.
            [cfg,~,reference]=localSetup(referenceSpeed,curvature);
            stream=RandStream('mt19937ar','Seed',7+referenceSpeed+1000*curvature);
            directions=randn(stream,5,400);unit=reference.factor\(directions./vecnorm(directions));
            result=terminalSafeSet.certificate(reference,cfg,sqrt(reference.certifiedLevel)*unit,10);
            testCase.verifyTrue(all(isfinite(result.contraction)));
            testCase.verifyLessThanOrEqual(max(result.contraction),reference.contraction);
            testCase.verifyLessThanOrEqual(max(result.holdFactor),reference.holdFactor);
            level=min(reference.certifiedLevel,terminalSafeSet.stateLevel(reference,cfg)/reference.holdFactor^2);
            rows=terminalSafeSet.certificate(reference,cfg,sqrt(level)*unit,0);
            testCase.verifyTrue(all(rows.rows));
        end
        function freshJacobiansLieInTheRecordedEnclosure(testCase,referenceSpeed,curvature)
            % Hypothesis H3 on new samples: the ray-averaged Jacobians of the
            % sampled error map along u = u* + K e on the certified set lie in
            % the recorded zonotope (coefficients within their bounds; the
            % principal residual plus the rank-one quadrature correction of
            % the mean-value identity within the residual norm), in box-scaled
            % coordinates.
            [cfg,~,reference]=localSetup(referenceSpeed,curvature);
            root=fileparts(fileparts(mfilename('fullpath')));
            entries=jsondecode(fileread(fullfile(root,'config','clfMatrices.json')));
            entry=entries(arrayfun(@(e)string(e.key)==nonlinearBicycleModel.clfKey(cfg,curvature),entries));
            certificate=entry.certificate;
            box=[cfg.clf.certificationLateralMeters;cfg.clf.certificationHeadingRadians;cfg.clf.certificationSpeedMetersPerSecond; ...
                cfg.clf.certificationLateralVelocityMetersPerSecond;cfg.clf.certificationYawRateRadiansPerSecond];
            inputBox=[cfg.clf.certificationSteeringRadians;cfg.clf.certificationBrakingRatio];
            stream=RandStream('mt19937ar','Seed',11+referenceSpeed+1000*curvature);
            directions=randn(stream,5,200);unit=reference.factor\(directions./vecnorm(directions));
            errors=unit.*sqrt([ones(1,120),rand(stream,1,80)]);
            inputs=reference.gain*errors;
            [a,b,gap]=terminalSafeSet.rayJacobians(reference,cfg,errors,inputs,cfg.controller.sampleTime,certificate.quadratureNodes);
            u=reshape(certificate.directions,35,[]);
            for j=1:size(errors,2)
                deviation=[diag(box)\(a(:,:,j)-reference.nominalA)*diag(box),diag(box)\(b(:,:,j)-reference.nominalB)*diag(inputBox)];
                x=reshape(deviation,[],1);coefficients=u.'*x;
                quadrature=norm(diag(box)\gap(:,j))/norm([diag(box)\errors(:,j);diag(inputBox)\inputs(:,j)]);
                testCase.verifyLessThanOrEqual(abs(coefficients),certificate.coefficientBounds(:)*(1+1e-9));
                testCase.verifyLessThanOrEqual(norm(reshape(x-u*coefficients,5,7))+quadrature,certificate.residualNorm*(1+1e-9));
            end
        end
        function trimHasZeroValueAndTheInitializationGuidancePreservesIt(testCase,referenceSpeed,curvature)
            [cfg,lane,reference]=localSetup(referenceSpeed,curvature);
            parameters=nonlinearBicycleModel.nominalGuidanceParameters(cfg,curvature);
            trim=[0;0;reference.state(3:6)];
            input=nonlinearBicycleModel.nominalFeedback(trim,reference.input,lane,reference,cfg,parameters);
            testCase.verifyEqual(input,reference.input,AbsTol=1e-12);
            testCase.verifyEqual(nonlinearBicycleModel.nominalValue(trim,lane,reference),0,AbsTol=1e-20);
            testCase.verifyGreaterThan(min(eig(reference.matrix)),0);
        end
        function quadraticHasStrictNonlinearDecreaseNearCruise(testCase,referenceSpeed,curvature,coordinate,direction)
            [cfg,lane,reference]=localSetup(referenceSpeed,curvature);
            scales=[cfg.clf.lateralPositionErrorScale;cfg.clf.headingErrorScale;cfg.clf.speedErrorScale; ...
                cfg.clf.lateralVelocityErrorScale;cfg.clf.yawRateErrorScale];
            e=zeros(5,1);e(coordinate)=direction*1e-3*scales(coordinate);
            x=[0;e(1);reference.state(3)+e(2);reference.state(4:6)+e(3:5)];
            % Witness: the input that minimizes the nonlinear successor value.
            successor=@(u)nonlinearBicycleModel.nominalValue(nonlinearBicycleModel.sample(x,u,cfg),lane,reference);
            input=fminsearch(successor,reference.input,optimset('TolX',1e-12,'TolFun',1e-16,'MaxFunEvals',4000,'MaxIter',2000));
            value=nonlinearBicycleModel.nominalValue(x,lane,reference);
            testCase.verifyLessThanOrEqual(successor(input),reference.contraction*value+1e-12);
        end
        function longitudinalPathPhaseDoesNotChangeTheValue(testCase,referenceSpeed,curvature)
            [~,lane,reference]=localSetup(referenceSpeed,curvature);
            values=zeros(1,3);stations=[0,47,-23];
            for index=1:numel(stations)
                [position,heading]=laneGeometry.referencePose(stations(index),.8,lane.referenceCurve);
                x=[position;heading+reference.state(3)+.2;reference.state(4:6)+[-.7;.1;.05]];
                values(index)=nonlinearBicycleModel.nominalValue(x,lane,reference);
            end
            testCase.verifyEqual(values,repmat(values(1),size(values)),RelTol=1e-10);
        end
        function retiredCostToGoAndExtraIterationSettingsAreRejected(testCase)
            for field=["evaluationSeconds","tailLevel"]
                testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('nominalClf',struct(field,1))), ...
                    'collisionAvoidanceController:invalidConfiguration');
            end
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('nonlinear',struct('maximumLinearizations',3))), ...
                'collisionAvoidanceController:invalidConfiguration');
        end
        function nominalFractionsMustLieInsideTheUnitInterval(testCase)
            for name=["lateralAccelerationFraction","frontForceFraction","brakingRatioLimit"]
                testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('nominalClf',struct(name,1))), ...
                    'collisionAvoidanceController:invalidConfiguration');
            end
            testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('nominalClf',struct('courseGain',0))), ...
                'collisionAvoidanceController:invalidConfiguration');
            for setting={struct('convergenceTimeConstantSeconds',0),struct('certificationLateralMeters',-1),struct('certificationYawRateRadiansPerSecond',0)}
                testCase.verifyError(@()collisionAvoidanceControllerConfig(struct('clf',setting{1})), ...
                    'collisionAvoidanceController:invalidConfiguration');
            end
        end
    end
end

function [cfg,lane,reference]=localSetup(speed,curvature)
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));
    lane=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',curvature,'length',200));
    reference=nonlinearBicycleModel.cruise(cfg,curvature);
end
