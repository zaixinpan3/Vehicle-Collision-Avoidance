function [states,firstDerivatives] = nrmmObserverRk4Interval(state,measurement,design,step,count)
%nrmmObserverRk4Interval Integrate the unchanged held-measurement observer.
% Returns every accepted RK4 substep and its first derivative so that the
% runtime can retain every error-bound update and operating-domain audit.
% Point dynamics do not depend on the separately propagated error bounds.
    states = zeros(numel(state),count+1);
    firstDerivatives = zeros(numel(state),count);
    states(:,1) = state;
    for index = 1:count
        first = localDerivative(state,measurement,design);
        second = localDerivative(state+0.5*step*first,measurement,design);
        third = localDerivative(state+0.5*step*second,measurement,design);
        fourth = localDerivative(state+step*third,measurement,design);
        state = state+(step/6)*(first+2*second+2*third+fourth);
        if any(~isfinite(state))
            error("onlineNrmmTrackingRuntime:nonfiniteObserverState", ...
                "The cascaded observer produced a nonfinite state.");
        end
        state(end) = mod(state(end)+pi,2*pi)-pi;
        states(:,index+1) = state;
        firstDerivatives(:,index) = first;
    end
end

function derivative = localDerivative(state,measurement,design)
    count = (numel(state)-7)/8;
    targetEnd = 6+6*count;
    targetState = reshape(state(7:targetEnd),6,count);
    targetOutputPredictor = reshape(state(targetEnd+1:end-1),2,count);
    observerEstimate = struct("bodyVelocity",state(1:2),"position",state(3:4), ...
        "targetState",targetState);
    vectorFieldInput = struct("yawRate",measurement.yawRate,"gnssVelocity",measurement.gnssVelocity, ...
        "bodyAcceleration",measurement.bodyAcceleration,"positionReference",state(5:6), ...
        "radarReference",targetOutputPredictor,"radarAvailable",measurement.radarDetectionAvailable);
    [observerDerivative,observerModel] = nrmmObserverVectorField(observerEstimate,vectorFieldInput,design);
    planarCross = [0,-1;1,0];
    targetPredictorDerivative = observerModel.targetPlantDerivative(1:2,:) ...
        -measurement.yawRate*planarCross*(targetOutputPredictor-targetState(1:2,:));
    yawDerivative = nrmmYawObserverDerivative(state(end),measurement.yawRate,measurement.correspondence,design);
    derivative = [observerDerivative.bodyVelocity;observerDerivative.position;observerModel.inertialVelocity; ...
        observerDerivative.targetState(:);targetPredictorDerivative(:);yawDerivative];
end
