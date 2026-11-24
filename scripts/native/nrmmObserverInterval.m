function [states, derivatives] = nrmmObserverInterval(state, measurement, design, step, count)
%nrmmObserverInterval Advance the packed NRMM observer over a held measurement.
    [states, derivatives] = nrmmObserverVectorField("integrate", state, measurement, design, step, count);
end
