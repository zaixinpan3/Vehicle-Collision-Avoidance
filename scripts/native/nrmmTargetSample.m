function [track,motion] = nrmmTargetSample(track,time,position,egoHeading,relativeHeading,radius,parameters)
%nrmmTargetSample Update one target using its synchronized inertial position.
    if track.detections==0
        assert(numel(position)==2,'The first target sample must contain a position.');
        track.state=[reshape(position,2,1);zeros(4,1)];
        track.time=time;track.lastReturn=time;track.heading=egoHeading;
        track.previousRadius=radius;track.speedUpper=parameters.design.domain.speedMaximum;
    end
    [track,motion] = onlineNrmmTrackingRuntime('targetSample',track,time,position, ...
        egoHeading,parameters.cfg,relativeHeading,radius,parameters.design,parameters.kernel, ...
        parameters.settings,parameters.bound);
end
