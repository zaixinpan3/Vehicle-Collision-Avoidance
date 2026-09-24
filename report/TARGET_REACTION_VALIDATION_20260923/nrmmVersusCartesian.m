function nrmmVersusCartesian()
%nrmmVersusCartesian Certificate box (Cartesian + NRMM jerk bound) versus the NRMM-family envelope.
% Offline diagnostic, run from the repository root. One representative target
% moves with constant speed-rate and constant sideslip (NRMM): 15 m/s, sideslip
% 0.005 rad, rear-axle distance 1.6 m, speed-rate 1 m/s^2. Estimate errors:
% position 0.2 m, velocity 0.1 m/s, acceleration 0.1 m/s^2 per axis, yaw
% 0.01 rad, yaw rate 0.005 rad/s. The certificate box is finiteFlow with the
% jerk bound the NRMM estimator publishes, hypot(vmax*wmax^2, 3*amax*wmax) with
% modelJerkMaximum = 0, for two sideslip domains; the envelope is the existing
% constant-curvature/constant-tangential-acceleration family envelope
% targetPrediction.errorEnvelope for the same estimate errors.
    addpath("controller","config");
    speed = 15;heading = 0;lr = 1.6;beta = 0.005;tangential = 1;
    yawRate = speed*sin(beta)/lr;curvature = yawRate/speed;
    velocity = speed*[cos(heading);sin(heading)];
    acceleration = tangential*[cos(heading);sin(heading)]+speed^2*curvature*[-sin(heading);cos(heading)];
    radius = [.2;.2;.1;.1;.1;.1;.01;.005];     % position, velocity, acceleration, yaw, yaw rate
    times = [1,2,3,4,4.8];
    for domain = [struct('name',"estimator-in-the-loop (beta_max 0.005)",'betaMax',0.005), ...
            struct('name',"nrmmTrackingConfig (beta_max 0.015)",'betaMax',0.015)]
        omegaMax = 20*sin(domain.betaMax)/lr;
        jerk = hypot(20*omegaMax^2,3*2*omegaMax);
        encounter = struct('center',[0;0;velocity;acceleration;heading;yawRate],'radius',radius, ...
            'contract',struct('jerkBound',jerk*[1;1],'yawAccelerationBound',0));
        [~,box] = targetPrediction.finiteFlow(encounter,times);
        capped = encounter;capped.contract.scalarAccelerationMaximum = 2;
        [~,cappedBox] = targetPrediction.finiteFlow(capped,times);
        fprintf("%s: J = %.3f m/s^3\n",domain.name,jerk);
        fprintf("  Cartesian certificate, position half-width per axis (m): %s\n",sprintf("%6.2f ",max(box(1:2,:),[],1)));
        fprintf("  same with |a| <= 2 m/s^2 cap (m):                        %s\n",sprintf("%6.2f ",max(cappedBox(1:2,:),[],1)));
        fprintf("  of which J t^3/6 (m):                                    %s\n",sprintf("%6.2f ",jerk*times.^3/6));
    end
    model = struct('hasTarget',true,'targetVelocityErrorBound',radius(3:4),'targetAccelerationErrorBound',radius(5:6), ...
        'targetAcceleration',acceleration,'targetYawRate',yawRate,'targetYawRateErrorBound',radius(8), ...
        'targetPositionErrorBound',radius(1:2),'targetYawErrorBound',radius(7), ...
        'targetPrediction',struct('initialSpeed',speed,'tangentialAcceleration',tangential,'curvature',curvature));
    model.targetPredictionSet = targetPrediction.initialSet(model);
    [position,yaw] = targetPrediction.errorEnvelope(times,model);
    fprintf("NRMM-family envelope (same estimate errors, no jerk term), position half-width (m): %s\n",sprintf("%6.2f ",max(position,[],1)));
    fprintf("  yaw half-width (rad): %s\n",sprintf("%6.3f ",yaw));
    fprintf("times (s): %s\n",sprintf("%6.1f ",times));
end
