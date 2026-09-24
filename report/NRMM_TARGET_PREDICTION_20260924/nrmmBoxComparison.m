function nrmmBoxComparison()
%nrmmBoxComparison Target box half-widths: Cartesian jerk contract versus nrmm-motion-v1.
% Offline diagnostic, run from the repository root. The target of
% report/TARGET_REACTION_VALIDATION_20260923/nrmmVersusCartesian.m: NRMM at
% 15 m/s, sideslip 0.005 rad, rear-axle distance 1.6 m, speed-rate 1 m/s^2;
% estimate errors 0.2 m, 0.1 m/s, 0.1 m/s^2 per axis, 0.01 rad, 0.005 rad/s.
% For the two estimator domains (sideslip maximum 0.005 and 0.015 rad, speed
% maximum 20 m/s, speed-rate maximum 2 m/s^2) it prints the position
% half-width per axis of: the Cartesian contract with the jerk bound the
% estimator publishes for a model error, hypot(vmax*wmax^2, 3*amax*wmax); and
% the nrmm-motion-v1 contract the estimator now publishes (curvature maximum
% sin(beta_max)/l_r, acceleration magnitude maximum hypot(amax, vmax*wmax)).
    addpath("controller","config");
    speed = 15;heading = 0;lr = 1.6;beta = 0.005;tangential = 1;
    yawRate = speed*sin(beta)/lr;curvature = yawRate/speed;
    velocity = speed*[cos(heading);sin(heading)];
    acceleration = tangential*[cos(heading);sin(heading)]+speed^2*curvature*[-sin(heading);cos(heading)];
    radius = [.2;.2;.1;.1;.1;.1;.01;.005];
    times = [1,2,3,4,4.8];
    center = [0;0;velocity;acceleration;heading;yawRate];
    for betaMax = [0.005,0.015]
        omegaMax = 20*sin(betaMax)/lr;
        jerk = hypot(20*omegaMax^2,3*2*omegaMax);
        cartesian = struct('center',center,'radius',radius, ...
            'contract',struct('kind',"finite-sensing-motion-v1",'jerkBound',jerk*[1;1],'yawAccelerationBound',0));
        nrmm = struct('center',center,'radius',radius, ...
            'contract',struct('kind',"nrmm-motion-v1",'jerkBound',[0;0],'yawAccelerationBound',0, ...
            'curvatureMaximum',sin(betaMax)/lr,'scalarAccelerationMaximum',hypot(2,20*omegaMax)));
        [~,box] = targetPrediction.finiteFlow(cartesian,times);
        [~,nrmmBox] = targetPrediction.finiteFlow(nrmm,times);
        fprintf("sideslip maximum %.3f rad (J = %.3f m/s^3, curvature maximum %.5f 1/m)\n", ...
            betaMax,jerk,sin(betaMax)/lr);
        fprintf("  Cartesian jerk contract, position half-width per axis (m): %s\n", ...
            sprintf("%6.2f ",max(box(1:2,:),[],1)));
        fprintf("  nrmm-motion-v1,          position half-width per axis (m): %s\n", ...
            sprintf("%6.2f ",max(nrmmBox(1:2,:),[],1)));
        fprintf("  nrmm-motion-v1,          yaw half-width (rad):             %s\n",sprintf("%6.3f ",nrmmBox(7,:)));
    end
    fprintf("times (s): %s\n",sprintf("%6.1f ",times));
end
