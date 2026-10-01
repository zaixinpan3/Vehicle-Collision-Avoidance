% Structure of the controller's CLF V = e'Pe, e = [e_y e_psi e_v v_y r] (straight trim),
% and the channels through which the one-step minimizer lowers V far from the path.
source='/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'));
D='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001';
h=0.05;lambda=-log(0.99)/h;
for speed=[8,15]
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));r=nonlinearBicycleModel.cruise(cfg,0);P=r.matrix;
    fprintf('v=%d m/s: P (e_y m, e_psi rad, e_v m/s, v_y m/s, r rad/s) =\n',speed);disp(P);
    slope=P(1,2)/P(2,2);
    fprintf('  V-minimizing heading for fixed e_y: -%.4f e_y; reaches pi/2 at |e_y|=%.1f m and pi at %.1f m\n',slope,pi/2/slope,pi/slope);
    fprintf('  required continuous decay rate %.4f 1/s; kinematic radius 2v/lambda = %.1f m (|de_y/dt|<=v, e_y term only)\n',lambda,2*speed/lambda);
end
for name=["greedy-crossing-8","greedy-curvedCrossing-8","greedy-turningCrossing-15","greedy-crossing-15"]
    load(fullfile(D,'greedy',name+".mat"),'res');speed=res.speed;
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));P=nonlinearBicycleModel.cruise(cfg,0).matrix;
    if contains(name,"curved"),road=struct('referenceCurve',struct('origin',[0;0],'heading',0,'curvature',.005,'length',200));
        P=nonlinearBicycleModel.cruise(cfg,.005).matrix;end
    fprintf('%s: start e=[%s], V0=%.4g\n',name,num2str(res.E(:,1).',' %.3g'),res.V(1));
    fprintf('   step  steer  brake | dV/V | first-order dV share by e_y e_psi e_v v_y r | speed  v_y\n');
    for k=1:min(12,numel(res.rho))
        e=res.E(:,k);de=res.E(:,k+1)-e;g=2*P*e;parts=g.*de;
        fprintf('   %3d %+.3f %+.3f | %+.4f | %+8.0f %+8.0f %+8.0f %+8.0f %+8.0f | %5.2f %+5.2f\n',k,res.U(1,k),res.U(2,k), ...
            res.V(k+1)/res.V(k)-1,parts,res.X(4,k+1),res.X(5,k+1));
    end
end
