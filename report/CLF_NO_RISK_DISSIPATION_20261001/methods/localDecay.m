% Local (linearized) one-step decay capability of the controller's CLF.
% For e1 = Ad e + Bd du, the best one-step ratio in direction e is
% e'Me/e'Pe with M = Ad'P Ad - Ad'P Bd (Bd'P Bd)^-1 Bd'P Ad.
source='/home/zai/.cache/collisionAvoidance/margin-5cm-20261001/source';
addpath(fullfile(source,'controller'),fullfile(source,'config'));
for speed=[8,15]
    cfg=collisionAvoidanceControllerConfig(struct('referenceSpeed',speed));r=nonlinearBicycleModel.cruise(cfg,0);
    h=cfg.controller.sampleTime;T=expm([r.continuousA,r.continuousB;zeros(2,7)]*h);Ad=T(1:5,1:5);Bd=T(1:5,6:7);P=r.matrix;
    M=Ad'*P*Ad-Ad'*P*Bd/(Bd'*P*Bd)*Bd'*P*Ad;S=sqrtm(P);G=S\M/S;G=(G+G')/2;[W,L]=eig(G);[lam,idx]=sort(diag(L),'descend');
    K=r.gain;Acl=Ad+Bd*K;H=S\(Acl'*P*Acl)/S;H=(H+H')/2;lqr=sort(eig(H),'descend');
    rng(3);N=200000;z=randn(5,N);z=z./vecnorm(z);e=S\z; % uniform on the P-unit sphere
    best=sum(e.*(M*e),1);
    worst=S\W(:,idx(1));worst=worst/norm(worst);
    fprintf('v=%d m/s: best one-step ratio over u, worst direction: %.5f (decrease %.3f%%), best direction %.5f\n',speed,lam(1),100*(1-lam(1)),lam(end));
    fprintf('   LQR policy one-step ratio range [%.5f, %.5f]\n',lqr(end),lqr(1));
    fprintf('   share of P-sphere directions where the 1%% one-step decrease is unattainable: %.3f; where no decrease at all: %.3f\n',mean(best>0.99),mean(best>=1));
    fprintf('   worst direction (unit, e=[ey epsi ev vy r]): [%s]\n',num2str(worst.',' %.3f'));
    fprintf('   eigen-ratios: %s\n',num2str(lam.',' %.5f'));
end
