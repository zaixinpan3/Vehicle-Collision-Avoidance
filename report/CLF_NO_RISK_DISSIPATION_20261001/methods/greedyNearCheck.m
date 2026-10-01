% Why does unconstrained greedy not recover in two near-path 15-m/s cases?
D='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001/greedy';
for name=["greedy-acceleratingHeadOn-15","greedy-brakingLead-15","indifferent-brakingLead-15"]
    load(fullfile(D,name+".mat"),'res');r=res;E=r.E;U=r.U;n=size(E,2);
    tail=max(1,n-400):n;tol=[.1;pi/180;.1;.05;.01];
    fprintf('%s: steps %d, final e=[%s]\n',name,r.steps,sprintf('%.3g ',E(:,end)));
    fprintf('  last 20 s: max|e| = [%s]; fraction of steps each tolerance violated = [%s]\n', ...
        sprintf('%.3g ',max(abs(E(:,tail)),[],2)),sprintf('%.2f ',mean(abs(E(:,tail))>tol,2)));
    du=diff(U(:,max(1,end-400):end),1,2);
    fprintf('  last 20 s: steering range [%.3f %.3f], mean|dsteer| %.3f, braking range [%.3f %.3f], mean|dbrake| %.3f, V range [%.3g %.3g]\n', ...
        min(U(1,end-400:end)),max(U(1,end-400:end)),mean(abs(du(1,:))),min(U(2,end-400:end)),max(U(2,end-400:end)),mean(abs(du(2,:))), ...
        min(r.V(tail)),max(r.V(tail)));
end
