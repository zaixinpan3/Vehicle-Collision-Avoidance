% Speed limit cycle of the 15-m/s head-on family after the lateral errors settle.
D='/home/zai/.cache/collisionAvoidance/clf-dissipation-20261001';
for name=["headOn","acceleratingHeadOn","brakingLead"]
    load(fullfile(D,'replay',name+"-15-full.mat"),'record');f=record.frame;time=[f.time]+0.05;e=[f.error1];
    lateral=all(abs(e([1,2,4,5],:))<=[.1;pi/180;.05;.01],1);
    settled=find(~lateral,1,'last')+1;window=settled:numel(f);ev=e(3,window);t=time(window);
    db=arrayfun(@(k)f(k).input(2)-f(k).anchorInputs(2,1),window);edge=abs(db)>0.1249;
    below=ev<-0.1;starts=t([below(1),~below(1:end-1)&below(2:end)]);
    fprintf('%s 15 m/s: lateral errors inside tolerance from %.2f s to 80 s; speed error in [%.3f, %.3f] m/s, %.0f%% of that time below -0.1 m/s;\n', ...
        name,t(1),min(ev),max(ev),100*mean(below));
    fprintf('   braking at its box edge in %.0f%% of those frames; excursions below -0.1 m/s start at %s s\n',100*mean(edge),num2str(starts,' %.1f'));
end
