% Usage: set speed, name, captureTimes (and optional variants) before running.
addpath('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/methods');
h=0.05;frames=round(max(captureTimes)/h)+1;
record=replayCase(speed,name,frames,'',captureTimes);
if ~exist('variants','var'),variants=["original","noSteerTrustFirst","noInputTrustFirst","noStateTrust","noInputTrust","noTrust","noEndpointCone","noStateTrustNoEndpoint"];end
all=struct([]);
for c=1:numel(record.capture)
    k=find(abs([record.frame.time]-record.capture(c).time)<1e-9,1);
    rows=ablateFrame(record.capture(c),record.configuration,record.frame(k),variants);
    for r=rows
        fprintf('%s %d t=%6.2f %-23s exact=%d init=%s flags=%-9s pcbf=%9.3g rho=%10.4g u0=[%8.4f %8.4f] u0-a=[%+7.4f %+7.4f] u1-a=[%+7.4f %+7.4f] V0=%10.4g dVaff=%+10.4g dVact=%+10.4g clr=%7.4f trustUse=%.2f futureSum=%+.4f endHeading=%+.4f\n', ...
            name,speed,r.time,r.variant,r.exactReproduction,r.initialization,mat2str(r.flags),r.pcbfOptimum,r.clfSlack,r.u0,r.u0MinusAnchor,r.u1MinusAnchor, ...
            r.V0,r.V1affine-r.V0,r.V1actual-r.V0,r.holdClearance,r.maxStateTrustUse,r.futureSteerSum,r.endHeadingShift);
    end
    if isempty(all),all=rows;else,all=[all,rows];end %#ok<AGROW>
end
save(sprintf('/home/zai/.cache/collisionAvoidance/rti-failure-analysis-20261001/ablation/%s-%d.mat',name,speed),'all','record','-v7.3');
