function result = checkParkedClearance(outputFile,cases,seed)
%checkParkedClearance Closed-form parked check against dense sampling in time (study script).
% For random parked ego poses and random targets on lines and circles (some
% braking to rest, forward or in reverse, some at rest), compares
% terminalSafeSet.parkedClearance with the least rectangle distance over
% 200001 samples in time of predictiveSafetyGeometry.predictTarget, up to the
% target's first exit from the range, its rest (plus 1 s) or one full turn of
% its circle (plus 1 s), whichever comes first; cases whose sampled duration
% would exceed 120 s are skipped. The sampling uses only the forecast, not
% the closed form's swept stretch. Counts only cases in which the two
% rectangles are apart at the start and the target is in range. Writes the
% summary as JSON to outputFile.
    if nargin<2,cases=3000;end
    if nargin<3,seed=11;end
    root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
    addpath(fullfile(root,'controller'),fullfile(root,'config'),fullfile(root,'solver','controller'));
    context=struct('range',50,'shape',[2.4;.95;0;0]);
    rng(seed);rows=zeros(0,6);skipped=0;
    for trial=1:cases
        pose=[0;0;2*pi*rand];
        q=[(rand(2,1)-.5)*40;2*pi*rand;(rand-.3)*15;(rand-.5)*6;0;1.6;2.4;.95;0;0];
        if rand<.6,q(6)=(rand-.5)*.4;end
        if rand<.1,q(5)=0;end
        if rand<.05,q(4)=0;end
        start=predictiveSafetyGeometry.rectangle(pose,context.shape,q(1:3),q(8:11));
        if start<=.05 || norm(q(1:2))>50,continue;end
        [distance,witness,track]=terminalSafeSet.parkedClearance(context,pose,q);
        V=q(4);A=q(5);k=sin(q(6))/q(7);
        horizon=Inf;
        if V*A<0,horizon=-V/A+1;elseif V==0 && A<=0,horizon=1;end
        if k~=0
            turn=2*pi/abs(k);
            if A==0,lap=turn/abs(V);
            elseif V*A>=0,lap=(-abs(V)+sqrt(V^2+2*abs(A)*turn))/abs(A);
            elseif turn>=V^2/(2*abs(A)),lap=Inf;
            else,lap=(abs(V)-sqrt(V^2-2*abs(A)*turn))/abs(A);
            end
            horizon=min(horizon,lap+1);
        end
        if V~=0 || A>0
            % first exit from the range, found by coarse stepping of the forecast
            coarse=0:.01:min(horizon,130);path=predictiveSafetyGeometry.predictTarget(q,coarse);
            out=find(vecnorm(path(1:2,:)-pose(1:2),2,1)>context.range,1);
            if ~isempty(out),horizon=min(horizon,coarse(out));end
        end
        if horizon>120,skipped=skipped+1;continue;end
        times=linspace(0,horizon,200001);poses=predictiveSafetyGeometry.predictTarget(q,times);
        inside=find(vecnorm(poses(1:2,:)-pose(1:2),2,1)>context.range,1);
        if ~isempty(inside),poses=poses(:,1:inside-1);end
        dense=Inf;
        near=find(vecnorm(poses(1:2,:)-pose(1:2),2,1)<distance+2*norm([2.4;.95])+.5);
        for j=near
            dense=min(dense,predictiveSafetyGeometry.rectangle(pose,context.shape,poses(1:3,j),poses(8:11,j)));
        end
        spacing=max(vecnorm(diff(poses(1:2,:),1,2),2,1));
        attained=predictiveSafetyGeometry.rectangle(pose,context.shape,witness(1:3),witness(8:11));
        rows(end+1,:)=[dense-distance,abs(attained-distance),track.fullCircle,V*A<0,spacing,isinf(track.exitSeconds)]; %#ok<AGROW>
    end
    result=struct('seed',seed,'cases',size(rows,1),'skippedLongerThan120Seconds',skipped, ...
        'fullCircles',nnz(rows(:,3)),'braking',nnz(rows(:,4)),'neverLeaving',nnz(rows(:,6)), ...
        'largestDenseMinusClosedMeters',max(rows(:,1)),'smallestDenseMinusClosedMeters',min(rows(:,1)), ...
        'largestWitnessMismatchMeters',max(rows(:,2)),'largestSampleSpacingMeters',max(rows(:,5)));
    fid=fopen(outputFile,'w');fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));fclose(fid);
    disp(result);
end
