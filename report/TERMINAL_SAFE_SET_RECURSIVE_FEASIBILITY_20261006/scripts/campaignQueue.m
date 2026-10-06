function campaignQueue(jobFile,worker,workers)
%campaignQueue Run every workers-th case of jobFile (starting at worker) in one session.
% A case whose summary exists, or which another worker has claimed, is skipped.
    lines=strtrim(splitlines(string(fileread(jobFile))));lines=lines(strlength(lines)>0);
    for k=worker:workers:numel(lines)
        parts=split(lines(k));
        speed=str2double(parts(1));name=parts(2);outdir=parts(3);noisy=parts(4)=="true";
        frames=str2double(parts(5));seed=str2double(parts(6));transition="ode45";
        if numel(parts)>=7,transition=parts(7);end
        stem=fullfile(outdir,sprintf('speed%g-%s',speed,name));if noisy,stem=stem+sprintf('-seed%d',seed);end
        if isfile(stem+".summary.txt"),continue;end
        claim=stem+".claim";
        if isfile(claim),continue;end
        fid=fopen(claim,'w');fprintf(fid,'%d',worker);fclose(fid);
        try
            campaignOne(speed,name,outdir,noisy,frames,seed,transition);
        catch exception
            fprintf('CASE FAILED %s: %s\n',stem,exception.message);
        end
        delete(claim);
    end
end
