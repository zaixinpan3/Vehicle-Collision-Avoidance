function runAlternatives(variant,outFile,pointsName,extra)
%runAlternatives Run alternativeCertificate for a variant on a set of operating points.
% variant: tokens joined by '+': secant | force | poly | sat | T<seconds> |
% box<scale> (certification box scaled by <scale>) | wide (box
% [2 .2 1 .3 .15]) | comp<n> | fbox<fraction> (force box fraction).
% pointsName: "default" (8 and 15 m/s, straight and 0.005 1/m, one
% certificate each) or "interval" (one certificate over 8-15 m/s and
% 0-0.005 1/m). Results are appended to outFile as JSON lines.
    if nargin<3,pointsName="default";end
    if nargin<4,extra=struct();end
    here=fileparts(mfilename('fullpath'));addpath(here);
    options=struct('Label',string(variant));
    tokens=split(string(variant),'+');
    for token=tokens.'
        t=char(token);
        if strcmp(t,'baseline')||isempty(t),continue;
        elseif strcmp(t,'secant'),options.Enclosure="secant";
        elseif strcmp(t,'force'),options.InputMap="force";
        elseif strcmp(t,'poly'),options.Lyapunov="polyQuadratic";
        elseif strcmp(t,'sat'),options.Saturation=true;options.DesignMargin=3e-3;
        elseif startsWith(t,'T'),options.TimeConstant=str2double(t(2:end));
        elseif strcmp(t,'wide'),options.Box=[2;.2;1;.3;.15];
        elseif startsWith(t,'box'),options.Box=str2double(t(4:end))*[1;.15;.75;.15;.08];
        elseif startsWith(t,'comp'),options.Components=str2double(t(5:end));
        elseif startsWith(t,'sched'),options.SchedulingComponents=str2double(t(6:end));
        elseif startsWith(t,'iter'),options.Iterations=str2double(t(5:end));
        elseif startsWith(t,'gs'),options.GainStep=str2double(t(3:end));
        elseif startsWith(t,'grow'),options.Growth=str2double(t(5:end));
        elseif startsWith(t,'fbox'),options.InputBox=[str2double(t(5:end));.125];
        else,error('runAlternatives:token','Unknown token %s.',t);
        end
    end
    for name=string(fieldnames(extra)).',options.(name)=extra.(name);end
    switch string(pointsName)
        case "default"
            sets={struct('speed',8,'curvature',0),struct('speed',8,'curvature',.005), ...
                struct('speed',15,'curvature',0),struct('speed',15,'curvature',.005)};
        case "straight"
            sets={struct('speed',8,'curvature',0),struct('speed',15,'curvature',0)};
        case "eight"
            sets={struct('speed',8,'curvature',0)};
        case "rest"
            sets={struct('speed',8,'curvature',.005),struct('speed',15,'curvature',0),struct('speed',15,'curvature',.005)};
        case {"intervalLow","intervalHigh"}
            if string(pointsName)=="intervalLow",speeds=[8,9.5,11.5];else,speeds=[11.5,13,15];end
            grid=struct('speed',{},'curvature',{});
            for speed=speeds,for curvature=[0,.005]
                grid(end+1)=struct('speed',speed,'curvature',curvature); %#ok<AGROW>
            end,end
            sets={grid};
            if ~isfield(options,'BoundarySamples'),options.BoundarySamples=3000;options.InteriorSamples=1500;end
        case "interval"
            grid=struct('speed',{},'curvature',{});
            for speed=[8,9.5,11.5,13,15],for curvature=[0,.0025,.005]
                grid(end+1)=struct('speed',speed,'curvature',curvature); %#ok<AGROW>
            end,end
            sets={grid};
            if ~isfield(options,'BoundarySamples'),options.BoundarySamples=3000;options.InteriorSamples=1500;end
        otherwise,error('runAlternatives:points','Unknown point set %s.',pointsName);
    end
    arguments=namedargs2cell(options);
    for index=1:numel(sets)
        points=sets{index};
        fprintf('\n##### %s speeds %s curvatures %s\n',string(variant),mat2str([points.speed]),mat2str([points.curvature]));
        try
            result=alternativeCertificate(points,arguments{:});
            if nargin>=2 && strlength(string(outFile))>0
                fid=fopen(outFile,'a');fprintf(fid,'%s\n',jsonencode(result));fclose(fid);
            end
        catch e
            fprintf('FAILED: %s\n',e.message);
            fprintf('%s\n',getReport(e,'extended','hyperlinks','off'));
        end
    end
end
