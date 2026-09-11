function binary = buildFialaIntervalVerifier(outputDirectory)
%buildFialaIntervalVerifier Build the MPFR-backed nonlinear model verifiers.
% Requires system MPFR/GMP development libraries. No dependency is vendored.
    arguments
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    clear fialaIntervalMex fialaFeedbackSampleMex
    for name=["fialaIntervalMex","fialaFeedbackSampleMex"]
        target=fullfile(outputDirectory,name+"."+mexext);
        if isfile(target),delete(target);end
        try
            mex('-R2018a','CXXFLAGS=$CXXFLAGS -std=c++17', ...
                fullfile(root,'controller',name+".cpp"),'-lmpfr','-lgmp', ...
                '-outdir',outputDirectory);
        catch exception
            if string(exception.identifier)~="MATLAB:mex:Error" || ~contains(exception.message,"is not a MEX file")
                rethrow(exception);
            end
            % Require actual execution of each fresh binary below when the
            % host's post-link inspection incorrectly rejects its format.
        end
        assert(isfile(target));
    end
    binary=fullfile(outputDirectory,"fialaIntervalMex."+mexext);
    addpath(outputDirectory);cleanup=onCleanup(@()rmpath(outputDirectory));rehash;
    point=[0;0;0;10;0;0;0;0];
    [residual,flow,jacobian]=fialaIntervalMex(point,point,zeros(6,8),zeros(6,1),ones(17,1));
    assert(all(isfinite([residual(:);flow(:);jacobian(:)])));
    assert(flow(1,1)<=10 && flow(1,2)>=10 && flow(3,1)==0 && flow(3,2)==0);
    % Exercise rejection without relying on a nonlinear integration fixture.
    check=fialaFeedbackSampleMex(point(1:6),point(1:6),point,zeros(2,6),zeros(6,1), ...
        ones(17,1),[.1;.005;1e-5;100;128;realmax],[1;1],[-1;-1;1;1;0;0],[]);
    assert(~check.accepted && string(check.reason)=="executionBounds");
end
