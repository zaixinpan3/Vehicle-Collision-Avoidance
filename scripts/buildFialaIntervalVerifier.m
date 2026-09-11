function binary = buildFialaIntervalVerifier(outputDirectory)
%buildFialaIntervalVerifier Build the MPFR-backed nonlinear model verifier.
% Requires system MPFR/GMP development libraries. No dependency is vendored.
    arguments
        outputDirectory (1,1) string
    end
    root=fileparts(fileparts(mfilename('fullpath')));
    if ~isfolder(outputDirectory),mkdir(outputDirectory);end
    binary=fullfile(outputDirectory,"fialaIntervalMex."+mexext);
    clear fialaIntervalMex
    if isfile(binary),delete(binary);end
    try
        mex('-R2018a','CXXFLAGS=$CXXFLAGS -std=c++17', ...
            fullfile(root,'controller','fialaIntervalMex.cpp'),'-lmpfr','-lgmp', ...
            '-outdir',outputDirectory);
    catch exception
        if string(exception.identifier)~="MATLAB:mex:Error" || ~contains(exception.message,"is not a MEX file")
            rethrow(exception);
        end
        % Some hosts reject the post-link inspection of a loadable binary.
        % Require fresh-binary execution below instead of accepting existence.
    end
    assert(isfile(binary));
    addpath(outputDirectory);cleanup=onCleanup(@()rmpath(outputDirectory));rehash;
    point=[0;0;0;10;0;0;0;0];
    [residual,flow,jacobian]=fialaIntervalMex(point,point,zeros(6,8),zeros(6,1),ones(17,1));
    assert(all(isfinite([residual(:);flow(:);jacobian(:)])));
    assert(flow(1,1)<=10 && flow(1,2)>=10 && flow(3,1)==0 && flow(3,2)==0);
end
