function version = setupJointNlpSolver(directory)
%setupJointNlpSolver Load an external CasADi MATLAB distribution with IPOPT.
% Download the Linux MATLAB zip from https://web.casadi.org/get/ and extract
% outside the repository. Pass its directory here or set CASADI_PATH. This
% function changes only the current MATLAB path; it does not install files.
    arguments
        directory (1,1) string = string(getenv('CASADI_PATH'))
    end
    if strlength(directory)==0 && exist('casadi.SX','class')~=8
        directory=fullfile(string(getenv('HOME')),'.local','opt','casadi-3.8.2-linux64-matlab2018b');
    end
    if strlength(directory)>0
        assert(isfolder(fullfile(directory,'+casadi')), ...
            'collisionAvoidanceController:missingCasadi','No CasADi MATLAB package at %s.',directory);
        addpath(directory);
    end
    assert(exist('casadi.SX','class')==8,'collisionAvoidanceController:missingCasadi', ...
        'Provide the external CasADi MATLAB directory or set CASADI_PATH.');
    version=string(casadi.CasadiMeta.version());
    assert(casadi.has_nlpsol('ipopt'),'collisionAvoidanceController:missingIpopt', ...
        'This CasADi distribution does not include the IPOPT plugin.');
end
