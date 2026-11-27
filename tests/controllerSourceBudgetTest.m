classdef controllerSourceBudgetTest < matlab.unittest.TestCase
    %controllerSourceBudgetTest Enforce the complete core source-file limit.

    methods (Test)
        function controllerAndConfigurationFitSevenSourceFiles(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            sources = [dir(fullfile(root,'controller','**','*.m')); ...
                dir(fullfile(root,'controller','**','*.cpp')); ...
                dir(fullfile(root,'controller','**','*.c')); ...
                dir(fullfile(root,'controller','**','*.h')); ...
                dir(fullfile(root,'controller','**','*.hpp')); ...
                dir(fullfile(root,'config','collisionAvoidanceControllerConfig.m'))];

            testCase.verifyLessThanOrEqual(numel(sources),7, ...
                'Core sources include all controller subdirectories and controller configuration.');
        end
        function observerFitsTwelveSourceFiles(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            sources = dir(fullfile(root,'estimator','**','*.m'));
            testCase.verifyLessThanOrEqual(numel(sources),12);
        end

        function bothAlgorithmsShareTheRootTheoryDocument(testCase)
            root = fileparts(fileparts(mfilename('fullpath')));
            testCase.verifyTrue(isfile(fullfile(root,'ALGORITHM.md')));
            for directory = ["controller","estimator"]
                theory = dir(fullfile(root,directory,'**','*THEORY*.md'));
                testCase.verifyEmpty(theory);
            end
        end
    end
end
