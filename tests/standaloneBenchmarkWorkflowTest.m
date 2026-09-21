classdef standaloneBenchmarkWorkflowTest < matlab.unittest.TestCase
    %standaloneBenchmarkWorkflowTest Reject stale or incomplete native evidence.
    methods (Test)
        function changedSourceInvalidatesTheBenchmark(testCase)
            code = ["(root/'controller').mkdir()", ...
                "source=root/'controller/model.m';source.write_text('original')", ...
                "before=b.source_state(root);source.write_text('changed')", ...
                "unittest.TestCase().assertRaises(RuntimeError,b.verify_unchanged,before,b.source_state(root))"];
            localPython(testCase,code);
        end

        function addedSourceInvalidatesTheBenchmark(testCase)
            code = ["(root/'controller').mkdir()", ...
                "before=b.source_state(root);(root/'controller/new.m').write_text('new')", ...
                "unittest.TestCase().assertRaises(RuntimeError,b.verify_unchanged,before,b.source_state(root))"];
            localPython(testCase,code);
        end

        function replacedExecutableInvalidatesTheBenchmark(testCase)
            code = ["exe=root/'controller-replay';exe.write_bytes(b'first')", ...
                "before=b.fingerprints([exe],root);b.verify_artifacts(before,root);exe.write_bytes(b'second')", ...
                "unittest.TestCase().assertRaises(RuntimeError,b.verify_artifacts,before,root)"];
            localPython(testCase,code);
        end

        function duplicateMeasurementsAreRejected(testCase)
            code = ["baseline={'frames':[{'file':'frame'}]}", ...
                "rows=[{'file':'frame','iteration':0}]*2", ...
                "unittest.TestCase().assertRaises(RuntimeError,b.checked_rows,baseline,rows,2)"];
            localPython(testCase,code);
        end

        function summaryUsesOnlyExecutableTiming(testCase)
            code = ["frame={'file':'frame','inherited':False,'scenario':'stationary','curvature':0,'frame':1,'horizonSteps':32,'seconds':[999]}", ...
                "baseline={'frames':[frame]}", ...
                "rows=[{'file':'frame','iteration':0,'status':1,'seconds':.01,'metrics':[.008,0,.001,0]}]", ...
                "b.checked_rows(baseline,rows,1);result=b.summarize(baseline,rows)", ...
                "assert result['groups']['all']['medianMs']==10 and not result['fullPipelineMeasured']"];
            localPython(testCase,code);
        end

        function fullPlanAdmissionStatusIsAcceptedInNativeMeasurements(testCase)
            code = ["baseline={'frames':[{'file':'frame'}]}", ...
                "rows=[{'file':'frame','iteration':0,'status':4,'seconds':.01,'metrics':[.003,.006,.001,1]}]", ...
                "b.checked_rows(baseline,rows,1)"];
            localPython(testCase,code);
        end
    end
end

function localPython(testCase,code)
    root=fileparts(fileparts(mfilename('fullpath')));
    prefix="import sys,tempfile,unittest;from pathlib import Path;sys.path.insert(0," ...
        +string(jsonencode(fullfile(root,'scripts')))+");import runNativeControllerBenchmark as b;" ...
        +"temporary=tempfile.TemporaryDirectory();root=Path(temporary.name);";
    % The temporary directory is reclaimed when the subprocess exits on error.
    command=prefix+strjoin(code,';')+";temporary.cleanup()";
    mark=char(39);replacement=[mark,char(34),mark,char(34),mark];
    quoted=[mark,strrep(char(command),mark,replacement),mark];
    [status,output]=system(['env -u LD_LIBRARY_PATH -u LD_PRELOAD python3 -c ',quoted]);
    testCase.verifyEqual(status,0,output);
end
