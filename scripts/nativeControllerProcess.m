classdef nativeControllerProcess < handle
    % Persistent JSON-lines client used by the independent MATLAB plant.
    properties (Access=private)
        child
        reader
        writer
    end
    methods
        function obj=nativeControllerProcess(executable)
            args=java.util.ArrayList();args.add(char(executable));
            builder=java.lang.ProcessBuilder(args);
            environment=builder.environment();
            environment.remove('LD_LIBRARY_PATH');environment.remove('LD_PRELOAD');
            builder.redirectError(javaMethod('appendTo','java.lang.ProcessBuilder$Redirect',java.io.File('/dev/stderr')));
            obj.child=builder.start();
            obj.reader=java.io.BufferedReader(java.io.InputStreamReader(obj.child.getInputStream()));
            obj.writer=java.io.PrintWriter(obj.child.getOutputStream(),true);
        end
        function reply=exchange(obj,request)
            obj.writer.println(jsonencode(request));line=obj.reader.readLine();
            assert(~isempty(line),'nativeControllerProcess:closed','Controller exited before replying.');
            reply=jsondecode(char(line));
            if strcmp(reply.type,'error')
                error('nativeControllerProcess:refused','%s',reply.message);
            end
        end
        function delete(obj)
            if ~isempty(obj.writer),obj.writer.close();end
            if ~isempty(obj.child),obj.child.destroy();end
        end
    end
end
