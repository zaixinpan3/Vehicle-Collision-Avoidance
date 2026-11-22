classdef casadiFunctionCallback < casadi.Callback
    % Wrap MATLAB numerical functions for CasADi, including an optional Jacobian.
    properties (Access=private)
        inputShapes
        outputShape
        implementation
        derivative
    end
    methods
        function self=casadiFunctionCallback(name,inputs,output,implementation,derivative)
            self@casadi.Callback();
            self.inputShapes=inputs;
            self.outputShape=output;
            self.implementation=implementation;
            if nargin==5
                self.derivative=casadiFunctionCallback([name,'_jacobian'],inputs, ...
                    [prod(output),prod(inputs(1,:))],derivative);
            end
            construct(self,name);
        end
        function release(self)
            % Release native directors while the MATLAB interpreter and
            % the function handles they call are still alive.
            jac=self.derivative;
            self.derivative=[];
            self.implementation=[];
            delete(self);
            if ~isempty(jac),delete(jac);end
        end
        function count=get_n_in(self)
            count=size(self.inputShapes,1);
        end
        function shape=get_sparsity_in(self,i)
            dims=self.inputShapes(i+1,:);
            shape=casadi.Sparsity.dense(dims(1),dims(2));
        end
        function shape=get_sparsity_out(self,~)
            shape=casadi.Sparsity.dense(self.outputShape(1),self.outputShape(2));
        end
        function result=eval(self,args)
            run=self.implementation;
            result={run(args)};
        end
        function yes=has_jacobian(self)
            yes=~isempty(self.derivative);
        end
        function j=get_jacobian(self,name,inames,onames,opts)
            dims=self.inputShapes(1,:);
            x=casadi.MX.sym('x',dims(1),dims(2));
            ignored=casadi.MX.sym('out',casadi.Sparsity(self.outputShape(1),self.outputShape(2)));
            gradient=self.derivative.call({x});
            j=casadi.Function(name,{x,ignored},gradient,inames,onames,opts);
        end
    end
end
