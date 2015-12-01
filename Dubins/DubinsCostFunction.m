%   DubinsCostFunction.m: Compute final time for objective function 
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        10/21/2013
%
%   Inputs:         vars            design variable 
%                   dubprob         supporting data 
%
%   Outputs:        J               objective function 
%                   dJdx            gradient vector 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [J, dJdx] = DubinsCostFunction(vars, dubprob)

    range = dubprob.numerics.range;
    
    tf = vars(range.tf);
    J = tf;
    
    if nargout > 1
        dtfdvars = zeros(dubprob.numerics.n_vars, 1);
        dtfdvars(range.tf) = 1;
        dJdx = dtfdvars;
    end
   
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%