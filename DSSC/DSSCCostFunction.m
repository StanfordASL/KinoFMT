%   DSSCCostFunction.m: Compute final time for objective function 
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        9/24/2014
%
%   Inputs:         vars            design variable 
%                   probinfo         supporting data 
%
%   Outputs:        J               objective function 
%                   dJdx            gradient vector 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [J, dJdx] = DSSCCostFunction(vars, probinfo)

    range = probinfo.numerics.range;
    
    tf = vars(range.tf);
    J = tf;
    
    if nargout > 1
        dtfdvars = zeros(probinfo.numerics.n_vars, 1);
        dtfdvars(range.tf) = 1;
        dJdx = dtfdvars;
    end
   
return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
