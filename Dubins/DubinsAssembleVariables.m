%   DubinsAssembleVariables.m: Assemble individual variables into vector of 
%                           design variables for optimization
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        10/21/2013
%
%   Inputs:         x, y            Cartesian spatial coordinates
%                   theta           Heading angle  
%                   u               Turnrate (heading derivative)  
%                   tf              final time (non-dim) 
%
%   Outputs:        vars            vector of optimization variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function vars = DubinsAssembleVariables(x,y,theta, u, tf)

vars = [x; y; theta; u; tf];

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%