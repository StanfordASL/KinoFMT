%   DubinsDecomposeVariables.m: Separate optimization design variable into 
%                           individual components 
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        10/21/2013
%
%   Inputs:         vars            vector of optimization variables  
%
%   Outputs:        x, y            Cartesian spatial coordinates
%                   theta           Heading angle  
%                   u               Turnrate (heading derivative)  
%                   tf              final time (non-dim) 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x,y,theta,u,tf] = DubinsDecomposeVariables(vars, range)

x = vars(range.x);
y = vars(range.y);
theta = vars(range.theta);
u = vars(range.u);
tf = vars(range.tf);

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%