%   SimpleUAVDecomposeVariables.m: Separate optimization design variable into 
%                           individual components 
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        4/22/2014
%
%   Inputs:         vars            vector of optimization variables  
%
%   Outputs:        x, y, z                    Cartesian spatial coordinates
%                   theta                      Heading angle
%                   utheta                     Turn rate
%                   uz                         Climb rate
%                   tf                         final time (non-dim) 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x,y,z,theta,utheta,uz,tf] = SimpleUAVDecomposeVariables(vars, range)

x = vars(range.x);
y = vars(range.y);
z = vars(range.z);
theta = vars(range.theta);
utheta = vars(range.utheta);
uz = vars(range.uz);
tf = vars(range.tf);

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%