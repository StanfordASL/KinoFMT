%   DSSCDecomposeVariables.m: Separate optimization design variable into 
%                           individual components 
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        9/24/2014
%
%   Inputs:         vars            vector of optimization variables  
%
%   Outputs:        x, y, z                    Cartesian spatial coordinates
%                   xdot, ydot, zdot           velocity components 
%                   ux, uy, uz                 unit thrust vector components
%                   eta                        throttle
%                   tf                         final time (non-dim) 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x,y,z,xdot,ydot,zdot,ux,uy,uz,eta,tf] = DSSCDecomposeVariables(vars, range)

x = vars(range.x);
y = vars(range.y);
z = vars(range.z);
xdot = vars(range.xdot);
ydot = vars(range.ydot);
zdot = vars(range.zdot);
ux = vars(range.ux);
uy = vars(range.uy);
uz = vars(range.uz);
eta = vars(range.eta);
tf = vars(range.tf);

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
