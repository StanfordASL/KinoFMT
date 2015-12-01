%   SimpleQuadDecomposeVariables.m: Separate optimization design variable into 
%                           individual components 
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        Mar 30, 2015
%
%   Inputs:         vars            vector of optimization variables  
%
%   Outputs:        x, y, z                    Cartesian spatial coordinates
%                   vx, vy, vz           velocity components 
%                   ux, uy, uz                 unit thrust vector components
%                   eta                        throttle
%                   tf                         final time (non-dim) 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x,y,z,vx,vy,vz,ux,uy,uz,eta,tf] = SimpleQuadDecomposeVariables(vars, range)

x = vars(range.x);
y = vars(range.y);
z = vars(range.z);
vx = vars(range.vx);
vy = vars(range.vy);
vz = vars(range.vz);
ux = vars(range.ux);
uy = vars(range.uy);
uz = vars(range.uz);
eta = vars(range.eta);
tf = vars(range.tf);

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
