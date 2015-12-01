%   DSSCAssembleVariables.m: Assemble individual variables into vector of 
%                           design variables for optimization
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        9/24/2014
%
%   Inputs:         x, y, z             Cartesian spatial coordinates
%                   xdot, ydot, zdot    Velocity components  
%                   ux, uy, uz          Thrust Direction
%                   eta                 Throttle level [0,1]
%                   tf                  final time (non-dim) 
%
%   Outputs:        vars                vector of optimization variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function vars = DSSCAssembleVariables(x,y,z,xdot,ydot,zdot,...
    ux,uy,uz,eta,tf)

vars = [x; y; z; xdot; ydot; zdot; ux; uy; uz; eta; tf];

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
