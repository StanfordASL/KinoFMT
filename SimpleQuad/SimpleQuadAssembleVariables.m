%   SimpleQuadAssembleVariables.m: Assemble individual variables into vector of 
%                           design variables for optimization
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        Mar 30, 2015
%
%   Inputs:         x, y, z             Cartesian spatial coordinates
%                   vx, vy, vz    Velocity components  
%                   ux, uy, uz          Thrust Direction
%                   eta                 Throttle level [0,1]
%                   tf                  final time (non-dim) 
%
%   Outputs:        vars                vector of optimization variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function vars = SimpleQuadAssembleVariables(x,y,z,vx,vy,vz,...
    ux,uy,uz,eta,tf)

vars = [x; y; z; vx; vy; vz; ux; uy; uz; eta; tf];

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
