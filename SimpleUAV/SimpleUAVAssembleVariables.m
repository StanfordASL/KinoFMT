%   SimpleUAVAssembleVariables.m: Assemble individual variables into vector of 
%                           design variables for optimization
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        4/22/2014
%
%   Inputs:         x, y, z             Cartesian spatial coordinates
%                   theta               Heading angle 
%                   utheta              Turn rate
%                   uz                  Climb rate
%                   tf                  final time (non-dim) 
%
%   Outputs:        vars                vector of optimization variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function vars = SimpleUAVAssembleVariables(x,y,z,theta,utheta,uz,tf)

vars = [x; y; z; theta; utheta; uz; tf];



return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
