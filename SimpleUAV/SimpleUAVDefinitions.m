%SimpleUAVDefinitions returns defining constants, attributes, labels, etc
%for the simple UAV optimization problem
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        Apr 30, 2014
%
%   Inputs:     
%
%   Outputs:
%
%   Notes:
%       - Dynamics based on simplified dubins+single integrator. See
%       Hwangbo & Kuffner's Efficient Two Phase 3D Motion Planning for
%       Small Fixed-wing UAVs for details
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function defs = SimpleUAVDefinitions()

defs.nStateDims = 4;        % (#) number of dimensions of state space
defs.stateLabels = {...
    'x';...
    'y';...
    'z';...
    'theta'};

defs.nControlDims = 2;      % (#) number of dimensions of control space 
defs.controlLabels = {...
    'utheta';...
    'uz'};

defs.nRobotParams = 3;      % (#) number of parameters that define and remain constant for the robot
defs.robotParamLabels = {...
    'Vhor';...
    'turnrate';...
    'climbrate'};

defs.optimizationFunction = @SimpleUAVOptimizer; % Function to be called to solve a optimization problem

end