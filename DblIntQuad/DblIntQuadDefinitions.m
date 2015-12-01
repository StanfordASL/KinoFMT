%DblIntQuadDefinitions returns defining constants, attributes, labels, etc
%for the linear, double-integrator quadrotor optimization problem
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        Mar 30, 2014
%
%   Inputs:     
%
%   Outputs:
%
%   Notes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function defs = DblIntQuadDefinitions()

defs.nStateDims = 6;        % (#) number of dimensions of state space
defs.stateLabels = {...
    'x';...
    'y';...
    'z';...
    'vx';...
    'vy';...
    'vz'};

defs.nControlDims = 3;      % (#) number of dimensions of control space 
defs.controlLabels = {...
    'ax';...
    'ay';...
    'az'};

defs.nRobotParams = 3;      % (#) number of parameters that define and remain constant for the robot
defs.robotParamLabels = {...
    'mass';...
    'thrustMax';...
    'maxPitchRoll'};

% system-specific functions
defs.CustomBVPOptimizer = @DblIntQuadOptimizer;
defs.TrajectorySmoother = @DblIntQuadConstrainedSmoother;
defs.CommsWriter = @DblIntQuadCommunicator;
% NOTE: other functions are not specified here but are called via
%       hard coding the names. This is because the other functions
%       (e.g. InitialGuess) do not have the same output format
%       so I don't want them to be confused with similarly titled 
%       functions in other problems.

end
