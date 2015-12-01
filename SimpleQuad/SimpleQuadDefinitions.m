%SimpleQuadDefinitions returns defining constants, attributes, labels, etc
%for the Simple Quadrotor optimization problem
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
function defs = SimpleQuadDefinitions()

defs.nStateDims = 6;        % (#) number of dimensions of state space
defs.stateLabels = {...
    'x';...
    'y';...
    'z';...
    'vx';...
    'vy';...
    'vz'};

defs.nControlDims = 4;      % (#) number of dimensions of control space 
defs.controlLabels = {...
    'ux';...
    'uy';...
    'uz';...
    'eta'};

defs.nRobotParams = 2;      % (#) number of parameters that define and remain constant for the robot
defs.robotParamLabels = {...
    'mass';...
    'thrustMax';...
    'maxPitchRoll'};

% system-specific functions
defs.ScaleFactors = @SimpleQuadScaleFactors;
defs.NumericData = @SimpleQuadNumericData;
defs.BoundaryValuesData = @SimpleQuadBoundaryValuesData;
defs.RobotData = @SimpleQuadRobotData;
defs.EnvironmentData = @SimpleQuadEnvironmentData;
defs.PrintSummary = @SimpleQuadPrintSummary;
defs.InitialGuess = @SimpleQuadInitialGuess;
defs.LinearConstraints = @SimpleQuadLinearConstraints;
defs.CostFunction = @SimpleQuadCostFunction;
defs.NonLinearConstraints = @SimpleQuadNonLinearConstraints;
defs.RecordSolution = @SimpleQuadRecordSolution;
defs.PlotResults = @SimpleQuadPlotResults;

end
