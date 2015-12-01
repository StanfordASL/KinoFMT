% Dubins_InputScript calls for solution of dubins optimal control problem
%   
%   AUTHOR: Ross Allen
%   DATE:   Oct 18, 2013
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
close all
profile off

% Add Paths to Necessary Functions
addpath([pwd, '/../']);

profile on

% Numerics
numerics.n_nodes = 30; % number of control nodes

% Vehicle
robot.V = 1;                  % (m/s)
robot.turnrate = 45;          % (deg/s)

% Boundary Values
boundary_values.x0 = 0;       % (m)
boundary_values.y0 = 0;       % (m)
boundary_values.theta0 = 0;   % (deg)
boundary_values.t0 = 0;       % (s)
boundary_values.xf = 2.3267;       % (m)
boundary_values.yf = -9.3934;       % (m)
boundary_values.thetaf = 90.927; % (deg)

% Environment
environment.xbounds = [-inf inf]; % (m, m)
environment.ybounds = [-inf inf]; % (m, m)

% Options
options.print_summary = true; % (boo)
options.plot_results = true;  % (b00)
options.solver = optimset('Algorithm','sqp','GradObj','on',...
    'GradConstr','on','DerivativeCheck','off', 'Display', 'off',...
    'TolFun', 1e-10, 'TolCon', 1e-10, 'TolX', 1e-10);
options.solver.MaxFunEvals = Inf; 
options.solver.MaxIter = 2000;
options.storeExitFlag = true;
options.storeSolverOutput = true;

% Consolidate Problem Information
dubprob.numerics = numerics;
dubprob.robot = robot;
dubprob.boundary_values = boundary_values;
dubprob.environment = environment;
dubprob.options = options;
clear numerics robot boundary_values environment options

% Call Solver
dubprob = DubinsOptimizer(dubprob);

profile viewer
