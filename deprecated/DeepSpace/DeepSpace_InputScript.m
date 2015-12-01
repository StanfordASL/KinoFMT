% DeepSpace_InputScript calls for solution of deep space spacecraft optimal
% control problem
%   
%   AUTHOR: Ross Allen
%   DATE:   Dec 6, 2013
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
close all

% Add Paths to Necessary Functions
addpath([pwd, '/../']);

% Numerics
numerics.n_nodes = 100; % number of control nodes

% Vehicle
robot.mass = 1;               % (kg)
robot.ThrustMax = 0.1;          % (N)

% Boundary Values
boundary_values.t0 = 0;       % (s)
boundary_values.x0 = 70;       % (m)
boundary_values.y0 = 70;       % (m)
boundary_values.z0 = 70;       % (m)
boundary_values.xdot0 = -1;       % (m/s)
boundary_values.ydot0 = -1;       % (m/s)
boundary_values.zdot0 = -1;       % (m/s)
boundary_values.xf = -70;       % (m)
boundary_values.yf = -70;        % (m)
boundary_values.zf = -70;       % (m)
boundary_values.xdotf = 0;       % (m/s)
boundary_values.ydotf = 0;       % (m/s)
boundary_values.zdotf = 0;       % (m/s)
% boundary_values.etaf = 0;     % (#)


% Environment
environment = [];
% environment.xbounds = [-inf inf]; % (m, m)
% environment.ybounds = [-inf inf]; % (m, m)

% Options
options.print_summary = true; % (boo)
options.plot_results = true;  % (b00)
options.solver = optimset('Algorithm','sqp','GradObj','on',...
    'GradConstr','on','DerivativeCheck','off', 'Display', 'on',...
    'TolFun', 1e-10, 'TolCon', 1e-10, 'TolX', 1e-10);
%     'TolFun', 5e-2, 'TolCon', 1e-4, 'TolX', 5e-2);
% options.solver.MaxFunEvals = Inf; 
options.solver.MaxIter = 200;
options.storeExitFlag = true;
options.storeSolverOutput = true;

% Consolidate Problem Information
probinfo.numerics = numerics;
probinfo.robot = robot;
probinfo.boundary_values = boundary_values;
probinfo.environment = environment;
probinfo.options = options;
clear numerics robot boundary_values environment options

% Call Solver
probinfo = DeepSpaceOptimizer(probinfo);

