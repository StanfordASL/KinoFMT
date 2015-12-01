% SimpleUAV_InputScript calls for solution of deep space spacecraft optimal
% control problem
%   
%   AUTHOR: Ross Allen
%   DATE:   Apr 23, 2014
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
close all

% Add Paths to Necessary Functions
addpath([pwd, '/../']);

% Numerics
numerics.n_nodes = 30; % number of control nodes

% Vehicle
robot.Vhor = 1;                     % (m/s) constant horizontal velocity
robot.turnrate = pi()/4;            % (rad/s) max turn rate
robot.climbrate = 0.5;              % (m/s) max climb rate

% Boundary Values
boundary_values.x0 = 0;       % (m)
boundary_values.y0 = 0;       % (m)
boundary_values.z0 = 0;       % (m)
boundary_values.theta0 = 0;   % (rad)
boundary_values.t0 = 0;       % (s)
boundary_values.xf = 5;       % (m)
boundary_values.yf = 0;       % (m)
boundary_values.zf = 0;       % (m)
boundary_values.thetaf = 8*pi();   % (rad)


% Environment
environment = [];


% Options
options.print_summary = true; % (boo)
options.plot_results = true;  % (b00)
options.solver = optimset(...
    'Algorithm',        'sqp',...
    'GradObj',          'on',...
    'GradConstr',       'on',...
    'DerivativeCheck',  'off',...
    'Display',          'on',...
    'MaxFunEvals',      Inf,...
    'MaxIter',          200,...
    'TolFun',           1e-5,...
    'TolCon',           1e-5,...
    'TolX',             1e-5...
    );
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
probinfo = SimpleUAVOptimizer(probinfo);