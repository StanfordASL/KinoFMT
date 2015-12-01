% DeepSpace_DataGenScript.m is used to generate training data to be used for a
% machine learning algorithm that classifies reachability sets.
%
%   Ross Allen, ASL, Stanford University
%   Dec 10, 2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

filename = 'DeepSpaceTestingData-CS229Final-Dec11-1.txt';
m = 100;                     % number of training examples
n = 6;                         % dimension of attribute variable
numerics.n_nodes = 30;         % number of discretization nodes

% extents of attributes to be tested
xf_extent = [0 10];         %(m)  x extents
yf_extent = [0 10];         %(m)  y extents
zf_extent = [0 10];         %(m)  y extents
xdot0_extent = [-2 2];         %(m)  x extents
ydot0_extent = [-2 2];         %(m)  y extents
zdot0_extent = [-2 2];         %(m)  y extents

% constants
x0 = 0;
y0 = 0;
z0 = 0;
xdotf = 0;
ydotf = 0;
zdotf = 0;
ThrustMax = 1;          %(N) maximum thrust
mass = 1;               %(kg) mass
extents = [xf_extent; yf_extent; zf_extent; xdot0_extent; ydot0_extent; zdot0_extent];

% Generate training data input values
halset = haltonset(n, 'Skip', 11000);            % halton set to sample input ranges
halset = scramble(halset, 'RR2'); % scramble the set with reverse-rad
halset = net(halset, m);   % extract first m samples of scrambled set
inputset = halset*diag([-1 1]*extents') + ones(m,n)*diag(extents(:,1));

% Environment
environment = [];

% Set solver options
options.print_summary = false; % (boo)
options.plot_results = false;  % (b00)
options.storeExitFlag = true;
options.storeSolverOutput = false;
options.solver = optimset('Algorithm','sqp','GradObj','on',...
    'GradConstr','on','DerivativeCheck','off', 'Display', 'off',...
    'TolFun', 1e-6, 'TolCon', 1e-6, 'TolX', 1e-6);
options.solver.MaxFunEvals = Inf; 
options.solver.MaxIter = 1000;

% Solve optimal control problem
outputset = NaN*ones(m,1);
exitflags = NaN*ones(m,1);
tic
for i = 1:m
    i
    
    % Set robot values
    robot.ThrustMax = ThrustMax;
    robot.mass = mass;
    
    % Set bounday values
    bv = inputset(i,:);
    boundary_values.t0 = 0;
    boundary_values.x0 = x0;
    boundary_values.y0 = y0;
    boundary_values.z0 = z0;
    boundary_values.xf = bv(1);
    boundary_values.yf = bv(2);
    boundary_values.zf = bv(3);
    boundary_values.xdot0 = bv(4);
    boundary_values.ydot0 = bv(5);
    boundary_values.zdot0 = bv(6);
    boundary_values.xdotf = xdotf;
    boundary_values.ydotf = ydotf;
    boundary_values.zdotf = zdotf;
    
    % Consolidate
    probinfo.numerics = numerics;
    probinfo.robot = robot;
    probinfo.boundary_values = boundary_values;
    probinfo.environment = environment;
    probinfo.options = options;
    clear robot boundary_values
    
    % Call Solver
    probinfo = DeepSpaceOptimizer(probinfo);
    
    % Save optimal cost and exit flag
    exitflags(i) = probinfo.solution.exitflag;
    outputset(i) = probinfo.solution.cost;
    
    clear probinfo
end
toc

% Save Data
trainingdata = [(1:m)' inputset outputset exitflags];

%% Write to file
dlmwrite(filename, trainingdata,'delimiter',';','precision', 5,  'newline', 'pc')