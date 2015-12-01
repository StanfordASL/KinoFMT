% Dubins_fmtPlan_PreCompScript.m peforms the precomputation necessary to
% solve a kinodynamic motion planning problem with Fast Marching Trees
%
%   Ross Allen, ASL, Stanford University
%   Nov 8, 2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clear
clc
close all

% Problem Parameters
n_samples = 100;                    %(#) number of sampled nodes in statespace
n_goalSamples = 5;                  %(#) ensure you sample from goal region
n_optControl = 10;                  %(#) number of disctrete points for optimal control subproblems
xinit = [-5; -5; 0];                %(m, m, deg) initial state
Xgoal = [4 6; 4 6; 160 200];        %(m, m, deg) goal region
xbounds = [-10 10];                 %(m m) x boundaries
ybounds = [-10 10];                 %(m m) y boundaries
num_dim = length(xinit);            %(#) dimension of state space
vel = 1;                            %(m/s) velocity of dubins
turnrate = 45;                      %(deg/s) max turnrate of dubins

% Sample Vertex Set (offline)
config_space = [xbounds; ybounds; [-360 360]];
V = HaltonSampling(num_dim, n_samples, config_space, 0, 0, true);
Vgoal = HaltonSampling(num_dim, n_goalSamples, Xgoal, 0, 0, true);
V = [V; Vgoal];
n_samples = n_samples + n_goalSamples;

% Precompute Costs (offline)
numerics.n_nodes = n_optControl;
environment.xbounds = xbounds;
environment.ybounds = ybounds;
options.print_summary = false; % (boo)
options.plot_results = false;  % (b00)
options.storeExitFlags = true;
options.storeSolverOutput = true;
options.solver = optimset('Algorithm','sqp','GradObj','on',...
    'GradConstr','on','DerivativeCheck','off', 'Display', 'off',...
    'TolFun', 1e-6, 'TolCon', 1e-6, 'TolX', 1e-6);
options.solver.MaxFunEvals = Inf; 
options.solver.MaxIter = 1000;
robot.V = vel;
robot.turnrate = turnrate;
costMatrix = NaN*ones(n_samples, n_samples);
trajMatrix = cell(n_samples, n_samples);
for i = 1:n_samples
    
    bv1 = V(i,:);
    boundary_values.x0 = bv1(1);
    boundary_values.y0 = bv1(2);
    boundary_values.theta0 = bv1(3);
    boundary_values.t0 = 0;
    
    for j = 1:n_samples
        disp([num2str(i) ', ' num2str(j)])
        
        if i == j
            costMatrix(i,j) = 0;
            trajMatrix{i,j} = [bv1(1)*ones(n_optControl,1) bv1(2)*ones(n_optControl,1) bv1(3)*ones(n_optControl,1)];
            continue;
        end
        
        % Set bounday values
        bv2 = V(j,:);
        boundary_values.xf = bv2(1);
        boundary_values.yf = bv2(2);
        boundary_values.thetaf = bv2(3);
        
        % Consolidate
        clear dubprob
        dubprob.numerics = numerics;
        dubprob.robot = robot;
        dubprob.boundary_values = boundary_values;
        dubprob.environment = environment;
        dubprob.options = options;
        
        % Call Solver
        dubprob = DubinsOptimizer(dubprob);
        
        % Save Cost and Trajectory
        costMatrix(i,j) = dubprob.solution.cost;
        if dubprob.solution.exitflag <= 0
            costMatrix(i,j) = Inf;
        end
        trajMatrix{i,j} = [dubprob.solution.x dubprob.solution.y dubprob.solution.theta];
    end
end



