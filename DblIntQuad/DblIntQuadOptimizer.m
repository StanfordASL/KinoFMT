% DblIntQuadCustomOptimization returns the optimal trajectory, control,
%   and cost for the linear quadrotor model (double integrator) 
%   
%
%   Author: Ross Allen
%   Date:   Apr 15, 2015
%
%   Inputs:
%
%   Outputs:
%       - solution.:
%           cost = (s) final time + weighted control application
%           (stateLabels) = optimal state at discrete times
%           (controlLabels) = optimal control at discrete times
%           exitflag = status at exit (positive value = success)
%
%   Notes:
%       - Assumes a control-penalizing cost function (as opposed to
%           control-constraints)
%       - while the analysis for optimization is performed on a
%           weighted minimum-time, minimum-control-effort cost
%           function, only the final time is reported as cost. 
%           TODO: record control effort to check for control
%           constraint violations
%       - asserts linear spacing of time vector of solution. This
%           differs from other approaches that use Chebyshev 
%           pseudospectral method that generates a cosine
%           spaced time vector 
%           (maybe could change this and just scal whatever time 
%           vector is generated in numerics. haven't decided which 
%           approach is better, yet).
%       - asserts a NED frame with a positive gravity constant
%       - note that this function serves a slightly different role 
%           than similarly titled functions for other systems.
%       - See Scmerling, Janson, Pavone: Optimal Sampling-Based 
%           Motion Planning under Differential Constraints: the 
%           Drift Case with Linear Affine Dynamics
%       - See Webb and van den Berg: Kinodynamic RRT*: Asymptotically
%           Optimal Motion Planning for Robots with Linear Dynamics
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = DblIntQuadOptimizer(probinfo, optFuncs)

% initialize outputs
% probinfo.solution = NaN;
probinfo.solution.exitflag = -2;
probinfo.solution.output = '';

% generate initial interval
%   NOTE: optFuncs is not used because output of DblIntQuadInitialGuess
%           is not standard for the InitialGuess Functions
[tf_min, tf_max] = DblIntQuadInitialGuess(probinfo);

% perform error checking
if tf_min < 0 || tf_min >= tf_max || isinf(tf_max) || ...
    isnan(tf_max-tf_min)

    probinfo.solution.output =...
         'time interval must be positive and finite';
    return;
end

% extract boundary values
bv = probinfo.boundary_values;
X0 = [bv.x0; bv.y0; bv.z0; bv.vx0; bv.vy0; bv.vz0];
Xf = [bv.xf; bv.yf; bv.zf; bv.vxf; bv.vyf; bv.vzf];

% extract control effort penalty weight
wR = probinfo.options.controlPenaltyWeight;

% hard code gravity constant that assumes NED frame (g positive)
g = 9.80665;    % (m/s/s)

% Calculate derivative of cost function at ends of time interval
ta = tf_min;
tb = tf_max;
dJa = DblIntQuadCostDerivative(tf_min, X0, Xf, wR, g);
dJb = DblIntQuadCostDerivative(tf_max, X0, Xf, wR, g);

if dJa*dJb > 0
    % derivative of cost function is same sign at both ends. 
    % invalid for bisection method
    probinfo.solution.output =...
         'Invalid time interval. Cost func sign matching';
    return;
end

% Begin iterations of bisection search
iter = 1;
solnFound = 0;
while iter < probinfo.options.solver.MaxIter

    % TODO: perform outer loop bisection search for optimal time
    %   while checking for control constraints.

    % evaluate middle of interval
    tc = (ta + tb)/2;
    dJc = DblIntQuadCostDerivative(tc, X0, Xf, wR, g);

    % check exit conditions
    if abs(dJc) < probinfo.options.solver.TolFun
        solnFound = true;
        sol.exitflag = 1;
        break;
    elseif (tb-ta)/2 < probinfo.options.solver.TolX
        solnFound = true;
        sol.exitflag = 2;
        break;
    end
    
    % iterate
    iter = iter + 1;

    if dJa*dJc > 0
        ta = tc;
        dJa = dJc;
    else
        tb = tc;
        dJb = dJc;
    end
 
end

if ~solnFound
    probinfo.solution.output = 'max iterations exceeded';
    probinfo.solution.exitflag = 0;
    return;
end

% time optimization succesful
sol.output = 'Successful optimization';

% generate linearly spaced time vector
tau = tc;
sol.t = linspace(0, tau, probinfo.numerics.n_nodes)';

% Caclculate optimal cost (Webb and van den Berg eqn 11)
xbar_tau = DblIntQuadXbar(tau, X0, g);
G_tau = DblIntQuadGramian(tau, wR);
d_tau = G_tau\(Xf-xbar_tau);

fval = tau + (Xf - xbar_tau)'*d_tau;

% calculate optimal trajectory at discrete times
for i = 1:probinfo.numerics.n_nodes

    % optimal cost
    %   NOTE: the cost recorded is just the final time, not the
    %   weighted integral of time and control effort. This is 
    %   because we are only concerned with final time and control
    %   effort is only included in the cost function as a relaxation
    %   of control constraints. It is important to not confuse
    %   these two notions of cost
    sol.cost = tau;

    % optimal states
    optState_ti = DblIntQuadOptState(sol.t(i), tau, X0, Xf, wR, g, d_tau);
    sol.x(i,1) = optState_ti(1,1);
    sol.y(i,1) = optState_ti(2,1);
    sol.z(i,1) = optState_ti(3,1);
    sol.vx(i,1) = optState_ti(4,1);
    sol.vy(i,1) = optState_ti(5,1);
    sol.vz(i,1) = optState_ti(6,1);

    % optimal control
    optControl_ti = DblIntQuadOptControl(sol.t(i), tau, X0,...
                    Xf, wR, g, d_tau);
    sol.ax(i,1) = optControl_ti(1,1);
    sol.ay(i,1) = optControl_ti(2,1);
    sol.az(i,1) = optControl_ti(3,1);
end

% consolidate results
probinfo.solution = sol;

end

