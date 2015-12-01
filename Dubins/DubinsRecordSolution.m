%   DubinsRecordSolution.m: Store and dimensionalize results data
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        10/21/2013
%
%   Inputs:         dubprob.scale,robot       dimensionalizatoin and other input data
%                   vars_opt                  optimal solution
%                   J_opt                     optimum cost
%                   exitflag                  (opt) exit flag of fmincon
%                   solveroutput              (opt) solver output such as iterations, function evals, etc
%
%   Outputs:        dubprob.solution          solution data               

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dubprob = DubinsRecordSolution(vars_opt, J_opt, dubprob,...
    exitflag, solveroutput)

[x,y,theta,u,tf] = DubinsDecomposeVariables(vars_opt, dubprob.numerics.range);
t0 = dubprob.boundary_values.non_dim.t0;
initnondim = dubprob.initial_guess.non_dim;
range = dubprob.numerics.range;

% initial guess
init(range.x) = dubprob.scale.R.*initnondim(range.x);   % m
init(range.y) = dubprob.scale.R.*initnondim(range.y);   % m
init(range.theta) = initnondim(range.theta)*180/pi();   % deg
init(range.u) = initnondim(range.u)*180/pi()/dubprob.scale.t; % deg/s
init(range.tf) = initnondim(range.tf)*dubprob.scale.t;  %s



% Solution
sol.x = dubprob.scale.R.*x;                             % m
sol.y = dubprob.scale.R.*y;                             % m
sol.theta = theta*180/pi();                             % deg
sol.u = u*180/pi()/dubprob.scale.t;                     % deg/s
sol.t = dubprob.scale.t*(tf - t0)*dubprob.numerics.t;   % s
sol.t = sol.t + dubprob.scale.t*t0;
sol.cost = dubprob.scale.t*J_opt;

dubprob.solution = sol;
dubprob.initial_guess.rescaled = init;

if nargin >= 4
    dubprob.solution.exitflag = exitflag;
end

if nargin >= 5
    dubprob.solution.solveroutput = solveroutput;
end

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
