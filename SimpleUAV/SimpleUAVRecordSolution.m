%   SimpleUAVRecordSolution.m: Store and dimensionalize results data
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        12/7/2013
%
%   Inputs:         probinfo.scale,robot       dimensionalizatoin and other input data
%                   vars_opt                  optimal solution
%                   J_opt                     optimum cost
%                   exitflag                  (opt) exit flag of fmincon
%                   solveroutput              (opt) solver output such as iterations, function evals, etc
%
%   Outputs:        probinfo.solution          solution data               

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = SimpleUAVRecordSolution(vars_opt, J_opt, probinfo,...
    exitflag, solveroutput)

range = probinfo.numerics.range;
scale = probinfo.scale;
[x,y,z,theta,utheta,uz,tf] = SimpleUAVDecomposeVariables(vars_opt, range);
t0 = probinfo.boundary_values.non_dim.t0;
initnondim = probinfo.initial_guess.non_dim;


% initial guess dimensionalized
init(range.x) = scale.R.*initnondim(range.x);   % m
init(range.y) = scale.R.*initnondim(range.y);   % m
init(range.z) = scale.R.*initnondim(range.z);   % m
init(range.theta) = initnondim(range.theta);    % rad
init(range.utheta) = initnondim(range.utheta)/scale.t;          % rad/s
init(range.uz) = initnondim(range.uz)*scale.R/scale.t;          % m/s
init(range.tf) = initnondim(range.tf)*scale.t;  %s



% Solution
sol.x = scale.R.*x;                             % m
sol.y = scale.R.*y;                             % m
sol.z = scale.R.*z;                             % m
sol.theta = theta;                              % rad
sol.utheta = utheta/scale.t;                    % rad/s               
sol.uz = uz*scale.R/scale.t;                    % m/s 
sol.t = probinfo.scale.t*(tf - t0)*probinfo.numerics.t;   % s
sol.t = sol.t + probinfo.scale.t*t0;

% Optimal Cost
sol.cost = probinfo.scale.t*J_opt;

probinfo.solution = sol;
probinfo.initial_guess.rescaled = init;

if nargin >= 4
    probinfo.solution.exitflag = exitflag;
end

if nargin >= 5
    probinfo.solution.solveroutput = solveroutput;
end

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%