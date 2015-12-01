%   DSSCRecordSolution.m: Store and dimensionalize results data
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        9/24/2014
%
%   Inputs:         probinfo.scale,robot       dimensionalizatoin and other input data
%                   vars_opt                  optimal solution
%                   J_opt                     optimum cost
%                   exitflag                  (opt) exit flag of fmincon
%                   solveroutput              (opt) solver output such as iterations, function evals, etc
%
%   Outputs:        probinfo.solution          solution data               

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = DSSCRecordSolution(vars_opt, J_opt, probinfo,...
    exitflag, solveroutput)

range = probinfo.numerics.range;
scale = probinfo.scale;
[x,y,z,xdot,ydot,zdot,ux,uy,uz,eta,tf] = DSSCDecomposeVariables(vars_opt, range);
t0 = probinfo.boundary_values.non_dim.t0;
initnondim = probinfo.initial_guess.non_dim;


% initial guess
init(range.x) = scale.R.*initnondim(range.x);   % m
init(range.y) = scale.R.*initnondim(range.y);   % m
init(range.z) = scale.R.*initnondim(range.z);   % m
init(range.xdot) = (scale.R/scale.t).*initnondim(range.xdot);   % m/s
init(range.ydot) = (scale.R/scale.t).*initnondim(range.ydot);   % m/s
init(range.zdot) = (scale.R/scale.t).*initnondim(range.zdot);   % m/s
init(range.ux) = initnondim(range.ux);   % #
init(range.uy) = initnondim(range.uy);   % #
init(range.uz) = initnondim(range.uz);   % #
init(range.eta) = initnondim(range.eta);   % #
init(range.tf) = initnondim(range.tf)*probinfo.scale.t;  %s



% Solution
sol.x = scale.R.*x;                             % m
sol.y = scale.R.*y;                             % m
sol.z = scale.R.*z;                             % m
sol.xdot = (scale.R/scale.t).*xdot;             % m/s
sol.ydot = (scale.R/scale.t).*ydot;             % m/s                
sol.zdot = (scale.R/scale.t).*zdot;             % m/s  
sol.ux = ux;                                    % #
sol.uy = uy;                                    % #                
sol.uz = uz;                                    % # 
sol.eta = eta;                                  % #
sol.t = probinfo.scale.t*(tf - t0)*probinfo.numerics.t;   % s
sol.t = sol.t + probinfo.scale.t*t0;
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
