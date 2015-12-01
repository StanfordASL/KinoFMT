%   SimpleQuadRecordSolution.m: Store and dimensionalize results data
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        Mar 30, 2015
%
%   Inputs:         probinfo.scale,robot       dimensionalizatoin and other input data
%                   vars_opt                  optimal solution
%                   J_opt                     optimum cost
%                   exitflag                  (opt) exit flag of fmincon
%                   solveroutput              (opt) solver output such as iterations, function evals, etc
%
%   Outputs:        probinfo.solution          solution data               

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function probinfo = SimpleQuadRecordSolution(vars_opt, J_opt, probinfo,...
    exitflag, solveroutput)

range = probinfo.numerics.range;
scale = probinfo.scale;
[x,y,z,vx,vy,vz,ux,uy,uz,eta,tf] = SimpleQuadDecomposeVariables(vars_opt, range);
t0 = probinfo.boundary_values.non_dim.t0;
initnondim = probinfo.initial_guess.non_dim;


% initial guess
init(range.x) = scale.R.*initnondim(range.x);   % m
init(range.y) = scale.R.*initnondim(range.y);   % m
init(range.z) = scale.R.*initnondim(range.z);   % m
init(range.vx) = (scale.R/scale.t).*initnondim(range.vx);   % m/s
init(range.vy) = (scale.R/scale.t).*initnondim(range.vy);   % m/s
init(range.vz) = (scale.R/scale.t).*initnondim(range.vz);   % m/s
init(range.ux) = initnondim(range.ux);   % #
init(range.uy) = initnondim(range.uy);   % #
init(range.uz) = initnondim(range.uz);   % #
init(range.eta) = initnondim(range.eta);   % #
init(range.tf) = initnondim(range.tf)*probinfo.scale.t;  %s



% Solution
sol.x = scale.R.*x;                             % m
sol.y = scale.R.*y;                             % m
sol.z = scale.R.*z;                             % m
sol.vx = (scale.R/scale.t).*vx;             % m/s
sol.vy = (scale.R/scale.t).*vy;             % m/s                
sol.vz = (scale.R/scale.t).*vz;             % m/s  
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
