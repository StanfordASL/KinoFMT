%   Dubins_SL1QP_Script attempts to solve the Dubins vehicle problem with
%   sequential l_1 Quadratic Programming (Sequential Convex Programming)
%
%   Ross Allen, ASL, Stanford University
%   Started: Oct 31, 2013
%
%   References:
%       - S. Boyd, "Convex Optimization II Lecture Notes: Sequential Convex
%       Optimization", 
%       http://www.stanford.edu/class/ee364b/lectures/seq_slides.pdf
%       - J. Schulman, J. Ho, A. Lee, I. Awwal, H. Bradlow, P. Abbeel,
%       "Finding Locally Optimal , Collision-Free Trajectories with
%       Sequential Convex Optimization"
%       - J. Nocedal, S. Wright, "Numerical Optimization: 2nd Ed", Section
%       18.8
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

% Dubins Parameters
numerics.n_nodes = 30;              % number of control nodes
robot.V = 1;                        % (m/s)
robot.turnrate = 45;                % (deg/s)
boundary_values.x0 = 0;           % (m)
boundary_values.y0 = 0;           % (m)
boundary_values.theta0 = 0;      % (deg)
boundary_values.t0 = 0;             % (s)
boundary_values.xf = 0;          % (m)
boundary_values.yf = 8/pi();        % (m)
boundary_values.thetaf = 180;    % (deg)
environment.xbounds = [-inf inf];   % (m, m)
environment.ybounds = [-inf inf];   % (m, m)

% Consolidate Problem Information
dubprob.numerics = numerics;
dubprob.robot = robot;
dubprob.boundary_values = boundary_values;
dubprob.environment = environment;
clear numerics robot boundary_values environment

% SQP Parameters
n_nodes = dubprob.numerics.n_nodes;
mu_0 = 0.5;               % initial penalty coefficient
% s_0 = [100*ones(2*n_nodes,1);...
%        pi()/12*ones(n_nodes,1);...
%        100*ones(n_nodes,1);...
%        2];% initial trust region size
s_0 = [pi()/4*ones(n_nodes,1); 2];% initial trust region size (just theta and tf)
c = 0.1;                % step acceptance parameter
tau_p = 1.1;            % trust region expansion factor
tau_m = 0.5;            % trust region shrinkage factor
k = 2;                  % penalty scaling factor
ftol = 1e-3;            % merit convergence threshold
xtol = 1e-3;            % solution convergence threshold
ctol = 1e-3;            % constraint satisfaction threshold
maxiter = 100;         % max number of iterations
cvx_quiet(true);

% Preprocessing
dubprob = DubinsScaleFactors(dubprob);
dubprob = DubinsNumericData(dubprob);
dubprob = DubinsBoundaryValuesData(dubprob);
dubprob = DubinsRobotData(dubprob);
dubprob = DubinsEnvironmentData(dubprob);
dubprob.numerics.n_ineqC.convex = length(DubinsConvexInequalityConstraints(...
    zeros(dubprob.numerics.n_vars,1), dubprob));
dubprob.numerics.n_ineqC.nonconvex = length(DubinsNonConvexInequalityConstraints(...
    zeros(dubprob.numerics.n_vars,1), dubprob));
dubprob.numerics.n_eqC.affine = length(DubinsAffineEqualityConstraints(...
    zeros(dubprob.numerics.n_vars,1), dubprob));
dubprob.numerics.n_eqC.nonaffine = length(DubinsNonAffineEqualityConstraints(...
    zeros(dubprob.numerics.n_vars,1), dubprob));
range = dubprob.numerics.range;

% Initialize variables
[oldX, dubprob] = DubinsFeasibleInitialGuess(dubprob, ...
    @DubinsAffineEqualityConstraints,...
    @DubinsConvexInequalityConstraints);
costVec(1) = DubinsCostFunction(oldX, dubprob);
nonAffineEq = DubinsNonAffineEqualityConstraints(oldX, dubprob);
mu = mu_0;
% mu = 10*costVec(1)/norm(nonAffineEq,1);
s = s_0;
oldphi = costVec(1) + mu*norm(nonAffineEq, 1);
phi(1) = oldphi;
phihat(1) = 0;
i = 2;
for PenaltyIteration = 1:maxiter
    if PenaltyIteration == maxiter
        disp('max iteration reached in PenaltyIteration');
        return;
    end
    
    for ConvexifyIteration = 1:maxiter
        if ConvexifyIteration == maxiter
            disp('max iteration reached in ConvexifyIteration');
            return;
        end
        
        % Convexify
        apprxAffineEq = @(Xvar) DubinsNonAffineEqualityConstraints(oldX, dubprob) + ...
            transpose(ComplexStepGradients(@DubinsNonAffineEqualityConstraints,...
            oldX, dubprob.numerics.n_eqC.nonaffine, dubprob))*(Xvar - oldX);
        
        for TrustRegionIteration = 1: maxiter
            if TrustRegionIteration == maxiter
                disp('max iteration reached in TrustRegionIteration');
                return;
            end
            
            % Find X that minimizes convexified problem
            cvx_begin
                variable X(dubprob.numerics.n_vars, 1)
%                 variable T(dubprob.numerics.n_ineqC,1) % slack variabl
                
                f_X = DubinsCostFunction(X, dubprob);
                g_X = DubinsConvexInequalityConstraints(X, dubprob);
                h_X = DubinsAffineEqualityConstraints(X, dubprob);
                h_hat_X = apprxAffineEq(X);
                
                minimize(f_X + mu*norm(h_hat_X,1))
%                 minimize(f_X + mu*(sum(T) + norm(h_X,1)))
                
                
                subject to
                    % Slack Constraint to use positive 1-norm (see J. Schulman, eqn 6)
%                     T >= 0;
%                     T >= g_X;

                    % Convex Inequality Constraints
                    g_X <= 0;
                    
                    % Affine Equality Constraints
                    h_X == 0;
                    
                    % Trust Region Constraint
                    abs(X([range.theta range.tf]) -...
                        oldX([range.theta range.tf])) <= s;        
            cvx_end
            
            % True Improvement (decrease with exact objective)
            costVec(i) = DubinsCostFunction(X, dubprob);
            nonAffineEq = DubinsNonAffineEqualityConstraints(X, dubprob);
            phi(i) = costVec(i) + mu*norm(nonAffineEq, 1);
            TrueImprove = oldphi - phi(i);
            
            % Model Improvement (predicted decrease)
            phihat(i) = cvx_optval;
            ModelImprove = oldphi - phihat(i);
            
            % Determine success or failure and change trust region
            if TrueImprove/ModelImprove <= c
                % Failure, shrink trust region
                s = s*tau_m;
            else
                % Success, grow trust region, propogate system
                s = s*tau_p;
                deltaX = norm(oldX - X);
                deltaphi = norm(oldphi - phi(i));
                oldX = X;
                oldphi = phi(i);
                i = i+1;
                break;
            end
            
        end
        
        % Check for satisfaction of merit or variable thresholds
        if deltaX < xtol || deltaphi < ftol
            break;
        end
        
    end
    
    % Check for satisfaction of constraint thresholds
    if norm(nonAffineEq) < ctol
        break;
    else   
        mu = k*mu;
    end
        
end
