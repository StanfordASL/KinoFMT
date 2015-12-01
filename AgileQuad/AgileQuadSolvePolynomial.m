% AgileQuadSolvePolynomial performs the matrix operations to solve for polynomial coefs
%
%	AUTHOR: Ross Allen
%	DATE:	Feb 20, 2015
%
%	NOTES:
%		See Ricther et. al. "Polynomial Trajectory Planning for Aggressive ..." Sec 3.1-3.3
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function p_coefs =...
            AgileQuadSolvePolynomial( polytrajinfo, Q, A, d, d_fixed, ...
            orderMat, matchMat_A, matchMat_d )

% Given Params
nVar = polytrajinfo.nVar;
nFixed = polytrajinfo.nFixed;

% Error Check

% Form A matrix and d vector  for unconstrained formulation
Au = orderMat*matchMat_A*A;
du = orderMat*matchMat_d*d;

% Form R matrix
% NOTE: this may be accelerated by calcing A^-1 once and reusing
R = ((Au')\Q)*(Au\eye(nVar));

% Form partitons of R and d
R_PP = R(nFixed+1:end,nFixed+1:end);
R_FP = R(1:nFixed,nFixed+1:end);
du_F = du;
du_F(nFixed+1:end,:) = [];

% Solve for free derivatives
du_P = R_PP\(R_FP');
du_P = -du_P*du_F;

% Map back to polynomial coefs
du_opt = [du_F; du_P];
p_coefs = Au\du_opt;

end
