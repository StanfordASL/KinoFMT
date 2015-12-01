% PolySolverTestScript.m is a script to develop and test a solver for polynomial trajectories
%
%	AUTHOR: Ross Allen
%	DATE:	Feb 9, 2015
%
%	NOTES:
%		See Ricther et. al. "Polynomial Trajectory Planning for Aggressive ..." to see motivation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

% Params
N = 9;		% Order of polynomial
nCoef = N+1;	% number of coefficients
minOrder = 4;	% order of derivative to be minimized
nSeg = 2;		% number of polynomial segments
nNodes = nSeg+1;% number of nodes to be connected
nFree = 8;     % total number of free variabled
nFixed = nSeg*nCoef-nFree; % total number of fixed variables

% Position and velocities to be met
%pos = cell(1,nSeg+1); 
pos = {[-1,0], [0,1], [1,0]};
%vel = {[0,1], [1,0], [0,-1]};
vel = cell(1,nSeg+1);
vel{1} = [0,1]; vel{3} =  [0,-1];
acc = cell(1,nSeg+1);
acc{1} = [0,0]; acc{3} =  [0,0];
jerk = cell(1,nSeg+1);
jerk{1} = [0,0]; jerk{3} =  [0,0];
snap = cell(1,nSeg+1);
snap{1} = [0,0]; snap{3} =  [0,0];
T = {0, 14.1421, 28.2843};

% Form Q matrix for snap minimization
for l = 1:nSeg
	Q{l} = zeros(nCoef,nCoef);
	for i = minOrder:nCoef
		for j = minOrder:nCoef
			Q{l}(i,j) = factorial(i)*factorial(j)*(T{l+1}^(i+j+1-2*minOrder) - ...
				T{l}^(i+j+1-2*minOrder))/(factorial(i-minOrder)*factorial(j-minOrder)*...
				(i+j+1-2*minOrder));
		end
	end
end
Qtot = blkdiag(Q{1}, Q{2});

% Form general A matrix
A = cell(1,nSeg);
for l = 1:nSeg
    A_T0 = zeros(nCoef/2, nCoef);
    A_T1 = zeros(nCoef/2, nCoef);
    for k = 0:nCoef/2-1
        %dk = k-1; % order of derivative
        for i = 0:N
            %pow = i-1;   % order of polynomial term
            if i >= k
                A_T0(k+1,i+1) = factorial(i)*T{l}^(i-k)/factorial(i-k);
                A_T1(k+1,i+1) = factorial(i)*T{l+1}^(i-k)/factorial(i-k);
            end
        end
    end
    A{1,l} = [A_T0; A_T1];
end
Atot = blkdiag(A{1,1}, A{1,2});


% Form general d vector
dxtot_uncon = [  ...
        pos{1}(1);...   % fixed position 1
        vel{1}(1);...   % fixed velocity 1
        acc{1}(1);...   % fixed acceleration 1
        jerk{1}(1);...  % fixed jerk 1
        snap{1}(1);...  % fixed snap 1
        pos{2}(1);...   % fixed position 2
        NaN;...         % free velocity 2
        NaN;...         % free acceleration 2
        NaN;...         % free jerk 2
        NaN;...         % free snap 2
        pos{2}(1);...   % fixed position 2
        NaN;...         % free velocity 2
        NaN;...         % free acceleration 2
        NaN;...         % free jerk 2
        NaN;...         % free snap 2
        pos{3}(1);...   % fixed position 3
        vel{3}(1);...   % fixed velocity 3
        acc{3}(1);...   % fixed acceleration 3
        jerk{3}(1);...  % fixed jerk 3
        snap{3}(1);...  % fixed snap 3
    ];

% Reform Atot for unconstrained formulation
Atot_uncon = Atot;

% Include matching constraints
for l = 1:nSeg-1
    if mod(nCoef,2)
        % nCoef not even
        disp('nCoef not even')
        break;
    end
    for i = nCoef/2:nCoef
        if isnan(dxtot_uncon((l-1)*nCoef+i,1))
            Atot_uncon((l-1)*nCoef+i,:) = ...
                Atot_uncon((l-1)*nCoef+i,:) - ...
                Atot_uncon(l*nCoef+i-nCoef/2,:);
        end
    end
end

% Reorder to place free constraints at bottom
Cmat = zeros(size(Atot_uncon));
fixedInd = 1;
freeInd = nFixed+1;
for i = 1:nCoef*nSeg
    if isnan(dxtot_uncon(i,1))
        Cmat(freeInd,i) = 1;
        freeInd = freeInd + 1;
    else
        Cmat(fixedInd,i) = 1;
        fixedInd = fixedInd + 1;
    end
end

% Form R matrix
% Note this may be accelerated by calcing A^-1 once and reusing
R = Cmat*((Atot_uncon')\Qtot)*(Atot_uncon\(Cmat'));

% Form partitons of R and d
R_PP = R(nFixed+1:end,nFixed+1:end);
R_FP = R(1:nFixed,nFixed+1:end);
d_F = dxtot_uncon;
d_F(isnan(d_F)) = 0;
d_F = Cmat*d_F;
d_F(nFixed+1:end) = [];

% Solve for free derivatives
d_P = R_PP\(R_FP');
d_P = -d_P*d_F;

% Map back to polynomial coefs
d_opt = [d_F; d_P];
p_coefs = Atot_uncon\(Cmat');
p_coefs = p_coefs*d_opt;

% Form A matrix and d vector for constraints (constrained form)
%dxtot_con = [  pos{1}(1);...   % fixed position
%        vel{1}(1);...   % fixed velocity
%        0.0;...         % fixed acceleration
%        0.0;...         % fixed jerk
%        0.0;...         % fixed snap
%        pos{2}(1);...   % fixed position
%        vel{2}(1);...   % fixed velocity
%        0;...           % free acceleration
%        0;...           % free jerk
%        0;...           % free snap
%        pos{2}(1);...   % fixed position
%        vel{2}(1);...   % fixed velocity
%        0;...           % free acceleration
%        0;...           % free jerk
%        0;...           % free snap
%        pos{3}(1);...   % fixed position
%        vel{3}(1);...   % fixed velocity
%        0.0;...         % fixed acceleration
%        0.0;...         % fixed jerk
%        0.0;...         % fixed snap
%    ];
%
%dfixed = [1,2,3,4,5,6,7,11,12,16,17,18,19,20];
%dfree = [8,9,10,13,14,15];
%
%A_con = cell(1,nSeg);
%for l = 1:nSeg
%    for i = 1:nCoef
%        for j = 1:nCoef
%            if ismember(i,dfixed)
%                if j>=minOrder
%                    A{l}(i,j) = 
%                end 
%            else
%            end
%        end
%    end
%end
%
%% Form A matrix and d vector for unconstrained formulation
%dxtot_uncon = [  ...
%        pos{1}(1);...   % fixed position 1
%        pos{2}(1);...   % fixed position 2
%        pos{3}(1);...   % fixed position 3
%        vel{1}(1);...   % fixed velocity 1
%        vel{3}(1);...   % fixed velocity 3
%        acc{1}(1);...   % fixed acceleration 1
%        acc{3}(1);...   % fixed acceleration 3
%        jerk{1}(1);...  % fixed jerk 1
%        jerk{3}(1);...  % fixed jerk 3
%        snap{1}(1);...  % fixed snap 1
%        snap{3}(1);...  % fixed snap 3
%        0;              % matching velocity at 2
%        0;              % matching acceleration at 2
%        0;              % matching jerk at 2
%        0;              % matching snap at 2
%    ];
%
%posRows = 1:nNodes
%velRowsFixed = posRows(end)+1:posRows(end)+3;
%accRowsFixed = velRows(end)+1:velRows(end)+3;
%jerkRowsFixed = accRows(end)+1:accRows(end)+3;
%snapRowsFixed = jerkRows(end)+1:jerkRows(end)+3;
%matchRows = snapRowsFixed(end)+1:snapRowsFixed(end)+1+4*(nNodes-2);
%velRowsFree = matchRows(end)+1:matchRows(end)+1+(nNodes-2);
%accRowsFree = velRowsFree(end)+1:velRowsFree(end)+1+(nNodes-2);
%jerkRowsFree = accRowsFree(end)+1:accRowsFree(end)+1+(nNodes-2);
%snapRowsFree = jerkRowsFree(end)+1:nCoef*nSeg;
%
%Atot_uncon = zeros(nCoef*nSeg,nCoef*nSeg);
%for i = 1:nCoef*nSeg
%    if ismember(i, posRows)
%    elseif ismember(i, velRowsFixed)
%    end
%end
%
%Apos = ones(1,nCoef);
%Aderiv = cell(minOrder,1);
%for i = 1:nCoef
%    for k = 1:minOrder
%        if i >= k
%            Aderiv{k}(1,i) = factorial(i)/factorial(i-k);
%        end
%    end
%end
%
%
%T1 = T{1}; T2 = T{2}; T3 = T{3};
%Ax = [1, T1, T1^2, T1^3, T1^4, T1^5, T1^6, T1^7, T1^8, T1^9;...
%    0, 1, 2*T1, 3*T1^2, 4*T1^3, 5*T1^4, 6*T1^5, 7*T1^6, 8*T1^7, 9*T1^8;...
%    0, 0, 1, 6*T 
        

