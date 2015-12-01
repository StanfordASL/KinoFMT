% PolySolverTestScript_V2.m is a script to develop and test a solver for polynomial trajectories
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

% Given Params
N = 9;		% Order of polynomial
wD = zeros(N+1,1); wD(5) = 1; % derivative minimization weight
wT = 1000;    % Time minimization weight
keys = [ ...    % keypoints or waypoints for trajectory 
    -3, 0, 0, 0;...
    -2, 1, 1, 0;...
%     -1, 0, 2, 0;...
%     0, -1, 3, 0;...
%     1, 0, 4, 0;...
%     2, 1, 5, 0;...
    3, 0, 6, 0;...
    ];          % ([m], [m], [m], [rad])
epsf = 1e-9;   % epsilon for floating point comparisons
lowVel = 10;   % [m/s] low velocity for timing estimate
T0 = 0;         % [s] initial time
plotRes = true;

% Derived parameters and one-time calculations
nCoef = N+1;	% number of coefficients
nKeys = size(keys, 1);  % number of keypoints to be connected
nOut = size(keys, 2);   % number of output states (should always be 4)
nSeg = nKeys - 1;		% number of polynomial segments
nTermVar = nCoef/2; % number of variables at terminal points of seg
nVar = nCoef*nSeg;  % number of variables
nExFixed = 2*nTermVar + 2*nSeg-2; % number of explicitly fixed variables
nImFixed = (nSeg-1)*(nTermVar-1);   % num of implicitly fixed vars (matching)
nFixed = nExFixed + nImFixed;       % total num of fixed vars
nFree = nVar - nFixed;  % total number of free variables 

% Error check inputs
if length(wD) ~= N+1
    disp('error: weight vector mis-dimension')
    return;
end
if any(wD < 0) || wT < 0 || all(abs(wD) < epsf);
    disp('error: weight vector invalid')
    return;
end
if nKeys < 2
    disp('error: need at least 2 keyframes')
    return;
end
if mod(nCoef,2)
    disp('error: currently require even number of coefs')
    return;
end

% Calculate keypoint linear distances and Estimate travel times
% TODO: perform some form of time scaling to have properly conditioned matrices
linDist = diag(-ones(nSeg,1),-1) + eye(nKeys);
linDist(1,:) = zeros(1,nKeys);
linDist = linDist*keys(:,1:3);
linDist = (linDist.*linDist)*ones(3,1);
linDist = sqrt(linDist);
T = linDist./lowVel;
T = T0 + cumsum(T);
%T = [0; 1; 2];      % added for debugging

% Form constraint  vector d
% NOTE: Assumes that vel, acc, jerk, snap are fixed to zero at beginning and end    
d_exFixed = zeros(nVar, 1);
d_exFixed(1:nTermVar) = 1;
d_exFixed(end-nTermVar+1:end) = 1;
d_exFixed(1:nTermVar:end) = 1;  % explicitly fixed variables
d_imFixed = repmat([zeros(nTermVar+1,1);ones(nTermVar-1,1)],nSeg-1,1);
d_imFixed = [d_imFixed; zeros(nCoef,1)];    % implicitly fixed varaibles (matching)
d_fixed = or(d_exFixed,d_imFixed);
if any(and(d_exFixed,d_imFixed)) || length(find(d_exFixed))~=nExFixed || length(find(d_imFixed))~=nImFixed
    disp('there is a misalignment with implicit and explicit constraint IDs')
    return;
end
% variables that are fixed implicitly or explicitly

nRepKeys = 2*nKeys-2;       % number of key when repetition is counted
repKeys = keys(2:end-1,:);
repKeys = [repKeys, repKeys];
repKeys = reshape(repKeys', nOut, nRepKeys-2)';
repKeys = [keys(1,:); repKeys; keys(end,:)];    % keys with repetition included
accordion = eye(nRepKeys);
accordion = [accordion, zeros(nRepKeys, nRepKeys*(nTermVar-1))];
accordion = reshape(accordion', nRepKeys, nVar)';   % matrix for exanding columns
d = accordion*repKeys;     % hold x, y, z, yaw constraint values in one matrix


% Form Q and A  matrices for derivative minimization (sec 3.1 - 3.3)
% Calculate gradient terms for time allocation (sec 3.4 and 3.6)
% TODO: verify validity of Q, A, Au matrice
Q = zeros(nVar, nVar, nCoef);
A = zeros(nVar, nVar);
%gradJMat = zeros(nVar,nVar,nCoef);  % gradient of J wrt T
gradJMat = cell(nSeg,1);  % gradient of J wrt T
matchMat_A = eye(nVar);     % matrix to enforce matching constraint at seg trans
matchMat_d = eye(nVar);     % matrix to enforce matching on d
orderMat = zeros(nVar, nVar);   % matrix to reorder A matrix and d vector
fixedInd = 1;
freeInd = nFixed + 1;
for l = 1:nSeg;                 % segment number being worked
    baseInd = (l-1)*nCoef;      % base row/column index of Q being worked
    gradJMat{l} = zeros(nCoef,nCoef,nCoef);

    for k = 0:N                 % order of derivative
        kk = k + 1;             % index of deriv weight
        krow = kk+baseInd;      % row index from 1:nVar

        % Construct contraint matching matrices
        if l < nSeg && kk > nTermVar+1
            matchMat_A(krow,krow+nTermVar) = -1;
            matchMat_d(krow,krow) = 0;
        end

        % Construct order matrix
        if d_fixed(krow)
            orderMat(fixedInd, krow) = 1;
            fixedInd = fixedInd + 1;
        else
            orderMat(freeInd, krow) = 1;
            freeInd = freeInd + 1;
        end

        for i = k:N
            ii = baseInd+i+1;   % row index to be inserted to
            iprod = factorial(i)/factorial(i-k);

            % Calculate constraint matrix
            if kk <= nTermVar
                A(baseInd+kk,ii) = iprod*T(l,1)^(i-k);
                A(baseInd+kk+nTermVar,ii) = iprod*T(l+1,1)^(i-k);
            end

            for j = k:N
                jj = baseInd+j+1;   % col index to be inserted to
                jprod = factorial(j)/factorial(j-k);

                tempProd = wD(kk)*iprod*jprod;

                % Calculate minimization matrix
                Q(ii,jj,kk) = tempProd*...
                    (T(l+1,1)^(i+j+1-2*k)-T(l,1)^(i+j+1-2*k))/(i+j+1-2*k);
                
                % Store gradient information
                gradJMat{l}(i+1,j+1,kk) = tempProd*T(l,1)^(i+j-2*k); 
            end
        end
    end
    
    % Sum over weights for gradient info
    gradJMat{l} = sum(gradJMat{l},3);

end
Q = sum(Q,3);   % sum over derivative weights for final Q matrix

% Error check orderMat
if ~(fixedInd == nFixed+1 && freeInd == nVar+1)
    disp('check orderMat creation')
    return;
end

clear baseInd l k i j kk ii jj krow iprod jprod fixedInd freeInd tempProd

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

% Calculate next time vector
gradJ = zeros(nSeg,1);
for l = 1:nSeg-1
    baseInd = (l-1)*nCoef;
    p_l = p_coefs(baseInd+1:baseInd+nCoef,1);
    p_l1 = p_coefs(baseInd+nCoef+1:baseInd+2*nCoef,1);
    gradJ(l,1) =  p_l'*gradJMat{l}*p_l - p_l1'*gradJMat{l}*p_l1; 
end
gradJ(nSeg,1) = p_l1'*gradJMat{nSeg}*p_l1 + wT;
Tnew = T - [0; gradJ];
clear l baseInd p_l p_l1

% Plot results
if plotRes
    nPlotPoints = 100;
    figure
    hold on
    xSnap2Intg = 0;
    for l = 1:nSeg
        baseInd = (l-1)*nCoef;
        tVec = linspace(T(l,1), T(l+1,1), nPlotPoints);
        xPos = polyval(flipud(p_coefs(1+baseInd:nCoef+baseInd,1)), tVec);
        yPos = polyval(flipud(p_coefs(1+baseInd:nCoef+baseInd,2)), tVec);
        zPos = polyval(flipud(p_coefs(1+baseInd:nCoef+baseInd,3)), tVec);
        plot3(xPos, yPos, zPos);
        
        % For debugging
        xSnap2 = (polyval(polyder(polyder(polyder(polyder(flipud(p_coefs(1+baseInd:nCoef+baseInd,1)))))), tVec)).^2;
        xSnap2Intg = trapz(tVec,xSnap2)+xSnap2Intg;
        for k = 4:N
            kk = k + 1;
            
        end
    end
    hold off
    
    clear l baseInd
end
