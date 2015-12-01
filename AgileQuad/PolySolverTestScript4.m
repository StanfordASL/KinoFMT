% PolySolverTestScript4.m is a script to develop and test a solver for polynomial trajectories
%
%	AUTHOR: Ross Allen
%	DATE:	Feb 19, 2015
%
%	NOTES:
%		See Ricther et. al. "Polynomial Trajectory Planning for Aggressive ..." to see motivation
%       - Version 3 redefines the time vector to be a set of delta
%       - Version 4 offloads some work to separate functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc
close all

% Given Params
N = 9;		% Order of polynomial
wD = zeros(N+1,1); wD(5) = 1; % derivative minimization weight
wT = 1000;    % Time minimization weight
gamT = 1;       % gradient descent step size
alphaT = 0.25;   % backtracking parameter
betaT = 0.5;    % backtracking parameter
keys = [ ...    % keypoints or waypoints for trajectory 
    -3, 0, 0, 0;...
    -2, 1, 1, 0;...
    -1, 0, 2, 0;...
%     0, -1, 3, 0;...
%     1, 0, 4, 0;...
%     2, 1, 5, 0;...
%     3, 0, 6, 0;...
    ];          % ([m], [m], [m], [rad])
epsf = 1e-9;   % epsilon for floating point comparisons
etaDescent = 1e-3;  % stopping criteria for time minimization
lowVel = 10;   % [m/s] low velocity for timing estimate
T0 = 0;         % [s] initial time
plotRes = false;

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
Tdel{1} = linDist./lowVel;
% Tcum = T0 + cumsum(Tdel{1});
Tdel{1}(1) = [];
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
polytrajinfo.N = N;
polytrajinfo.wD = wD;
polytrajinfo.nCoef = nCoef;
polytrajinfo.nSeg = nSeg;
polytrajinfo.nTermVar = nTermVar;
polytrajinfo.nVar = nVar;
polytrajinfo.nFixed = nFixed;
polytrajinfo.d_fixed = d_fixed;
[Q, A, gradJCell, matchMat_A, matchMat_d, orderMat] = ...
    AgileQuadFormPolyMats( polytrajinfo, Tdel{1} );

% Solve for polynomial coefficients and calculate cost with time
iter = 1;
p_coefs{iter} = AgileQuadSolvePolynomial( polytrajinfo, Q, A, d, d_fixed,...
    orderMat, matchMat_A, matchMat_d);
J{iter} = p_coefs{iter}(:,1)'*Q*p_coefs{iter}(:,1) + wT*ones(1,nSeg)*Tdel{1};

% Calculate gradient of cost function with respect to time
% NOTE: this uses only x-polynomial to perform time allocation
gradJ = zeros(nSeg,1);
for l = 1:nSeg
    baseInd = (l-1)*nCoef;
    p_l = p_coefs{1}(baseInd+1:baseInd+nCoef,1);
    gradJ(l,1) =  p_l'*gradJCell{l}*p_l +wT; 
end
gradJnorm2 = gradJ'*gradJ;
clear l baseInd p_l

p_cur = p_coefs{1};
while gradJnorm2 > etaDescent
    
    % Perform backtracking line search to determine next time vector
    Tnext = Tdel{iter} -  gamT*gradJ;
    Tnext = max( Tnext, zeros(nSeg,1));
    if ~all(Tnext)
        % There is a zero element in Tnext, invalid case
        Jnext = Inf;
    else
%         [Q, A, gradJCell, ~, ~, ~] = AgileQuadFormPolyMats(polytrajinfo, Tnext);
%         p_next = AgileQuadSolvePolynomial( polytrajinfo, Q, A, d, d_fixed, orderMat, ...
%                 matchMat_A, matchMat_d);
%         Jnext = p_next(:,1)'*Q*p_next(:,1) + wT*ones(1,nSeg)*Tnext;
        [Q, ~, gradJCell, ~, ~, ~] = AgileQuadFormPolyMats(polytrajinfo, Tnext);
        Jnext = p_cur(:,1)'*Q*p_cur(:,1) + wT*ones(1,nSeg)*Tnext;
%         Jnext = p_coefs{iter}(:,1)'*Q*p_coefs{iter}(:,1) + wT*ones(1,nSeg)*Tnext;
    end
    while Jnext > J{iter} - alphaT*gamT*gradJ'*gradJ;
        gamT = betaT*gamT;
        Tnext = Tdel{iter} -  gamT*gradJ;
        Tnext = max( Tnext, zeros(nSeg,1));
        if ~all(Tnext)
            % There is a zero element in Tnext, invalid case
            Jnext = Inf;
        else
%             [Q, A, gradJCell, ~, ~, ~] = AgileQuadFormPolyMats(polytrajinfo, Tnext);
%             p_next = AgileQuadSolvePolynomial( polytrajinfo, Q, A, d, d_fixed, orderMat, ...
%                     matchMat_A, matchMat_d);
%             Jnext = p_next(:,1)'*Q*p_next(:,1) + wT*ones(1,nSeg)*Tnext;
            [Q, ~, gradJCell, ~, ~, ~] = AgileQuadFormPolyMats(polytrajinfo, Tnext);
            Jnext = p_cur(:,1)'*Q*p_cur(:,1) + wT*ones(1,nSeg)*Tnext;
%             Jnext = p_coefs{iter}(:,1)'*Q*p_coefs{iter}(:,1) + wT*ones(1,nSeg)*Tnext;
        end
    end
    
    % update iter count
    iter = iter + 1;
    
    % update coeficients, cost, and time vector
%     p_coefs{iter} = p_next;
%     p_coefs{iter} = AgileQuadSolvePolynomial( polytrajinfo, Q, A, d, d_fixed, orderMat, ...
%         matchMat_A, matchMat_d);
    J{iter} = Jnext;
    Tdel{iter} = Tnext;
    gamT = 1;   % reset line search
    
    % update gradient vector wrt time
    gradJ = zeros(nSeg,1);
    for l = 1:nSeg
        baseInd = (l-1)*nCoef;
%         p_l = p_coefs{iter}(baseInd+1:baseInd+nCoef,1);
        p_l = p_cur(baseInd+1:baseInd+nCoef,1);
        gradJ(l,1) =  p_l'*gradJCell{l}*p_l + wT; 
    end
    gradJnorm2 = gradJ'*gradJ;
    clear l baseInd p_l
end

% Plot results
if plotRes
    nPlotPoints = 100;
    figure
    hold on
    xSnap2Intg = 0;
    for l = 1:nSeg
        baseInd = (l-1)*nCoef;
        tVec = linspace(0, Tdel(l,1), nPlotPoints);
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
