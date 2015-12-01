% PolySolverTestScript5.m is a script to develop and test a solver for polynomial trajectories
%
%	AUTHOR: Ross Allen
%	DATE:	Apr 30, 2015
%
%	NOTES:
%		See Ricther et. al. "Polynomial Trajectory Planning for Aggressive ..." to see motivation
%       - Version 3 redefines the time vector to be a set of delta
%       - Version 4 offloads some work to separate functions
%       - Version 5 re-interprets cost function gradients with respect to
%       time and uses numerical techniques of computation
%       - Version 6 is working to integrate with motion planning. Removes
%       time optimization since this is handled in the motion planning
%       phase
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% assumes mpinfo with solution information exists
yawcen = -1.68;

% Given Params
N = 9;		% Order of polynomial
wD = zeros(N+1,1); wD(5) = 1; % derivative minimization weight
keyStateNums = mpinfo.optPath;
epsf = 1e-9;   % epsilon for floating point comparisons
plotRes = true;

% Derived parameters and one-time calculations
keys = mpinfo.stateMat(keyStateNums, 1:3);  % keypoints or waypoints for trajectory
nCoef = N+1;	% number of coefficients
nKeys = size(keys, 1);  % number of keypoints to be connected
keys = [keys, yawcen*ones(nKeys,1)];    % add yaw keypoints
nOut = size(keys, 2);   % number of output states (should always be 4)
nSeg = nKeys - 1;		% number of polynomial segments
nTermVar = nCoef/2; % number of variables at terminal points of seg
nVar = nCoef*nSeg;  % number of variables
nExFixed = 2*nTermVar + 2*nSeg-2; % number of explicitly fixed variables
nImFixed = (nSeg-1)*(nTermVar-1);   % num of implicitly fixed vars (matching)
nFixed = nExFixed + nImFixed;       % total num of fixed vars
nFree = nVar - nFixed;  % total number of free variables 

% Extract timing vector
Tdel = zeros(nSeg, 1);  % time steps between segments
for i = 1:nSeg
    Tdel(i,1) = mpinfo.costMat(mpinfo.evalMat(...
        mpinfo.optimalPath(i,1),mpinfo.optimalPath(i,2)),1);
end
clear i

% Error check inputs
if length(wD) ~= N+1
    disp('error: weight vector mis-dimension')
    return;
end
if any(wD < 0) || all(abs(wD) < epsf);
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
[Q, A, ~, matchMat_A, matchMat_d, orderMat] = ...
    AgileQuadFormPolyMats( polytrajinfo, Tdel );

% Solve for polynomial coefficients and calculate cost with time
p_coefs = AgileQuadSolvePolynomial( polytrajinfo, Q, A, d, d_fixed,...
    orderMat, matchMat_A, matchMat_d);

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
        set(gca, 'ZDir', 'reverse')
        
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
