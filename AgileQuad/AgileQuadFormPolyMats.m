% AgileQuadFormPolyMats is a function to formulate the optimization and constraint matrices
%
%	AUTHOR: Ross Allen
%	DATE:	Feb 19, 2015
%
%	NOTES:
%		See Ricther et. al. "Polynomial Trajectory Planning for Aggressive ..." Sec 3.1-3.3
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Q, A, gradJCell, matchMat_A, matchMat_d, orderMat] =...
            AgileQuadFormPolyMats( polytrajinfo, Tdel )

% Given Params
N = polytrajinfo.N;             % order of polynomial
wD = polytrajinfo.wD;           % derivative minimization weight
nCoef = polytrajinfo.nCoef;     % number of coefficients
nSeg = polytrajinfo.nSeg;		% number of polynomial segments
nTermVar = polytrajinfo.nTermVar; % number of variables at terminal points of seg
nVar = polytrajinfo.nVar;       % number of variables
nFixed = polytrajinfo.nFixed;       % total num of fixed vars
d_fixed = polytrajinfo.d_fixed; % vec of implicit or explicit constraints

fullcomp = false;       % boolean to determine if full computation

% Error Check
if ~(nargout == 1 || nargout == 6)
    disp('function only defined for 1 or 6 outputs')
    return;
elseif nargout == 6
    fullcomp = true;
end

% Form Q and A  matrices for derivative minimization (sec 3.1 - 3.3)
% Calculate gradient terms for time allocation (sec 3.4 and 3.6)
Q = zeros(nVar, nVar, nCoef);

if fullcomp
    A = zeros(nVar, nVar);
    gradJCell = cell(nSeg,1);   % gradient of J wrt T
    matchMat_A = eye(nVar);     % matrix to enforce matching constraint at seg trans
    matchMat_d = eye(nVar);     % matrix to enforce matching on d
    orderMat = zeros(nVar, nVar);   % matrix to reorder A matrix and d vector
end

fixedInd = 1;
freeInd = nFixed + 1;
for l = 1:nSeg;                 % segment number being worked
    baseInd = (l-1)*nCoef;      % base row/column index of Q being worked
    if fullcomp
        gradJCell{l} = zeros(nCoef,nCoef,nCoef);
    end

    for k = 0:N                 % order of derivative
        kk = k + 1;             % index of deriv weight
        krow = kk+baseInd;      % row index from 1:nVar

        if fullcomp
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
        end

        for i = k:N
            ii = baseInd+i+1;   % row index to be inserted to
            iprod = FactorialRatioMEX(i,k);
            %iprod = factorial(i)/factorial(i-k);

            % Calculate constraint matrix
            if fullcomp && kk <= nTermVar
                if i == k
                    A(baseInd+kk,ii) = iprod;
                end
                A(baseInd+kk+nTermVar,ii) = iprod*Tdel(l,1)^(i-k);
            end

            for j = k:N
                jj = baseInd+j+1;   % col index to be inserted to
                jprod = FactorialRatioMEX(j,k);
                %jprod = factorial(j)/factorial(j-k);

                tempProd = wD(kk)*iprod*jprod;

                % Calculate minimization matrix
                Q(ii,jj,kk) = tempProd*...
                    (Tdel(l,1)^(i+j+1-2*k))/(i+j+1-2*k);
                
                if fullcomp
                    % Store gradient information
                    gradJCell{l}(i+1,j+1,kk) = tempProd*Tdel(l,1)^(i+j-2*k);
                end 
            end
        end
    end
    
    if fullcomp
        % Sum over weights for gradient info
        gradJCell{l} = sum(gradJCell{l},3);
    end

end

Q = sum(Q,3);   % sum over derivative weights for final Q matrix

% Error check orderMat
if ~(fixedInd == nFixed+1 && freeInd == nVar+1)
    disp('check orderMat creation')
    return;
end
