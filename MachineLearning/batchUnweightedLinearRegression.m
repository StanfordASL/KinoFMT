function [theta] = batchUnweightedLinearRegression(xdat, ydat)

% xfit = [ ---xfit1--- ]
%        [ ---xfit2--- ] etc

% Unweighted Least Squares
X = [ones(length(xdat(:,1)),1) xdat];
theta = (X'*X)\(X'*ydat);
% yfit = [ones(length(xfit(:,1)),1), xfit]*theta;

end