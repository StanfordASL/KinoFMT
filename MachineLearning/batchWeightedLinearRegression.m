function [theta, yfit] = batchWeightedLinearRegression(xdat, ydat, tau, xfit, xExtent)

% Weighted Least Squares
m = length(xdat(:,1));
X = [ones(m,1) xdat];
W = zeros(m,m);
yfit = zeros(length(xfit(:,1)),1);
theta = zeros(1+size(xfit,2),size(xfit,1) );

for i=1:length(xfit(:,1))
    x = [1;xfit(i,:)'];
    for j=1:m
        errorVector = (x-[1;xdat(j,:)'])./[1; xExtent'];
        W(j,j) = exp(-errorVector'*errorVector/(2*tau^2));
    end
    XWX = X'*W*X;
%     if rcond(XWX) > 1e-15
        theta(:,i) = XWX\(X'*W*ydat);
        yfit(i) = theta(:,i)'*x;
%     else
%         theta = inf*ones(size(x));
%         yfit(i) = NaN;
%     end
end

end