%   ComplexStepGradients.m: Computes the gradient of a function at a given
%   point using the complex step method
%
%   Ross Allem, ASL, Stanford University
%   Based on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        12/1/2013
%
%   Inputs:         f               function for gradient evaluation
%                   x               point of gradient evaluation
%                   dimf            dimension of output of f
%                   params          additional params needed to comput f(x) optional
%
%   Outputs:        gradf           gradient of f at x
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function gradf = ComplexStepGradients(f,x,dimf,params)
    
dimx = length(x);
gradf = zeros(dimx,dimf);
h = 1e-20; 
x0 = x;

if nargin == 4
    for n = 1:dimx
        x = x0;
        x(n) = x(n) + h*1i;
        gradf(n,:) = imag(f(x,params))./h;
    end
    
elseif nargin == 3
    for n = 1:dimx  
        x = x0;
        x(n) = x(n) + h*1i;
        gradf(n,:) = imag(f(x))./h;
    end
end

return;