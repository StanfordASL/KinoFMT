%   ChebyshevMatrices.m: form cosine-spaced vector of independent variable, 
%                           Chebyshev differentiation and integration matrices
%
%   Ross Allen, ASL, Stanford University
%   Michael Colonno, ADL, Stanford University 
%
%   Started:        6/27/11
%   Last Updated:   Feb 13, 2014
%
%   Inputs:         N               order of polynomial approximation 
%                                   (N+1 control nodes) 
%
%   Outputs:        x               cosine-spaced vector {0 --> 1} (non-dim) 
%                   D               differentiation matrices (non-dim)
%                   w               Clenshaw-Curtis weights
%                   I               integration matrix (non-dim)
%                   err             nonzero if an error occurs
%
%
%   References: 
%              Q. Gong, I. M. Ross, F. Fahroo, "A Chebyshev Pseudospectral
%               Method for Nonlinear Constrained Optimal Control Problems",
%               Joint 48th IEEE Conference on Decision and Control and 28th
%               Chinese Control Conference, Shanghai, P.R. China, December
%               16-18, 2009
%
%              L. Trfethen, "Spectral Methods in MATLAB", SIAM 2000,
%               Page 128
%
%   Note:
%               Francisco Capistran found problems/inaccuracies in using
%               the integration matrix. Use with caution.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x, D, w, I, err] = ChebyshevPseudoNumerics(N)

n = N+1;

err = 0; 

% nodes and weights
[x, w] = clencurt(N);

% differentiation matrix
D = zeros(n);
for i = 1:n
    for j = 1:n
        
        ii = i - 1; jj = j - 1;
        ci = 1; cj = 1;  
        if (ii == 0 || ii == (n-1))      
            ci = 2;
        elseif (jj == 0 || jj == (n-1))      
            cj = 2;
        end
    
        if (ii == jj)     
            D(i,j) = -0.5*x(j)/(1 - x(j)^2);       
        else
            D(i,j) = (ci/cj)*((-1)^(ii + jj))/(x(i) - x(j));
        end
        
    end
end

% fix corners
D(1,1) = (2*(n-1)^2 + 1)/6; D(n,n) = -D(1,1);
D(1,n) = D(1,n)/2; D(n,1) = D(n,1)/2;


% remap x to {0 ---> 1} domain
x = 0.5.*(1 - cos((0:(n-1)).*pi/(n-1))');

% remap D to {0 ---> 1} domain
D = -2*D; 

% integration matrix
I = D(2:end,2:end)\eye(n-1,n-1); 
I = [zeros(1,n); zeros(n-1,1), I]; 

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%