%   DblIntQuadInitialGuess.m: Generate a starting guess for optimization 
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        Apr 28, 2015
%
%   Inputs:         probinfo         global probinfo data structure   
%
%   Outputs:
%       t_init  =   lower bound for bisection search
%       t_end   =   upper bound for bisection search     
%
%   Notes:
%       - This function is significantly different in output than
%           similarly named functions in other problems. This
%           function only generates an interval for bisection 
%           search that is used to find the zero of a nonlinear
%           analytical function. Similarly named functions in
%           other problems generates estimates of state and 
%           control values at discrete times 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [tf_min, tf_max] = DblIntQuadInitialGuess(probinfo)

% Simple implementation
tf_min = probinfo.options.tf_min;

% check if cost threshold is set and use that if so
tf_max = probinfo.options.costThreshold;
if isnan(tf_max)
    tf_max = probinfo.options.tf_max;
end
if isnan(tf_max)
    disp('tf_max not defined');
end

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
