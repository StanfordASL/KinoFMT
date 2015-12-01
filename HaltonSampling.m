function sampled_nodes = HaltonSampling( num_dim, num_samples, ...
    sample_space, skip_size, leap_size, scram)
%HaltonSampling generates the Halton set for given state space for a given
%number of samples
%
%   Ross Allen, ASL, Stanford Univeristy
%   Started: Nove 8, 2013
%
%   Inputs:
%       num_dim:                number of dimensions
%       num_samples:            number of samples             
%       sample_space:           num_dim x 2 matrix holding sample space bounds
%       skip_size:              number of initial points to omit
%       leap_size:              number of elements to leap in halton
%       scram                   boolean indicating if scrambled or not
%
%   Outputs:
%       sampled_nodes:          sampled nodes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if num_dim ~= size(sample_space,1)
    disp('dimension of sample_space must equal num_dim')
    sampled_nodes = NaN;
    return
end

% Generate Halton set
halset = haltonset(num_dim, 'Skip', skip_size, 'Leap', leap_size);

% Scramble
if scram
    halset = scramble(halset, 'RR2');
end

% Extract samples
halset = net(halset, num_samples);

% Map to sample space
sampled_nodes = halset*diag([-1 1]*sample_space') + ...
    ones(num_samples, num_dim)*diag(sample_space(:,1));
end

