% SimpleUAVStateSampler samples a specified range of a state space using
% halton sequencing
%
%   Ross Allen, ASL, Stanford University
%   Apr 30, 2014
%
%   Notes:
%       - Samples state space and forms stateMat
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ mpinfo ] = SimpleUAVStateSampler( mpinfo )

sysdefs = mpinfo.systemDefs;
sampling = mpinfo.sampling;

halton_skip = randi(1e6,1); % randomly generate number of initial halton sequence to omit

mpinfo.stateMat = HaltonSampling(...
    sysdefs.nStateDims, sampling.nSamples, sampling.stateSampleRange,...
    halton_skip, 0, true);

% Check for biasing of heading samples and implement normal dist if so
if (isfield(sampling, 'bias') && isfield(sampling.bias, 'heading') && sampling.bias.heading.useBias)
    biasedHeading = normrnd(sampling.bias.heading.mu, sampling.bias.heading.sigma, sampling.nSamples, 1);
    biasedHeading = wrapToPi(biasedHeading);
    mpinfo.stateMat(:,4) = biasedHeading;
end
end

