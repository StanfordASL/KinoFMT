%OnlineErrorCheck validates user inputs at online initiation
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        July 8, 2015
%
%   Inputs:
%
%   Outputs:
%
%   Notes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function err = OnlineErrorCheck(mpinfo)

err.flag = 0;
err.string = 'valid input';

% Check initial and goal states are with sampling bounds
if any(~isnan(mpinfo.termStates.Xstart)) && ...
        (any(mpinfo.termStates.Xstart' < mpinfo.sampling.stateSampleRange(:,1)) || ...
        any(mpinfo.termStates.Xstart' > mpinfo.sampling.stateSampleRange(:,2)))
   
        err.flag = 1;
        err.string = 'Xstart not within sampling bounds';
        return;
end
if any(mpinfo.termStates.goalRegion(:,1) < mpinfo.sampling.stateSampleRange(:,1)) || ...
        any(mpinfo.termStates.goalRegion(:,1) > mpinfo.sampling.stateSampleRange(:,2)) || ...
        any(mpinfo.termStates.goalRegion(:,2) < mpinfo.sampling.stateSampleRange(:,1)) || ...
        any(mpinfo.termStates.goalRegion(:,2) > mpinfo.sampling.stateSampleRange(:,2))
   
        err.flag = 1;
        err.string = 'goalRegion not within sampling bounds';
        return;
end

% Check that nGoalSamples makes sense
if all(mpinfo.termStates.goalRegion(:,1) == mpinfo.termStates.goalRegion(:,2)) ...
        && mpinfo.sampling.nGoalSamples > 1
    err.flag = 1;
    err.string = 'goalRegion is singular point. Please rerun precomputation with nGoalSamples = 1';
end

% Check obstacles
if size(mpinfo.obstacles.obstacleSpace,1) > size(mpinfo.sampling.stateSampleRange,1)
    err.flag = 1;
    err.string = 'Obstacle space must have less than or equal number of dimensions as the state space';
end

% Check environment format
if isfield(mpinfo.environment, 'bounds') && ...
        ~isempty(mpinfo.environment.bounds)

    [nConfDim, nCols] = size(mpinfo.environment.bounds);
    if nCols ~= 2 || ~(nConfDim == 2 || nConfDim == 3) 
        err.flag = 1;
        err.string = 'Environment bounds mis formated';
    end

    if any(mpinfo.environment.bounds(:,1) > ...
            mpinfo.environment.bounds(:,2))
        err.flag = 1;
        err.string = 'evironment bounds min greater than max';
    end

end

% Check time scaling
if isfield(mpinfo, 'smoother') && isfield(mpinfo.smoother, 'timeScaling') && ...
    mpinfo.smoother.timeScaling < 1

    err.flag = 1;
    err.string = 'timeScaling must not accelerate time segments';

end

end
