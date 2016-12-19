%MotionPlannerFrameworkOnlineMPC repeatedly performs the online computation phase
%   of a motion planning problem for a general system
%
%   Ross Allen, ASL, Stanford University
%
%   Started:    Feb 2016
%
%   Inputs:
%
%   Outputs:
%
%   Notes:
%       - Matlab 2012b or later should be used as certain Matlab functions
%       experience adverse 'legacy' effects that will cause erros if older
%       versions are used (e.g. svmtrain, intersect, ismember, etc)
%       - 'mpinfo' stands for motion planning information and holds all
%       relevant data for a motion planning problem (i.e. sampling info, robot
%       info, machine learning info, etc.)
%       - This has been generalized to a system-agnostic function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [mpinfo_complete, mpc_info] = MotionPlannerFrameworkOnlineMPC(mpinfo_base)

% Ensure that proper path variables are established
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../ObstacleSets/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);
addpath([pwd, '/../MEXFuncs/']);

% initiate planner
if (isfield(mpinfo_base,'profiler') && mpinfo_base.profiler == true)
    profile on
end

% intial run of planner
replan = true;
iterationTimer = tic;
[mpinfo_complete, exitCond] = MotionPlannerFrameworkOnline(mpinfo_base);

% store tcp client to avoid trying to open duplicates
mpinfo_base.comms.tcpClient = mpinfo_complete.comms.tcpClient;
   
% Rerun until success
while (exitCond ~= 1)
   pause(mpinfo_base.onlineOptions.obstacleCheckTime);
   [mpinfo_complete, exitCond] = MotionPlannerFrameworkOnline(mpinfo_base);
end

% clock when trajectory has been sent
trajectoryTimer = tic;

% store most recent motion plan
mpinfo_recent = mpinfo_complete;

% Run iterations if necessary
mpc_iter = 0;

while (replan)
    
    % increment
    mpc_iter = mpc_iter+1;
    
    % save at every iteration
    mpc_info.exitCond(mpc_iter) = exitCond;
    mpc_info.Xprior{mpc_iter} = mpinfo_recent.termStates.Xprior;
    mpc_info.spheres{mpc_iter} = mpinfo_recent.obstacles.spheres;
    
    % save results if successful
    if (exitCond == 1)

        % store salient results from previous plan
        mpc_info.Xstart{mpc_iter} = mpinfo_complete.termStates.Xstart;
        mpc_info.Xgoal{mpc_iter} = mpinfo_complete.termStates.Xgoal;
        mpc_info.Tdel{mpc_iter} = mpinfo_complete.smoother.Tdel;
        mpc_info.splineCoefs{mpc_iter} = mpinfo_complete.smoother.splineCoefs;
        mpc_info.optPath{mpc_iter} = mpinfo_complete.optPath;
        mpc_info.optCost{mpc_iter} = mpinfo_complete.optCost;
        mpc_info.treeParents{mpc_iter} = mpinfo_complete.treeParents;
        if isfield(mpinfo_complete.termStates, 'XstartNom')
           mpc_info.XstartNom{mpc_iter} = mpinfo_complete.termStates.XstartNom; 
        end
        
        
    end
    
    % Check if iteration limit hit
    if (mpc_iter >= mpinfo_base.onlineOptions.maxMPCIter)
        disp('Max iterations for MPC reached. Exiting model predictive control...');
        replan = false;
        mpc_info.iterationTime{mpc_iter} = toc(iterationTimer);
        break;
    end
    
    % Wait for MPC horizon time and start timer
    mpc_info.iterationTime{mpc_iter} = toc(iterationTimer);
    if (exitCond == 0)
        % Only wait a short time
        pause(mpinfo_base.onlineOptions.obstacleCheckTime);
    else
        % Give it enough time for the quad to respond and settle
        pause(mpinfo_base.onlineOptions.PX4_SPLINE_START_DELAY_T_SEC_REL)
    end
    iterationTimer = tic;
    
    % Calculate estimated target state
    %%%%% NOTE: THIS IS NOT GENERALIZED, ONLY WORKS FOR 6DOF DOUBLE
    %%%%% INTEGRATOR
    mpinfo_base.termStates.XtargetEst = zeros(1,6);
    tEstSpline = toc(trajectoryTimer) - ...
        mpinfo_base.onlineOptions.PX4_SPLINE_START_DELAY_T_SEC_REL;
    segIter = 1;
    tCumSegEnd = 0;
    tCumSegStart = 0;
    nCoef = mpinfo_complete.smoother.nCoef;
    while (segIter <= length(mpinfo_complete.smoother.Tdel))
        tCumSegStart = tCumSegEnd;
        tCumSegEnd = tCumSegEnd + mpinfo_complete.smoother.Tdel(segIter);
        if tEstSpline < tCumSegEnd;
            break;
        end
        segIter = segIter+1;
    end
    if segIter > length(mpinfo_complete.smoother.Tdel)
        % use the final point in the spline, with zero vel, do not extend
        tPoly = mpinfo_complete.smoother.Tdel(end);
        baseInd = (segIter-2)*nCoef;

    else
        % calculate time in polynomial
        tPoly = tEstSpline - tCumSegStart;
        if tPoly < 0
            tPoly = 0;
        end
        baseInd = (segIter-1)*nCoef;
    end
    mpinfo_base.termStates.XtargetEst(1) = polyval(flipud(...
        mpinfo_complete.smoother.splineCoefs(1+baseInd:nCoef+baseInd,1)),tPoly);
    mpinfo_base.termStates.XtargetEst(2) = polyval(flipud(...
        mpinfo_complete.smoother.splineCoefs(1+baseInd:nCoef+baseInd,2)),tPoly);
    mpinfo_base.termStates.XtargetEst(3) = polyval(flipud(...
        mpinfo_complete.smoother.splineCoefs(1+baseInd:nCoef+baseInd,3)),tPoly);
    mpinfo_base.termStates.XtargetEst(4) = polyval(polyder(flipud(...
        mpinfo_complete.smoother.splineCoefs(1+baseInd:nCoef+baseInd,1))),tPoly);
    mpinfo_base.termStates.XtargetEst(5) = polyval(polyder(flipud(...
        mpinfo_complete.smoother.splineCoefs(1+baseInd:nCoef+baseInd,2))),tPoly);
    mpinfo_base.termStates.XtargetEst(6) = polyval(polyder(flipud(...
        mpinfo_complete.smoother.splineCoefs(1+baseInd:nCoef+baseInd,3))),tPoly);
    
    % Record values for next iteration
    mpc_info.XtargetEst{mpc_iter+1} = mpinfo_base.termStates.XtargetEst;
    mpc_info.tEstSpline{mpc_iter+1} = tEstSpline;
    
    % Run new iteration of plan and catch vicon terminated exit condition
    % to avoid overwriting most recent version of mpinfo_complete
    [mpinfo_recent, exitCond] = MotionPlannerFrameworkOnline(mpinfo_base);
    if (exitCond == 1)
        trajectoryTimer = tic;
        mpinfo_complete = mpinfo_recent;
    elseif (exitCond == 2)
        % goal satisfaction
        replan = false;
    end
%     try
%        mpinfo_complete = MotionPlannerFrameworkOnline(mpinfo_base);
%        
%     catch exitCondition
%         switch exitCondition.identifier
%             case 'Planner:goalSatisfaction'
%                 warning('Vicon indicates goal region satisfaction');
%                 replan = false;
%             otherwise
%                 rethrow(exitCondition);
%         end
%             
%     end
    
end

% Profiler
if (isfield(mpinfo_base,'profiler') && mpinfo_base.profiler == true)
    profile viewer
    profsave(profile('info'), [mpinfo_base.savefile, '_profiler'])
end

% Close communications
if (isfield(mpinfo_base.comms, 'tcpClient') && ...
        strcmp(mpinfo_base.comms.tcpClient.Status, 'open'))
    fclose(mpinfo_base.comms.tcpClient);
    delete(mpinfo_base.comms.tcpClient);
end

% Save data
save([mpinfo_base.savefile, '_MPC']);

end