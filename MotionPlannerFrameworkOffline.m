%MotionPlannerFrameworkOffline performs the offline, precomputation phase
%   of a motion planning problem for a general system
%
%   Ross Allen, ASL, Stanford University
%
%   Started:    June 26, 2015
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

function [mpinfo] = MotionPlannerFrameworkOffline(mpinfo, planningPhase)

%% Offline Phase

% Ensure that proper path variables are established
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../ObstacleSets/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);

% Perform initial input error checks and retrieve system definitions
if planningPhase <= 0
	% TODO: Add input error check
    mpinfo.inputError = InitialErrorCheck(mpinfo);
    if mpinfo.inputError.flag > 0
        disp('User input invalid. Check inputError code');
        return;
    end
    mpinfo.progress.InitialErrorCheck = true;
    mpinfo.systemDefs = mpinfo.definitionsFunction();
    mpinfo.progress.RetrieveSystemDefs = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
end

% Sample State Space
if planningPhase  <= 1
    disp('----------------------------------------')
    disp('Sampling State Space')
    mpinfo = StateSampler(mpinfo);
    mpinfo.progress.SampleStateSpace = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

% Solve Initial Set of 2PBVPs
if planningPhase <= 2
    disp('----------------------------------------')
    disp('Solving Initial Set of Optimal 2PBVPs')
    mpinfo = Solve2PBVPsInitialSet(mpinfo);
    mpinfo.progress.SolveTraining2PBVPs = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    save([mpinfo.savefile, '_phase', num2str(planningPhase-1)]);
    disp('----------------------------------------')
end

% Train SVM Neighborhood Classifier
if planningPhase <= 3
    disp('----------------------------------------')
    disp('Training SVM Reachability Classifier')
    mpinfo = TrainNeighborClassifier(mpinfo);
    mpinfo.progress.TrainSVM = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

% Solve Remaining 2PBVPs Based on Classification
if planningPhase <= 4
    disp('----------------------------------------')
    disp('Solving Remaining 2PBVPs Based on Predicted Reachability')
    mpinfo = Solve2PBVPsRemainingSet(mpinfo);
    mpinfo.progress.SolveRemaining2PBVPs = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    save([mpinfo.savefile, '_phase', num2str(planningPhase-1)]);
    disp('----------------------------------------')
end

% Train Cost Estimator
if planningPhase <= 5
    disp('----------------------------------------')
    disp('Training Cost Estimator')
    mpinfo = TrainCostEstimator(mpinfo);
    mpinfo.progress.TrainCostEstimator = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

% Reform Matrices to Accomondate Initial and Goal States
if planningPhase <= 6
    disp('----------------------------------------')
    disp('Reforming Data Matrices to Incorporate Start and Goal')
    mpinfo = ReformDataMatrices(mpinfo);
    mpinfo.progress.ReformMatrices = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

% Sort Neighborhoods for current samples
if planningPhase <= 7
    disp('----------------------------------------')
    disp('Sorting Neighborhoods')
    mpinfo = SortNeighborhoods(mpinfo,...
        [mpinfo.sampling.nGoalSamples+2:mpinfo.nTotSamples]);
    mpinfo.progress.SortNeighborhoods = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

save([mpinfo.savefile, '_precomputed']);

disp('----------------------------------------')
disp('Precomputation Complete!')
disp('----------------------------------------')

end
