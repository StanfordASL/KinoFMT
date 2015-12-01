%DSSCMotionPlannerInitial solves the initial steps for a motion planning
%   problem that utilizes paralization
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        Sep 25, 2014
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
%       - System dynamics are based on a point-mass spacecraft in a 6D
%       state space (x,y,z,vx,vy,vz) with control space being the thrust
%       vector which can point arbitrarily, yet magnitude limited. 
%       Mass change assumed negligible
%       - Part 1 solves everything upto and including the training
%       of the SVM classifier. The solution to the remaining 2PBVPs
%       is handled in parallel threads
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mpinfo, planningPhase] = DSSCParaMotionPlannerInitial(mpinfo, planningPhase)

%% Offline Phase

% Ensure that proper path variables are established
addpath([pwd, '/../']);
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../ObstacleSets/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);

% Retrieve System Definitions
if planningPhase <= 0
    mpinfo.systemDefs = DSSCDefinitions;
    mpinfo.progress.RetrieveSystemDefs = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
end

% Sample State Space
if planningPhase  <= 1
    disp('----------------------------------------')
    disp('Sampling State Space')
    mpinfo = DSSCStateSampler(mpinfo);
    mpinfo.progress.SampleStateSpace = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    disp('----------------------------------------')
end

% Solve Initial Set of 2PBVPs
if planningPhase <= 2
    disp('----------------------------------------')
    disp('Solving Initial Set of Optimal 2PBVPs')
    mpinfo = DSSCSolve2PBVPsInitialSet(mpinfo);
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
    mpinfo = DSSCTrainNeighborClassifier(mpinfo);
    mpinfo.progress.TrainSVM = true;
    planningPhase = planningPhase + 1;
    save(mpinfo.savefile);
    save([mpinfo.savefile, '_phase', num2str(planningPhase-1)]);
    disp('----------------------------------------')
end


disp('----------------------------------------')
disp('DSSC Motion Planning Part 1 Complete')
disp('Ready for parallel computations')
disp('----------------------------------------')
end


