%DSSCParaMotionPlannerCompileData compiles the data generated during the 
%   parallel computation phase
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mpinfo, planningPhase] = DSSCParaMotionPlannerCompileData(mpinfo, planningPhase)

%% Offline Phase

nThreads = 450;     % ensure this matches nThreads in DSSCParaMotionPlannerPythonDriver

% Ensure that proper path variables are established
addpath([pwd, '/../']);
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../ObstacleSets/']);

% Unpack variables to accessed and modified
n2PBVPs = mpinfo.sampling.n2PBVPs;  %(#) number of 2pt BVPs to solve for trainng ML
costMat = mpinfo.costMat;
evalMat = mpinfo.evalMat;
trajMat = mpinfo.trajMat;
controlMat = mpinfo.controlMat;

% Check for Valid Inputs
if planningPhase < 4
    disp('Error occured in input. Data not prepared for final steps')
    disp('planningPhase < 4')
    disp('Exiting DSSCParaMotionPlanningCompileData prematurely ...')
    return;
end

% Loop through all directories containing parallel data
for k = 1:nThreads
    % access data
    curFileString = ['run',num2str(k),'/subset2PBVPData.mat'];
    load(curFileString);
    
    % compile data
    for l = 1:size(pEvalList,1)
        evalMat(pEvalList(l,1),pEvalList(l,2)) = l+n2PBVPs;
    end
    n2PBVPs = n2PBVPs+pN2PBVPs;
    costMat = cat(1,costMat,pCostMat);
    trajMat = cat(1,trajMat,pTrajMat);
    controlMat = cat(1,controlMat,pControlMat);

    % clear temporary parallel data so there is no uninteded use
    clear pN2PBVPs pCostMat pEvalList pTrajMat pControlMat
end

% Store variables
mpinfo.sampling.n2PBVPs = n2PBVPs;
mpinfo.costMat = costMat;
mpinfo.trajMat = trajMat;
mpinfo.evalMat = evalMat;
mpinfo.controlMat = controlMat;

planningPhase = planningPhase + 1;

save(mpinfo.savefile);
end


