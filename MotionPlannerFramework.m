%MotionPlannerFramework solves a motion planning problem for a general system
%
%   Ross Allen, ASL, Stanford University
%
%   Started:        Nov 11, 2015
%
%   Inputs:
%
%   Outputs:
%
%   Notes:
%       - To avoid having to make changes in multiple locations, the complete
%       motion planner just calls the subcomponents for offline and online
%       - Matlab 2012b or later should be used as certain Matlab functions
%       experience adverse 'legacy' effects that will cause erros if older
%       versions are used (e.g. svmtrain, intersect, ismember, etc)
%       - 'mpinfo' stands for motion planning information and holds all
%       relevant data for a motion planning problem (i.e. sampling info, robot
%       info, machine learning info, etc.)
%       - This has been generalized to a system-agnostic function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mpinfo] = MotionPlannerFramework(mpinfo, planningPhase)

% offline phase
mpinfo = MotionPlannerFrameworkOffline(mpinfo, planningPhase);

% online phase
mpinfo = MotionPlannerFrameworkOnline(mpinfo);


end

