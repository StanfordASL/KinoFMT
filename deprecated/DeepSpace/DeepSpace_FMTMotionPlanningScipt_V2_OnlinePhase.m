% DeepSpace_FMTMotionPlanningScipt_V2_OnlinePhase.m attempts to solve the kinodynamic motion
% plannning problem. This only executes the online phase and minimizes
%
%   Ross Allen, ASL, Stanford University
%   Jan 28, 2013
%
%   NOTES:
%   - "DeepSpace" refers to a spacecraft in deepspace (non-orbiting)
%   - After incorporation of initial and goal states, it is assumed they
%   are identified and indexed by 1 and 2, respectively. Note that c++ code
%   will have these states indexed and identified by 0 and 1,
%   respectively. This will need to adapted when goal regions are included
%   - nSamples = number of sampled states (not including init and goal)
%   - ntateDims = dimension of state space (length of state vector)
%   - nTrajNodes = number of nodes used to approximate a trajectory
%
%   NOTES on V2:
%   - uses new training data format (see PrecompScript_V2)
%   - 'IDs' refer to the row index of a state in stateMat
%   - 'CaseNums' refers to the case number of a successful 2PBVP problem 
%   (i.e. the row index of a specific initStateIDs and finalStateIDs pair)
%   - n2PBVPs specifically refers to the number of successful 2PBVPs
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

s=dbstatus;
save('myBreakpoints.mat', 's');
clear all
load('myBreakpoints.mat');
dbstop(s);
clc
close all

% Add path for machine learning (ML) functions
addpath([pwd, '/../']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);

load('DeepSpaceAllOfflineSave_2000x10000x10_Mar11-2014.mat')
tic
%% Approximate neighborhood of initial and goal states with ML
disp('Approximating Neighboorhoods of Initial and Goal States')
stateMat(1,:) = Xstart;
stateMat(2,:) = Xgoal;
init_neighbors_feature_matrix  = deep_space_extract_2PBVP_features( ...
    repmat(Xstart,nSamples+2,1), stateMat(1:end,:) );
reachable_from_Xstart    = is_reachable( svm_output,...
    init_neighbors_feature_matrix);
goal_neighbors_feature_matrix = deep_space_extract_2PBVP_features( ...
    stateMat(1:end,:), repmat(Xgoal,nSamples+2,1) );
Xgoal_reachable_from = is_reachable(svm_output,...
    goal_neighbors_feature_matrix);

%% Optimally connect initial and state to its approximate out-neighboorhood
disp('Optimally Connecting Xstart to out-Neighborhood') % NOTE remove later
clear boundary_values
boundary_values.x0 = Xstart(1);
boundary_values.y0 = Xstart(2);
boundary_values.z0 = Xstart(3);
boundary_values.xdot0 = Xstart(4);
boundary_values.ydot0 = Xstart(5);
boundary_values.zdot0 = Xstart(6);
boundary_values.t0 = 0;
outNeighbors = [];
for j = 2:nSamples+2 
    if reachable_from_Xstart(j) == 1
        % Set boundary values
        boundary_values.xf = stateMat(j,1);
        boundary_values.yf = stateMat(j,2);
        boundary_values.zf = stateMat(j,3);
        boundary_values.xdotf = stateMat(j,4);
        boundary_values.ydotf = stateMat(j,5);
        boundary_values.zdotf = stateMat(j,6);
        
        % Clear previous problem
        clear bvpinfo
        bvpinfo.numerics = numerics;
        bvpinfo.robot = robot;
        bvpinfo.boundary_values = boundary_values;
        bvpinfo.environment = environment;
        bvpinfo.options = options;
        
        % Call Solver
        bvpinfo = DeepSpaceOptimizer(bvpinfo);
        
        % Save Cost, Trajectory and Control and neighborhood
        if bvpinfo.solution.exitflag > 0
            n2PBVPs = n2PBVPs+1;
            curCost = bvpinfo.solution.cost;
            evalMat(1,j) = n2PBVPs;
            costMat(n2PBVPs,1) = curCost;
            trajMat(n2PBVPs,1,:) = bvpinfo.solution.x;
            trajMat(n2PBVPs,2,:) = bvpinfo.solution.y;
            trajMat(n2PBVPs,3,:) = bvpinfo.solution.z;
            trajMat(n2PBVPs,4,:) = bvpinfo.solution.xdot;
            trajMat(n2PBVPs,5,:) = bvpinfo.solution.ydot;
            trajMat(n2PBVPs,6,:) = bvpinfo.solution.zdot;
            controlMat(n2PBVPs,1,:) = bvpinfo.solution.ux;
            controlMat(n2PBVPs,2,:) = bvpinfo.solution.uy;
            controlMat(n2PBVPs,3,:) = bvpinfo.solution.uz;
            controlMat(n2PBVPs,4,:) = bvpinfo.solution.eta;
            outNeighbors = [outNeighbors; [j curCost]];
            if isempty(inNeighborCell{1,j})
                inNeighborCell{1,j}(1,:) = [1 curCost];
            else
                inPos = find(inNeighborCell{1,j}(:,2) > curCost,1,'first');
                inNeighborCell{1,j} = [inNeighborCell{1,j}(1:inPos,:);...
                    inNeighborCell{1,j}(inPos:end,:)];
                inNeighborCell{1,j}(1,:) = [1 curCost];
            end
        end
    end
end
[~,sortIX] = sort(outNeighbors(:,2));
outNeighbors = outNeighbors(sortIX,:);
outNeighborCell{1,1} = outNeighbors';


%% Optimally connect goal state to its approximate in-neighboorhood
disp('Optimally Connecting Xgoal to in-Neighborhood')
clear boundary_values
boundary_values.xf = Xgoal(1);
boundary_values.yf = Xgoal(2);
boundary_values.zf = Xgoal(3);
boundary_values.xdotf = Xgoal(4);
boundary_values.ydotf = Xgoal(5);
boundary_values.zdotf = Xgoal(6);
boundary_values.t0 = 0;
inNeighbors = [];
for i = [1, 3:nSamples+2]
    if Xgoal_reachable_from(i) == 1
        % Set boundary values
        boundary_values.x0 = stateMat(i,1);
        boundary_values.y0 = stateMat(i,2);
        boundary_values.z0 = stateMat(i,3);
        boundary_values.xdot0 = stateMat(i,4);
        boundary_values.ydot0 = stateMat(i,5);
        boundary_values.zdot0 = stateMat(i,6);
        
        % Clear previous problem
        clear bvpinfo
        bvpinfo.numerics = numerics;
        bvpinfo.robot = robot;
        bvpinfo.boundary_values = boundary_values;
        bvpinfo.environment = environment;
        bvpinfo.options = options;
        
        % Call Solver
        bvpinfo = DeepSpaceOptimizer(bvpinfo);
        
        % Save Cost, Trajectory and Control
        if bvpinfo.solution.exitflag > 0
            n2PBVPs = n2PBVPs+1;
            curCost = bvpinfo.solution.cost;
            evalMat(i,2) = n2PBVPs;
            costMat(n2PBVPs,1) = curCost;
            trajMat(n2PBVPs,1,:) = bvpinfo.solution.x;
            trajMat(n2PBVPs,2,:) = bvpinfo.solution.y;
            trajMat(n2PBVPs,3,:) = bvpinfo.solution.z;
            trajMat(n2PBVPs,4,:) = bvpinfo.solution.xdot;
            trajMat(n2PBVPs,5,:) = bvpinfo.solution.ydot;
            trajMat(n2PBVPs,6,:) = bvpinfo.solution.zdot;
            controlMat(n2PBVPs,1,:) = bvpinfo.solution.ux;
            controlMat(n2PBVPs,2,:) = bvpinfo.solution.uy;
            controlMat(n2PBVPs,3,:) = bvpinfo.solution.uz;
            controlMat(n2PBVPs,4,:) = bvpinfo.solution.eta;
            inNeighbors = [inNeighbors; [i curCost]];
            if isempty(outNeighborCell{i,1})
                outNeighborCell{i,1}(:,1) = [2; curCost];
            else
                outPos = find(outNeighborCell{i,1}(2,:) > curCost,1,'first');
                outNeighborCell{i,1} = [outNeighborCell{i,1}(:,1:outPos), outNeighborCell{i,1}(:,outPos:end)];
                outNeighborCell{i,1}(:,outPos) = [2; curCost];
            end
        end
    end 
end
[~,sortIX] = sort(inNeighbors(:,2));
inNeighbors = inNeighbors(sortIX,:);
inNeighborCell{1,2} = inNeighbors;


%% Save/Load workspace for NOTE debugging
save('tempFMTworkspace2')
% load('tempFMTworkspace2')

%% Begin Fast Marching Trees Algorithm
disp('Running FMT Algorithm')
P = [];                     % Path from Xstart to Xgoal
V = 1:nSamples+2;           % set of all states
Vgoal = 2;                  % goal state(s)
E = [];                     % edges
W = V;                      % states not in tree
W(1) = [];                  % remove Xstart
H = 1;                      % states in tree (just Xstart)
z = 1;                      % current state
NzOut = outNeighborCell{z,1};     % set of nearest outgoing neighbors
cost2Come = zeros(1,nSamples+2);    % cost to arrive at each state

while isempty(intersect(Vgoal, z))
    Hnew = [];
    Xnear = intersect(NzOut, W);        %hmmm
    for j = 1:length(Xnear)
        x = Xnear(j);
        NxIn = inNeighborCell{1,x};
        Ynear = intersect(NxIn, H);     %hmmmm
        yMin = -1; 
        xMinCost2Come = inf;
        for i=1:length(Ynear)
            y = Ynear(i);
            yxCost2Come = cost2Come(y) + costMat(evalMat(y,x),1);
            if yxCost2Come <= xMinCost2Come
                yMin = y;
                xMinCost2Come = yxCost2Come;
            end
        end
        
        % Check Collision
        nodes2check = reshape(trajMat(evalMat(y,x),1:3,:),3,nTrajNodes)'; 
        collision = areStatesCollided(nodes2check,obstacles,spaceTilingRes,xyz_bounds,true);
        if collision % change costMat
            costMat(evalMat(y,x),1) = Inf;
        end
        
        if ~collision
            E = [E; yMin x];
            Hnew = [Hnew x];
            W(find(W==x,1)) = [];
            cost2Come(x) = xMinCost2Come; %hmmmmm
        end
    end
    H = [H Hnew];
    H(find(H==z,1)) = [];
    if isempty(H)
        disp('FMT Fail: Set H has emptied before Xgoal reached')
        return
    end
    [~, zind] = min(cost2Come(H));
    z = H(zind);
    NzOut = outNeighborCell{z,1};
end

P = FMTPath(1, Vgoal, E, cost2Come);
toc
save('tempFMTworkspace3')

%% Plot Results

% Trajectory
arrowScale = 20;
arrowHead= 5;
arrowThick = 2;
figure
hold on
for i = 1:length(obstacles.cuboids.vertices)  
    [Acol, bcol, ~, ~] = vert2lcon(obstacles.cuboids.vertices{i}');
    plotregion(-Acol, -bcol, [], [], [0.7,0.2,0.3],1);
end
plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
    'MarkerFaceColor','g', 'MarkerSize', 10)
quiver3(stateMat(1,1),stateMat(1,2),stateMat(1,3),...
    stateMat(1,4),stateMat(1,5),stateMat(1,6),arrowScale, 'MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
for i = 1:size(P,1)
    plot3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),'b*')
    quiver3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),...
        stateMat(P(i,2),4),stateMat(P(i,2),5),stateMat(P(i,2),6),arrowScale, 'MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
    tempVar = reshape(trajMat(evalMat(P(i,1),P(i,2)),1:3,:),3,nTrajNodes)';
    plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-')
end
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))