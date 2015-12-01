% Dubins_fmtPlanScript.m attempts to solve an obstacle free motion planning
% problem with Fast Marching Trees
%
%   Ross Allen, ASL, Stanford University
%   Nov 20, 2013
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
close all

load('Dubins_10k_precomp_trajectories_Nov14.mat')

% Parameters
rn = 10;                    % cost radius of neigboorhood
collisionFree = 1;          % place holder because no obstacles present

%%% Need to compute cost from xinit to all other points %%%

% For simplicity, I'm just going to use a different initial position
xinit = 3;

V_state = V;
V_nearout = cell(n_samples,1);
V_nearin = cell(n_samples,1);
V_cost2come = inf*ones(n_samples,1);
V = [1:n_samples]';                                        % line 1
Vgoal = V(end-size(Vgoal,1)+1:end);
E = [];                                                         % line 1
W = V; W(xinit) = [];                                               % line 2
H = V(xinit);                                                       % line 2
z = xinit;                                                          % line 3
Nz = Near_naive(costMatrix,z,rn);                               % line 4
V_nearout{z} = Nz;                                                 % line 5
for i=Nz
    V_nearin{i} = [V_nearin{i} z];
end
V_cost2come(z) = 0;

while isempty(intersect(z,Vgoal))        % line 6
    Hnew = [];                                      % line 7
    Xnear = intersect([V_nearout{z}], W); %hmmmmm???                  % line 8
    for j = 1:length(Xnear)                                 % line 9
        x = Xnear(j);            % line 9
        Nx = Near_naive(costMatrix, x, rn);                  % line 10
        V_nearout{x} = Nx;                                            % line 11
        for i=Nx
            V_nearin{i} = [V_nearin{i} x];
        end
        Ynear = intersect([V_nearin{x}], H); %hmmmm???   % line 12
        ymin = -1; xmincost2come = inf;
        for i=1:length(Ynear)
            y = Ynear(i);
            yxcost2come = V_cost2come(y) + costMatrix(y,x);
            if yxcost2come <= xmincost2come
                ymin = y;
                xmincost2come = yxcost2come;
            end
        end
        if collisionFree
            E = [E; ymin x];
            Hnew = [Hnew x];
            W(find(W==x,1)) = [];
            V_cost2come(x) = xmincost2come; %hmmmm, not sure about this
        end
    end
    H = [H Hnew];
    H(find(H==z,1)) = [];
    if isempty(H)
        return
    end
    [~, zind] = min(V_cost2come(H));
    z = H(zind);
end

P = FMTPath(xinit, Vgoal, E, V_cost2come);

