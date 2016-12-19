% Calculate average path cost and computation time and max vel
avgCost = 0;
avgTime = 0;
avgNodes = 0;
counter = 0;
maxVel = 0;
for i = 1:nTrials
    if (trial_info.exitCond(i) == 1 || trial_info.exitCond(i) == -2)
        avgCost = avgCost + trial_info.optCost{i};
        avgTime = avgTime + trial_info.onlineCompTime{i};
        avgNodes = avgNodes + length(trial_info.optPath{i});
        counter = counter + 1;
%         if (trial_info.exitCond(i) == 1)
%             for j = 1:length(trial_info.optPath{i})-1
%                 nodeA = trial_info.optPath{i}(j);
%                 nodeB = trial_info.optPath{i}(j+1);
%                 stateA = mpinfo_trial{i}.stateMat(nodeA,1:3);
%                 stateB = mpinfo_trial{i}.stateMat(nodeB,1:3);
%                 dist = norm(stateB-stateA); 
%                 dt = mpinfo_trial{i}.costMat(mpinfo_trial{i}.evalMat(nodeA,nodeB));
%                 avgVel = dist/dt;
%                 if avgVel > maxVel
%                     maxVel = avgVel;
%                 end
%             end
%         end
    end
end
clear i j dist nodeA nodeB avgVel dt stateA stateB
avgCost = avgCost/counter;
avgTime = avgTime/counter;
avgNodes = avgNodes/counter;

% Calculate failure rate
nFullSuccess = length(find(trial_info.exitCond == 1));
nPartSuccess = length(find(trial_info.exitCond == -2));
failRate = 100*(nTrials - nFullSuccess - nPartSuccess)/nTrials;
