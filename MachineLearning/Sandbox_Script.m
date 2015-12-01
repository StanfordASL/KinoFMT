% Sandbox_Script
close all

originpt = 1;
maxCostToPlot = 50;
infcost = find(isinf(costMat(originpt,:)));
smallcosts = find(costMat(originpt,:) < maxCostToPlot);

speeds = sqrt(sum(stateMat(:,4:6).^2,2));

figure
hold on
plot3(stateMat(originpt,1),stateMat(originpt,2),stateMat(originpt,3), 'k^', 'MarkerSize',10) 
scatter3(stateMat(smallcosts,1),stateMat(smallcosts,2),stateMat(smallcosts,3), [],costMat(originpt,smallcosts), 'filled')
quiver3(stateMat(smallcosts,1),stateMat(smallcosts,2),stateMat(smallcosts,3),...
    stateMat(smallcosts,4),stateMat(smallcosts,5),stateMat(smallcosts,6))
% quiver3(stateMat(originpt,1),stateMat(originpt,2),stateMat(originpt,3),...
%     stateMat(originpt,4),stateMat(originpt,5),stateMat(originpt,6))
grid on

figure
hold on
plot3(stateMat(originpt,1),stateMat(originpt,2),stateMat(originpt,3), 'k^', 'MarkerSize',10)
scatter3(stateMat(infcost,1),stateMat(infcost,2),stateMat(infcost,3), [],costMat(originpt,infcost), 'filled')
quiver3(stateMat(infcost,1),stateMat(infcost,2),stateMat(infcost,3),...
    stateMat(infcost,4),stateMat(infcost,5),stateMat(infcost,6))
xlabel('x-pos')
ylabel('y-pos')
zlabel('z-pos')
title('Points with infinite cost 2PBVPs from a single origin point', 'FontSize',34)
grid on
