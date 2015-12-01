clear
close all
clc

addpath([pwd, '/../']);
addpath([pwd, '/../MachineLearning/']);
addpath([pwd, '/../MachineLearning/Reachability Classifier/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);

load('WAFR_Data_1.mat')

arrowScale = 20;
arrowHead= 5;
arrowThick = 2;


% Trajectory
figure('Color',[1 1 1])
subplot(2,2,1)
hold on
if exist('obstacles', 'var')
    for i = 1:length(obstacles.cuboids.vertices)
        [Acol, bcol, ~, ~] = vert2lcon(obstacles.cuboids.vertices{i}');
        plotregion(-Acol, -bcol, [], [], [0.7,0.2,0.3],0.2);
    end
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
plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
    'MarkerFaceColor','r', 'MarkerSize', 10)
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))
xlabel('x-pos')
ylabel('y-pos')
zlabel('z-pos')
axis off

subplot(2,2,3)
hold on
if exist('obstacles', 'var')
    for i = 1:length(obstacles.cuboids.vertices)
        [Acol, bcol, ~, ~] = vert2lcon(obstacles.cuboids.vertices{i}');
        plotregion(-Acol, -bcol, [], [], [0.7,0.2,0.3],0.2);
    end
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
plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
    'MarkerFaceColor','r', 'MarkerSize', 10)
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))
xlabel('x-pos')
ylabel('y-pos')
zlabel('z-pos')
view(0,0)
axis off

subplot(2,2,2)
hold on
if exist('obstacles', 'var')
    for i = 1:length(obstacles.cuboids.vertices)
        [Acol, bcol, ~, ~] = vert2lcon(obstacles.cuboids.vertices{i}');
        plotregion(-Acol, -bcol, [], [], [0.7,0.2,0.3], 1);
    end
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
plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
    'MarkerFaceColor','r', 'MarkerSize', 10)
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))
xlabel('x-pos')
ylabel('y-pos')
zlabel('z-pos')
view(111,32)
axis off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tree
figure('Color',[1 1 1])
hold on
if exist('obstacles', 'var')
    for i = 1:length(obstacles.cuboids.vertices)
        [Acol, bcol, ~, ~] = vert2lcon(obstacles.cuboids.vertices{i}');
        plotregion(-Acol, -bcol, [], [], [0.7,0.2,0.3],1);
    end
end

plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
    'MarkerFaceColor','g', 'MarkerSize', 10)
quiver3(stateMat(1,1),stateMat(1,2),stateMat(1,3),...
    stateMat(1,4),stateMat(1,5),stateMat(1,6),arrowScale, 'MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
for i = 1:size(E,1)
    plot3(stateMat(E(i,2),1),stateMat(E(i,2),2),stateMat(E(i,2),3),'b*','MarkerSize', 0.1)
    tempVar = reshape(trajMat(evalMat(E(i,1),E(i,2)),1:3,:),3,nTrajNodes)';
    plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'y.-', 'MarkerSize', 0.1)
end
for i = 1:size(P,1)
    plot3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),'b*')
    tempVar = reshape(trajMat(evalMat(P(i,1),P(i,2)),1:3,:),3,nTrajNodes)';
    plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'b.-', 'LineWidth',10)
end
plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
    'MarkerFaceColor','r', 'MarkerSize', 10)
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))
axis off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% controls
figure('Color',[1 1 1])

subplot(1,2,1)
hold on
if exist('obstacles', 'var')
    for i = 1:length(obstacles.cuboids.vertices)
        [Acol, bcol, ~, ~] = vert2lcon(obstacles.cuboids.vertices{i}');
        plotregion(-Acol, -bcol, [], [], [0.7,0.2,0.3], 0.05);
    end
end
plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
    'MarkerFaceColor','g', 'MarkerSize', 10)
quiver3(stateMat(1,1),stateMat(1,2),stateMat(1,3),...
    stateMat(1,4),stateMat(1,5),stateMat(1,6),arrowScale, 'MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
for i = 1:size(P,1)
    plot3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),'b*')
%     quiver3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),...
%         stateMat(P(i,2),4),stateMat(P(i,2),5),stateMat(P(i,2),6),arrowScale, 'MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
    tempVar = reshape(trajMat(evalMat(P(i,1),P(i,2)),1:3,:),3,nTrajNodes)';
    plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-')
    contVar = reshape(controlMat(evalMat(P(i,1),P(i,2)),1:3,:),3,nTrajNodes)';
    quiver3(tempVar(:,1),tempVar(:,2),tempVar(:,3),...
        contVar(:,1),contVar(:,2),contVar(:,3), 0.25,'color', 'r','MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
end
plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
    'MarkerFaceColor','r', 'MarkerSize', 10)
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))
xlabel('x-pos')
ylabel('y-pos')
zlabel('z-pos')
axis off
view(111,32)

subplot(1,2,2)
hold on
if exist('obstacles', 'var')
    for i = 1:length(obstacles.cuboids.vertices)
        [Acol, bcol, ~, ~] = vert2lcon(obstacles.cuboids.vertices{i}');
        plotregion(-Acol, -bcol, [], [], [0.7,0.2,0.3], 0.05);
    end
end
plot3(stateMat(1,1),stateMat(1,2),stateMat(1,3),'g^',...
    'MarkerFaceColor','g', 'MarkerSize', 10)
quiver3(stateMat(1,1),stateMat(1,2),stateMat(1,3),...
    stateMat(1,4),stateMat(1,5),stateMat(1,6),arrowScale, 'MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
for i = 1:size(P,1)
    plot3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),'b*')
%     quiver3(stateMat(P(i,2),1),stateMat(P(i,2),2),stateMat(P(i,2),3),...
%         stateMat(P(i,2),4),stateMat(P(i,2),5),stateMat(P(i,2),6),arrowScale, 'MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
    tempVar = reshape(trajMat(evalMat(P(i,1),P(i,2)),1:3,:),3,nTrajNodes)';
    plot3(tempVar(:,1),tempVar(:,2),tempVar(:,3),'k.-')
    contVar = reshape(controlMat(evalMat(P(i,1),P(i,2)),1:3,:),3,nTrajNodes)';
    quiver3(tempVar(:,1),tempVar(:,2),tempVar(:,3),...
        contVar(:,1),contVar(:,2),contVar(:,3), 0.25,'color', 'r','MaxHeadSize', arrowHead, 'LineWidth', arrowThick)
end
plot3(stateMat(2,1),stateMat(2,2),stateMat(2,3),'ro',...
    'MarkerFaceColor','r', 'MarkerSize', 10)
xlim(xyz_bounds(1,:))
ylim(xyz_bounds(2,:))
zlim(xyz_bounds(3,:))
xlabel('x-pos')
ylabel('y-pos')
zlabel('z-pos')
axis off
view(111,32)
