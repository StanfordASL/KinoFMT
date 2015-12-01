% boundary_values.x0 = stateMat(45,1);
% boundary_values.y0 = stateMat(45,2);
% boundary_values.z0 = stateMat(45,3);
% boundary_values.xdot0 = stateMat(45,4);
% boundary_values.ydot0 = stateMat(45,5);
% boundary_values.zdot0 = stateMat(45,6);
% boundary_values.t0 = 0;
% boundary_values.xf = stateMat(38,1);
% boundary_values.yf = stateMat(38,2);
% boundary_values.zf = stateMat(38,3);
% boundary_values.xdotf = stateMat(38,4);
% boundary_values.ydotf = stateMat(38,5);
% boundary_values.zdotf = stateMat(38,6);
% clear bvpinfo
% bvpinfo.numerics = numerics;
% bvpinfo.robot = robot;
% bvpinfo.boundary_values = boundary_values;
% bvpinfo.environment = environment;
% bvpinfo.options = options;
% % Call solver
% bvpinfo = DeepSpaceOptimizer(bvpinfo);

A = [initStateIDs(testingSampleTotCaseNums) finalStateIDs(testingSampleTotCaseNums)];
B = [initStateIDs(trainingSampleTotCaseNums) finalStateIDs(trainingSampleTotCaseNums)];

for ai = 1:length(A)
    for bi = 1:length(B)
        if isequal(A(ai,:),B(bi,:))
            disp([num2str(ai), ', ', num2str(bi)])
        end
    end
end