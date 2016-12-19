% DblIntQuad_NeighborSVMVisual attempts to visualize the accuracy of the 
%   SVM classifier
%
% AUTHOR:   Ross Allen
% DATE:     Feb 29, 2016

clear xVals yVals vVals x_i y_i v_i initState finalState featVec reachableSet
clear probinfo mainTitleStr subTitleStr savefile blankStr
clear fig plotInit plotMisClass plotNonReachable plotReachable
close all

xVals = -15:1:15;
yVals = -15:1:15;
vVals = 0:2:14;
% vVals = 6;
initState = zeros(1,6);

for v_i = 1:length(vVals)
    
    % set velocity for initial state
    initState(5) = vVals(v_i);
    
    % set up plot
    fig = figure; hold on;
    blankStr = sprintf([blanks(1)]);
    mainTitleStr = 'SVM Neighbor Classification Accuracy';
    subTitleStr = sprintf('Initial Velocity = [0.0, %0.1f] m/s', vVals(v_i));
    subTitleStr = sprintf(['\\fontsize{10}' subTitleStr]);
    title({mainTitleStr; subTitleStr; blankStr});
    xlabel('x-position [m]', 'FontSize',12,'FontWeight','bold');
    ylabel('y-position [m]', 'FontSize',12,'FontWeight','bold');
    plot3(initState(1), initState(2), initState(3), 'ms', 'MarkerSize', 5, 'MarkerFaceColor','m');
    plotInit = quiver(initState(1), initState(2), initState(4), initState(5), 'm', 'MarkerSize', 10, 'LineWidth', 5, 'MaxHeadSize', 1.0);
    
    % iterate through x,y coords
    for x_i = 1:length(xVals)
        for y_i = 1:length(yVals)
            finalState{v_i}{x_i, y_i} = ...
                [xVals(x_i), yVals(y_i), 0, 0, 0, 0];
            featVec = mpinfo.learning.neighbor.featureSet(...
                initState, finalState{v_i}{x_i, y_i});
            reachableSet{v_i}(x_i, y_i) = is_reachable( ...
                mpinfo.learning.neighbor.svm_output, featVec);
            
            
            clear probinfo
            probinfo.numerics.n_nodes = mpinfo.sampling.nTrajNodes;
            probinfo.robot = mpinfo.robot;
            probinfo.environment = mpinfo.environment;
            probinfo.options = mpinfo.offlineOptions;
            probinfo.options.print_summary = false;
            probinfo.options.plot_results = false;
            probinfo.options.costThreshold = NaN;
            probinfo.boundary_values.x0 = initState(1);
            probinfo.boundary_values.y0 = initState(2);
            probinfo.boundary_values.z0 = initState(3);
            probinfo.boundary_values.vx0 = initState(4);
            probinfo.boundary_values.vy0 = initState(5);
            probinfo.boundary_values.vz0 = initState(6);
            probinfo.boundary_values.xf = finalState{v_i}{x_i, y_i}(1);
            probinfo.boundary_values.yf = finalState{v_i}{x_i, y_i}(2);
            probinfo.boundary_values.zf = finalState{v_i}{x_i, y_i}(3);
            probinfo.boundary_values.vxf = finalState{v_i}{x_i, y_i}(4);
            probinfo.boundary_values.vyf = finalState{v_i}{x_i, y_i}(5);
            probinfo.boundary_values.vzf = finalState{v_i}{x_i, y_i}(6);
            probinfo = DblIntQuadOptimizer(probinfo, 0);
            if isfield(probinfo.solution, 'cost')
%                 if probinfo.solution.cost < mpinfo.learning.neighborCostThreshold
%                     plot3(xVals(x_i), yVals(y_i), probinfo.solution.cost, 'b.', 'MarkerSize',1);
%                 else
%                     plot3(xVals(x_i), yVals(y_i), probinfo.solution.cost, 'k.', 'MarkerSize',1);
%                 end
            else
                disp('No Solution');
            end
            
            if (reachableSet{v_i}(x_i, y_i))
                plotReachable = plot3(xVals(x_i), yVals(y_i), 0, 'bo');
                if isfield(probinfo.solution, 'cost') && ...
                        probinfo.solution.cost > mpinfo.learning.neighborCostThreshold
                    plotMisClass = plot3(xVals(x_i), yVals(y_i), 0, 'r*', 'MarkerSize',12);
                end
            else
                plotNonReachable = plot3(xVals(x_i), yVals(y_i), 0, 'k^');
                if isfield(probinfo.solution, 'cost') && ...
                        probinfo.solution.cost <= mpinfo.learning.neighborCostThreshold
                    plotMisClass = plot3(xVals(x_i), yVals(y_i), 0, 'r*', 'MarkerSize',12);
                end
            end
        end
    end
    
    % Set up legend
    axis square
    legend([plotInit plotReachable plotNonReachable plotMisClass],...
        'initial state', 'reachable', 'non-reachable', 'misclassification',...
        'Location','southeast')
    tightfig(fig);
    savefile = ['figures/SVMVisual_initVel_', num2str(vVals(v_i))];
    print(fig, savefile, '-dsvg')
    hold off
end