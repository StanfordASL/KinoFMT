% DeepSpace_SVMDebugScript.m attempts to find the best training parameters
% for SVM classifier of the Deep Space Spacecraft reachable set
%
%   Ross Allen, ASL, Stanford University
%   Mar 4, 2014
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

s=dbstatus;
save('myBreakpoints.mat', 's');
clear all
load('myBreakpoints.mat');
dbstop(s);
clc
close all

% Add path for machine learning (ML) functions
addpath([pwd, '/Reachability Classifier/']);
addpath([pwd, '/../../../../planningdata/GenKinoFMT/']);

% General Parameters
precompMatFile = 'DeepSpacePrecompute_200x10_Jan28-2014.mat';


% Machine Learning Parameters
neighborCostThreshold = NaN;        % (cost) radius of neighborhood  NOTE: this should be changed to be based off average cost
nMLTrainingSamples = [90];            % (#) number of training examples for ML (note:~the square of the value is used for training). NaN->use all samples
nMLTestingSamples = 20;            % (#) number of training examples for ML (note:~the square of the value is used for training). NaN->use all samples
MaxIter = 5e5;      % (#) max number iterations for training
polyorders = [2];           % (#) order of polynomial of SVM kernel 
boxconstraints = [0.01];   % (#) weighting parameter 
kktviolationlevels = [0];
nRepeatTrainings = 5;

% Load Precomputed Data
if exist('precompTextFiles')
    trajMat = Text2Matrix([precompTextFiles, 'Trajectory.txt']);
    costMat = Text2Matrix([precompTextFiles, 'Cost.txt']);
    stateMat = Text2Matrix([precompTextFiles, 'StateID.txt']);
elseif exist('precompMatFile')
    load(precompMatFile);
else
    disp('Error: no precomputed data supplied. Exiting...')
    return
end
costMat = costMat(:,:,1); % remove neighbor data and recompute later
stateMat(:,1) = [];       % remove state ID tag (doesn't alighn with ML code)
if exist('controlMat')
    clear controlMat
end
[nSamples, nStateDims, nTrajNodes, ~] = size(trajMat);
disp('The information from the text files is done being imported')

% Calculate neighborhood cost threshold
if ~exist('neighborCostThreshold') || isnan(neighborCostThreshold)
    apprxNumNeighbors = nSamples/2;
    sortedCostMat = sort(costMat,2);
    neighborCostThreshold = sum(sortedCostMat(:,apprxNumNeighbors+1))/nSamples;
end
disp(['Neighborhood Cost Threshold = ', num2str(neighborCostThreshold')])
disp('------------------------')

counter = 0;
for i1 = 1:length(polyorders)
    for i2 = 1:length(boxconstraints)
        for i3 = 1:length(kktviolationlevels)
            skipRepeated = false;
            for i4 = 1:length(nMLTrainingSamples)
                for i5 = 1:nRepeatTrainings
                    counter = counter + 1;
                    disp(['Number training samples = ', num2str(nMLTrainingSamples(i4)), '^2']);
                    disp(['KKT violation level = ', num2str(kktviolationlevels(i3))]);
                    disp(['Box constraint = ', num2str(boxconstraints(i2))]);
                    disp(['Order of polynomial kernel = ', num2str(polyorders(i1))]);
                    
                    % Train Machine Learning Algorithms
                    train_options = svmset('MaxIter', MaxIter, 'polyorder', polyorders(i1),...
                        'kktviolationlevel', kktviolationlevels(i3), 'boxconstraint', boxconstraints(i2),'display','off');
                    if ~skipRepeated
                        trainingSamples = randsample(nSamples, nMLTrainingSamples(i4));
                        trainingStateMat = stateMat(trainingSamples, :);
                        trainingCostMat = costMat(trainingSamples, trainingSamples);
                        try
                            [svm_output, n_training_errors, percent_training_errors] = ...
                                reachability_classifier(trainingStateMat, trainingCostMat, ...
                                neighborCostThreshold, @deep_space_extract_2PBVP_features, train_options);
                            disp('SVM Training Converged')
                        catch err
                            disp('SVM Training Failed to Converge')
                            skipRepeated = true;
                            svm_output = NaN;
                            n_training_errors = NaN;
                            percent_training_errors = NaN;
                        end
                    else
                        disp('Skipping Repeated or Larger Training Sets Due to Prior Failure of Convergence');
                        svm_output = NaN;
                        n_training_errors = NaN;
                        percent_training_errors = NaN;
                    end
                    disp(['Number of training errors = ', num2str(n_training_errors)]);
                    disp(['Training error percentage = ', num2str(percent_training_errors)]);
                    
                    % Test SVM Algorithm
                    if ~isnan(n_training_errors)
                        testingSamples = [1:nSamples]';
                        testingSamples(trainingSamples) = [];
                        testingSamples = randsample(testingSamples, nMLTestingSamples);
                        testingStateMat = stateMat(testingSamples,:);
                        testingCostMat = costMat(testingSamples, testingSamples);
                        testingExamplesMat1 = zeros( nMLTestingSamples^2, nStateDims ); % Stores the set of all initial points for 2PBVP's
                        testingExamplesMat2 = zeros( nMLTestingSamples^2, nStateDims ); % Stores the set of all initial points for 2PBVP's
                        testing_reachability = zeros( nMLTestingSamples^2, 1 ); % Stores the reachability indicators (1 = reachable, -1 = unreachable)
                        testingCostDiffVec = zeros( nMLTestingSamples^2, 1 );
                        k = 0;
                        for k2 = 1:nMLTestingSamples
                            for k1 = 1:nMLTestingSamples
                                if k1 ~= k2
                                    k = k + 1;
                                    testingCostDiffVec(k) = testingCostMat(k1,k2)-neighborCostThreshold;
                                    testingExamplesMat1(k,:)  = testingStateMat( k1, : );
                                    testingExamplesMat2(k,:)  = testingStateMat( k2, : );
                                    if testingCostMat(k1,k2) <= neighborCostThreshold
                                        testing_reachability(k) = 1;    % reachable
                                    else
                                        testing_reachability(k) = -1;   % unreachable
                                    end
                                end
                            end
                        end
                        if k == 0
                            error('No feasible cases found for testing reachabiltiy classifier.');
                        else
                            testingExamplesMat1( (k+1):end, : )  = [];
                            testingExamplesMat2( (k+1):end, : )  = [];
                            testing_reachability( (k+1):end )    = [];
                            testingCostDiffVec( (k+1):end ) = [];
                        end
                        testing_data   = deep_space_extract_2PBVP_features( testingExamplesMat1, testingExamplesMat2);
                        testing_predicted_reachability = svmclassify(svm_output, testing_data);
                        n_testing_errors = nnz(testing_predicted_reachability - testing_reachability);
                        percent_testing_errors = 100*n_testing_errors/k;
                        diff_from_thresh_for_errors{counter} = testingCostDiffVec(find(testing_predicted_reachability - testing_reachability));
                    else
                        n_testing_errors = NaN;
                        percent_testing_errors = NaN;
                    end
                    disp(['Number of testing errors = ', num2str(n_testing_errors)]);
                    disp(['Testing error percentage = ', num2str(percent_testing_errors)]);
                    disp('------------------------')
                    
                    DebugData(counter,:) = [polyorders(i1), boxconstraints(i2), kktviolationlevels(i3), nMLTrainingSamples(i4), i5, n_training_errors, percent_training_errors, n_testing_errors, percent_testing_errors];
                end
            end
        end
    end
end
