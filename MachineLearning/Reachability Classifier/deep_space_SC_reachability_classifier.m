
%% CS 229 Machine Learning Project
% Ross Allen
% Ashley A. Clark
% Joseph A. Starek
% February 6th, 2013

function deep_space_SC_reachability_classifier
clc; close all; clear all;

%% Enter Input

training_filename       = 'Data/DeepSpaceTrainingData-IROS_InitialSubmission.txt';      % Name and extension of training data file
testing_filename        = 'Data/DeepSpaceTestingData-IROS_InitialSubmission.txt';       % Name and extension of testing data file
linreg_results_filename = '../DeepSpaceResults_CostClassifier_IROS_InitialSubmission.mat';

train_options               = svmset('MaxIter', 60000);
extract_2PBVP_features      = @deep_space_extract_2PBVP_features;
save_figures                = true;
validation_mode             = 'none';       	% Type of validation method to use: 'holdout', 'kFold', 'leave_one_out', 'none'
plot_mode                   = 'publication';  	% Mode to use for displaying plots: 'display', 'publication'
sigma_threshold             = [-1, 0, 1];       % Signed number of standard deviations from mean cost for setting the reachability cost threshold
holdout_percent             = 30;              	% Percentage of training data to hold out for Holdout Cross-Validation
K                           = 10;              	% Number of training data partitions for K-Fold Cross-Validation

positive_label_format               = 'ob';
positive_label_linewidth            = 3;
positive_label_facecolor            = 'none';
negative_label_format               = 'dr';
negative_label_linewidth            = 3;
negative_label_facecolor            = 'none';
linreg_positive_label_format     	= '.';
linreg_positive_label_linewidth     = 2;
linreg_positive_label_facecolor     = 'none';
linreg_negative_label_format     	= '+m';
linreg_negative_label_linewidth     = 2;
linreg_negative_label_facecolor     = 'none';
linreg_positive_markersize          = 12;
linreg_positive_color               = 'b'; %'k';
linreg_negative_markersize          = 12;
linreg_negative_color               = 'r'; %[0 0.5 0];


%% Process Training and Testing Data

% Training labels
TRUE        = 1;
FALSE       = -1;

% Test if training and test data files exist
if fopen( training_filename ) == -1
    error('Be sure to enter a valid filename for the training file.');
end
test_file_exists = not( fopen( testing_filename ) == -1 );

% Read in and plot input training data
[ x_f, y_f, z_f, xdot_0, ydot_0, zdot_0, cost, ~ ] = read_data( training_filename );

% Extract the matrix of feature vectors from the matrix of states
m               = length(x_f);
stateMat1       = [ zeros(m,3), xdot_0, ydot_0, zdot_0 ];
stateMat2       = [ x_f, y_f, z_f, zeros(m,3) ];
training_data   = extract_2PBVP_features( stateMat1, stateMat2 );
%training_data = extract_features( x_f, y_f, z_f, xdot_0, ydot_0, zdot_0 );
%m   = size(training_data,1);
n   = size(training_data,2);

% Read in and plot input testing data
if test_file_exists
    [ x_f_test, y_f_test, z_f_test, xdot_0_test, ydot_0_test, zdot_0_test, cost_test, run_number_test ] = read_data( testing_filename );
    m_test = length(cost_test);
end

% Extract the matrix of feature vectors from the matrix of states
%testing_data = extract_features( x_f_test, y_f_test, z_f_test, xdot_0_test, ydot_0_test, zdot_0_test );


%% Add Linear Weighted Regression Data (if Available)
if fopen( linreg_results_filename ) == -1
    error('Be sure to enter a valid filename for the weighted linear regression results file.');
end
linreg_file_exists = not( fopen( testing_filename ) == -1 );

if linreg_file_exists
    load( linreg_results_filename, 'featureResults' );
    linreg_estimatedCosts	= featureResults{1}.estimatedCosts;
end


%% Compute Reachability Set Approximations for Various Cost Thresholds

for sigma = sigma_threshold
    
    %% Label Training Data

    % Determine reachability within cost threshold
    cost_threshold          = mean(cost) + sigma*std(cost);
    training_reachability   = FALSE.*ones(m,1);
    training_reachability( cost <= cost_threshold ) = TRUE;
    
    
    %% Run the SVM Algorithm
    test_options = struct( ...
        'showplot', train_options.showplot ...
    );

    % Run SVM on portions of the training data, using one of the cross-validation techniques
    if strcmpi(validation_mode, 'holdout'),
        [svm_output, n_errors, error_percent] = ...
            holdOut_cross_validation( holdout_percent, training_data, training_reachability, train_options, test_options );
        error_string            = 'Number of Errors (Held-out Data)';
        error_percent_string    = 'Percentage Error (Held-out Data)';

    elseif strcmpi(validation_mode, 'kFold')
        [svm_output, n_errors, error_percent] = ...
            kFold_cross_validation( K, training_data, training_reachability, train_options, test_options );
        error_string            = ['Number of Errors (Avg over each (1/', num2str(k), ')-th of Data)'];
        error_percent_string    = ['Percentage Error (Avg over each (1/', num2str(k), ')-th of Data)'];

    elseif strcmpi(validation_mode, 'leave_one_out')
        [svm_output, n_errors, error_percent] = ...
            leaveOneOut_cross_validation( training_data, training_reachability, train_options, test_options );
        error_string            = 'Number of Errors (Avg over each Data Pt)';
        error_percent_string    = 'Percentage Error (Avg over each Data Pt)';
        
    elseif strcmpi(validation_mode, 'none')
        svm_output              = { svm_train( training_data, training_reachability, train_options ) };
        n_errors                = NaN;
        error_percent           = NaN;
        error_string            = 'Number of Validation Errors (N/A)';
        error_percent_string    = 'Percentage Validation Error (N/A)';
    end
    
    
    %% Apply the Fit SVM Parameters to the Test Set from Test Set Data File
    
    % Apply classifier to test data
    if test_file_exists
        test_reachability   = FALSE.*ones(m_test,1);
        prediction_times    = zeros(m_test,1);

        for j = 1:m_test
            tic;
            %test_data_point         = extract_features( x_f_test(j), y_f_test(j), z_f_test(j), xdot_0_test(j), ydot_0_test(j), zdot_0_test(j) );
            test_data_point         = extract_2PBVP_features( [ 0, 0, 0, xdot_0_test(j), ydot_0_test(j), zdot_0_test(j) ], [ x_f_test(j), y_f_test(j), z_f_test(j), 0, 0, 0 ] );
            test_reachability(j)	= is_reachable( svm_output, test_data_point );
            prediction_times(j)     = toc;
        end
        avg_prediction_time = mean(prediction_times);
        [n_test_errors, test_error_percent] = count_errors( test_reachability, (cost_test < cost_threshold) );
    end
    
    
    %% Plot the Results
    
    % Plot the set of reachable states
    figure;
    if test_file_exists
        subplot(1,2,1); 
    end;
    hold on;
    scatter3( x_f( training_reachability == TRUE ), y_f( training_reachability == TRUE ), z_f( training_reachability == TRUE ), ...
        positive_label_format, 'Linewidth', positive_label_linewidth, 'MarkerFaceColor', positive_label_facecolor );
    scatter3( x_f( training_reachability == FALSE ), y_f( training_reachability == FALSE ), z_f( training_reachability == FALSE ), ...
        negative_label_format, 'Linewidth', negative_label_linewidth, 'MarkerFaceColor', negative_label_facecolor );
    view([43, 22]);
    box off; grid on;
    legend('Reachable', 'Unreachable', 'Location', 'SouthOutside');
    xlabel('x_f [m]');
    ylabel('y_f [m]');
    zlabel('z_f [m]');
    title(['Set of ', num2str(cost_threshold), '-Cost Reachable States']);
    
    if test_file_exists
        subplot(1,2,2); hold on;
        scatter3( x_f_test( test_reachability == true ), y_f_test( test_reachability == true ), z_f_test( test_reachability == true ), ...
            positive_label_format, 'Linewidth', positive_label_linewidth, 'MarkerFaceColor', positive_label_facecolor );
        scatter3( x_f_test( test_reachability == false ), y_f_test( test_reachability == false ), z_f_test( test_reachability == false ), ...
            negative_label_format, 'Linewidth', negative_label_linewidth, 'MarkerFaceColor', negative_label_facecolor );
        view([43, 22]);
        box off; grid on;
        legend('Reachable', 'Unreachable', 'Location', 'SouthOutside');
        xlabel('x_f [m]');
        ylabel('y_f [m]');
        zlabel('z_f [m]');
        title(['Estimate of the ', num2str(cost_threshold), '-Cost Reachable States']);

        figure;
        hold on;
        plot( run_number_test( test_reachability == true ), cost_test( test_reachability == true ), ...
            positive_label_format, 'Linewidth', positive_label_linewidth, 'MarkerFaceColor', positive_label_facecolor );
        plot( run_number_test( test_reachability == false ), cost_test( test_reachability == false ), ...
            negative_label_format, 'Linewidth', negative_label_linewidth, 'MarkerFaceColor', negative_label_facecolor );
        if linreg_file_exists
            plot( run_number_test( linreg_estimatedCosts <= cost_threshold ), cost_test( linreg_estimatedCosts <= cost_threshold ), ...
                linreg_positive_label_format, 'Linewidth', linreg_positive_label_linewidth, 'MarkerFaceColor', linreg_positive_label_facecolor, ...
                'MarkerSize', linreg_positive_markersize, 'Color', linreg_positive_color );
            plot( run_number_test( linreg_estimatedCosts > cost_threshold ), cost_test( linreg_estimatedCosts > cost_threshold ), ...
                linreg_negative_label_format, 'Linewidth', linreg_negative_label_linewidth, 'MarkerFaceColor', linreg_negative_label_facecolor, ...
                'MarkerSize', linreg_negative_markersize, 'Color', linreg_negative_color );
        end
        V = axis;
        plot( [V(1), V(2)], cost_threshold.*ones(2,1), '-k', 'Linewidth', 2 );
        xlabel('Run');
        ylabel('Cost');
        title(['Test Data Costs with Reachability Cost Threshold ', num2str(cost_threshold), ' (\sigma = ', num2str(sigma), ')']);
        grid on;
    end
    
    
    %% Report the Results
    fprintf( 1, '\nCost Threshold: \t\t\t%f (sigma = %f)\n', cost_threshold, sigma );
    fprintf( 1, 'Validation Method: \t\t\t%s\n', validation_mode );
    fprintf( 1, '%s: \t%d\n', error_string, n_errors );
    fprintf( 1, '%s: \t%f%%\n', error_percent_string, error_percent );
    if test_file_exists
        fprintf( 1, 'Number of Test Set Errors: \t%d\n', n_test_errors );
        fprintf( 1, 'Percentage Test Set Errors: %f%%\n', test_error_percent );
        fprintf( 1, 'Average Prediction Time: \t%f\n', avg_prediction_time );
    end
    fprintf( 1, '\n');
    
end


%% Adjust and Save Plots
if save_figures
    if test_file_exists
        figure_names = {'DeepSpaceSC_training_data_feasibility', 'DeepSpaceSC_test_data_feasibility', ...
            'DeepSpaceSC_reachability_-1sigma', 'DeepSpaceSC_cost_reachability_-1sigma', ...
            'DeepSpaceSC_reachability_0sigma', 'DeepSpaceSC_cost_reachability_0sigma', ...
            'DeepSpaceSC_reachability_1sigma', 'DeepSpaceSC_cost_reachability_1sigma'};
    else
        figure_names = {'DeepSpaceSC_training_data_feasibility', ...
            'DeepSpaceSC_reachability_-1sigma', ...
            'DeepSpaceSC_reachability_0sigma', ...
            'DeepSpaceSC_reachability_1sigma'};
    end

    set_plot_properties( 'mode', plot_mode );
    savefigs('Format', 'fig', 'Save', 'all', 'SaveDir', 'Results', 'ZipName', 'DeepSpaceSC_figs', 'SaveAs', figure_names );
    savefigs('Format', 'png', 'Save', 'all', 'SaveDir', 'Results', 'ZipName', 'DeepSpaceSC_pngs', 'SaveAs', figure_names );
end


end


function [x_f, y_f, z_f, xdot_0, ydot_0, zdot_0, cost, run_number] = read_data( filename )
    % Read in training data
    fid         = fopen( filename );
    data        = textscan( fid, '%d%f%f%f%f%f%f%f%d', 'Headerlines', 0, 'Delimiter', ';' );
    run_number  = data{1};
    x_f         = data{2};
    y_f         = data{3};
    z_f         = data{4};
    xdot_0      = data{5};
    ydot_0      = data{6};
    zdot_0      = data{7};
    cost        = data{8};
    exit_flag   = data{9};

    % Visualize infeasible data
    figure; hold on;
    scatter3( x_f( exit_flag == 1 ), y_f( exit_flag == 1 ), z_f( exit_flag == 1 ), '.b' );
    scatter3( x_f( exit_flag ~= 1 ), y_f( exit_flag ~= 1 ), z_f( exit_flag ~= 1 ), 'sk', 'MarkerFaceColor', 'r' );
    view([43, 22]);
    box off; grid on;
    legend('Feasible', 'Infeasible', 'Location', 'EastOutside');
    xlabel('x_f [m]');
    ylabel('y_f [m]');
    zlabel('z_f [m]');
    title('Distribution of Input Data');
    
    % Use only feasible data: reject any 2PBVP solutions that did not converge
    x_f         = x_f( exit_flag > 0 );
    y_f         = y_f( exit_flag > 0 );
    z_f         = z_f( exit_flag > 0 );
    xdot_0      = xdot_0( exit_flag > 0 );
    ydot_0      = ydot_0( exit_flag > 0 );
    zdot_0      = zdot_0( exit_flag > 0 );
    cost        = cost( exit_flag > 0 );
    run_number  = 1:length(x_f);
end

% function feature_matrix = extract_features( x_f, y_f, z_f, xdot_0, ydot_0, zdot_0 )
%     feature_matrix = [ ...
%         x_f, ...
%         y_f, ...
%         z_f, ...
%         xdot_0, ...
%         ydot_0, ...
%         zdot_0, ...
%         x_f.^2, ...
%         y_f.^2, ...
%         z_f.^2, ...
%         xdot_0.^2, ...
%         ydot_0.^2, ...
%         zdot_0.^2, ...
%         abs(xdot_0), ...
%         abs(ydot_0), ...
%         abs(zdot_0), ...
%         sqrt(x_f.^2 + y_f.^2 + z_f.^2), ...
%         sqrt(xdot_0.^2 + ydot_0.^2 + zdot_0.^2), ...
%         sqrt(x_f.^2 + y_f.^2 + z_f.^2 + xdot_0.^2 + ydot_0.^2 + zdot_0.^2), ...
%         x_f./xdot_0, ...
%         y_f./ydot_0, ...
%         z_f./zdot_0, ...
%         (x_f./xdot_0).^2, ...
%         (y_f./ydot_0).^2, ...
%         (z_f./zdot_0).^2, ...
%         sqrt( (x_f./xdot_0).^2 + (y_f./ydot_0).^2 + (z_f./zdot_0).^2 ), ...
%         sqrt(x_f.^2 + y_f.^2 + z_f.^2) ./ sqrt(xdot_0.^2 + ydot_0.^2 + zdot_0.^2)
%     ];
% end

