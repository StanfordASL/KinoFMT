
function svm_output = svm_train( training_data, training_labels, train_options )
    
    if strcmpi( train_options.method, 'QP' )
        options   	= optimset('display', train_options.display, 'maxiter', train_options.max_iter);
    elseif strcmpi( train_options.method, 'SMO' )
        options   	= statset('display', train_options.display, 'maxiter', train_options.max_iter);
    end
    
    if size(training_data,2) > 2
        % Cannot display data of dimension greater than 2
        train_options.showplot = false;
    end
    if ~strcmpi( train_options.kernel_function, 'rbf' )
        % Suppresses rbf warning message
        train_options.rbf_sigma = [];
    end
    if ~strcmpi( train_options.kernel_function, 'mlp' )
        % Suppresses mlp warning message
        train_options.mlp_params = [];
    end
    if ~strcmpi( train_options.kernel_function, 'polynomial' )
        % Suppresses polynomial warning message
        train_options.polyorder = [];
    end
    
    svm_output 	= svmtrain( training_data, training_labels, ...
        'autoscale', train_options.autoscale, ...
        'boxconstraint', train_options.C, ...
        'kernelcachelimit', train_options.kernel_cachelimit, ...
        'kernel_function', train_options.kernel_function, ...
        'kktviolationlevel', train_options.violation_level, ...
        'method', train_options.method, ...
        'mlp_params', train_options.mlp_params, ...
        'options', options, ...
        'polyorder', train_options.polyorder, ...
        'rbf_sigma', train_options.rbf_sigma, ...
        'showplot', train_options.showplot, ...
        'tolkkt', train_options.kkt_tol );
end