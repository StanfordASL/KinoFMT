
function train_options = svmset( varargin )
% SVMSET
%     Kernel function options:
%     'linear'      ? Linear kernel, meaning dot product:   K(x,z) = x'*z.
%     'quadratic'   ? Quadratic kernel:                     K(x,z) = (x'*z + c)^2
%     'polynomial'  ? Polynomial kernel (default order 3):  K(x,z) = (x'*z + c)^(polyorder)
%     'rbf'         ? Gaussian Radial Basis Function kernel with a default scaling factor, sigma, of 1: K(x,z) = exp( -(1/(2*rbf_sigma^2))*||x - z|| )
%     'mlp'         ? Multilayer Perceptron kernel with default scale [1 ?1].  Uses layers of sigmoid functions, e.g. phi(v_i) = tanh(v_i).
%     @kfun         ? Function handle to a kernel function. A kernel function must be of the form: function K = kfun(U, V)
%                       The returned value, K, is a matrix of size M-by-N, where U and V have M and N rows respectively
% 
%     Method options:
%     'QP'          ? Quadratic programming (2-norm soft-margin SVM).
%     'SMO'         ? Sequential Minimal Optimization
%     'LS'          ? Least squares.
%
%     Name                  Value
%     'kernel_function'     A string or a function handle specifying the
%                           kernel function used to represent the dot
%                           product in a new space. The value can be one of
%                           the following:
%                           'linear'     - Linear kernel or dot product
%                                          (default). In this case, svmtrain
%                                          finds the optimal separating plane
%                                          in the original space.
%                           'quadratic'  - Quadratic kernel
%                           'polynomial' - Polynomial kernel with default
%                                          order 3. To specify another order,
%                                          use the 'polyorder' argument.
%                           'rbf'        - Gaussian Radial Basis Function
%                                          with default scaling factor 1. To
%                                          specify another scaling factor,
%                                          use the 'rbf_sigma' argument.
%                           'mlp'        - Multilayer Perceptron kernel (MLP)
%                                          with default weight 1 and default
%                                          bias -1. To specify another weight
%                                          or bias, use the 'mlp_params'
%                                          argument.
%                           function     - A kernel function specified using
%                                          @(for example @KFUN), or an
%                                          anonymous function. A kernel
%                                          function must be of the form
%  
%                                          function K = KFUN(U, V)
%  
%                                          The returned value, K, is a matrix
%                                          of size M-by-N, where M and N are
%                                          the number of rows in U and V
%                                          respectively.
%  
%     'rbf_sigma'           A positive number specifying the scaling factor
%                           in the Gaussian radial basis function kernel.
%                           Default is 1.
%  
%     'polyorder'           A positive integer specifying the order of the
%                           polynomial kernel. Default is 3.
%  
%     'mlp_params'          A vector [P1 P2] specifying the parameters of MLP
%                           kernel.  The MLP kernel takes the form:
%                           K = tanh(P1*U*V' + P2),
%                           where P1 > 0 and P2 < 0. Default is [1,-1].
%  
%     'method'              A string specifying the method used to find the
%                           separating hyperplane. Choices are:
%                           'SMO' - Sequential Minimal Optimization (SMO)
%                                   method (default). It implements the L1
%                                   soft-margin SVM classifier.
%                           'QP'  - Quadratic programming (requires an
%                                   Optimization Toolbox license). It
%                                   implements the L2 soft-margin SVM
%                                   classifier. Method 'QP' doesn't scale
%                                   well for TRAINING with large number of
%                                   observations.
%                           'LS'  - Least-squares method. It implements the
%                                   L2 soft-margin SVM classifier.
%  
%    'tolkkt'              A positive scalar that specifies the tolerance
%                          with which the Karush-Kuhn-Tucker (KKT) conditions
%                          are checked for method 'SMO'. Default is
%                          1.0000e-003.
%  
%    'kktviolationlevel'   A scalar specifying the fraction of observations
%                          that are allowed to violate the KKT conditions for
%                          method 'SMO'. Setting this value to be positive
%                          helps the algorithm to converge faster if it is
%                          fluctuating near a good solution. Default is 0.
%  
%    'kernelcachelimit'    A positive scalar S specifying the size of the
%                          kernel matrix cache for method 'SMO'. The
%                          algorithm keeps a matrix with up to S * S
%                          double-precision numbers in memory. Default is
%                          5000. When the number of points in TRAINING
%                          exceeds S, the SMO method slows down. It's
%                          recommended to set S as large as your system
%                          permits.
%  
%    'boxconstraint'       The box constraint C for the soft margin. C can be
%                          a positive numeric scalar or a vector of positive
%                          numbers with the number of elements equal to the
%                          number of rows in TRAINING.
%                          Default is 1.
%                          * If C is a scalar, it is automatically rescaled
%                            by N/(2*N1) for the observations of group one,
%                            and by N/(2*N2) for the observations of group
%                            two, where N1 is the number of observations in
%                            group one, N2 is the number of observations in
%                            group two. The rescaling is done to take into
%                            account unbalanced groups, i.e., when N1 and N2
%                            are different.
%                          * If C is a vector, then each element of C
%                            specifies the box constraint for the
%                            corresponding observation.
%  
%     'autoscale'          A logical value specifying whether or not to
%                          shift and scale the data points before training.
%                          When the value is true, the columns of TRAINING
%                          are shifted and scaled to have zero mean unit
%                          variance. Default is true.
%  
%     'showplot'           A logical value specifying whether or not to show
%                          a plot. When the value is true, svmtrain creates a
%                          plot of the grouped data and the separating line
%                          for the classifier, when using data with 2
%                          features (columns). Default is false.
%
%     'display'            Level of display.  'off', 'iter', or 'final'.
%

    % Default values
    autoscale           = true;             % If true, automatically centers data at their mean and scales to unit standard deviation before training.
    C                   = 1;                % Value of the box constraint C for the soft margin. C can be a scalar, or a vector of the same length as the training data.
    kernel_cachelimit   = 5000;             % Value that specifies the size of the kernel matrix cache for the SMO training method.
    kernel_function     = 'polynomial';     % Kernel function used to map the training data into kernel space.
    violation_level     = 0;                % A value in [0,1) for the fraction of samples allowed to violate the KKT conditions for the SMO training method.
    method              = 'SMO';            % Method used to find the separating hyperplane.
    mlp_params          = [1, -1];          % Parameters of the Multilayer Perceptron (mlp) kernel.
    display             = 'final';          % String that specifies the info that is displayed as the algorithm runs: 'off', 'iter', or 'final'.
    max_iter            = 50000;            % Maximum number of iterations of the main loop.
    polyorder           = 4;                % Order of the polynomial kernel.
    rbf_sigma           = 1;                % Scaling factor (sigma) in the radial basis function kernel.
    showplot            = true;             % Indicates whether to plot the grouped data and separating line.
    kkt_tol             = 1e-3;             % Tolerance with which the KKT conditions are checked for the SMO training method.

    % Set to user-defined values
    for k = 1:2:length(varargin)
        if strcmpi( varargin{k}, 'autoscale' )
            autoscale   = varargin{k+1};
        elseif strcmpi( varargin{k}, 'boxconstraint' )
            C           = varargin{k+1};
        elseif strcmpi( varargin{k}, 'kernelcachelimit' )
            kernel_cachelimit           = varargin{k+1};
        elseif strcmpi( varargin{k}, 'kernel_function' )
            kernel_function           = varargin{k+1};
        elseif strcmpi( varargin{k}, 'kktviolationlevel' )
            violation_level           = varargin{k+1};
        elseif strcmpi( varargin{k}, 'method' )
            method           = varargin{k+1};
        elseif strcmpi( varargin{k}, 'mlp_params' )
            mlp_params           = varargin{k+1};
        elseif strcmpi( varargin{k}, 'display' )
            display           = varargin{k+1};
        elseif strcmpi( varargin{k}, 'MaxIter' )
            max_iter           = varargin{k+1};
        elseif strcmpi( varargin{k}, 'polyorder' )
            polyorder           = varargin{k+1};
        elseif strcmpi( varargin{k}, 'rbf_sigma' )
            rbf_sigma           = varargin{k+1};
        elseif strcmpi( varargin{k}, 'showplot' )
            showplot          = varargin{k+1};
        elseif strcmpi( varargin{k}, 'tolkkt' )
            kkt_tol           = varargin{k+1};
        end
    end
    
    % Generate the svmtrain settings structure
    train_options = struct( ...
        'autoscale', autoscale, ...
        'C', C, ...
        'kernel_cachelimit', kernel_cachelimit, ...
        'kernel_function', kernel_function, ...
        'violation_level', violation_level, ...
        'method', method, ...
        'mlp_params', mlp_params, ...
        'display', display, ...
        'max_iter', max_iter, ...
        'polyorder', polyorder, ...
        'rbf_sigma', rbf_sigma, ...
        'showplot', showplot, ...
        'kkt_tol', kkt_tol ...
    );

end