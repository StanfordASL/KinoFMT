% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(X(21) + mu*norm(NAEC + gradNAEC*(X - oldX), 1))
%   subject to
%      - turnrate - X(16) <= 0
%      - turnrate - X(17) <= 0
%      - turnrate - X(18) <= 0
%      - turnrate - X(19) <= 0
%      - turnrate - X(20) <= 0
%     X(16) - turnrate <= 0
%     X(17) - turnrate <= 0
%     X(18) - turnrate <= 0
%     X(19) - turnrate <= 0
%     X(20) - turnrate <= 0
%     xlower - X(1) <= 0
%     xlower - X(2) <= 0
%     xlower - X(3) <= 0
%     xlower - X(4) <= 0
%     xlower - X(5) <= 0
%     X(1) - xupper <= 0
%     X(2) - xupper <= 0
%     X(3) - xupper <= 0
%     X(4) - xupper <= 0
%     X(5) - xupper <= 0
%     ylower - X(6) <= 0
%     ylower - X(7) <= 0
%     ylower - X(8) <= 0
%     ylower - X(9) <= 0
%     ylower - X(10) <= 0
%     X(6) - yupper <= 0
%     X(7) - yupper <= 0
%     X(8) - yupper <= 0
%     X(9) - yupper <= 0
%     X(10) - yupper <= 0
%     t0 - X(21) <= 0
%     X(1) - x0 == 0
%     X(5) - xf == 0
%     X(6) - y0 == 0
%     X(10) - yf == 0
%     X(11) - th0 == 0
%     X(15) - thf == 0
%     abs(X(11) - oldX(11)) <= s_th
%     abs(X(12) - oldX(12)) <= s_th
%     abs(X(13) - oldX(13)) <= s_th
%     abs(X(14) - oldX(14)) <= s_th
%     abs(X(15) - oldX(15)) <= s_th
%     abs(X(21) - oldX(21)) <= s_tf
%
% with variables
%        X  21 x 1
%
% and parameters
%     NAEC  15 x 1
% gradNAEC  15 x 21
%       mu   1 x 1    positive
%     oldX  21 x 1
%     s_tf   1 x 1
%     s_th   5 x 1
%       t0   1 x 1
%      th0   1 x 1
%      thf   1 x 1
% turnrate   1 x 1
%       x0   1 x 1
%       xf   1 x 1
%   xlower   1 x 1
%   xupper   1 x 1
%       y0   1 x 1
%       yf   1 x 1
%   ylower   1 x 1
%   yupper   1 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.NAEC, ..., params.yupper, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2014-01-01 21:07:22 -0500.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
