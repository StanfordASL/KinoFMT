%   DubinsPlotResults.m: Plot results data
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        10/21/2013
%
%   Inputs:         dubprob.solution
%
%   Outputs:        err                 nonzero if error is encountered 
%                   plots

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function err = DubinsPlotResults(dubprob)

err = 0;

sol = dubprob.solution;
range = dubprob.numerics.range;

% Trajectory
figure
hold on
plot(dubprob.initial_guess.rescaled(range.x), dubprob.initial_guess.rescaled(range.y), '--')
plot(sol.x, sol.y)
xlabel('x-position (m)')
ylabel('y-position (m)')
legend('Initial Guess', 'Solution')
title('Dubins Vehicle Trajectory')
hold off

% Heading
figure
hold on
plot(sol.t, dubprob.initial_guess.rescaled(range.theta), '--')
plot(sol.t, sol.theta)
xlabel('time (s)')
ylabel('heading (deg)')
legend('Initial Guess', 'Solution')
title('Heading Angle vs Time')
hold off

% Control History (turnrate
figure
hold on
plot(sol.t, dubprob.initial_guess.rescaled(range.u), '--')
plot(sol.t, sol.u)
xlabel('time (s)')
ylabel('turnrate (deg/s)')
legend('Initial Guess', 'Solution')
title('Control History: Turnrate vs Time')
hold off

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%