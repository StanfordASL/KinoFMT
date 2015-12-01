%   SimpleUAVPlotResults.m: Plot results data
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        4/23/2014
%
%   Inputs:         probinfo.solution
%
%   Outputs:        err                 nonzero if error is encountered 
%                   plots

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function err = SimpleUAVPlotResults(probinfo)

err = 0;

sol = probinfo.solution;
range = probinfo.numerics.range;

% Trajectory
figure
hold on
plot3(probinfo.initial_guess.rescaled(range.x), probinfo.initial_guess.rescaled(range.y), probinfo.initial_guess.rescaled(range.z), '--')
plot3(sol.x, sol.y, sol.z)
xlabel('x-position (m)')
ylabel('y-position (m)')
zlabel('z-position (m)')
legend('Initial Guess', 'Solution')
title('Simple UAV Trajectory')
hold off

% Heading
figure
hold on
plot(sol.t, probinfo.initial_guess.rescaled(range.theta), '--')
plot(sol.t, sol.theta)
xlabel('time (s)')
ylabel('heading (deg)')
legend('Initial Guess', 'Solution')
title('Heading Angle vs Time')
hold off

% Control History (turnrate)
figure
hold on
plot(sol.t, probinfo.initial_guess.rescaled(range.utheta), '--')
plot(sol.t, sol.utheta)
xlabel('time (s)')
ylabel('turnrate (rad/s)')
legend('Initial Guess', 'Solution')
title('Control History: Turnrate vs Time')
hold off

% Control History (climbrate)
figure
hold on
plot(sol.t, probinfo.initial_guess.rescaled(range.uz), '--')
plot(sol.t, sol.uz)
xlabel('time (s)')
ylabel('climbrate (m/s)')
legend('Initial Guess', 'Solution')
title('Control History: Climbrate vs Time')
hold off


return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%