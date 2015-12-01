%   SimpleQuadPlotResults.m: Plot results data
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        Mar 30, 2015
%
%   Inputs:         probinfo.solution
%
%   Outputs:        err                 nonzero if error is encountered 
%                   plots

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function err = SimpleQuadPlotResults(probinfo)

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
title('Simple Quadrotor Trajectory')
hold off

% Velocities
figure
hold on
plot(sol.t, probinfo.initial_guess.rescaled(range.vx), 'b--')
plot(sol.t, sol.vx, 'b' )
plot(sol.t, probinfo.initial_guess.rescaled(range.vy), 'r--')
plot(sol.t, sol.vy, 'r' )
plot(sol.t, probinfo.initial_guess.rescaled(range.vz), 'g--')
plot(sol.t, sol.vz, 'g' )
xlabel('time (s)')
ylabel('velocity (m/s)')
legend('vx Initial Guess', 'vx Solution', 'vy Initial Guess', 'vy Solution', 'vz Initial Guess', 'vz Solution')
title('Simple Quadrotor Velocity')
hold off

% Thrust Unit Vector
figure
hold on
plot(sol.t, probinfo.initial_guess.rescaled(range.ux), 'b--')
plot(sol.t, sol.ux, 'b' )
plot(sol.t, probinfo.initial_guess.rescaled(range.uy), 'r--')
plot(sol.t, sol.uy, 'r' )
plot(sol.t, probinfo.initial_guess.rescaled(range.uz), 'g--')
plot(sol.t, sol.uz, 'g' )
xlabel('time (s)')
ylabel('unit vector components (#)')
legend('ux Initial Guess', 'ux Solution', 'uy Initial Guess', 'uy Solution', 'uz Initial Guess', 'uz Solution')
title('Control History: Thrust Unit Vector Components')

% Throttle
figure
hold on
plot(sol.t, probinfo.initial_guess.rescaled(range.eta), 'b--')
plot(sol.t, sol.eta, 'b' )
xlabel('time (s)')
ylabel('throttle (#)')
legend('eta Initial Guess', 'eta Solution')
title('Control History: Throttle')
hold off

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
