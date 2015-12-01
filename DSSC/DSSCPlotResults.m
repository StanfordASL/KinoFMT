%   DSSCPlotResults.m: Plot results data
%
%   Ross Allen, ASL, Stanford University 
%
%   Started:        9/29/2014
%
%   Inputs:         probinfo.solution
%
%   Outputs:        err                 nonzero if error is encountered 
%                   plots

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function err = DSSCPlotResults(probinfo)

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
title('Deep Space Spacecraft Trajectory')
hold off

% Velocities
figure
hold on
plot(sol.t, probinfo.initial_guess.rescaled(range.xdot), 'b--')
plot(sol.t, sol.xdot, 'b' )
plot(sol.t, probinfo.initial_guess.rescaled(range.ydot), 'r--')
plot(sol.t, sol.ydot, 'r' )
plot(sol.t, probinfo.initial_guess.rescaled(range.zdot), 'g--')
plot(sol.t, sol.zdot, 'g' )
xlabel('time (s)')
ylabel('velocity (m/s)')
legend('xdot Initial Guess', 'xdot Solution', 'ydot Initial Guess', 'ydot Solution', 'zdot Initial Guess', 'zdot Solution')
title('Deep Space Spacecraft Velocity')
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