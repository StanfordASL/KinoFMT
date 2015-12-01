%   DblIntQuadPlotResults.m: Plot results data
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

function err = DblIntQuadPlotResults(probinfo)

err = 0;

sol = probinfo.solution;

% Trajectory
figure
hold on
plot3(sol.x, sol.y, sol.z)
set(gca,'ZDir','reverse')   % plot z-axis in reverse for NED
xlabel('x-position (m)')
ylabel('y-position (m)')
zlabel('z-position (m)')
legend('Solution')
title('Linear Quadrotor Trajectory')
hold off

% Velocities
figure
hold on
plot(sol.t, sol.vx, 'b' )
plot(sol.t, sol.vy, 'r' )
plot(sol.t, sol.vz, 'g' )
xlabel('time (s)')
ylabel('velocity (m/s)')
legend('vx Solution', 'vy Solution', 'vz Solution')
title('Linear Quadrotor Velocity')
hold off

% Thrust Unit Vector
figure
hold on
plot(sol.t, sol.ax, 'b' )
plot(sol.t, sol.ay, 'r' )
plot(sol.t, sol.az, 'g' )
xlabel('time (s)')
ylabel('unit vector components (#)')
legend('ux Solution', 'uy Solution', 'uz Solution')
title('Control History: Thrust Unit Vector Components')

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
