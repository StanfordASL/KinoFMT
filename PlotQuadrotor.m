%PlotQuadrotor generates a plot of obstacles 
%
%   Ross Allen, ASL, Stanford University
%
%   Started:    March 2016
%
%   Inputs:
%
%   Outputs:
%
%   Notes:  Currently plots simple, flat, unrotated quadrotor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function err = PlotQuadrotor(quadPos, armRad, rotorRad)

err = 1;

ar2 = armRad*sqrt(2);

% determine rotor centers and plot
r1 = quadPos + [ar2 ar2 0];
r2 = quadPos + [-ar2 -ar2 0];
r3 = quadPos + [-ar2 ar2 0];
r4 = quadPos + [ar2 -ar2 0];
viscircles([r1(1), r1(2)], rotorRad, 'Color', [0 0 1]);
viscircles([r2(1), r2(2)], rotorRad, 'Color', [0 0 1]);
viscircles([r3(1), r3(2)], rotorRad, 'Color', [0 0 1]);
viscircles([r4(1), r4(2)], rotorRad, 'Color', [0 0 1]);


% Plot arms
arm12 = [r1; r2];
arm34 = [r3; r4];
plot3(arm12(:,1), arm12(:,2), arm12(:,3), 'b-');
plot3(arm34(:,1), arm34(:,2), arm34(:,3), 'b-');

% Plot rotors


end