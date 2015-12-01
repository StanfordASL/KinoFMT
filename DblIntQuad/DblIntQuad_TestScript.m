% Testing calculations of optimal control of linear double integrator model.
%
%	See: Webb and van den Berg: Kinodynamic RRT*


clear
close all
clc

probinfo.options.tf_min = 1e-6;
probinfo.options.tf_max = 10;
probinfo.options.costThreshold = NaN;
probinfo.options.controlPenaltyWeight = 1000000000;

probinfo.numerics.n_nodes = 10; % hmmmm

probinfo.options.solver.MaxIter = 100;
probinfo.options.solver.TolFun = 1e-12;
probinfo.options.solver.TolX = 1e-12;

% probinfo.boundary_values.x0 = 4;
% probinfo.boundary_values.y0 = 4;
% probinfo.boundary_values.z0 = -1;
% probinfo.boundary_values.vx0 = 0;
% probinfo.boundary_values.vy0 = 0;
% probinfo.boundary_values.vz0 = 0;
% probinfo.boundary_values.xf = 1;
% probinfo.boundary_values.yf = 1;
% probinfo.boundary_values.zf = -2.5;
% probinfo.boundary_values.vxf = 0;
% probinfo.boundary_values.vyf = 0;
% probinfo.boundary_values.vzf = 0;
probinfo.boundary_values.x0 = 0;
probinfo.boundary_values.y0 = 0;
probinfo.boundary_values.z0 = -1;
probinfo.boundary_values.vx0 = 0;
probinfo.boundary_values.vy0 = 0;
probinfo.boundary_values.vz0 = 0;
probinfo.boundary_values.xf = 0;
probinfo.boundary_values.yf = 0;
probinfo.boundary_values.zf = -2;
probinfo.boundary_values.vxf = 0;
probinfo.boundary_values.vyf = 0;
probinfo.boundary_values.vzf = -5;

probinfo = DblIntQuadNumericData(probinfo);
tic
probinfo = DblIntQuadOptimizer(probinfo);
toc
% x0 = [0; 0; 0; 0; 0; 0];
% x1 = [0; 0; -1; 0; 0; 0];
% 
% 
% nS = 6;
% nC = 3;
% g = 9.81;       % (m/s^2)
% wR = 5;
% funcDel = 1e-6;
% stepDel = 1e-6;
% iterMax = 1000;
% 
% A = zeros(nS,nS);
% A(1,4) = 1;
% A(2,5) = 1;
% A(3,6) = 1;
% 
% B = zeros(nS,nC);
% B(4,1) = 1;
% B(5,2) = 1;
% B(6,3) = 1;
% 
% c = zeros(nS,1);
% c(6) = g;
% 
% R = wR*eye(nC);
% R_inv = (1/wR)*eye(nC);
% 
% t_a = 0.001;
% t_b = 1000;
% 
% % Bisection method to find optimal time
% tic
% 
% Jp_a = DblIntQuadCostDerivative(t_a, x0, x1, wR, g);
% Jp_b = DblIntQuadCostDerivative(t_b, x0, x1, wR, g);
% 
% if Jp_a*Jp_b > 0
%     disp('select a different interval');
%     return;
% end
% 
% iter = 1;
% 
% while iter < iterMax
% 
%     t_c = (t_a + t_b)/2;
%     Jp_c = DblIntQuadCostDerivative(t_c, x0, x1, wR, g);
% 
%     if abs(Jp_c) < funcDel || (t_b-t_a)/2 < stepDel
%         tprime = t_c;
%         solnTime = toc;
%         disp('Solution found')
%         disp(['Jprime = 0 at t = ', num2str(tprime)]);
%         break;
%     end 
% 
%     iter = iter + 1;
% 
%     if Jp_a*Jp_c < 0
%         t_b = t_c;
%         Jp_b = Jp_c;
%     else
%         t_a = t_c;
%         Jp_a = Jp_c;
%     end
% end
% 
% % Calculate optimal trajectory and control
% 
% x_0 = DblIntQuadOptState(0, tprime, x0, x1, wR, g);
% x_1_v2 = DblIntQuadOptState(tprime, tprime, x0, x1, wR, g);
%  
% ABR = zeros(12,12);
% ABR(1,4) = 1;
% ABR(2,5) = 1;
% ABR(3,6) = 1;
% ABR(4,10) = 1/wR;
% ABR(5,11) = 1/wR;
% ABR(6,12) = 1/wR;
% ABR(10,7) = -1;
% ABR(11,8) = -1;
% ABR(12,9) = -1;
% 
% Abar = [A B*R_inv*B'; zeros(6,6) -A'];
% 
% t = pi();
% c1 = t/wR;
% c2 = t^2/(2*wR);
% c3 = t^3/(6*wR);
% expAbar = eye(12,12);
% expAbar(1,4) = t;
% expAbar(2,5) = t;
% expAbar(3,6) = t;
% expAbar(1,7) = -c3;
% expAbar(1,10) = c2;
% expAbar(2,8) = -c3;
% expAbar(2,11) = c2;
% expAbar(3,9) = -c3;
% expAbar(3,12) = c2;
% expAbar(4,7) = -c2;
% expAbar(4,10) = c1;
% expAbar(5,8) = -c2;
% expAbar(5,11) = c1;
% expAbar(6,9) = -c2;
% expAbar(6,12) = c1;
% expAbar(10,7) = -t;
% expAbar(11,8) = -t;
% expAbar(12,9) = -t;
