% Testing calculations of optimal control of linearized quadrotor model.
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

probinfo.boundary_values.x0 = 0;
probinfo.boundary_values.y0 = 0;
probinfo.boundary_values.z0 = -1;
probinfo.boundary_values.vx0 = 0;
probinfo.boundary_values.vy0 = 0;
probinfo.boundary_values.vz0 = 0;
probinfo.boundary_values.phi0 = 0;
probinfo.boundary_values.theta0 = 0;
probinfo.boundary_values.dphi0 = 0;
probinfo.boundary_values.dtheta0 = 0;
probinfo.boundary_values.xf = 0;
probinfo.boundary_values.yf = 0;
probinfo.boundary_values.zf = -2;
probinfo.boundary_values.vxf = 0;
probinfo.boundary_values.vyf = 0;
probinfo.boundary_values.vzf = -5;
probinfo.boundary_values.phif = 0;
probinfo.boundary_values.thetaf = 0;
probinfo.boundary_values.dphif = 0;
probinfo.boundary_values.dthetaf = 0;

probinfo = LinQuadNumericData(probinfo);
tic
probinfo = LinQuadOptimizer(probinfo);
toc
