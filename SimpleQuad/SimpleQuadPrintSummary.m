%   SimpleQuadPrintSummary.m: Display problem summary to the console (to be added)
%
%   Ross Allen, ASL, Stanford Univeristy
%   Base on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        Mar 30, 2015
%
%   Inputs:         probinfo         Dubins Car Problem data      
%
%   Outputs:        err             nonzero if error occurred                

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function err = SimpleQuadPrintSummary(probinfo)

err = 0;        % placeholder 

if (probinfo.options.print_summary)
    
    disp('=====================================================');
    disp('           Simple Quadrotor Problem ');
    disp('=====================================================');

    % print out launch site data
     disp('%%%%%%%%%%%%%%%%%%%%%%%% Initial State %%%%%%%%%%%%%%%%%%%%%%%%%%%%');
     disp(' ');
     disp(['x-position:                            ',num2str(probinfo.boundary_values.x0,'%10.6f'), ' m']);
     disp(['y-position:                            ',num2str(probinfo.boundary_values.y0,'%10.6f'), ' m']);
     disp(['z-position:                            ',num2str(probinfo.boundary_values.z0,'%10.6f'), ' m']);
     disp(['x-velocity:                            ',num2str(probinfo.boundary_values.vx0,'%10.6f'), ' m/s']);
     disp(['y-velocity:                            ',num2str(probinfo.boundary_values.vy0,'%10.6f'), ' m/s']);
     disp(['z-velocity:                            ',num2str(probinfo.boundary_values.vz0,'%10.6f'), ' m/s']);
     disp(['start time:                            ',num2str(probinfo.boundary_values.t0,'%10.6f'), ' s']);
     disp(' ');
     disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
     disp(' ');
    
    
    % print out final conditions
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%% Final State %%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
    disp(' ');
    disp(['x-position:                            ',num2str(probinfo.boundary_values.xf,'%10.6f'), ' m']);
    disp(['y-position:                            ',num2str(probinfo.boundary_values.yf,'%10.6f'), ' m']);
    disp(['z-position:                            ',num2str(probinfo.boundary_values.zf,'%10.6f'), ' m']);
    if isfield(probinfo.boundary_values, 'vxf') 
        disp(['x-velocity:                            ',num2str(probinfo.boundary_values.vxf,'%10.6f'), ' m/s']);
        disp(['y-velocity:                            ',num2str(probinfo.boundary_values.vyf,'%10.6f'), ' m/s']);
        disp(['z-velocity:                            ',num2str(probinfo.boundary_values.vzf,'%10.6f'), ' m/s']);
    end
    if isfield(probinfo.boundary_values, 'etaf') 
        disp(['Throttle:                            ',num2str(probinfo.boundary_values.etaf,'%10.6f'), ' #']);
    end
    disp(' ');
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
    disp(' ');
    disp('Calculating all Constants: Complete');
    
end

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
