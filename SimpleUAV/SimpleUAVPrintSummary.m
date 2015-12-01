%   DubinsPrintSummary.m: Display problem summary to the console (to be added)
%
%   Ross Allen, ASL, Stanford Univeristy
%   Base on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        12/7/2013
%
%   Inputs:         probinfo         Dubins Car Problem data      
%
%   Outputs:        err             nonzero if error occurred                

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function err = SimpleUAVPrintSummary(probinfo)

err = 0;        % placeholder 

if (probinfo.options.print_summary)
    
    disp('=====================================================');
    disp('            Simple UAV Problem ');
    disp('=====================================================');

    % print out launch site data
     disp('%%%%%%%%%%%%%%%%%%%%%%%% Initial State %%%%%%%%%%%%%%%%%%%%%%%%%%%%');
     disp(' ');
     disp(['x-position:                            ',num2str(probinfo.boundary_values.x0,'%10.6f'), ' m']);
     disp(['y-position:                            ',num2str(probinfo.boundary_values.y0,'%10.6f'), ' m']);
     disp(['z-position:                            ',num2str(probinfo.boundary_values.z0,'%10.6f'), ' m']);
     disp(['heading:                               ',num2str(probinfo.boundary_values.theta0,'%10.6f'), ' rad']);
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
    if isfield(probinfo.boundary_values, 'thetaf') 
        disp(['heading:                               ',num2str(probinfo.boundary_values.thetaf,'%10.6f'), ' rad']);
    end
    disp(' ');
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
    disp(' ');
    disp('Calculating all Constants: Complete');
    
end

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%