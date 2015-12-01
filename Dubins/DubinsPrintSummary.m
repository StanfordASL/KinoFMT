%   DubinsPrintSummary.m: Display problem summary to the console (to be added)
%
%   Ross Allen, ASL, Stanford Univeristy
%   Base on work by: Michael Colonno, ADL, Stanford University 
%
%   Started:        10/18/2013
%
%   Inputs:         dubprob         Dubins Car Problem data      
%
%   Outputs:        err             nonzero if error occurred                

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function err = DubinsPrintSummary(dubprob)

err = 0;        % placeholder 

if (dubprob.options.print_summary)
    
    disp('=====================================================');
    disp('               Dubins Vehicle Problem ');
    disp('=====================================================');

    % print out launch site data
     disp('%%%%%%%%%%%%%%%%%%%%%%%% Initial State %%%%%%%%%%%%%%%%%%%%%%%%%%%%');
     disp(' ');
     disp(['x-position:                            ',num2str(dubprob.boundary_values.x0,'%10.6f'), ' m']);
     disp(['y-position:                            ',num2str(dubprob.boundary_values.y0,'%10.6f'), ' m']);
     disp(['heading:                               ',num2str(dubprob.boundary_values.theta0,'%10.6f'), ' deg']);
     disp(['start time:                            ',num2str(dubprob.boundary_values.t0,'%10.6f'), ' s']);
     disp(' ');
     disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
     disp(' ');
    
    
    % print out final conditions
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%% Final State %%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
    disp(' ');
    disp(['x-position:                            ',num2str(dubprob.boundary_values.xf,'%10.6f'), ' m']);
    disp(['y-position:                            ',num2str(dubprob.boundary_values.yf,'%10.6f'), ' m']);
    if isfield(dubprob.boundary_values, 'thetaf')
       disp(['heading:                               ',num2str(dubprob.boundary_values.thetaf,'%10.6f'), ' deg']);
    end
    disp(' ');
    disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
    disp(' ');
    disp('Calculating all Constants: Complete');
    
end

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%