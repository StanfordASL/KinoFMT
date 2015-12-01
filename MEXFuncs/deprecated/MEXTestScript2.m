% Test the CheckCollisionMEX.cpp func for accuracy and speed

addpath([pwd, '/../']);
addpath([pwd, '/../ObstacleSets/']);

clear ans delt_mex delt_mfile mex_2_mfile mex_check mfile_check nodeIter nodes2check obvp_iter s

% loop through trajMat testing collision checker
delt_mex = 0;
delt_mfile = 0;
for obvp_iter = 1:size(mpinfo_complete.trajMat,1)
    
    nodes2check = reshape(mpinfo_complete.trajMat(obvp_iter,1:3,:),3,mpinfo_complete.sampling.nTrajNodes)';

    % Run Mex collision checking
    tic; 
    mex_check = CheckCollisionMEX(nodes2check', mpinfo_complete.obstacles.cuboids.ulVerts', ...
        mpinfo_complete.environment.bounds);
    delt_mex = delt_mex + toc;

    % Run mfile collision checking
    tic;
    for nodeIter = 1:mpinfo_complete.sampling.nTrajNodes-1
        mfile_check = checkCollision(nodes2check(nodeIter,:), nodes2check(nodeIter+1,:),...
            mpinfo_complete.obstacles.cuboids.ulVerts, mpinfo_complete.environment.bounds);
        if ~mfile_check
            break;
        end
    end
    delt_mfile = delt_mfile + toc;

    % compare results
    if mex_check ~= mfile_check
        disp(['Error: algorithm disagreement at obvp_iter ', num2str(obvp_iter)]);
        disp(['mex_check = ', num2str(mex_check)]);
        disp(['mfile_check = ', num2str(mfile_check)]);
        break;
    end

end

% Compare timing
mex_2_mfile = delt_mex/delt_mfile
