% AgileQuadSplineSmoother is a function that generates a smooth, polynomial spline
%    trajectory from a set of waypoints and checks collisions
%
%	AUTHOR: Ross Allen
%	DATE:	July 17, 2015
%
%   INPUTS:
%
%   OUTPUTS:
%
%	NOTES:
%		See Ricther et. al. "Polynomial Trajectory Planning for Aggressive ..." to see motivation
%       - DblIntQuadSmoother wraps AgileQuadSplineSmoother in an iterative
%           loop with a collision checker
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mpinfo] = DblIntQuadConstrainedSmoother(mpinfo)

    % extract data
    ulVerts = mpinfo.obstacles.cuboids.ulVerts;
    bounds = mpinfo.environment.bounds;
    spheres = mpinfo.obstacles.spheres;
    yaw = mpinfo.smoother.yaw;
    timeScaling = mpinfo.smoother.timeScaling;
    options = mpinfo.smoother;
    nCheckNodes = options.nCheckNodes;
    nTrajNodes = mpinfo.sampling.nTrajNodes;
    
    % generate optimal path state keys and timing vector
    keyIDNums = mpinfo.optPath;
    nEdges = size(mpinfo.optPath,1)-1;
    keyCaseNums = diag(mpinfo.evalMat(keyIDNums(1:end-1), keyIDNums(2:end)));
    keyTrajNums = [ones(nEdges,1); nTrajNodes];
    keys = zeros(nEdges+1,3);
    Tdel = zeros(nEdges, 1);  % time steps between segments
    for i = 1:nEdges
        keys(i,:) = mpinfo.trajMat(keyCaseNums(i),1:3,keyTrajNums(i));
        if mpinfo.smoother.flatPath
            % flatten path to 2D as a hack because throttle model is no
            % good right now
            keys(i,3) = keys(1,3);
        end
        Tdel(i,1) = mpinfo.costMat(keyCaseNums(i),1);
        Tdel(i,1) = Tdel(i,1)*timeScaling;
    end
    keys(end, :) = mpinfo.stateMat(keyIDNums(end), 1:3);
    % record initial and final velocity for enforcment in spline
    keyVelInit = mpinfo.stateMat(keyIDNums(1),4:6);
    keyVelFinal = mpinfo.stateMat(keyIDNums(end),4:6);
    if mpinfo.smoother.flatPath
        % flatten path to 2D as a hack because throttle model is no
        % good right now
        keys(end,3) = keys(1,3);
        keyVelInit(1,3) = 0;
        keyVelFinal(1,3) = 0;
    end
    clear i

    % generate trajectory and check trajectory collisions
    [smoother, valid] = Gen_n_Check(keys, keyCaseNums, keyTrajNums, keyVelInit, keyVelFinal,...
                mpinfo.trajMat, nTrajNodes, nCheckNodes, Tdel, yaw, mpinfo.environment.nWorkDims, bounds,...
                ulVerts, spheres, mpinfo.smoother.flatPath, mpinfo.smoother.collisionSpeedCheck);
    
    if (valid)
        mpinfo.smoother = smoother;
    end
    mpinfo.smoother.options = options;
    mpinfo.smoother.valid = valid;
        
end

function [smoother, valid]  = Gen_n_Check(keys, keyCaseNums, keyTrajNums, keyVelInit, keyVelFinal,...
                trajMat, nTrajNodes, nCheckNodes, Tdel, yaw, nWorkDims, bounds, ulVerts, spheres, flatPath, colSpeedCheck)
        
%     collisionSeg = NaN;
    % generate polynomial trajectory
    smoother = DblIntQuadSplineSmoother(keys, Tdel, keyVelInit, keyVelFinal, yaw);
    valid = true;

    nSeg = smoother.nSeg;
    nCoef = smoother.nCoef;
    Tdel = smoother.Tdel;
    splineCoefs = smoother.splineCoefs;

    % check collisions
    for curSeg = 1:nSeg
        baseInd = (curSeg-1)*nCoef;
        tVec = linspace(0, Tdel(curSeg,1), nCheckNodes);
        xpos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,1)), tVec);
        ypos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,2)), tVec);
        zpos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,3)), tVec);
        for j = 1:nCheckNodes-1
            nodeA = [xpos(j) ypos(j) zpos(j)];
            nodeB = [xpos(j+1) ypos(j+1) zpos(j+1)];
%             collisionFree = checkCollision(nodeA, nodeB,...
%                 ulVerts, bounds);
            collisionFree = CollisionCheckerMEX(nodeA, nodeB, nWorkDims,...
                bounds, ulVerts', spheres');
            inCollision = ~collisionFree;
            if inCollision
                break;
            end
        end
        if inCollision % update optimal path and timing vector
            % determine bisection node in original trajectoy accounting for
            % change of segments at keyTrajNum = 1
            if keyTrajNums(curSeg+1) == 1
                bisectNode = floor((nTrajNodes+keyTrajNums(curSeg))/2);
            else
                bisectNode = floor((keyTrajNums(curSeg+1)+keyTrajNums(curSeg))/2);
            end
            if bisectNode == keyTrajNums(curSeg)
%                 disp('Smoother failed due to obstacle proximity');
                smoother = NaN;
                valid = false;
                return;
            end
            % Expand keyTrajNums by one slot at curSeg and fill with the
            % bisecting node in trajectory
            keyTrajNums = [keyTrajNums(1:curSeg);...
                            bisectNode;...
                            keyTrajNums(curSeg+1:end)];
            % Expand keyCaseNums by one slot at curSeg and fill with copy of
            % keyCaseNum in collision
            keyCaseNums = [keyCaseNums(1:curSeg);...
                            keyCaseNums(curSeg);...
                            keyCaseNums(curSeg+1:end)];
            % Retrieve key of bisecting node and approximate time vector
            keys = [keys(1:curSeg,:); zeros(1,3); keys(curSeg+1:end,:)];
            keys(curSeg+1,:) = trajMat(keyCaseNums(curSeg+1), 1:3, keyTrajNums(curSeg+1));
            if flatPath
                % flatten path to 2D as a hack because throttle model is no
                % good right now
                keys(curSeg+1,3) = keys(1,3);
        	end
            Tdel = [Tdel(1:curSeg-1); colSpeedCheck*Tdel(curSeg)/2; colSpeedCheck*Tdel(curSeg)/2; Tdel(curSeg+1:end)];
            
            % recursive call to Gen_n_Check
            [smoother, valid] = Gen_n_Check(keys, keyCaseNums, keyTrajNums, keyVelInit, keyVelFinal,...
                trajMat, nTrajNodes, nCheckNodes, Tdel, yaw, nWorkDims, bounds, ulVerts, spheres, flatPath, colSpeedCheck);

            % to break or not to break?
            break;
        end
    end
    end
