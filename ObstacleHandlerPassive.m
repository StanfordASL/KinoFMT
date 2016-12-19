% ObstacleHandlerPassive gets passive (pre-programmed cuboid) obstacles and
% encodes them for a message to ViconWifiComm over TCP
%
%   Ross Allen, ASL, Stanford University
%   Jan 26, 2016
%
%   Notes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mpinfo, tcpSendBuf] = ObstacleHandlerPassive(mpinfo)
    
% Get passive obstacles
mpinfo = GetCuboidObstacles(mpinfo);

% Count number of obstacles
mpinfo.obstacles.nPassiveObs = size(mpinfo.obstacles.cuboids.ulVerts,1)/2;

tcpSendBuf = '';

% Xmit passive obstacles to ViconWifiComm for reactive control
if (mpinfo.comms.xmitPassiveObs)
    vertices = mpinfo.obstacles.cuboids.ulVerts;
    
    if (mpinfo.obstacles.reduceBounds > 0 )
        % Reduce virtual obstacle bounds to improve potential field
        % performance
        del = 0.001;
        for i = 1:mpinfo.obstacles.nPassiveObs
           xLen = vertices(2*i,1) - vertices(2*i-1,1); 
           if (2*mpinfo.obstacles.reduceBounds < xLen)
               vertices(2*i-1,1) = vertices(2*i-1,1) + ...
                   mpinfo.obstacles.reduceBounds;
               vertices(2*i,1) = vertices(2*i,1) - ...
                   mpinfo.obstacles.reduceBounds;
           else
               vertices(2*i-1,1) = vertices(2*1-1,1) + ...
                   (xLen/2 - del);
               vertices(2*i,1) = vertices(2*1,1) - ...
                   (xLen/2 - del);
           end
           clear xLen
           yLen = vertices(2*i,2) - vertices(2*i-1,2); 
           if (2*mpinfo.obstacles.reduceBounds < yLen)
               vertices(2*i-1,2) = vertices(2*i-1,2) + ...
                   mpinfo.obstacles.reduceBounds;
               vertices(2*i,2) = vertices(2*i,2) - ...
                   mpinfo.obstacles.reduceBounds;
           else
               vertices(2*i-1,2) = vertices(2*1-1,2) + ...
                   (yLen/2 - del);
               vertices(2*i,2) = vertices(2*1,2) - ...
                   (yLen/2 - del);
           end
           clear yLen
           zLen = vertices(2*i,3) - vertices(2*i-1,3); 
           if (2*mpinfo.obstacles.reduceBounds < zLen)
               vertices(2*i-1,3) = vertices(2*i-1,3) + ...
                   mpinfo.obstacles.reduceBounds;
               vertices(2*i,3) = vertices(2*i,3) - ...
                   mpinfo.obstacles.reduceBounds;
           else
               vertices(2*i-1,3) = vertices(2*1-1,3) + ...
                   (zLen/2 - del);
               vertices(2*i,3) = vertices(2*1,3) - ...
                   (zLen/2 - del);
           end
           clear zLen
        end
    end
    
    % write obstacle vertices to string buffer
    tcpSendBuf = num2str(...
        reshape(vertices', 1,...
        3*2*mpinfo.obstacles.nPassiveObs), '%+11.3e');

end


end