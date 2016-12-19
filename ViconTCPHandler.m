% HandleViconTCP establishes a client on the ViconWifiComm TCP server
% (PlannerServer.cpp) and pass quadrotor and obstacle data back and forth.
% Quadrotor and dynamic obstacles are passed to the MATLAB planner, static
% obstacles are passed back to ViconWifiComm
%
%   Ross Allen, ASL, Stanford University
%   Feb 1, 2016
%
%   Notes:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [mpinfo, continuePlanning] = ViconTCPHandler(mpinfo, tcpSendBuf)

continuePlanning = true;

% Establish connection to server
if (mpinfo.comms.connectVicon)
    
    if (~isfield(mpinfo.comms, 'tcpClient') || strcmp(mpinfo.comms.tcpClient.Status, 'closed'))
        
        % set TCP Params
        mpinfo.comms.tcpClient = tcpip('localhost', mpinfo.comms.tcpPort, 'NetworkRole', 'Client');
        set(mpinfo.comms.tcpClient, 'InputBufferSize', ...
            (3*mpinfo.systemDefs.nStateDims+4*mpinfo.obstacles.maxActiveObs)*...
            (mpinfo.comms.recvCharsPerVal+1)); % set buffer size
        set(mpinfo.comms.tcpClient, 'Timeout', mpinfo.comms.tcpTimeout);   % define a timeout
        set(mpinfo.comms.tcpClient, 'Terminator', 'LF');    % set terminator as line feed
        
        % open the TCP/IP connection to the server
        fopen(mpinfo.comms.tcpClient);
        
    end
    
    % Transmit passive obstacles. Also acts as first part of handshake to
    % tell ViconWifiComm to prepare current quadrotor state for
    % transmission
    fwrite(mpinfo.comms.tcpClient, tcpSendBuf);

    % Read in data from buffer and then flush
    tcpRecvBuf = fscanf(mpinfo.comms.tcpClient);
    flushinput(mpinfo.comms.tcpClient);
    
    % Extract numerical data
    mpinfo.comms.recvBuffer = str2num(tcpRecvBuf);
    clear tcpRecvBuf;    % delete temporary data storage
    
    % Check that data is appropriate form
    % expected [start state (nStateDims), goal region (2*nStateDims),
    % active obsacles (4*nActiveObs)];
    if (mod(size(mpinfo.comms.recvBuffer,2) - ...
            3*mpinfo.systemDefs.nStateDims, 4) ~= 0)
        
        disp('MPC termination detected');
        continuePlanning = false;
        return;
        
    end
    
    
end

end