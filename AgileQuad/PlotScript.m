close all

nPlotPoints = 100;
viconLogFile = '../../AgileQuad/ViconWifiComm/Debug/curViconLog.txt';

% Try plotting smoothed trajectory
try

    % extract information
    smoother = mpinfo.smoother;
    nCoef = smoother.nCoef;
    nSeg = smoother.nSeg;
    Tdel = smoother.Tdel;
    splineCoefs = smoother.splineCoefs;
    
    figure
    hold on

    for l = 1:nSeg
        baseInd = (l-1)*nCoef;
        tVec = linspace(0, Tdel(l,1), nPlotPoints);
        xPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,1)), tVec);
        yPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,2)), tVec);
        zPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,3)), tVec);
        plot3(xPos, yPos, zPos);
    end 
    
    % try plotting vicon
    try
        viconData = importdata(viconLogFile);
        plot3(viconData.data(:,3),viconData.data(:,4),viconData.data(:,5));
    catch
        disp('Invalid vicon data');
    end
    xlabel('x [m]')
    ylabel('y [m]')
    zlabel('z [m]')
    view(151,-10);
    camroll(180);
    hold off
    
    % Try plotting position
    figure
    hold on
    try
        tbase = 0;
        for l = 1:nSeg
            baseInd = (l-1)*nCoef;
            dtVec = linspace(0, Tdel(l,1), nPlotPoints);
            cumtVec = dtVec + tbase;
            xPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,1)), dtVec);
            yPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,2)), dtVec);
            zPos = polyval(flipud(splineCoefs(1+baseInd:nCoef+baseInd,3)), dtVec);
            plot(cumtVec, xPos, '-b');
            plot(cumtVec, yPos, '-g');
            plot(cumtVec, zPos, '-r');
            tbase = cumtVec(end);
        end 
    catch
        disp('failed to plot smoothed trajectory')
    end
    xlabel('time [s]')
    ylabel('pos [m]')
    hold off
    
    
    % Try plotting velocity
    figure
    hold on
    try
        tbase = 0;
        for l = 1:nSeg
            baseInd = (l-1)*nCoef;
            dtVec = linspace(0, Tdel(l,1), nPlotPoints);
            cumtVec = dtVec + tbase;
            xVel = polyval(polyder(flipud(splineCoefs(1+baseInd:nCoef+baseInd,1))), dtVec);
            yVel = polyval(polyder(flipud(splineCoefs(1+baseInd:nCoef+baseInd,2))), dtVec);
            zVel = polyval(polyder(flipud(splineCoefs(1+baseInd:nCoef+baseInd,3))), dtVec);
            plot(cumtVec, xVel, '-b');
            plot(cumtVec, yVel, '-g');
            plot(cumtVec, zVel, '-r');
            tbase = cumtVec(end);
        end 
    catch
        disp('failed to plot smoothed trajectory')
    end
    xlabel('time [s]')
    ylabel('vel [m/s]')
    hold off
    
catch
    
    figure
    hold on

    for i = 1:length(p_coefs)

        for l = 1:nSeg
            baseInd = (l-1)*nCoef;
            tVec = linspace(0, Tdel{i}(l,1), nPlotPoints);
            xPos = polyval(flipud(p_coefs{i}(1+baseInd:nCoef+baseInd,1)), tVec);
            yPos = polyval(flipud(p_coefs{i}(1+baseInd:nCoef+baseInd,2)), tVec);
            zPos = polyval(flipud(p_coefs{i}(1+baseInd:nCoef+baseInd,3)), tVec);
            plot3(xPos, yPos, zPos);
            hold on
        end

    end

    figure
    hold on

    for l = 1:nSeg
        baseInd = (l-1)*nCoef;
        tVec = linspace(0, Tdel{1}(l,1), nPlotPoints);
        xPos = polyval(flipud(p_coefs{1}(1+baseInd:nCoef+baseInd,1)), tVec);
        yPos = polyval(flipud(p_coefs{1}(1+baseInd:nCoef+baseInd,2)), tVec);
        zPos = polyval(flipud(p_coefs{1}(1+baseInd:nCoef+baseInd,3)), tVec);
        plot3(xPos, yPos, zPos, 'b*');
        hold on
    end

    for l = 1:nSeg
        baseInd = (l-1)*nCoef;
        tVec = linspace(0, Tdel{end}(l,1), nPlotPoints);
        xPos = polyval(flipud(p_coefs{end}(1+baseInd:nCoef+baseInd,1)), tVec);
        yPos = polyval(flipud(p_coefs{end}(1+baseInd:nCoef+baseInd,2)), tVec);
        zPos = polyval(flipud(p_coefs{end}(1+baseInd:nCoef+baseInd,3)), tVec);
        plot3(xPos, yPos, zPos, 'ro');
        hold on
    end
    
end

