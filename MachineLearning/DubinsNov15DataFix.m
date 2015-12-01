%This script corrects a scaling error that was saved into Nov 15 (and
%earlier) training data for dubins vehicle

scalefactor = 4/pi();

data = load('DubinsTrainData-229Milestone-ConstHeading-Nov15-1.txt');

data(:,5) = data(:,5)*scalefactor;

dlmwrite('DubinsTrainData-229Milestone-ConstHeading-Nov15-1-FIXED.txt',...
    data,'delimiter',';','precision', 5,  'newline', 'pc');
%%%%%%%%%%%%%%%%%%
clear
%%%%%%%%%%%%%%%%%%
scalefactor = 4/pi();

data = load('DubinsTrainData-229Milestone-UnconstHeading-Nov15-1.txt');

data(:,5) = data(:,5)*scalefactor;

dlmwrite('DubinsTrainData-229Milestone-UnconstHeading-Nov15-1-FIXED.txt',...
    data,'delimiter',';','precision', 5,  'newline', 'pc');
%%%%%%%%%%%%%%%%%%
clear
%%%%%%%%%%%%%%%%%%
scalefactor = 4/pi();

data = load('DubinsTestData-229Milestone-ConstHeading-Nov15-1.txt');

data(:,5) = data(:,5)*scalefactor;

dlmwrite('DubinsTestData-229Milestone-ConstHeading-Nov15-1-FIXED.txt',...
    data,'delimiter',';','precision', 5,  'newline', 'pc');
%%%%%%%%%%%%%%%%%%
clear
%%%%%%%%%%%%%%%%%%
scalefactor = 4/pi();

data = load('DubinsTestData-229Milestone-UnconstHeading-Nov15-1.txt');

data(:,5) = data(:,5)*scalefactor;

dlmwrite('DubinsTestData-229Milestone-UnconstHeading-Nov15-1-FIXED.txt',...
    data,'delimiter',';','precision', 5,  'newline', 'pc');