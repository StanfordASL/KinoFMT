# DSSCParaMotionPlannerPythonDriver is the python script that calls the parallel
# computations on Stanford's barley cluster
#
#   Ross Allen
#
#   Sep 25, 2014
#
#   NOTES:
#       - The driver was adapted from the example given at:
#           https://web.stanford.edu/group/farmshare/cgi-bin/wiki/index.php/MatCarlo
#
################################################################################

import os

n = 0
nThreads = 450      # default maximum is 480
nSamples = 5000      # number of samples drawn from the state space
loadfile = "'DSSC_Para_5000x10000x10_Sep25-2014_phase3.mat'"

nBase, nRem = divmod(nSamples, nThreads)    # number of cases per thread and number of threads with extra case

for initState in range(1, nThreads+1):
    n += 1

    matlabstartup = '''
load(%s);
runNum = %d;
nRem = %d;
nBase = %d;

if runNum <= nRem
    nInitStates = nBase+1;
    initStateRange = (nBase+1)*(runNum-1)+1:(nBase+1)*runNum;
else
    nInitStates = nBase;
    initStateRange = nRem*(nBase+1) + ...
        [nBase*(runNum-1-nRem)+1:nBase*(runNum-nRem)];
end 

[ pN2PBVPs, pCostMat, pEvalList, pTrajMat, pControlMat ] = ...
    DSSCSolve2PBVPsParaSubset( mpinfo, initStateRange );

listOfVars = {'pN2PBVPs', 'pCostMat', 'pEvalList', 'pTrajMat', 'pControlMat'};
save('run%d/subset2PBVPData.mat', listOfVars{:});

''' % (loadfile, n, nRem, nBase, n)

    qsubscript = '''#!/bin/bash

#$ -N run%d
#$ -o run%d/job.out
#$ -e run%d/job.error
#$ -cwd
#$ -S /bin/bash
##$ -l testq=1

module load matlab
matlab -nodesktop -singleCompThread < run%d/run.m
''' % (n,n,n,n)

    os.mkdir('run%d' % n)

    runfile = open('run%d/run.m' % n, 'w')
    runfile.write(matlabstartup)
    runfile.close()

    qsubfile = open('run%d/run.submit' % n, 'w')
    qsubfile.write(qsubscript)
    qsubfile.close()

    os.system('qsub run%d/run.submit' % n) 
