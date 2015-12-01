#include "compilePrecomputedData.h"

using namespace std;

vector <vector <double>*> getSampledStateID(string stateIDFile){
    
    FlatMatrix stateMat = text2FlatMatrix(stateIDFile);
    
    vector <vector <double>*> sampledStates;
    for(int iter1 = 0; iter1 < stateMat.getDimensions().at(0); iter1++){   
        vector <double>* curState = new vector <double>();  // instantiate new pointer to a vector of doubles
        for(int iter2 = 0; iter2 < stateMat.getDimensions().at(1); iter2++){
            vector <int> index;
            index.push_back(iter1);
            index.push_back(iter2);
            curState->push_back(stateMat.value_at(index));
        }
        sampledStates.push_back(curState);
    }
    return sampledStates;
}


vector <vector <BVP_info> > getPrecomputedData(string trajectoryFile, string costFile){
    FlatMatrix costMat = text2FlatMatrix(costFile);
    FlatMatrix trajMat = text2FlatMatrix(trajectoryFile);
    
    vector <vector <BVP_info> > precompData;
    for(int iter4 = 0; iter4 < trajMat.getDimensions().at(3); iter4++){
        vector <BVP_info> allCurStateBVPs;
        for(int iter1 = 0; iter1 < trajMat.getDimensions().at(0); iter1++){
            BVP_info curBVP;
            for(int iter3 = 0; iter3 < trajMat.getDimensions().at(2); iter3++){
                vector <double>* curInterpState = new vector <double>();
               for(int iter2 = 0; iter2 < trajMat.getDimensions().at(1); iter2++){
                   vector <int> index;
                   index.push_back(iter1);
                   index.push_back(iter2);
                   index.push_back(iter3);
                   index.push_back(iter4);
                   curInterpState->push_back(trajMat.value_at(index));
               }
                curBVP.trajectory.push_back(curInterpState);
            }
            vector <int> index;
            index.push_back(iter4);
            index.push_back(iter1);
            curBVP.cost = costMat.value_at(index);
            allCurStateBVPs.push_back(curBVP);
        }
        precompData.push_back(allCurStateBVPs);
    }
    return precompData;
}
