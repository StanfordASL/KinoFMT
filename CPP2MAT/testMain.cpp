/* 
 * File:   testMain.cpp
 * Author: Ross Allen
 *
 * Created on January 17, 2014, 12:08 PM
 */

#include <cstdlib>
#include <vector>
#include <string>
#include <iostream>

#include "text2FlatMatrix.h"
#include "FlatMatrix.h"
#include "compilePrecomputedData.h"

using namespace std;

/*
 * 
 */
//struct BVP_info{
//    vector <vector <double>*> trajectory;
//    double cost;
//};

int main(int argc, char** argv) {
    
//    int darr[] = {2, 3, 4};  // create array that will be used to initialize vector
//    vector<int> dims(darr, darr + sizeof(darr) / sizeof(darr[0]));        // create vector and initialize from array
//    double varr[] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, \
//            150, 160, 170, 180, 190, 200, 210, 220, 230, 240}; 
//    vector<double> vals(varr, varr + sizeof(varr) / sizeof(varr[0]));
//    
//    FlatMatrix A = FlatMatrix(dims, vals);

    string StateIDFileName = "../../../../planningdata/GenKinoFMT/DubinsPrecomputeV2_5x10_Jan20-2014_StateID.txt";
    FlatMatrix stateMat = text2FlatMatrix(StateIDFileName);
    string CostFileName = "../../../../planningdata/GenKinoFMT/DubinsPrecomputeV2_5x10_Jan20-2014_Cost.txt";
    FlatMatrix costMat = text2FlatMatrix(CostFileName);
    string TrajectoryFileName = "../../../../planningdata/GenKinoFMT/DubinsPrecomputeV2_5x10_Jan20-2014_Trajectory.txt";
    FlatMatrix trajMat = text2FlatMatrix(TrajectoryFileName);
    
    vector < vector <double>* > sampledStates = getSampledStateID("../../../../planningdata/GenKinoFMT/DubinsPrecomputeV2_5x10_Jan20-2014_StateID.txt"); 
    vector <vector <BVP_info> > trajData = getPrecomputedData("../../../../planningdata/GenKinoFMT/DubinsPrecomputeV2_5x10_Jan20-2014_Trajectory.txt", "../../../../planningdata/GenKinoFMT/DubinsPrecomputeV2_5x10_Jan20-2014_Cost.txt");
    
    cout << sampledStates.at(3)->at(1) << "\n";
    cout <<trajData.at(2).at(0).trajectory.at(6)->at(2) << "\n";
//    vector <int> testind;
//    testind.push_back(2);
//    testind.push_back(2);
//    testind.push_back(4);
//    testind.push_back(3);
//    cout << trajMat.value_at(testind);
    
//    vector <vector <double>*> sampledStates;
//    for(int iter1 = 0; iter1 < stateMat.getDimensions().at(0); iter1++){   
//        vector <double>* curState = new vector <double>();  // instantiate new pointer to a vector of doubles
//        for(int iter2 = 0; iter2 < stateMat.getDimensions().at(1); iter2++){
//            vector <int> index;
//            index.push_back(iter1);
//            index.push_back(iter2);
//            curState->push_back(stateMat.value_at(index));
//        }
//        sampledStates.push_back(curState);
//    }
//    
//    vector <vector <BVP_info> > precompData;
//    for(int iter4 = 0; iter4 < trajMat.getDimensions().at(3); iter4++){
//        vector <BVP_info> allCurStateBVPs;
//        for(int iter1 = 0; iter1 < trajMat.getDimensions().at(0); iter1++){
//            BVP_info curBVP;
//            for(int iter3 = 0; iter3 < trajMat.getDimensions().at(2); iter3++){
//                vector <double>* curInterpState = new vector <double>();
//               for(int iter2 = 0; iter2 < trajMat.getDimensions().at(1); iter2++){
//                   vector <int> index;
//                   index.push_back(iter1);
//                   index.push_back(iter2);
//                   index.push_back(iter3);
//                   index.push_back(iter4);
//                   curInterpState->push_back(trajMat.value_at(index));
//               }
//                curBVP.trajectory.push_back(curInterpState);
//            }
//            vector <int> index;
//            index.push_back(iter4);
//            index.push_back(iter1);
//            curBVP.cost = costMat.value_at(index);
//            allCurStateBVPs.push_back(curBVP);
//        }
//        precompData.push_back(allCurStateBVPs);
//    }
//
//    cout << sampledStates.at(3)->at(1) << "\n";
//    cout << precompData.at(2).at(0).trajectory.at(6)->at(2) << "\n";
    //    copy(fMat.getDimensions().begin(), fMat.getDimensions().end(), ostream_iterator<int>(cout, " "));
    
    return 0;
}

