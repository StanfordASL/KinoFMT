#include "text2FlatMatrix.h"
#include <iostream>

using namespace std;
FlatMatrix text2FlatMatrix(string fileName){
    
    string line;  // string to contain current line from file
    ifstream fileObj (fileName.c_str());  // object that contains file (note need to pass a constant char* not string)
    
    if(fileObj.is_open()){  // check that file is open
        
        // Get the number of dimensions for the FlatMatrix from the first line
        getline(fileObj, line);  //get first line
        unsigned pos = line.find_last_of(":");  // Find location of ":"
        string curStr = line.substr(pos+1);  // create substring of everything after ":"
        int numDims = atoi(curStr.c_str());  // convert substring to integer

        
        // Get size of each dimension of the FlatMatrix and create vector
        int numVals = 1;  // total number of elements in matrix
        vector <int> dims;
        for(int iter = 0; iter < numDims; iter++){
            getline(fileObj, line);  // get new line
            pos = line.find_last_of(":");  // Find location of ":"
            curStr = line.substr(pos+1);  // create substring of everything after ":
            dims.push_back(atoi(curStr.c_str())); // convert string to int and store in vector
            numVals = numVals*dims.at(iter);
        }

        
        // Skip next line since it is just the word 'values'
        getline(fileObj, line);
        
        // Get the value of all the elements of the matrix
        vector <double> vals;
        double base;  // base of number in scientific notation
        double ex; // exponent of number in scientific notation
        for(int iter = 0; iter < numVals; iter++){
            getline(fileObj, line);  // get new line
            pos = line.find_first_of("e");  //Position of exponent
            curStr = line.substr(0,pos);  //string of everything to left of e
            base = atof(curStr.c_str());  // convert string to double
            curStr = line.substr(pos+2); // substring after e and + or -
            ex = atof(curStr.c_str());
            vals.push_back(base*pow(10,ex));  ;
        }

        return FlatMatrix(dims, vals);
        
    }else{
        cout << "Invalid File Path Passed to text2FlatMatrix.cpp \n";
        return FlatMatrix();
    } 
    
}