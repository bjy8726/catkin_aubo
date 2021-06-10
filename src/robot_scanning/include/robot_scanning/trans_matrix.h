#ifndef TRANS_MATRIX_H_
#define TRANS_MATRIX_H_

#include <Eigen/Dense>
#include <fstream>
#include <iostream>

using namespace std;

class TransMatrix
{
public:
    TransMatrix();
    virtual ~TransMatrix();
    Eigen::Matrix4d getTransMatrix(Eigen::Vector3d &T,Eigen::Quaterniond &Q);
    Eigen::Matrix4d getTransMatrix(std::vector<double> &posture);
    void writeTrackPoints2File(Eigen::Matrix4d &TrackPoints_obj,fstream &outFile_obj);
    void newTrackPointsFileByTransMatrix(std::string &text_init,std::string &text_target,Eigen::Matrix4d &T_init2target);
    void printfMatrix4d(Eigen::Matrix4d &T,const std::string &matrixName);


private:
    /* data */
};

#endif
