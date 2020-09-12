#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.h>

using namespace std;

int main()
{

    Eigen::Quaterniond qa(0.55,0.3,0.2,0.2),qb(-0.1,0.3,-0.7,0.2);
    qa.normalize();
    qb.normalize();

    Eigen::Vector3d ta(0.7,1.1,0.2),tb(-0.1,0.4,0.8);
    Eigen::Matrix3d ra=qa.toRotationMatrix();
    Eigen::Matrix3d rb=qb.toRotationMatrix();

    Eigen::Matrix4d Ta=Eigen::Matrix4d::Identity(),Tb=Eigen::Matrix4d::Identity();
    Ta.block(0,0,3,3)=ra;
    Ta.block(0,3,3,1)=ta;

    Tb.block(0,0,3,3)=rb;
    Tb.block(0,3,3,1)=tb;

    Eigen::Matrix4d transform_matrix;
    transform_matrix=Ta.inverse()*Tb;

    cout<<"transform_matrix:"<<endl<<transform_matrix<<endl;
    return 0;
}
