#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include "./getRotationVector.hpp"

namespace VelodynePoints
{
    GetRotationVector::GetRotationVector() : 
        R3d (Eigen::Matrix3d::Identity()),
        R (Eigen::Matrix4d::Identity()),
        tpclX(0.0),
        tpclY(0.0),
        tpclZ(0.0),
        pitch(0.0),
        roll(0.0),
        yaw(0.0)
    {
        get_param();
    }

    GetRotationVector::~GetRotationVector()
    {
    }

    void GetRotationVector::get_param()
    {
        ros::param::get("/use_velo/x", x);
        ros::param::get("/use_velo/y", y);
        ros::param::get("/use_velo/z", z);
    }


    void GetRotationVector::setOverRotate4()
    {
        R(0,0) = R3d(0,0);
        R(0,1) = R3d(0,1);
        R(0,2) = R3d(0,2);
 
        R(1,0) = R3d(1,0);
        R(1,1) = R3d(1,1);
        R(1,2) = R3d(1,2);
 
        R(2,0) = R3d(2,0);
        R(2,1) = R3d(2,1);
        R(2,2) = R3d(2,2);

        R(0,3) = tpclX + x; 
        R(1,3) = tpclY + y; 
        R(2,3) = tpclZ + z;
        R(3,3) = 1.0;
    }


    void GetRotationVector::eulerToRote()
    {
        Eigen::Matrix3d Rpi;
        Eigen::Matrix3d Rya;
        Eigen::Matrix3d Rro;

        Rpi << 1, 0, 0,  
               0, cos(pitch), -sin(pitch),  
               0, sin(pitch), cos(pitch); 

        Rya << cos(yaw), 0, sin(yaw), 
               0, 1, 0, 
               -sin(yaw), 0, cos(yaw);

        Rro << cos(roll), -sin(roll), 0, 
               sin(roll), cos(roll), 0, 
               0, 0, 1; 

        R3d = Rya * Rro * Rpi;
    }

    void GetRotationVector::transformPointCloud()
    {
        eulerToRote();
        setOverRotate4();
    }
}
