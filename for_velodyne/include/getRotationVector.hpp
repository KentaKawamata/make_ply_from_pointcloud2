#ifndef GET_ROTATION_VECTOR_H
#define GET_ROTATION_VECTOR_H

#include <Eigen/Core>
#include <Eigen/LU>

namespace VelodynePoints
{
    class GetRotationVector
    {
    private:

        Eigen::Matrix3d R3d;

        void get_param();
        void eulerToRote();
        void setOverRotate4();

    public:

        double roll;
        double pitch;
        double yaw;

        double tpclX;
        double tpclY;
        double tpclZ;
    
        Eigen::Matrix4d R;

        GetRotationVector();
        ~GetRotationVector();
        void transformPointCloud();
    
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}


#endif //GET_ROTATION_VECTOR_H
