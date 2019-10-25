#ifndef MAKE_PLY_H
#define MAKE_PLY_H

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/LU>

#include "./../include/getRotationVector.hpp"
#include "./../include/editCloud.hpp"

namespace VelodynePoints
{
    class ROStoPCL {

    private:

        unsigned int count;

        std::string file_path;
        std::string file_name;

        std::string lis_header_id;
        std::string lis_child_id;

        std::string cloud_frame;

        tf2::Vector3 translation;
        tf2::Quaternion rotation;

        ros::Subscriber cloud_sub;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        Eigen::Matrix4d R;

        void get_params();
        void getPointCloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msgs);
        void make_ply_data(const std::string &path);
        void make_datas(geometry_msgs::TransformStamped &ts);
        void quaternion_to_euler(geometry_msgs::TransformStamped &ts); 
        void quaternion_to_vector(geometry_msgs::TransformStamped &ts);

    public:

        ROStoPCL(ros::NodeHandle &nh);
        ~ROStoPCL();
        void run();

        GetRotationVector *rotevec;
        EditCloud *edit; 

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //MAKE_PLY_H