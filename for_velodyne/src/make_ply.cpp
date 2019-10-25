#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include "./../include/make_ply.hpp"

namespace VelodynePoints
{
    ROStoPCL::ROStoPCL(ros::NodeHandle &nh) : 
        cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>()),
        cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        R (Eigen::Matrix4d::Identity()),
        cloud_frame ("/velodyne_points"),
        count (0),
        cloud_sub(nh.subscribe(cloud_frame, 1, &ROStoPCL::getPointCloud_callback, this))
    {
        get_params();
        rotevec = new GetRotationVector();
        edit = new EditCloud();
    }

    ROStoPCL::~ROStoPCL()
    {
        delete rotevec; 
        delete edit;
    }

    void ROStoPCL::get_params()
    {
        ros::param::get("/use_velo/lis_header_id", lis_header_id);
        ros::param::get("/use_velo/lis_child_id", lis_child_id);
    
        ros::param::get("/use_velo/file_path", file_path);
        ros::param::get("/use_velo/file_name", file_name);
    }


    void ROStoPCL::getPointCloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msgs)
    {
        pcl::fromROSMsg(*cloud_msgs, *cloud_pcl);
        ROS_INFO("GET POINTCLOUD");
    }


    void ROStoPCL::make_ply_data(const std::string &path)
    {
        pcl::transformPointCloud(*cloud_pcl, *cloud_pcl, R); 
        edit->filter(cloud_pcl);
        *cloud += *cloud_pcl;
        
        if(count%10 == 0)
        {
            std::string savename = path 
                                 + file_name 
                                 + std::to_string(count) 
                                 + ".ply"; 
    
            pcl::io::savePLYFileASCII(savename, *cloud);
            ROS_INFO_STREAM("Save PointCloud : " + savename);
        }
    }


    void ROStoPCL::make_datas(geometry_msgs::TransformStamped &ts)
    {
        quaternion_to_vector(ts);
        make_ply_data(file_path);
    }

    void ROStoPCL::quaternion_to_euler(geometry_msgs::TransformStamped &ts)
    {
        translation.setValue(ts.transform.translation.x,
                             ts.transform.translation.y,
                             ts.transform.translation.z);

        rotevec->tpclZ = translation.getZ();
        rotevec->tpclY = translation.getY();
        rotevec->tpclX = translation.getX();

        rotation.setValue(ts.transform.rotation.x,
                          ts.transform.rotation.y,
                          ts.transform.rotation.z,
                          ts.transform.rotation.w);
        tf2::Matrix3x3 m(rotation);

        m.getRPY(rotevec->roll, 
                 rotevec->pitch, 
                 rotevec->yaw);
    }


    void ROStoPCL::quaternion_to_vector(geometry_msgs::TransformStamped &ts)
    {
        quaternion_to_euler(ts);
        rotevec->transformPointCloud();
        R = rotevec->R;
    }


    void ROStoPCL::run()
    {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::Rate rate(1.0);

            while(ros::ok())
            {
            geometry_msgs::TransformStamped ts;
            try
            {
                ts = tfBuffer.lookupTransform(lis_header_id, lis_child_id, ros::Time(0));
            }
            catch(tf2::TransformException &ex)
            {
                ROS_ERROR("%s", ex.what());
                ros::Duration(1.0).sleep();
                ros::spinOnce();
                continue;
            }

            if(cloud_pcl->size()>0)
            {
                make_datas(ts);
                count++;
            }
            else
            {
                ROS_INFO("NO POINTCLOUD DATA");
            }

            rate.sleep();
            ros::spinOnce();
        }    
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "superimpose_velo_points");
    ros::NodeHandle nh;

    VelodynePoints::ROStoPCL *get_pcl;
    get_pcl = new VelodynePoints::ROStoPCL(nh);
    get_pcl->run();

    delete get_pcl; 
   
    return 0;
}
