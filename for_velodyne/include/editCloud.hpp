#ifndef EDIT_CLOUD_H
#define EDIT_CLOUD_H

#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/LU>

namespace VelodynePoints
{
    class EditCloud
    {
    public:
        EditCloud();
        ~EditCloud();
        void filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

    private:

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;
        float voxel_size;
        bool make_regi_data;

        int meanK;
        float mulThresh;

        float plane_threshold;

        float x_min; 
        float x_max; 
        float y_min; 
        float y_max; 
        float z_min; 
        float z_max; 

        void get_param();
        //void smooth(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
        void detect_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
        void range_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
        void outline(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
        void voxel_grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
};
}

#endif //EDIT_CLOUD_H
