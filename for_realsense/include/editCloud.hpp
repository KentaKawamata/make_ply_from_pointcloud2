#ifndef EDIT_CLOUD_H
#define EDIT_CLOUD_H

#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/LU>

class EditCloud
{
public:
    EditCloud();
    ~EditCloud();
    void filter();
    void filter_for_regi();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr over_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr under_cloud;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

private:

    float voxel_size;
    bool make_regi_data;

    int meanK;
    float mulThresh;

    float x_min; 
    float x_max; 
    float y_min; 
    float y_max; 
    float z_min; 
    float z_max; 
    float under_z_min; 
    float under_z_max; 

    void get_param();
    void detect_plane();
    void voxel_grid();
    void smooth();
    void rangeFilter_over();
    void rangeFilter_under();
    void outline();
};

#endif //EDIT_CLOUD_H