#include <ros/ros.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <cmath>
#include <iostream>

#include "./../include/editCloud.hpp"

namespace VelodynePoints
{
    EditCloud::EditCloud() :
        filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>)
    {
        get_param();
    }

    EditCloud::~EditCloud()
    {
    }

    void EditCloud::get_param()
    {
        ros::param::get("/use_velo/voxel_size", voxel_size);
        ros::param::get("/use_velo/make_data_for_registration", make_regi_data);

        ros::param::get("/use_velo/meanK", meanK);
        ros::param::get("/use_velo/mulThresh", mulThresh);

        ros::param::get("/use_velo/plane_threshold", plane_threshold);

        ros::param::get("/use_velo/range_x_min", x_min);
        ros::param::get("/use_velo/range_x_max", x_max);
        ros::param::get("/use_velo/range_y_min", y_min);
        ros::param::get("/use_velo/range_y_max", y_max);
        ros::param::get("/use_velo/range_z_min", z_min);
        ros::param::get("/use_velo/range_z_max", z_max);
    }

    /*
    void EditCloud::smooth(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);//Kdtreeの作成

        pcl::PointCloud<pcl::PointNormal> mls_points;//出力する点群の格納場所を作成

        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

        mls.setComputeNormals (true);//法線の計算を行うかどうか

        // 各パラメーターの設定
        mls.setInputCloud (cloud);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.03);

        mls.process (mls_points);//再構築

        pcl::copyPointCloud(mls_points, *cloud);
    }
    */


    void EditCloud::detect_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
    
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(plane_threshold);

        int i=0;
        int nr_points = cloud->points.size();

        while(cloud->points.size() > 0.3*nr_points)
        {
            seg.setInputCloud (cloud);
            seg.segment (*inliers, *coefficients);
    
            if(inliers->indices.size()==0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (cloud);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*cloud);
        }
    }


    void EditCloud::range_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        pcl::PassThrough<pcl::PointXYZ> passX;
        passX.setInputCloud(cloud);
        passX.setFilterFieldName("x");
        passX.setFilterLimits(x_min, x_max);
        passX.setFilterLimitsNegative(false);
        passX.filter(*cloud);

        pcl::PassThrough<pcl::PointXYZ> passY;
        passY.setInputCloud(cloud);
        passY.setFilterFieldName("y");
        passY.setFilterLimits(y_min, y_max);
        passY.setFilterLimitsNegative(false);
        passY.filter(*cloud);

        pcl::PassThrough<pcl::PointXYZ> passZ;
        passZ.setInputCloud(cloud);
        passZ.setFilterFieldName("z");
        passZ.setFilterLimits(z_min, z_max);
        passZ.setFilterLimitsNegative(false);
        passZ.filter(*cloud);
    }


    void EditCloud::outline(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(meanK);
        sor.setStddevMulThresh(mulThresh);
        sor.filter(*cloud);
    }


    void EditCloud::voxel_grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        std::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> sor (new pcl::VoxelGrid<pcl::PointXYZ>);
        sor->setInputCloud(cloud);
        sor->setLeafSize(voxel_size, voxel_size, voxel_size);
        sor->filter(*cloud);
    }

    void EditCloud::filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        detect_plane(cloud);
        range_filter(cloud);
        //voxel_grid(cloud);
        //outline(cloud);
    }
}
