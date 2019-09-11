#include <ros/ros.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <iostream>

#include "./../include/editCloud.hpp"

EditCloud::EditCloud() : 
    cloud (new pcl::PointCloud<pcl::PointXYZ>()),
    over_cloud (new pcl::PointCloud<pcl::PointXYZ>()),
    under_cloud (new pcl::PointCloud<pcl::PointXYZ>())
{
    get_param();
}

EditCloud::~EditCloud(){

}

void EditCloud::get_param()
{
    ros::param::get("/ply_from_pc2/voxel_size", voxel_size);

    ros::param::get("/ply_from_pc2/range_under_z_min", under_z_min);
    ros::param::get("/ply_from_pc2/range_under_z_max", under_z_max);

    ros::param::get("/ply_from_pc2/range_x_min", x_min);
    ros::param::get("/ply_from_pc2/range_x_max", x_max);
    ros::param::get("/ply_from_pc2/range_y_min", y_min);
    ros::param::get("/ply_from_pc2/range_y_max", y_max);
    ros::param::get("/ply_from_pc2/range_z_min", z_min);
    ros::param::get("/ply_from_pc2/range_z_max", z_max);
}

void EditCloud::smooth()
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

void EditCloud::rangeFilter_under()
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
    passZ.setFilterLimits(under_z_min, under_z_max);
    passZ.setFilterLimitsNegative(false);
    passZ.filter(*cloud);
}

void EditCloud::rangeFilter_over()
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

void EditCloud::outline()
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);

    /*     
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
    sor1.setInputCloud(cloud);
    sor1.setMeanK(50);
    sor1.setStddevMulThresh(0.01);
    sor1.filter(*cloud);
    */
    //pcl::PLYWriter writer;
    //writer.write<pcl::PointXYZ> ("/home/kawa/program/calc3D/data/outline.ply", *newCloud, false);
}

void EditCloud::voxel_grid()
{
    std::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> sor (new pcl::VoxelGrid<pcl::PointXYZ>);
    sor->setInputCloud(cloud);
    sor->setLeafSize(voxel_size, voxel_size, voxel_size);
    sor->filter(*cloud);
}

void EditCloud::filter()
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    //over_cloud
    pcl::copyPointCloud(*over_cloud, *cloud);
    rangeFilter_over();
    outline();
    voxel_grid();
    pcl::copyPointCloud(*cloud, *over_cloud);

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    //under_cloud
    pcl::copyPointCloud(*under_cloud, *cloud);
    rangeFilter_under();
    outline();
    voxel_grid();
    pcl::copyPointCloud(*cloud, *under_cloud);
}