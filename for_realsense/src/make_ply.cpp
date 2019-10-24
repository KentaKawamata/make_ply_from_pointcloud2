#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include "./../include/make_ply.hpp"

ROStoPCL::ROStoPCL(ros::NodeHandle &nh) : 
    over_cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>()),
    under_cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>()),
    R (Eigen::Matrix4d::Identity()),
    under_R (Eigen::Matrix4d::Identity()),
    lis_conf("/track/confidence/sample"),
    over_cloud_frame ("/cam_1/depth/color/points"),
    under_cloud_frame ("/cam_2/depth/color/points"),
    count (0),
    conf_sub(nh.subscribe(lis_conf, 1, &ROStoPCL::getConfidence_callback, this)),
    over_cloud_sub(nh.subscribe(over_cloud_frame, 1, &ROStoPCL::getOverPointCloud_callback, this)),
    under_cloud_sub(nh.subscribe(under_cloud_frame, 1, &ROStoPCL::getUnderPointCloud_callback, this))
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
    ros::param::get("/ply_from_pc2/lis_header_id", lis_header_id);
    ros::param::get("/ply_from_pc2/lis_child_id", lis_child_id);
    
    ros::param::get("/ply_from_pc2/first_position_x", first_position_x);
    ros::param::get("/ply_from_pc2/first_position_y", first_position_y);
    ros::param::get("/ply_from_pc2/first_position_z", first_position_z);

    ros::param::get("/ply_from_pc2/use_translation", use_translate);
    ros::param::get("/ply_from_pc2/use_rotation", use_rotation);

    ros::param::get("/ply_from_pc2/file_path", file_path);
    ros::param::get("/ply_from_pc2/regi_path", regi_path);
    ros::param::get("/ply_from_pc2/file_no_rote_path", file_no_rote_path);
    ros::param::get("/ply_from_pc2/regi_no_rote_path", regi_no_rote_path);
    ros::param::get("/ply_from_pc2/file_name", file_name);
}

void ROStoPCL::getConfidence_callback(const std_msgs::UInt32& msg)
{
    confidence = msg.data;
    //ROS_INFO(" GET CONFIDENCE");
}

void ROStoPCL::getOverPointCloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msgs)
{
    pcl::fromROSMsg(*cloud_msgs, *over_cloud_pcl);
    ROS_INFO("GET POINTCLOUD   === OVER ===");
}

void ROStoPCL::getUnderPointCloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msgs)
{
    pcl::fromROSMsg(*cloud_msgs, *under_cloud_pcl);
    ROS_INFO("GET POINTCLOUD   === UNDER ===");
}

void ROStoPCL::make_ply_data_for_regi(const std::string &path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cp_over_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cp_under_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*over_cloud_pcl, *cp_over_cloud);
    pcl::copyPointCloud(*under_cloud_pcl, *cp_under_cloud);

    pcl::transformPointCloud(*cp_over_cloud, *cp_over_cloud, R); 
    pcl::transformPointCloud(*cp_under_cloud, *cp_under_cloud, under_R); 

    pcl::copyPointCloud(*cp_over_cloud, *(edit->over_cloud));
    pcl::copyPointCloud(*cp_under_cloud, *(edit->under_cloud));
    edit->filter_for_regi();

    pcl::copyPointCloud(*(edit->over_cloud), *cp_over_cloud);
    pcl::copyPointCloud(*(edit->under_cloud), *cp_under_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud += *cp_over_cloud;
    *cloud += *cp_under_cloud;

    std::string savename = path 
                         + file_name 
                         + std::to_string(count) 
                         + ".ply"; 

    //ROS_INFO("Save PointCloud for Registration");
    pcl::io::savePLYFileASCII(savename, *cloud);
}

void ROStoPCL::make_ply_data(const std::string &path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cp_over_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cp_under_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::copyPointCloud(*over_cloud_pcl, *cp_over_cloud);
    pcl::copyPointCloud(*under_cloud_pcl, *cp_under_cloud);

    pcl::transformPointCloud(*cp_over_cloud, *cp_over_cloud, R); 
    pcl::transformPointCloud(*cp_under_cloud, *cp_under_cloud, under_R); 

    pcl::copyPointCloud(*cp_over_cloud, *(edit->over_cloud));
    pcl::copyPointCloud(*cp_under_cloud, *(edit->under_cloud));
    edit->filter();

    pcl::copyPointCloud(*(edit->over_cloud), *cp_over_cloud);
    pcl::copyPointCloud(*(edit->under_cloud), *cp_under_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud += *cp_over_cloud;
    *cloud += *cp_under_cloud;

    std::string savename = path 
                         + file_name 
                         + std::to_string(count) 
                         + ".ply"; 

    
    //ROS_INFO("Save PointCloud for Calcrate");
    pcl::io::savePLYFileASCII(savename, *cloud);
}


void ROStoPCL::make_datas(geometry_msgs::TransformStamped &ts)
{
    // enable T265 rotation ////////////////////
    use_rotation = true;
    quaternion_to_vector(ts);

    make_ply_data(file_path);
    make_ply_data_for_regi(regi_path);
    ROS_INFO("Save PointCloud enable T265 data");
    ////////////////////////////////////////////

    // disable T265 rotation ///////////////////
    use_rotation = false;
    quaternion_to_vector(ts);

    make_ply_data(file_no_rote_path);
    make_ply_data_for_regi(regi_no_rote_path);
    ROS_INFO("Save PointCloud disable T265 data");
    ////////////////////////////////////////////
}

void ROStoPCL::quaternion_to_euler(geometry_msgs::TransformStamped &ts)
{
    translation.setValue(ts.transform.translation.x,
                         ts.transform.translation.y,
                         ts.transform.translation.z);

    /***********************************************************
     * < Must not toutch tpclZ, tpclY and tpclX !!!!!      >
     * < tpclZ, tpclY and tpclX are completely correct !!! >
     *         \  ^__^
     *          \ (oo)\______
     *           (__)\    )\/\
     *              ||----w |
     *              ||    ||
     ***********************************************************/
    //////////////////////////////////
    if(use_translate)
    {
        rotevec->tpclZ =  translation.getX();
        rotevec->tpclY = -translation.getZ();
        rotevec->tpclX = -translation.getY();
    }
    else
    {
        rotevec->tpclX = 0.0;
        rotevec->tpclY = 0.0;
        rotevec->tpclZ = 0.0;
    }
    //////////////////////////////////

    rotation.setValue(ts.transform.rotation.x,
                      ts.transform.rotation.y,
                      ts.transform.rotation.z,
                      ts.transform.rotation.w);

    tf2::Matrix3x3 m(rotation);

    //rotevec->get_movement(translation, m);

    m.getRPY(RrosX, RrosY, RrosZ);

    /*******************************************************************
     * Must not toutch roll, pitch and yaw !!!!!
     * Roll, pitch and yaw are completely correct !!!
     * 
     *    　　　　　　　_,ｨ￣￣￣￣ 7ｰ ､
     *　　 　 _,. -'" .| ' ''　　'　　''|l　　｀ ｰｫ､_
     *　　　/ , 　,,　|　'　'　　''　'|l　'　　'' '|;;;;;;l 　＿＿
     *　　 /　　　　| ::　　　　　 |l 　 　 　.|;;;;;;|￣r--ｧ￣､＼__
     *　　/___ , , , ,| ::　　　　　 |　　,　　,,|;;;;;{ --'-=（=　乂 rﾄ}
     *　 ﾑ==ｧ--､ｰ､- - - - ァ=======ｲ{ZZZZZZｲ三ｴﾆｲ==-
     *　　 {Yヽ0ﾉ} } ＼!!!＿/;;;;;{：{0} }};;;;;ｲ}￣
     *　　　 | | | |-t==t-''￣￣~ | | | l|i:::i:l|
     *　　　.|_U_l|　|:i::i:l| 　 　 　　ﾞ|_U_l|::::::l|
     *　 　｜l　ｌ|　 | ::::l|　 　 　 　 | l　l|::::::l|
     *　 　 |r-､l| 　 Y=Y}　 　 　　 Y^ 米o;}}
     *　 　 ﾄ-.イ 　　ﾄ-ィ| 　 　 　 ｀ﾄ-ィ|Y;/
     *　 　,|　 ｌ| 　 　 |　 l|　　 　 　 ,|　i, |/
     *　　(0)(0) 　　　!n人9､　　／ｨ(0)(0)}
     *　/ri￣|^}　　　{ T￣|^}／Ｘ:;; {T==ｲｺ
     *　|二=--{､　　|三三三}＼/三r-----{
     *　｀ーー‐'　　　￣￣￣　　｀~└─―┘
     *****************************************************************/
    ////////////////////////////////////////////////////////////
    if(use_rotation)
    {
        rotevec->roll  = RrosX;
        rotevec->pitch = -RrosY;
        rotevec->yaw   = -RrosZ;
        rotevec->under_pitch = -RrosY - (40.0*M_PI/180);
    }
    else
    {
        rotevec->roll  = 0.0;
        rotevec->pitch = -RrosY;
        rotevec->yaw   = 0.0;
        rotevec->under_pitch = -RrosY - (40.0*M_PI/180);
    }
    ////////////////////////////////////////////////////////////
}

void ROStoPCL::quaternion_to_vector(geometry_msgs::TransformStamped &ts)
{
    quaternion_to_euler(ts);
    rotevec->transformPointCloud();

    R = rotevec->R;
    under_R = rotevec->under_R;
}

void ROStoPCL::getCharacter()
{
    ROS_INFO("===== PLEASE INPUT 's' or 'q' KEY =====");

    while(true)
    {
        std::cin >> character;
        
        if(character == 's')
        {
            break;
        }
        else if(character == 'q')
        {
            ROS_INFO("===== THIS NODE CLOSE =====");
            exit(0);
        }
        else
        {
            ROS_ERROR("PLEASE INPUT KEY 's' or 'q' ONLY");
        } 
    }
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
            getCharacter();
            ros::Duration(1.0).sleep();
            ros::spinOnce();
            continue;
        }

        over_pc_num = over_cloud_pcl->size();
        under_pc_num = under_cloud_pcl->size();

        if((over_pc_num>0 && under_pc_num>0) && confidence>=2)
        {
            make_datas(ts);
            count++;
        }
        else
        {
            ROS_INFO("NO POINTCLOUD DATA");
        }

        getCharacter();
        
        rate.sleep();
        ros::spinOnce();
    }    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ply_from_pc2");
    ros::NodeHandle nh;

    ROStoPCL *get_pcl;
    get_pcl = new ROStoPCL(nh);
    get_pcl->run();

    delete get_pcl; 
   
    return 0;
}
