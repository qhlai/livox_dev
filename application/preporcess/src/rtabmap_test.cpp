#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <std_srvs/Empty.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <queue>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud2.h>
// #include <occupancy_map_msgs/OccupancyMap.h>
#include <visualization_msgs/MarkerArray.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#define USE_ROS_SUB true;
using POINTCLOUD_TYPE = pcl::PointXYZRGB;
using POINTCLOUD = pcl::PointCloud<POINTCLOUD_TYPE>;
sensor_msgs::PointCloud2 cloud_map;
sensor_msgs::PointCloud2 cloud_voxel;
std::queue<POINTCLOUD> cloud_voxel_sum_queue;
POINTCLOUD cloud_voxel_sum;
// pcl::PointCloud<POINTCLOUD_TYPE> cloud_voxel_sum;
sensor_msgs::PointCloud2 cloud_map_voxel_sum;
Eigen::Isometry3d eigen_transform;

bool if_trans=false;

void tf_trans(){
    // tf2_ros::Buffer tfBuffer;
    
    tf::TransformListener listener;
    double extrapolation_limit = 3.0; // 设置允许的最大误差值
    listener.setExtrapolationLimit(ros::Duration(extrapolation_limit));
    std::string target_frame = "map";  // 目标坐标系
    std::string source_frame = "camera_color_optical_frame";  // 源坐标系
    listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));
    tf::StampedTransform transform;
    try {
        listener.lookupTransform(target_frame, source_frame, ros::Time(0), transform);
        // tf::transformTFToEigen(transform, eigen_transform);
        // 获取平移向量 
        tf::Vector3 translation = transform.getOrigin();
        eigen_transform.translation() << translation.x(), translation.y(), translation.z();

        // 获取旋转四元数
        tf::Quaternion rotation = transform.getRotation();
        
        // Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z());
        auto rot = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z()).toRotationMatrix();
        // eigen_transform.rotate(rot);
        eigen_transform.linear() =rot;
        std::cout <<"rotation: "<< rotation.w()<< rotation.x()<<rotation.y()<<rotation.z()<< std::endl
                <<rot.matrix()<< std::endl;

        std::cout << std::endl << eigen_transform.matrix() << std::endl;
        if_trans = true;
    } catch (tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        // 处理异常情况
        if_trans = false;
    }   
}

void show_octmap(){

    pcl::octree::OctreePointCloud<POINTCLOUD_TYPE> octree(0.01); // 设置
    // octree.setInputCloud(&cloud_voxel_sum);
    // octree.addPointsFromInputCloud();

    // 执行八叉树构建
    // octree.buildOctree();

}

void cloud_map_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_map = *msg;
    //std::cout <<"cloud_map_cb: "<< cloud_map.header.frame_id << std::endl;//map
    ROS_INFO("Received point cloud with width: %d, height: %d", msg->width, msg->height);
}

void voxel_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_voxel = *msg;
    //std::cout <<"voxel_cb: "<< cloud_voxel.header.frame_id << std::endl;//camera_color_optical_frame
    POINTCLOUD cloud;
    pcl::fromROSMsg(cloud_voxel, cloud);
    // POINTCLOUD::Ptr transformedCloud(new POINTCLOUD);
    POINTCLOUD transformedCloud;
    // Eigen::Affine3d eigen_transform_Affine3d;
    // tf::transformEigenToTF(eigen_transform, eigen_transform);
    if(if_trans){
        pcl::transformPointCloud(cloud, transformedCloud, eigen_transform.matrix());
        // Eigen::Isometry3d tran =Eigen::Isometry3d::Identity();
        // pcl::transformPointCloud(cloud, transformedCloud, tran.matrix());
    }
// transformedCloud=cloud;
    cloud_voxel_sum_queue.push(transformedCloud);
    if(cloud_voxel_sum_queue.size() > 50) {
        cloud_voxel_sum_queue.pop();
    }
    ROS_INFO("Received point cloud with width: %d, height: %d", msg->width, msg->height);
}

bool gen_sum_pc(){
    if(cloud_voxel_sum_queue.size() == 0) {
        ROS_INFO("cloud_voxel_sum is empty");
        return false;
    }
    // ROS_INFO("save_cb");
    pcl::fromROSMsg(cloud_map, cloud_voxel_sum);
    // cloud += cloud_map;
    for(uint32_t i = 0; i < cloud_voxel_sum_queue.size(); i++) {
        cloud_voxel_sum += cloud_voxel_sum_queue.front();
        // cloud_voxel_sum.pop();
    }


#if 1
static uint16_t voxel_cout = 0;
if(voxel_cout >= 30)
{
    voxel_cout=0;
#if 1
{
    pcl::PointCloud<POINTCLOUD_TYPE>::Ptr cloud_ptr(new pcl::PointCloud<POINTCLOUD_TYPE>(cloud_voxel_sum));
    pcl::PointCloud<POINTCLOUD_TYPE>::Ptr cloud_filtered(new pcl::PointCloud<POINTCLOUD_TYPE>);
    pcl::StatisticalOutlierRemoval<POINTCLOUD_TYPE> sor;
    sor.setInputCloud(cloud_ptr);
    sor.setMeanK(30);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);
    cloud_voxel_sum = *cloud_filtered;
}
#endif

#if 1
    {
        // voxel_cout=0;
        pcl::PointCloud<POINTCLOUD_TYPE>::Ptr cloud_ptr(new pcl::PointCloud<POINTCLOUD_TYPE>(cloud_voxel_sum));
        pcl::PointCloud<POINTCLOUD_TYPE>::Ptr cloud_filtered(new pcl::PointCloud<POINTCLOUD_TYPE>);
        pcl::VoxelGrid<POINTCLOUD_TYPE> sor;
        sor.setInputCloud(cloud_ptr);
        sor.setLeafSize(0.01, 0.01, 0.01);
        sor.filter(*cloud_filtered);
        cloud_voxel_sum = *cloud_filtered;
    }
#endif

}
#endif

    std::cout <<"storage analyse: "<< cloud_voxel_sum.size() << ", " << cloud_voxel_sum_queue.size() << std::endl;
    return true;
    // pcl::toROSMsg(cloud_voxel_sum, cloud_map_voxel_sum);
}
bool save_pc_cb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
#if 0
    if(cloud_map.width*cloud_map.height == 0) {
        ROS_INFO("cloud_map is empty");
        return false;
    }
    ROS_INFO("save_cb");
    pcl::PointCloud<POINTCLOUD_TYPE> cloud;
    pcl::fromROSMsg(cloud_map, cloud);
    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    return true;
#endif
#if 0
    if(cloud_voxel_sum_queue.size() == 0) {
        ROS_INFO("cloud_voxel_sum is empty");
        return false;
    }
    ROS_INFO("save_cb");
    pcl::PointCloud<POINTCLOUD_TYPE> cloud;
    pcl::fromROSMsg(cloud_map, cloud);
    // cloud += cloud_map;
    for(uint32_t i = 0; i < cloud_voxel_sum_queue.size(); i++) {
        cloud += cloud_voxel_sum_queue.front();
        // cloud_voxel_sum.pop();
    }
    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    return true;
#endif

#if 1
    if(gen_sum_pc()){
        pcl::io::savePCDFileASCII("/home/hanglok/output/test_pcd.pcd", cloud_voxel_sum);
        return true;
    }
    else{
        return false;
    }
    
#endif
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle nh;


#ifdef USE_ROS_SUB
    tf_trans();

    ros::Subscriber sub_cloudmap = nh.subscribe<sensor_msgs::PointCloud2>("/rtabmap/cloud_map", 1, cloud_map_cb);
    ros::Subscriber sub_voxel = nh.subscribe<sensor_msgs::PointCloud2>("/voxel_cloud", 1, voxel_cb);
    ros::Publisher  pub_cloudmap = nh.advertise<sensor_msgs::PointCloud2>("/cloud_map_sum", 1);
    // ros::Subscriber sub_voxel = nh.subscribe<sensor_msgs::PointCloud2>("/voxel", 1, voxel_cb);
    ros::ServiceServer service_savemap = nh.advertiseService("save_pc", save_pc_cb);
    // ros::Publisher map_pub = nh.advertise<occupancy_map_msgs::OccupancyMap>("/octomap_binary", 1);
#else 

#endif

    ros::Rate rate(3);
    while (ros::ok())
    {
#ifdef USE_ROS_SUB
        gen_sum_pc();
        pcl::toROSMsg(cloud_voxel_sum, cloud_map_voxel_sum);
        pub_cloudmap.publish(cloud_map_voxel_sum);
#endif


        ros::spinOnce();
        ros::Duration(rate).sleep();
    }
    
    ros::spin();

    return 0;
}
