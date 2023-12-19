#include <ros/ros.h>
#include <rtabmap_msgs/MapData.h>
#include <rtabmap_msgs/NodeData.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void mapDataCallback(const rtabmap_msgs::MapData::ConstPtr& msg) {
    PointCloudT::Ptr cloud(new PointCloudT);
    for (const auto& node : msg->nodes) {
        // node.wordPts
        for (size_t i = 0; i < node.wordPts.size(); ++i) {
            const auto& point = node.wordPts[i];
            PointT pclPoint;
            pclPoint.x = point.x;
            pclPoint.y = point.y;
            pclPoint.z = point.z;
            pclPoint.r = point.r;
            pclPoint.g = point.g;
            pclPoint.b = point.b;
            cloud->push_back(pclPoint);
        }
    }
    // 打印点云大小
    std::cout << "点云中的点数量为: " << cloud->size() << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_data_to_pointcloud");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/rtabmap/mapData", 1, mapDataCallback);

    ros::spin();

    return 0;
}