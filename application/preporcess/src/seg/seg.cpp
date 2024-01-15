#include "seg.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
// extern loglevel_e loglevel;


// #include "../base/common.hpp"
namespace PointCloud_process{

    Segment::Segment(){
        // cloud_all=new pcl::PointCloud<PointRGB>;
        // pcl::PointCloud<PointRGB>::Ptr tmp(new pcl::PointCloud<PointRGB>);
        // cloud_all=tmp;
        cloud_all = pcl::PointCloud<PointRGB>::Ptr(new pcl::PointCloud<PointRGB>);
        viewer=boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("planar segment"));

    }


cv::Mat Segment::projection(POINTCLOUD::Ptr cloud){

    //pcl_viewer '/home/hanglok/pc_ws/src/application/preporcess/dataset/1.pcd'  -ax 5

    cv::Mat depthMap = cv::Mat::zeros(Lidar_FOV_Height, Lidar_FOV_Width, CV_32FC1);
        // 遍历点云，更新深度图

#define Lidar_Axis_Trans 0
#if Lidar_Axis_Trans
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform(0,0)=0;transform(0,1)=-1;transform(0,2)=0;transform(0,3)=0;
    transform(1,0)=1;transform(1,1)=0;transform(1,2)=0;transform(1,3)=0;
    transform(2,0)=0;transform(2,1)=0;transform(2,2)=1;transform(2,3)=0;
    transform(3,0)=0;transform(3,1)=0;transform(3,2)=0;transform(3,3)=1;
    // 应用变换
    POINTCLOUD::Ptr transformed_cloud(new POINTCLOUD());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    for (const auto& point : transformed_cloud->points) {
#else
    for (const auto& point : cloud->points) {
#endif

#if Lidar_Axis_Trans
        // 假设点云已经在相机坐标系中
        int u = static_cast<int>(point.x * Lidar_Focal_Length / point.z + Lidar_FOV_Width / 2);
        int v = static_cast<int>(point.y * Lidar_Focal_Length / point.z + Lidar_FOV_Height / 2);
#else
        // 机器人坐标系
        int u = static_cast<int>(-point.y * Lidar_Focal_Length / point.x + Lidar_FOV_Width / 2);
        int v = static_cast<int>(-point.z * Lidar_Focal_Length / point.x + Lidar_FOV_Height / 2);
#endif
        
        if (u >= 0 && u < Lidar_FOV_Width && v >= 0 && v < Lidar_FOV_Height) {
            depthMap.at<float>(v, u) = point.x;
            std::cout << u << ", "<< v << std::endl;
        }
    }
    // 转换深度图为可视化格式（可选）
    cv::Mat displayMap;
    cv::normalize(depthMap, displayMap, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imwrite("/home/uestc/pc_ws/src/1.png",displayMap);
    cv::imshow("Depth Map", displayMap);
    cv::waitKey(0);
    return depthMap;

}

void Segment::backprojection(POINTCLOUD::Ptr cloud){

}

//直通滤波器对点云进行处理
void Segment::cloudPassThrough(POINTCLOUD::Ptr cloud,const char *axis,int min,int max)
{
     pcl::PassThrough<POINTTYPE> passthrough;
     passthrough.setInputCloud(cloud);//输入点云
     passthrough.setFilterFieldName(axis);//对z轴进行操作
     passthrough.setFilterLimits(min,max);//设置直通滤波器操作范围
     passthrough.filter(*cloud);//);//执行滤波

}
void Segment::Plane_fitting_normal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input)
{
    // pointcloud_size<POINTTYPE>(cloud);

    // 创建法向量估计对象
    pcl::NormalEstimation<POINTTYPE, pcl::Normal> ne;
    ne.setInputCloud(cloud_input);

    // 创建一个用于存储法向量的PointCloud
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // 估计法向量
    pcl::search::KdTree<POINTTYPE>::Ptr tree(new pcl::search::KdTree<POINTTYPE>);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);  // 设置法向量估计半径
    ne.compute(*cloud_normals);


    // logit(logDEBUG3) << cloud_normals->width <<","<< cloud_normals->height <<","<< sizeof(cloud_normals->points[0]);
    pointcloud_size<pcl::Normal>(cloud_normals);
    // pointcloud_size(cloud_normals);

    // 创建分割对象
    pcl::SACSegmentationFromNormals<POINTTYPE, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(DisThre);  // 设置距离阈值

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());


    // 设置输入点云和法向量
    seg.setInputCloud(cloud_input);
    seg.setInputNormals(cloud_normals);

    // 创建一个对象来存储分割结果
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);


    // std::cout << "分割结果: " << inliers->indices.size() << " 个点被识别为平面" << std::endl;
	// //判断是否分割成功
	// if (inliers->indices.size() == 0)
	// {
	// 	PCL_ERROR("Could not estimate a planar model for the given dataset.");
	// 	return (-1);
	// }
	// std::cerr << std::endl << "Model coefficients: " << coefficients->values[0] << " "
	// 	<< coefficients->values[1] << " "
	// 	<< coefficients->values[2] << " "
	// 	<< coefficients->values[3] << std::endl << std::endl;


    int m=0;
    while (cloud_input->size() > 100)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_norm_p(new pcl::PointCloud<pcl::Normal>);
        // 设置输入点云和法向量
        seg.setInputCloud(cloud_input);
        seg.setInputNormals(cloud_normals);

        // 分割平面
        seg.segment(*inliers, *coefficients);


        if (inliers->indices.size() == 0)
        {
            PCL_ERROR("Could not estimate a planar model for the given dataset.");
            break;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_input);
        extract.setIndices(inliers);
        extract.filter(*cloud_plane);//输出平面

        pcl::ExtractIndices<pcl::Normal> extract_normal;
        extract_normal.setInputCloud(cloud_normals);
        extract_normal.setIndices(inliers);
        // extract_normal.filter(*cloud_plane_normals);//输出平面

        if (cloud_plane->size()>40)
        {
            m++;
            // 可视化相关的代码
            R = rand() % (256) + 0;
            G = rand() % (256) + 0;
            B = rand() % (256) + 0;
            cout<<"cloud_cluster->size()="<<cloud_plane->size()<<endl;
            pcl::PointCloud<PointRGB>::Ptr cloud_cluster(new pcl::PointCloud<PointRGB>);
            for(std::size_t k=0; k<cloud_plane->size(); k++ )
            {
                PointRGB thispoint;
                thispoint.x=cloud_plane->points[k].x;
                thispoint.y=cloud_plane->points[k].y;
                thispoint.z=cloud_plane->points[k].z;
                thispoint.r=R;
                thispoint.g=G;
                thispoint.b=B;
                cloud_cluster->push_back(thispoint);
                cloud_all->push_back(thispoint);
            }
            output_plane(cloud_cluster,m);
        }
        // 移除plane
        extract.setNegative(true);
        extract.filter(*cloud_p);
        *cloud_input = *cloud_p;
        
        extract_normal.setNegative(true);
        extract_normal.filter(*cloud_norm_p);
        *cloud_normals = *cloud_norm_p;

    }

}

void Segment::Plane_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input)
{

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(300);
    seg.setDistanceThreshold(DisThre);

    int m=0;
    while (cloud_input->size() > 100)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
        seg.setInputCloud(cloud_input);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            break;
        }
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_input);
        extract.setIndices(inliers);
        extract.filter(*cloud_plane);//输出平面

        if (cloud_plane->size()>40)
        {
            m++;
            // 可视化相关的代码
            R = rand() % (256) + 0;
            G = rand() % (256) + 0;
            B = rand() % (256) + 0;
            cout<<"cloud_cluster->size()="<<cloud_plane->size()<<endl;
            pcl::PointCloud<PointRGB>::Ptr cloud_cluster(new pcl::PointCloud<PointRGB>);
            for(std::size_t k=0; k<cloud_plane->size(); k++ )
            {
                PointRGB thispoint;
                thispoint.x=cloud_plane->points[k].x;
                thispoint.y=cloud_plane->points[k].y;
                thispoint.z=cloud_plane->points[k].z;
                thispoint.r=R;
                thispoint.g=G;
                thispoint.b=B;
                cloud_cluster->push_back(thispoint);
                cloud_all->push_back(thispoint);
            }
            output_plane(cloud_cluster,m);
        }
        // 移除plane
        extract.setNegative(true);
        extract.filter(*cloud_p);
        *cloud_input = *cloud_p;
    }

}

void Segment::output_plane(pcl::PointCloud<PointRGB>::Ptr cloud_plane,int begin)
{
    std::stringstream ss;
    ss << "plane_" << begin<< ".pcd";
    pcl::io::savePCDFileBinary(ss.str(), *cloud_plane);
    cout << ss.str() << "保存完毕！！！" << endl;

    std::string str;
    ss >> str;
    pcl::visualization::PointCloudColorHandlerCustom<PointRGB> color(cloud_plane, R, G, B);
    viewer->addPointCloud<PointRGB>(cloud_plane, color, str);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, str);
}

};
