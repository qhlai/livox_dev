
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
// #include "tools/tools_utils.hpp"


#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#define DisThre 0.03//平面分割阈值
#include "tools/tools_value_redefine.hpp"
#include "tools/tools_logger_lite.hpp"

#include <boost/thread/thread.hpp>

using POINTTYPE = pcl::PointXYZ;
using POINTNORMALTYPE = pcl::PointXYZINormal;
using POINTCLOUD = pcl::PointCloud<POINTTYPE>;
using PointRGB = pcl::PointXYZRGB;
uint8_t R;
uint8_t G;
uint8_t B;
pcl::PointCloud<PointRGB>::Ptr cloud_all(new pcl::PointCloud<PointRGB>);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("planar segment"));
void Plane_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input);
void output_plane(pcl::PointCloud<PointRGB>::Ptr cloud_plane,int begin);
void  cloudPassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const char *zhou,int min,int max);
FILE *fpWrite=fopen("cloud_all.txt","w");//a续写，w清除后写入

template <typename PointT>
void pointcloud_size(typename pcl::PointCloud<PointT>::Ptr cloud) {
    size_t numPoints = cloud->width * cloud->height;
    size_t pointSize = sizeof(cloud->points[0]);
    size_t totalSize = numPoints * pointSize;
    std::cout << "PointCloud 的内存占用为: " << totalSize << " 字节. " <<cloud->width <<","<< cloud->height <<","<<sizeof(cloud->points[0])<< std::endl;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_merge(pcl::PointCloud<pcl::PointXYZI>::Ptr& xyz_cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals_cloud){
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    // 设置 combined_cloud 的大小
    combined_cloud->resize(normals_cloud->size());

    // 将 normals_cloud 中的法线数据复制到 combined_cloud 中
    for (size_t i = 0; i < normals_cloud->size(); ++i) {
        combined_cloud->points[i].x = xyz_cloud->points[i].x;
        combined_cloud->points[i].y = xyz_cloud->points[i].y;
        combined_cloud->points[i].z = xyz_cloud->points[i].z;
        combined_cloud->points[i].intensity = xyz_cloud->points[i].intensity;
        combined_cloud->points[i].normal_x = normals_cloud->points[i].normal_x;
        combined_cloud->points[i].normal_y = normals_cloud->points[i].normal_y;
        combined_cloud->points[i].normal_z = normals_cloud->points[i].normal_z;
    }
    return combined_cloud;

}

//直通滤波器对点云进行处理
void  cloudPassThrough(POINTCLOUD::Ptr cloud,const char *axis,int min,int max)
{
     pcl::PassThrough<POINTTYPE> passthrough;
     passthrough.setInputCloud(cloud);//输入点云
     passthrough.setFilterFieldName(axis);//对z轴进行操作
     passthrough.setFilterLimits(min,max);//设置直通滤波器操作范围
     passthrough.filter(*cloud);//);//执行滤波

}


void Plane_fitting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input)
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

void output_plane(pcl::PointCloud<PointRGB>::Ptr cloud_plane,int begin)
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



loglevel_e loglevel = logDEBUG4;

int main(int argc, char** argv)
{
    // log_test();
    // dbg(42, "hello world", false);
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string input_file;
    std::string output_file;
    if (argc < 3)
    {
        std::cout << "Usage: pcd_to_ply input.pcd output.ply" << std::endl;

        logit(logWARN) << "defalut " ;
        std::string path_prefix="/home/uestc/ros/mid360_ws/src/application/preporcess/dataset/";

        input_file =path_prefix+"2.pcd";
        output_file =path_prefix+"3.pcd";
        // dbg(input_file, output_file);
        // return -1;
    }else{
        input_file = argv[1];
        output_file = argv[2];
    }

    POINTCLOUD::Ptr cloud(new POINTCLOUD);
    POINTCLOUD::Ptr cloud_inner(new POINTCLOUD);
    POINTCLOUD::Ptr cloud_outer(new POINTCLOUD);

    if (pcl::io::loadPCDFile<POINTTYPE>(input_file, *cloud) == -1)
    {
        // log(logFATAL) << "Failed to load PCD file: " << input_file ;
        logit(logFATAL) << "Failed to load PCD file " ;
        return -1;

    }
    
    viewer->setWindowName("Plane Model Segmentation");
    viewer->setBackgroundColor(0, 0, 0);
    //直通滤波
    cloudPassThrough(cloud,"y",-20,20);
    cloudPassThrough(cloud,"x",5,50);
    cloudPassThrough(cloud,"z",-5,15);
    Plane_fitting(cloud);
    for(std::size_t i=0; i< cloud_all->size(); i++)
        fprintf(fpWrite,"%2.3f %2.3f %2.3f %d %d %d \n",cloud_all->points[i].x,cloud_all->points[i].y,cloud_all->points[i].z,cloud_all->points[i].r,cloud_all->points[i].g,cloud_all->points[i].b);
    fclose(fpWrite);
    cout << "cloud_all.txt 保存完毕！！！" << endl;

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    // pointcloud_size<POINTTYPE>(cloud);
    // // 创建法向量估计对象
    // pcl::NormalEstimation<POINTTYPE, pcl::Normal> ne;
    // ne.setInputCloud(cloud);

    // // 创建一个用于存储法向量的PointCloud
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // // 估计法向量
    // pcl::search::KdTree<POINTTYPE>::Ptr tree(new pcl::search::KdTree<POINTTYPE>);
    // ne.setSearchMethod(tree);
    // ne.setRadiusSearch(0.03);  // 设置法向量估计半径
    // ne.compute(*cloud_normals);

    // logit(logDEBUG3) << cloud_normals->width <<","<< cloud_normals->height <<","<< sizeof(cloud_normals->points[0]);
    // pointcloud_size<pcl::Normal>(cloud_normals);
    // // pointcloud_size(cloud_normals);

    // // 创建分割对象
    // pcl::SACSegmentationFromNormals<POINTTYPE, pcl::Normal> seg;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(100);
    // seg.setDistanceThreshold(0.01);  // 设置距离阈值


    // // 设置输入点云和法向量
    // seg.setInputCloud(cloud);
    // seg.setInputNormals(cloud_normals);

    // // 创建一个对象来存储分割结果
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // // 分割平面
    // seg.segment(*inliers, *coefficients);

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


	// //根据分割结果填充平面内和平面外点云
	// cloud_inner->width = inliers->indices.size();
	// cloud_inner->height = 1;
	// cloud_inner->points.resize(cloud_inner->width * cloud_inner->height);
 
	// cloud_outer->width = cloud->points.size() - inliers->indices.size();
	// cloud_outer->height = 1;
	// cloud_outer->points.resize(cloud_outer->width * cloud_outer->height);

 
	// //创建一个数组，大小为点云总数,初始化为0
	// std::vector<int> p_flag(cloud->points.size());
 
	// //将平面内的点标记
	// for (size_t i = 0; i < inliers->indices.size(); ++i)
	// 	p_flag[inliers->indices[i]] = 1;
 
	// // for (size_t i = 0,j=0 ; i < (*cloud).points.size(); ++i)
	// // {
	// // 	//遍历，找出平面外的点
	// // 	if (p_flag[i] == 0)
	// // 	{
	// // 		cloud_outer->points[j].x = (*cloud).points[i].x;
	// // 		cloud_outer->points[j].y = (*cloud).points[i].y;
	// // 		cloud_outer->points[j].z = (*cloud).points[i].z;
	// // 		++j;
	// // 		std::cerr << "outer points index:\t" << i << "\t" << (*cloud).points[i].x << "\t"
	// // 			<< (*cloud).points[i].y << "\t"
	// // 			<< (*cloud).points[i].z << std::endl;
	// // 	}
	// // }
 
	// // //打印出平面外的点
	// // std::cerr << std::endl << "Outer points: " << cloud_outer->points.size() << std::endl;
	// // for (size_t i = 0; i < cloud_outer->points.size(); ++i)
	// // {
	// // 	std::cerr << "\t" << (*cloud_outer).points[i].x << "\t"
	// // 		<< (*cloud_outer).points[i].y << "\t"
	// // 		<< (*cloud_outer).points[i].z << std::endl;
	// // }
 
	// // //平面内的点
	// // std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
	// // for (size_t i = 0; i < inliers->indices.size(); ++i)
	// // {
	// // 	cloud_inner->points[i].x = (*cloud).points[inliers->indices[i]].x;
	// // 	cloud_inner->points[i].y = (*cloud).points[inliers->indices[i]].y;
	// // 	cloud_inner->points[i].z = (*cloud).points[inliers->indices[i]].z;
	// // 	std::cerr << "index:\t" << inliers->indices[i] << "\t" << (*cloud).points[inliers->indices[i]].x << "\t"
	// // 		<< (*cloud).points[inliers->indices[i]].y << "\t"
	// // 		<< (*cloud).points[inliers->indices[i]].z << std::endl;
	// // }
 
	// // //图形化显示
	// // //创建PCLVisualzer对象
	// // pcl::visualization::PCLVisualizer viewer("Plane Model Segmentation");
 
	// // int v1(1);
	// // int v2(2);
	
	// // //创建视角v1，v2
	// // viewer.createViewPort(0.0, 0.0, 0.5, 1.0,v1);
	// // viewer.createViewPort(0.5, 0.0, 1.0, 1.0,v2);
	// // //设置背景颜色为白色
	// // viewer.setBackgroundColor(255, 255, 255, v1);
	// // viewer.setBackgroundColor(255, 255, 255, v2);
	// // //添加直角坐标，放大1000倍
	// // viewer.addCoordinateSystem(1000,v1);
	// // viewer.addCoordinateSystem(1000,v2);
	
	// // //设置点云颜色
	// // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_origin(cloud, 255, 0, 0);
	// // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in(cloud_inner, 255, 0, 0);
	// // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out(cloud_outer, 0, 0, 255);
 
 
	// // viewer.addPointCloud(cloud, cloud_origin, "v1", v1);
	// // viewer.addPointCloud(cloud_outer, cloud_out, "v2", v2);
	// // viewer.addPointCloud(cloud_inner, cloud_in, "v3", v2);
 
	// // //设置点云的大小，point_size默认为1，这里设置为1000，突出显示
	// // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1000,"v1",v1);
	// // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1000,"v2",v2);
	// // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1000,"v3",v2);
	// // viewer.spin();
 
	return (0);

    // // 使用分割结果提取平面点云
    // POINTCLOUD::Ptr cloud_plane(new POINTCLOUD);
    // pcl::ExtractIndices<POINTTYPE> extract;
    // extract.setInputCloud(cloud);
    // extract.setIndices(inliers);
    // extract.setNegative(false);
    // extract.filter(*cloud_plane);

    // pcl::PCDWriter writer;
    // writer.write(output_file, *cloud_plane);
    
    // return 0;
}