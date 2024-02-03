#include "seg1.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>//使用OMP需要添加的头文件
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/console/parse.h>
#include <pcl/features/boundary.h>
#include <pcl/common/centroid.h>
#include <pcl/surface/mls.h> //最小二乘 重采样平滑
#include <pcl/surface/poisson.h>  //泊松重建
#include <pcl/geometry/polygon_mesh.h> //MESH
#include <pcl/surface/gp3.h>  //贪心三角形

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/filters/statistical_outlier_removal.h>

// #include <pcl/io/polygon_mesh_io.h>

#include <vtkPolyLine.h>
// extern loglevel_e loglevel;


// #include "../base/common.hpp"
namespace PointCloud_process1{

    template class Segment<pcl::PointXYZINormal>;
    template class Segment<pcl::PointXYZI>;
    template class Segment<pcl::PointXYZ>;
    template class Segment<pcl::PointXYZRGB>;

    // template <typename PointT>
    // Segment<PointT>::Segment(){
    //     // cloud_all=new pcl::PointCloud<PointRGB>;
    //     // pcl::PointCloud<PointRGB>::Ptr tmp(new pcl::PointCloud<PointRGB>);
    //     // cloud_all=tmp;
    //     cloud_all = pcl::PointCloud<PointRGB>::Ptr(new pcl::PointCloud<PointRGB>);
    //     viewer=boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("planar segment"));

    // }

    // template <typename PointT>
    // auto Segment<PointT>::projection(typename PointCloudT::Ptr cloud)->cv::Mat {

    //     //pcl_viewer '/home/hanglok/pc_ws/src/application/preporcess/dataset/1.pcd'  -ax 5

    //     cv::Mat depthMap = cv::Mat::zeros(Lidar_FOV_Height, Lidar_FOV_Width, CV_32FC1);
    //         // 遍历点云，更新深度图

    // #define Lidar_Axis_Trans 0
    // #if Lidar_Axis_Trans
    //     Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    //     transform(0,0)=0;transform(0,1)=-1;transform(0,2)=0;transform(0,3)=0;
    //     transform(1,0)=1;transform(1,1)=0;transform(1,2)=0;transform(1,3)=0;
    //     transform(2,0)=0;transform(2,1)=0;transform(2,2)=1;transform(2,3)=0;
    //     transform(3,0)=0;transform(3,1)=0;transform(3,2)=0;transform(3,3)=1;
    //     // 应用变换
    //     typename PointCloudT::Ptr transformed_cloud(new POINTCLOUD());
    //     pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    //     for (const auto& point : transformed_cloud->points) {
    // #else
    //     for (const auto& point : cloud->points) {
    // #endif

    // #if Lidar_Axis_Trans
    //         // 假设点云已经在相机坐标系中
    //         int u = static_cast<int>(point.x * Lidar_Focal_Length / point.z + Lidar_FOV_Width / 2);
    //         int v = static_cast<int>(point.y * Lidar_Focal_Length / point.z + Lidar_FOV_Height / 2);
    // #else
    //         // 机器人坐标系
    //         int u = static_cast<int>(-point.y * Lidar_Focal_Length / point.x + Lidar_FOV_Width / 2);
    //         int v = static_cast<int>(-point.z * Lidar_Focal_Length / point.x + Lidar_FOV_Height / 2);
    // #endif
            
    //         if (u >= 0 && u < Lidar_FOV_Width && v >= 0 && v < Lidar_FOV_Height) {
    //             depthMap.at<float>(v, u) = point.x;
    //             std::cout << u << ", "<< v << std::endl;
    //         }
    //     }
    //     // 转换深度图为可视化格式（可选）
    //     cv::Mat displayMap;
    //     cv::normalize(depthMap, displayMap, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    //     cv::imwrite("/home/uestc/pc_ws/src/1.png",displayMap);
    //     cv::imshow("Depth Map", displayMap);
    //     cv::waitKey(0);
    //     return depthMap;

    // }
    // template <typename PointT>
    // auto Segment<PointT>::
    // template <typename PointT>
    // auto Segment<PointT>::backprojection(POINTCLOUD::Ptr cloud)->void {

    // }
    template <typename PointT>
    auto Segment<PointT>::load_pointcloud(std::string path)->void
    {

        if (pcl::io::loadPCDFile<PointT>(path, *cloud) == -1)
        {
            // log(logFATAL) << "Failed to load PCD file: " << input_file ;
            // logit(logFATAL) << "Failed to load PCD file " ;
            std::cout << "Failed to load PCD file " << std::endl;
            // return -1;

        }
    }

    template <typename PointT>
    auto Segment<PointT>::save_pointcloud(std::string path)->void
    {
        // // FILE *fpWrite=fopen("cloud_all.txt","w");//a续写，w清除后写入
        // FILE *fpWrite=fopen("/home/lqh/ros/pc_ws/src/pointcloud_dev/cloud_all.txt","w");//a续写，w清除后写入
        // for(std::size_t i=0; i< seg.cloud_all->size(); i++)
        //     fprintf(fpWrite,"%2.3f %2.3f %2.3f %d %d %d \n",seg.cloud_all->points[i].x,seg.cloud_all->points[i].y,seg.cloud_all->points[i].z,seg.cloud_all->points[i].r,seg.cloud_all->points[i].g,seg.cloud_all->points[i].b);
        // fclose(fpWrite);
        // cout << "cloud_all.txt 保存完毕！！！" << endl;

        // std::cout << "PLY saved as PCD file!" << std::endl;
        if (pcl::io::savePCDFileBinary(path, *cloud_all) == -1)
        {
            // log(logFATAL) << "Failed to load PCD file: " << input_file ;
            // logit(logFATAL) << "Failed to load PCD file " ;
            std::cout << "Failed to load PCD file " << std::endl;
            // return -1;

        }else{
            std::cout << "saved" << std::endl;
        
        }
    }
//重采样平滑点云


template <typename PointT>
auto Segment<PointT>::SmoothPointcloud()->void
{
    // pcl::PointCloud<PointT> mls_points;   //输出MLS
    // can't use it , TODO: fix it
	// 对点云重采样 
	std::cout<<"begin smooth: size " << cloud->size() << std::endl;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

	pcl::MovingLeastSquares<PointT, PointT> mls;  // 定义最小二乘实现的对象mls
	mls.setSearchMethod(tree);    // 设置KD-Tree作为搜索方法
	mls.setComputeNormals(false);  //设置在最小二乘计算中是否需要存储计算的法线
	mls.setInputCloud(cloud);        //设置待处理点云
	mls.setPolynomialOrder(2);             // 拟合2阶多项式拟合
	mls.setPolynomialFit(false);  // 设置为false可以 加速 smooth
	mls.setSearchRadius(0.03); // 单位m.设置用于拟合的K近邻半径
	mls.process(*cloud);        //输出
	std::cout << "success smooth, size: " << cloud->size() << std::endl;




    // // 创建MovingLeastSquares对象
    // pcl::MovingLeastSquares<PointT, PointT> mls;
    // mls.setInputCloud(cloud);
    // mls.setSearchRadius(0.03); // 设置搜索半径
    // mls.setPolynomialFit(true); // 启用多项式拟合
    // mls.setPolynomialOrder(1); // 设置多项式拟合阶数

    // // 执行平滑滤波
    // // pcl::PointCloud<pcl::PointXYZI>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointXYZI>);
    // mls.process(*cloud);
    // std::cout << "success smooth, size: " << cloud->size() << std::endl;


}

//贪心三角化算法得到Mesh

template <>
auto Segment<pcl::PointXYZRGB>::greedy_traingle_GenerateMesh(typename PointCloudT::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)->pcl::PolygonMesh::Ptr
{	
	// pcl::StopWatch time;

	std::cout << "begin  mesh..." << std::endl;

	// typename PointCloudT::Ptr cloud_out(new pcl::PointCloud<PointT>());
	
	// SmoothPointcloud(cloud_in, cloud_out);
	// EraseInvalidPoints(cloud_out);

	// 将点云位姿、颜色、法线信息连接到一起
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*cloud_in, *normals, *cloud_with_normals);

	//定义搜索树对象
    typename pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree->setInputCloud(cloud_with_normals);

	// 三角化
	pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;   // 定义三角化对象
	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh); //存储最终三角化的网络模型

	// 设置三角化参数
	gp3.setSearchRadius(0.1);  //设置搜索时的半径，也就是KNN的球半径
	gp3.setMu(2.5);  //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100);    //设置样本点最多可搜索的邻域个数，典型值是50-100

	gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到的三角形内角的最小的角度为10°
	gp3.setMaximumAngle(2 * M_PI / 3); // 设置三角化后得到的三角形内角的最大角度为120°

	gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
	gp3.setNormalConsistency(false);  //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

	gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
	gp3.setSearchMethod(tree);   //设置搜索方式
    // gp3.setInputNormals(normals);
	gp3.reconstruct(*triangles);  //重建提取三角化
	// cloud_with_normals->width = cloud_with_normals->height = 0;
	// std::cout << "success traingles, time(s) "<< time.getTimeSeconds() << std::endl;
	return triangles;
}

template <>
auto Segment<pcl::PointXYZI>::greedy_traingle_GenerateMesh(typename PointCloudT::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)->pcl::PolygonMesh::Ptr 
{	
	// pcl::StopWatch time;

	std::cout << "begin  mesh..." << std::endl;

	// typename PointCloudT::Ptr cloud_out(new pcl::PointCloud<PointT>());
	
	// SmoothPointcloud(cloud_in, cloud_out);
	// EraseInvalidPoints(cloud_out);
    // 删除无效点
    // std::vector<int> indices;
    // pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
	// 将点云位姿、颜色、法线信息连接到一起
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::concatenateFields(*cloud_in, *normals, *cloud_with_normals);

	//定义搜索树对象
    typename pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>);
    tree->setInputCloud(cloud_with_normals);

	// 三角化
	pcl::GreedyProjectionTriangulation<pcl::PointXYZINormal> gp3;   // 定义三角化对象
	// pcl::PolygonMesh triangles; //存储最终三角化的网络模型
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

	// 设置三角化参数
	gp3.setSearchRadius(0.3);  //设置搜索时的半径，也就是KNN的球半径
	gp3.setMu(2.5);  //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
	gp3.setMaximumNearestNeighbors(100);    //设置样本点最多可搜索的邻域个数，典型值是50-100

	gp3.setMinimumAngle(M_PI / 18); // 设置三角化后得到的三角形内角的最小的角度为10°
	gp3.setMaximumAngle(2 * M_PI / 3); // 设置三角化后得到的三角形内角的最大角度为120°

	gp3.setMaximumSurfaceAngle(M_PI / 4); // 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
	gp3.setNormalConsistency(false);  //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

	gp3.setInputCloud(cloud_with_normals);     //设置输入点云为有向点云
	gp3.setSearchMethod(tree);   //设置搜索方式
    // gp3.setInputNormals(normals);
	gp3.reconstruct(*triangles);  //重建提取三角化
	// cloud_with_normals->width = cloud_with_normals->height = 0;
	// std::cout << "success traingles, time(s) "<< time.getTimeSeconds() << std::endl;
	return triangles;
}
template <typename PointT>
auto Segment<PointT>::greedy_traingle_GenerateMesh(typename PointCloudT::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)->pcl::PolygonMesh::Ptr
{	
	std::cout << "not implement yet" << std::endl;
	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
	return triangles;
}



template <>
auto Segment<pcl::PointXYZRGB>::poisson_reconstruction_GenerateMesh(typename PointCloudT::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)->pcl::PolygonMesh::Ptr
{	
	std::cout << "begin  mesh..." << std::endl;
	// 将点云位姿、颜色、法线信息连接到一起
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*cloud_in, *normals, *cloud_with_normals);

	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh); //存储最终三角化的网络模型


    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    //poisson.setDegree(2);
    poisson.setDepth(8);
    poisson.setSolverDivide (6);
    poisson.setIsoDivide (6);

    poisson.setConfidence(false); 
    poisson.setManifold(false); 
    poisson.setOutputPolygons(false); 

    poisson.setInputCloud(cloud_with_normals);
    poisson.reconstruct(*mesh);

	return mesh;
}

template <>
auto Segment<pcl::PointXYZI>::poisson_reconstruction_GenerateMesh(typename PointCloudT::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)->pcl::PolygonMesh::Ptr 
{	
	std::cout << "begin  mesh..." << std::endl;
	// 将点云位姿、颜色、法线信息连接到一起
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::concatenateFields(*cloud_in, *normals, *cloud_with_normals);
	pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh); //存储最终三角化的网络模型
    pcl::Poisson<pcl::PointXYZINormal> poisson;
    //poisson.setDegree(2);
    poisson.setDepth(8);
    poisson.setSolverDivide (6);
    poisson.setIsoDivide (6);

    poisson.setConfidence(false); 
    poisson.setManifold(false); 
    poisson.setOutputPolygons(false); 

    poisson.setInputCloud(cloud_with_normals);
    poisson.reconstruct(*mesh);

	return mesh;
}
template <typename PointT>
auto Segment<PointT>::poisson_reconstruction_GenerateMesh(typename PointCloudT::Ptr cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normals)->pcl::PolygonMesh::Ptr
{	
	std::cout << "not implement yet" << std::endl;
	pcl::PolygonMesh::Ptr (new pcl::PolygonMesh);
	return triangles;
}

    template <typename PointT>
    auto Segment<PointT>::pointcloud_finetune()->void
    {


        // 下采样，同时保持点云形状特征
        pcl::VoxelGrid<PointT> downSampled;  //创建滤波对象
        downSampled.setInputCloud (cloud);            //设置需要过滤的点云给滤波对象
        downSampled.setLeafSize (0.01f, 0.01f, 0.01f);  //设置滤波时创建的体素体积为1cm的立方体
        downSampled.filter (*cloud);           //执行滤波处理，存储输出

            //直通滤波
        cloudPassThrough(cloud,"y",-20,20);
        cloudPassThrough(cloud,"x",5,50);
        cloudPassThrough(cloud,"z",-5,15);


        // SmoothPointcloud(); // have bug here

        // 删除无效点
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud);

        // Set the number of neighboring points to analyze for each point
        sor.setMeanK(80);//设置在进行统计时考虑查询点临近点数
        // Set the standard deviation multiplier threshold
        sor.setStddevMulThresh(1.0);//设置判断是否为离群点的阀值:均值+1.0*标准差

        // Create an empty container for the filtered points
        // typename PointCloudT::Ptr cloud_filtered(new PointCloudT);

        // Apply the filter
        sor.filter(*cloud);
    }
//直通滤波器对点云进行处理
    template <typename PointT>
    auto Segment<PointT>::cloudPassThrough(typename PointCloudT::Ptr cloud,const char *axis,int min,int max)->void
    {
        typename pcl::PassThrough<PointT> passthrough;
        passthrough.setInputCloud(cloud);//输入点云
        passthrough.setFilterFieldName(axis);//对z轴进行操作
        passthrough.setFilterLimits(min,max);//设置直通滤波器操作范围
        passthrough.filter(*cloud);//);//执行滤波

    }
    template <typename PointT>
    auto Segment<PointT>::normal_viz(typename PointCloudT::Ptr cloud_input)->void
    {
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals=clac_normal(cloud_input);
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        viewer->addPointCloud<PointT>(cloud_input, "cloud");

        // 添加法线到可视化
        viewer->addPointCloudNormals<PointT, pcl::Normal>(cloud_input, cloud_normals, 10, 0.05, "normals");
    }

    template <typename PointT>
    auto Segment<PointT>::clac_normal(typename PointCloudT::Ptr cloud_input)->pcl::PointCloud<pcl::Normal>::Ptr
    {
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
        pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;//OMP加速
        ne.setInputCloud(cloud_input);
        ne.setNumberOfThreads(10);//设置openMP的线程数
        // 估计法向量
        
        ne.setSearchMethod(tree);
        
        ne.setRadiusSearch(0.3);  // 设置法向量估计半径
        // ne.setKSearch(10);//点云法向计算时，需要所搜的近邻点大小
        ne.compute(*cloud_normals);

        return cloud_normals;
    }
    template <typename PointT>
    auto Segment<PointT>::Plane_fitting_cluster_growth(typename PointCloudT::Ptr cloud_input)->void
    {

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals=clac_normal(cloud_input);
        typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
        //区域增长聚类分割对象	< 点 ， 法 线 > 
        pcl::RegionGrowing<PointT, pcl::Normal> reg; 
        reg.setMinClusterSize (50);	//最小的聚类的点数
        reg.setMaxClusterSize (1000000);//最大的聚类的点数
        reg.setSearchMethod (tree);	//搜索方式
        reg.setNumberOfNeighbours (30); //设置搜索的邻域点的个数
        reg.setInputCloud (cloud_input);	//输入点
        //reg.setIndices (indices); 
        reg.setInputNormals (cloud_normals);	//输入的法线
        reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);//设置平滑度 法线差值阈值
        reg.setCurvatureThreshold (0.5);	//设置曲率的阀值

            // 获取聚类结果
        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        // 遍历和处理每个聚类
        int j = 0;    
        int m = 0;
        for (const auto& cluster : clusters) {
            typename PointCloudT::Ptr cloud_cluster(new PointCloudT);
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_cluster(new pcl::PointCloud<pcl::Normal>);
            for (const auto& idx : cluster.indices){
                cloud_cluster->points.push_back(cloud_input->points[idx]); // 添加点到当前簇
                cloud_normals_cluster->points.push_back(cloud_normals->points[idx]);
            }
                

            
            // *cloud_cluster=typename PointCloudT(cloud_input, cluster.indices);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            cloud_normals_cluster->width = cloud_normals_cluster->points.size();
            cloud_normals_cluster->height = 1;
            cloud_normals_cluster->is_dense = true;

            if (cloud_cluster->size()>80)
            {
                m++;

                cout<<"cloud_cluster->size()="<<cloud_cluster->size()<<endl;

                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cloud, centroid);
                std::cout << "Centroid: (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")" << std::endl;
#if 0
                // 可视化相关的代码
                R = rand() % (256) + 0;
                G = rand() % (256) + 0;
                B = rand() % (256) + 0;
                pcl::PointCloud<PointRGB>::Ptr cloud_cluster_vis(new pcl::PointCloud<PointRGB>);
                for(std::size_t k=0; k<cloud_cluster->size(); k++ )
                {
                    PointRGB thispoint;
                    thispoint.x=cloud_cluster->points[k].x;
                    thispoint.y=cloud_cluster->points[k].y;
                    thispoint.z=cloud_cluster->points[k].z;
                    thispoint.r=R;
                    thispoint.g=G;
                    thispoint.b=B;
                    cloud_cluster_vis->push_back(thispoint);
                    cloud_all->push_back(thispoint);
                }
                output_plane(cloud_cluster_vis,m);
#else   
                std::cout << "cloud_cluster->size()=" << cloud_cluster->size() << std::endl;
                std::cout << "cloud_normals_cluster->size()=" << cloud_normals_cluster->size() << std::endl;
                pcl::PolygonMesh::Ptr mesh =greedy_traingle_GenerateMesh(cloud_cluster,cloud_normals_cluster);
                output_plane(mesh,m);
#endif
            }
        }     
    }
    template <typename PointT>
    auto Segment<PointT>::Plane_fitting_cluster_growth_v(typename PointCloudT::Ptr cloud_input)->void
    {

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals=clac_normal(cloud_input);

        //区域增长聚类分割对象	< 点 ， 法 线 > 
        pcl::RegionGrowing<PointT, pcl::Normal> reg; 
        typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>);
        reg.setMinClusterSize (50);	//最小的聚类的点数
        reg.setMaxClusterSize (1000000);//最大的聚类的点数
        reg.setSearchMethod (tree);	//搜索方式
        reg.setNumberOfNeighbours (30); //设置搜索的邻域点的个数
        reg.setInputCloud (cloud_input);	//输入点
        //reg.setIndices (indices); 
        reg.setInputNormals (cloud_normals);	//输入的法线
        reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);//设置平滑度 法线差值阈值
        reg.setCurvatureThreshold (1.0);	//设置曲率的阀值

            // 获取聚类结果
        std::vector<pcl::PointIndices> clusters;
        reg.extract(clusters);

        // 遍历和处理每个聚类
        int j = 0;    
        int m = 0;
        for (const auto& cluster : clusters) {
            typename PointCloudT::Ptr cloud_cluster(new PointCloudT);

            for (const auto& idx : cluster.indices){
                cloud_cluster->points.push_back(cloud_input->points[idx]); // 添加点到当前簇
            }
                
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            if (cloud_cluster->size()>40)
            {
                m++;
                // 可视化相关的代码
                R = rand() % (256) + 0;
                G = rand() % (256) + 0;
                B = rand() % (256) + 0;
                cout<<"cloud_cluster->size()="<<cloud_cluster->size()<<endl;
                pcl::PointCloud<PointRGB>::Ptr cloud_cluster_vis(new pcl::PointCloud<PointRGB>);
                for(std::size_t k=0; k<cloud_cluster->size(); k++ )
                {
                    PointRGB thispoint;
                    thispoint.x=cloud_cluster->points[k].x;
                    thispoint.y=cloud_cluster->points[k].y;
                    thispoint.z=cloud_cluster->points[k].z;
                    thispoint.r=R;
                    thispoint.g=G;
                    thispoint.b=B;
                    cloud_cluster_vis->push_back(thispoint);
                    cloud_all->push_back(thispoint);
                }
                output_plane(cloud_cluster_vis,m);
            }
        }     
    }

    template <typename PointT>
    auto Segment<PointT>::Plane_fitting_cluster_eu(typename PointCloudT::Ptr cloud_input)->void
    {

        // 创建KdTree对象
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud_input);

        // 创建一个用于提取聚类的EuclideanClusterExtraction对象
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(0.8); // 2cm
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(50000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_input);
        ec.extract(cluster_indices);

        int m = 0;
        // 遍历每个簇
        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
            typename PointCloudT::Ptr cloud_cluster(new PointCloudT);
            for (const auto& idx : it->indices)
                cloud_cluster->points.push_back(cloud_input->points[idx]); // 添加点到当前簇
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
            // std::stringstream ss;
            // ss << "cloud_cluster_" << j << ".pcd";
            // pcl::io::savePCDFileASCII(ss.str(), *cloud_cluster); // 保存簇

            // if(cloud_cluster->size()<100){
            //     std::cout << "plane less than 100, exit."  << std::endl;
            //     break;
            // }
            if (cloud_cluster->size()>40)
            {
                m++;
                // 可视化相关的代码
                R = rand() % (256) + 0;
                G = rand() % (256) + 0;
                B = rand() % (256) + 0;
                cout<<"cloud_cluster->size()="<<cloud_cluster->size()<<endl;
                pcl::PointCloud<PointRGB>::Ptr cloud_cluster_vis(new pcl::PointCloud<PointRGB>);
                for(std::size_t k=0; k<cloud_cluster->size(); k++ )
                {
                    PointRGB thispoint;
                    thispoint.x=cloud_cluster->points[k].x;
                    thispoint.y=cloud_cluster->points[k].y;
                    thispoint.z=cloud_cluster->points[k].z;
                    thispoint.r=R;
                    thispoint.g=G;
                    thispoint.b=B;
                    cloud_cluster_vis->push_back(thispoint);
                    cloud_all->push_back(thispoint);
                }
                output_plane(cloud_cluster_vis,m);
            }
            j++;
        }


    }
    template <typename PointT>
    auto Segment<PointT>::point2mesh(typename PointCloudT::Ptr cloud,pcl::Normal normals)->pcl::PolygonMesh::Ptr
    {
        pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
        pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
        // typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        // tree->setInputCloud(cloud);
        // gp3.setSearchRadius(0.025);
        // gp3.setMu(2.5);
        // gp3.setMaximumNearestNeighbors(100);
        // gp3.setMaximumSurfaceAngle(M_PI / 4);
        // gp3.setMinimumAngle(M_PI / 18);
        // gp3.setMaximumAngle(2 * M_PI / 3);
        // gp3.setNormalConsistency(false);
        // gp3.setInputCloud(cloud);
        // gp3.setInputNormals(normals);
        // gp3.setSearchMethod(tree);
        // gp3.reconstruct(*mesh);
        return mesh;
    }    
    template <typename PointT>
    auto Segment<PointT>::Plane_fitting_normal(typename PointCloudT::Ptr cloud_input)->void
    {
        // pointcloud_size<PointT>(cloud);

        // 创建法向量估计对象
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals=clac_normal(cloud_input);
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);

        // logit(logDEBUG3) << cloud_normals->width <<","<< cloud_normals->height <<","<< sizeof(cloud_normals->points[0]);
        pointcloud_size<pcl::Normal>(cloud_normals);
        // pointcloud_size(cloud_normals);

        // 创建分割对象
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(DisThre);  // 设置距离阈值

        typename PointCloudT::Ptr cloud_plane(new  PointCloudT());


        // 设置输入点云和法向量
        seg.setInputCloud(cloud_input);
        seg.setInputNormals(cloud_normals);

        // 创建一个对象来存储分割结果
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        int m=0;
        while (cloud_input->size() > 100)
        {
            typename PointCloudT::Ptr cloud_p(new PointCloudT);
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
            std::cout << "remaining points: " << cloud_input->size() << std::endl;
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(cloud_input);
            extract.setIndices(inliers);
            extract.filter(*cloud_plane);//输出平面

            pcl::ExtractIndices<pcl::Normal> extract_normal;
            extract_normal.setInputCloud(cloud_normals);
            extract_normal.setIndices(inliers);
            // extract_normal.filter(*cloud_plane_normals);//输出平面
            if(cloud_plane->size()<100){
                std::cout << "plane less than 100, exit."  << std::endl;
                break;
            }
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
    // template <typename PointT>
    // auto Segment<PointT>::Plane_fitting(typename PointCloudT::Ptr cloud_input)->void
    // {

    //     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    //     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //     pcl::SACSegmentation<PointT> seg;
    //     typename PointCloudT::Ptr cloud_plane(new typename PointCloudT());

    //     seg.setOptimizeCoefficients(true);
    //     seg.setModelType(pcl::SACMODEL_PLANE);
    //     seg.setMethodType(pcl::SAC_RANSAC);
    //     seg.setMaxIterations(300);
    //     seg.setDistanceThreshold(DisThre);

    //     int m=0;
    //     while (cloud_input->size() > 100)
    //     {
    //         typename PointCloudT::Ptr cloud_p(new typename PointCloudT);
    //         seg.setInputCloud(cloud_input);
    //         seg.segment(*inliers, *coefficients);
    //         if (inliers->indices.size() == 0)
    //         {
    //             break;
    //         }
    //         pcl::ExtractIndices<PointT> extract;
    //         extract.setInputCloud(cloud_input);
    //         extract.setIndices(inliers);
    //         extract.filter(*cloud_plane);//输出平面

    //         if (cloud_plane->size()>40)
    //         {
    //             m++;
    //             // 可视化相关的代码
    //             R = rand() % (256) + 0;
    //             G = rand() % (256) + 0;
    //             B = rand() % (256) + 0;
    //             cout<<"cloud_cluster->size()="<<cloud_plane->size()<<endl;
    //             pcl::PointCloud<PointRGB>::Ptr cloud_cluster(new pcl::PointCloud<PointRGB>);
    //             for(std::size_t k=0; k<cloud_plane->size(); k++ )
    //             {
    //                 PointRGB thispoint;
    //                 thispoint.x=cloud_plane->points[k].x;
    //                 thispoint.y=cloud_plane->points[k].y;
    //                 thispoint.z=cloud_plane->points[k].z;
    //                 thispoint.r=R;
    //                 thispoint.g=G;
    //                 thispoint.b=B;
    //                 cloud_cluster->push_back(thispoint);
    //                 cloud_all->push_back(thispoint);
    //             }
    //             output_plane(cloud_cluster,m);
    //         }
    //         // 移除plane
    //         extract.setNegative(true);
    //         extract.filter(*cloud_p);
    //         *cloud_input = *cloud_p;
    //     }

    // }
    template <typename PointT>
    auto Segment<PointT>::output_plane(pcl::PointCloud<PointRGB>::Ptr cloud_plane,int begin)->void
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

    template <typename PointT>
    auto Segment<PointT>::output_plane(pcl::PolygonMesh::Ptr cloud_plane,int begin)->void
    {


        std::string mesh_id="mesh"+std::to_string(begin);
        viewer->addPolygonMesh(*cloud_plane, mesh_id);

        // 设置Mesh的渲染属性
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_FLAT, mesh_id);

        R = (rand() % (256) + 128)/255;
        G = (rand() % (256) + 128)/255;
        B = (rand() % (256) + 128)/255;

        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, R, G, B, mesh_id);

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.8, 0.8, mesh_id);
        // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "mesh");
    }
    // template <typename PointT>
    // auto Segment<PointT>::addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
    //                                     POINTCLOUD &adjacent_supervoxel_centers,
    //                                     std::string supervoxel_name,
    //                                     pcl::visualization::PCLVisualizer::Ptr & viewer)->void
    // {

    //     vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
    //     vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
    //     vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();
    //     //Iterate through all adjacent points, and add a center point to adjacent point pair
    //     for (auto adjacent_itr = adjacent_supervoxel_centers.begin (); adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
    //     {
    //         points->InsertNextPoint (supervoxel_center.data);
    //         points->InsertNextPoint (adjacent_itr->data);
    //     }
    //     // Create a polydata to store everything in
    //     vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
    //     // Add the points to the dataset
    //     polyData->SetPoints (points);
    //     polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
    //     for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
    //         polyLine->GetPointIds ()->SetId (i,i);

    //     cells->InsertNextCell (polyLine);
    //     // Add the lines to the dataset
    //     polyData->SetLines (cells);
    //     viewer->addModelFromPolyData (polyData,supervoxel_name);
    // }


    template <typename PointT>
    auto Segment<PointT>::get_cmd_parm(int argc, char** argv)->bool {
          if (argc < 2){
                pcl::console::print_error ("Syntax is: %s <pcd-file> \n "

                                            "--NT Dsables the single cloud transform \n"

                                            "-v <voxel resolution>\n-s <seed resolution>\n"

                                            "-c <color weight> \n-z <spatial weight> \n"

                                            "-n <normal_weight>\n", argv[0]);
                return false;
            }

            bool disable_transform = pcl::console::find_switch (argc, argv, "--NT");
            float voxel_resolution = 0.008f;
            bool voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
            if (voxel_res_specified)
                pcl::console::parse (argc, argv, "-v", voxel_resolution);


            bool seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
            if (seed_res_specified)
                pcl::console::parse (argc, argv, "-s", seed_resolution);


            float color_importance = 0.2f;

            if (pcl::console::find_switch (argc, argv, "-c"))
                pcl::console::parse (argc, argv, "-c", color_importance);


            float spatial_importance = 0.4f;
            if (pcl::console::find_switch (argc, argv, "-z"))
                pcl::console::parse (argc, argv, "-z", spatial_importance);


            float normal_importance = 1.0f;
            if (pcl::console::find_switch (argc, argv, "-n"))
                pcl::console::parse (argc, argv, "-n", normal_importance);
    }
    template <typename PointT>
    auto Segment<PointT>::init_display()->void {
        viewer->setWindowName("Plane Model Segmentation");
        viewer->setBackgroundColor(0, 0, 0);
    }

    template <typename PointT>
    auto Segment<PointT>::display()->void {
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }      
    }
};
