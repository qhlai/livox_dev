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
namespace PointCloud_process{

    Segment::Segment(){
        // cloud_all=new pcl::PointCloud<PointRGB>;
        // pcl::PointCloud<PointRGB>::Ptr tmp(new pcl::PointCloud<PointRGB>);
        // cloud_all=tmp;
        cloud_all = pcl::PointCloud<PointRGB>::Ptr(new pcl::PointCloud<PointRGB>);
        viewer=boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("planar segment"));

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
