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
#include <pcl/segmentation/supervoxel_clustering.h>


//VTK include needed for drawing graph lines

#include <vtkPolyLine.h>

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

    template <>
    auto Segment<pcl::PointXYZRGB>::Cluster_super_voxel(typename PointCloudT::Ptr cloud_input)->void
    {	
        typename pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (voxel_resolution, seed_resolution);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_inputa;
        cloud_inputa= PC_RGB2RGBA(cloud_input);
        std::cout<< "2" << std::endl;
        super.setInputCloud (cloud_inputa);
        super.setColorImportance (color_importance);
        super.setSpatialImportance (spatial_importance);
        super.setNormalImportance (normal_importance);
        std::map <std::uint32_t, typename  pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;
        pcl::console::print_highlight ("Extracting supervoxels!\n");
        std::cout<< "3" << std::endl;
        super.extract (supervoxel_clusters);
        pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
        viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");
          pcl::PointCloud<PointLT>::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();

        viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

        PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);

        *cloud_all+=*PC_RGBA2RGB(voxel_centroid_cloud);
        //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
        // viewer->addPointCloudNormals<PointNT> (sv_normal_cloud,1,0.05f, "supervoxel_normals");
        pcl::console::print_highlight ("Getting supervoxel adjacency\n");
        std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
        super.getSupervoxelAdjacency (supervoxel_adjacency);
        pcl::console::print_highlight ("Getted supervoxel adjacency\n");
        //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
        for (auto label_itr = supervoxel_adjacency.cbegin (); label_itr != supervoxel_adjacency.cend (); )
        {
            //First get the label
            std::uint32_t supervoxel_label = label_itr->first;
            //Now get the supervoxel corresponding to the label
            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);
            //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
            pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
            for (auto adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
            {
            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
            adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
            }
            //Now we make a name for this polygon
            std::stringstream ss;
            ss << "supervoxel_" << supervoxel_label;
            //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
            addSupervoxelConnectionsToViewer(supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
            //Move iterator forward to next label
            label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
        }


    }   
    template <typename PointT>
    auto Segment<PointT>::Cluster_super_voxel(typename PointCloudT::Ptr cloud_input)->void
    {

        std::cout<< "not imply yet" << std::endl;
 
    }

    //  template <typename PointT>
    // auto Segment<PointT>::addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
    //                              PointCloudT &adjacent_supervoxel_centers,
    //                               std::string supervoxel_name,
    //                               pcl::visualization::PCLVisualizer::Ptr & viewer)->void
    // {
    
    // }
     template <typename PointT>
auto Segment<PointT>::addSupervoxelConnectionsToViewer (pcl::PointXYZRGBA &supervoxel_center,
                                 pcl::PointCloud<pcl::PointXYZRGBA> &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  pcl::visualization::PCLVisualizer::Ptr & viewer)->void
    {
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();
    //Iterate through all adjacent points, and add a center point to adjacent point pair
    for (auto adjacent_itr = adjacent_supervoxel_centers.begin (); adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
    {
        points->InsertNextPoint (supervoxel_center.data);
        points->InsertNextPoint (adjacent_itr->data);
    }
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
    // Add the points to the dataset
    polyData->SetPoints (points);
    polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
    for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
        polyLine->GetPointIds ()->SetId (i,i);
    cells->InsertNextCell (polyLine);
    // Add the lines to the dataset
    polyData->SetLines (cells);
    viewer->addModelFromPolyData (polyData,supervoxel_name);
    }
 template <typename PointT>
auto Segment<PointT>::PC_RGB2RGBA (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input)->pcl::PointCloud<pcl::PointXYZRGBA>::Ptr{
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGBA>);

    cloud_output->resize(cloud_input->size());
    // 假设你已经从某处加载了点云数据到cloud对象中
    std::cout<< "size: " << cloud_output->size() << std::endl;
    // 遍历点云，将RGB转换为RGBA

    for (size_t i = 0; i < cloud_input->points.size(); ++i)
    {
        // 获取RGB颜色值
        uint8_t r = cloud_input->points[i].r;
        uint8_t g = cloud_input->points[i].g;
        uint8_t b = cloud_input->points[i].b;

        // 将RGB值设置为RGBA，Alpha通道默认为255（不透明）
        uint32_t rgba = (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);

        cloud_output->points[i].x = cloud->points[i].x;
        cloud_output->points[i].y = cloud->points[i].y;
        cloud_output->points[i].z = cloud->points[i].z;
        cloud_output->points[i].rgba = *reinterpret_cast<float*>(&rgba);
    }
    return cloud_output;
}
 template <typename PointT>
auto Segment<PointT>::PC_RGBA2RGB (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input)->pcl::PointCloud<pcl::PointXYZRGB>::Ptr{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output(new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud_output->resize(cloud_input->size());
    // 假设你已经从某处加载了点云数据到cloud对象中
    std::cout<< "size: " << cloud_output->size() << std::endl;
    // 遍历点云，将RGB转换为RGBA

    for (size_t i = 0; i < cloud_input->points.size(); ++i)
    {
        // pcl::PointXYZRGBA rgba_point = input_cloud->points[i];

        cloud_output->points[i].x = cloud_input->points[i].x;
        cloud_output->points[i].y = cloud_input->points[i].y;
        cloud_output->points[i].z = cloud_input->points[i].z;
        cloud_output->points[i].r = cloud_input->points[i].r;
        cloud_output->points[i].g = cloud_input->points[i].g;
        cloud_output->points[i].b = cloud_input->points[i].b;
    }
    return cloud_output;
}


};
