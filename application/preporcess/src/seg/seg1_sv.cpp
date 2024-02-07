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

 
};
