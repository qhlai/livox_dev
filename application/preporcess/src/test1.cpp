
#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
// #include "tools/tools_utils.hpp"


#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件al_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include<pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/console/parse.h>
#include "base/common.hpp"
#include "seg/seg1.hpp"
// #include "seg/seg1.hpp"


#include "tools/tools_value_redefine.hpp"
// #include "tools/tools_logger_lite.hpp"

#include "map/rbg_pointcloud.hpp"
#include <boost/thread/thread.hpp>


// using POINTTYPE = pcl::PointXYZ;
// using POINTNORMALTYPE = pcl::PointXYZINormal;
// using POINTCLOUD = pcl::PointCloud<POINTTYPE>;
// using PointRGB = pcl::PointXYZRGB;

// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("planar segment"));

// https://blog.csdn.net/xx970829/article/details/123751443





// loglevel_e loglevel = logDEBUG4;

int main(int argc, char** argv)
{
    // log_test();
    // dbg(42, "hello world", false);
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string input_file;
    std::string output_file;

    if (argc < 3){
        pcl::console::print_error ("Syntax is: %s <pcd-file> \n "

                                    "--NT Dsables the single cloud transform \n"

                                    "-v <voxel resolution>\n-s <seed resolution>\n"

                                    "-c <color weight> \n-z <spatial weight> \n"

                                    "-n <normal_weight>\n", argv[0]);
        return false;
    }
    int pointcloud_type = 0;
    int display_type = 0; // 0 pointcloud  1 greedy_traingle mesh  2 poisson mesh
    pcl::console::parse (argc, argv, "-t", pointcloud_type);
    pcl::console::parse (argc, argv, "-i", input_file);
    pcl::console::parse (argc, argv, "-o", output_file);
    pcl::console::parse (argc, argv, "-p", display_type);
    bool disable_transform = pcl::console::find_switch (argc, argv, "--NT");
    // std::unique_ptr<PointCloud_process1::Segment> seg(new PointCloud_process1::Segment);
    // PointCloud_process1::Segment<pcl::PointXYZRGB> seg;
    // PointCloud_process1::Segment<pcl::PointXYZI> seg;
    // void* seg; // dirty code
    std::cout << "pointcloud_type: " << pointcloud_type << std::endl;
    if (pointcloud_type==0)
    {
        std::cout<<"PointCloud<pcl::PointXYZI> "<<std::endl;
        PointCloud_process1::Segment<pcl::PointXYZI> seg;
        seg.load_pointcloud(input_file);
        seg.pointcloud_finetune();

        // pcl::io::loadPointCloud()
        seg.init_display();
        seg.display_type=display_type;
        seg.disable_transform=disable_transform;
        seg.Plane_fitting_cluster_growth(seg.cloud);

        seg.display();

        seg.save_pointcloud(output_file);

    }else if (pointcloud_type==1)
    {
        std::cout << "pcl::PointXYZRGB" << std::endl;
        PointCloud_process1::Segment<pcl::PointXYZRGB> seg;
        seg.load_pointcloud(input_file);
        seg.pointcloud_finetune();
        // pcl::io::loadPointCloud()
        seg.init_display();
        seg.display_type=display_type;
        seg.disable_transform=disable_transform;
        seg.Plane_fitting_cluster_growth(seg.cloud);
        
        seg.display();

        seg.save_pointcloud(output_file);
    }
    
    // seg.load_pointcloud(input_file);

    // // pcl::io::loadPointCloud()
    // seg.init_display();

    // seg.Plane_fitting_cluster_growth(seg.cloud);

    // for(std::size_t i=0; i< seg.cloud_all->size(); i++)
    //     fprintf(fpWrite,"%2.3f %2.3f %2.3f %d %d %d \n",seg.cloud_all->points[i].x,seg.cloud_all->points[i].y,seg.cloud_all->points[i].z,seg.cloud_all->points[i].r,seg.cloud_all->points[i].g,seg.cloud_all->points[i].b);
    // fclose(fpWrite);
    // cout << "cloud_all.txt 保存完毕！！！" << endl;

    // seg.display();

	return (0);

}