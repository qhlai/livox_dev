
#pragma once

#include <set>
// C++
#include <atomic>
#include <unordered_set>
#include <mutex>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>

#include "../tools/tools_kd_hash.hpp"

namespace khmap{
cv::RNG g_rng = cv::RNG(0);

#define KH_MAP_DEBUG

enum Pts_Info_Tag {
    INTENSITY,
    GRAY,
    BGR,
    BGR_INTENSITY
};

struct Pts_Info_Taged {
    enum Pts_Info_Tag type;
    union 
    {
        uint8_t m_intensity;
        uint8_t m_gray;    
        uint8_t m_bgr[3];
        uint8_t m_bgr_intensity[3];
    };
};



class RGB_pts
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#if 0 //性能差20%左右
    std::atomic<double> m_pos[3];
    std::atomic<double> m_rgb[3];
    std::atomic<double> m_cov_rgb[3];
    std::atomic<double> m_gray;
    std::atomic<double> m_cov_gray;
    std::atomic<int> m_N_gray;
    std::atomic<int> m_N_rgb;
#else
    double m_pos[ 3 ] = { 0 };
    // float m_rgb[ 3 ] = { 0 };
    
    //  bgr_intensity[3] = {0};
    Pts_Info_Taged m_pts_info; // 节约内存空间 24Byte 到8Byte
    // double m_gray = 0;
    // double m_cov_gray = 0;
    // int    m_N_gray = 0;
    // int    m_N_rgb = 0;
    // uint32_t    m_pt_index = 0;
#endif
    void clear()
    {
        m_pos[0]=0;
        m_pos[1]=0;
        m_pos[2]=0;
        m_pts_info.type=BGR;
        m_pts_info.m_bgr[0]=0;
        m_pts_info.m_bgr[1]=0;
        m_pts_info.m_bgr[2]=0;

        // m_rgb[ 0 ] = 0;
        // m_rgb[ 1 ] = 0;
        // m_rgb[ 2 ] = 0;
        // m_gray = 0;
        // m_cov_gray = 0;
        // m_N_gray = 0;
        // m_N_rgb = 0;
        // m_obs_dis = 0;
        // m_last_obs_time = 0;
        // int r = g_rng.uniform( 0, 256 );
        // int g = g_rng.uniform( 0, 256 );
        // int b = g_rng.uniform( 0, 256 );
        // m_dbg_color = cv::Scalar( r, g, b );
        // m_rgb = TypeDefs::Vector3Type(255, 255, 255);
    };

    RGB_pts()
    {
        // m_pt_index = g_pts_index++;
        clear();
    };
    ~RGB_pts(){};

    // void set_pos( const TypeDefs::Vector3Type &pos );
    // TypeDefs::Vector3Type          get_pos();
    // TypeDefs::Vector3Type          get_rgb();
    // TypeDefs::Matrix3Type        get_rgb_cov();
    // pcl::PointXYZI get_pt();
    // void update_gray( const double gray, double obs_dis = 1.0 );
    // int update_rgb( const TypeDefs::Vector3Type &rgb, const double obs_dis, const TypeDefs::Vector3Type obs_sigma, const double obs_time );
    // void Intensity2Rgb( int colormap_type=cv::COLORMAP_JET);
  private:
    // friend class boost::serialization::access;
    // template < typename Archive >
    // void serialize( Archive &ar, const unsigned int version )
    // {
    //     ar &m_pos;
    //     ar &m_rgb;
    //     ar &m_pt_index;
    //     ar &m_cov_rgb;
    //     ar &m_gray;
    //     ar &m_N_rgb;
    //     ar &m_N_gray;
    // }
};
// class 
using RGB_pt_ptr = std::shared_ptr< RGB_pts >;


class RGB_Voxel
{
  public:
    std::vector< RGB_pt_ptr > m_pts_in_grid;
    double                   m_last_visited_time =0;
    RGB_Voxel() = default;
    ~RGB_Voxel() = default;
    void add_pt( RGB_pt_ptr &rgb_pts ) { m_pts_in_grid.push_back( rgb_pts ); }
};

struct Plane{
public:
    Eigen::Vector3f a;
    Eigen::Vector3f b;
    Eigen::Vector3f c;
    Eigen::Vector3f d;
};

struct Plane_1{
    float x, y, z;  // 位置
    float length, width; // 尺寸
    float nx, ny, nz;   // 法线向量
};

using RGB_voxel_ptr = std::shared_ptr< RGB_Voxel >;
using Voxel_set_iterator = std::unordered_set< std::shared_ptr< RGB_Voxel > >::iterator;

class Global_map
{
    uint8_t                   m_map_major_version = 0;
    uint8_t                   m_map_minor_version = 2;
    // std::vector< RGB_pt_ptr >                    m_rgb_pts_vec;
    Hash_map_3d< long, RGB_pt_ptr >   m_hashmap_3d_pts;
    Hash_map_3d< long, std::shared_ptr< RGB_Voxel > > m_hashmap_voxels;
    
    std::vector<std::unordered_set< std::shared_ptr< RGB_Voxel > >> m_voxels_recent_visited;
    std::vector< std::shared_ptr< RGB_pts > >          m_pts_last_hitted;
    double                                   m_minimum_pts_size = 0.05; // 5cm minimum distance.
    double                                   m_voxel_resolution = 0.1;
    double                                   m_maximum_depth_for_projection = 200;
    double                                   m_minimum_depth_for_projection = 3;

    // std::unordered_map< int, std::unordered_map< int, std::unordered_map< int, RGB_Voxel > > > m_map_3d_hash_map;
    // std::mutex m_mutex;
    // void       insert( const int &x, const int &y, const int &z, const RGB_pt_ptr &target )
    // {
    //     std::lock_guard< std::mutex > lock( m_mutex );
    //     m_map_3d_hash_map[ x ][ y ][ z ].add_pt( target );
    // }

    // int if_exist( const int &x, const int &y, const int &z )
    // {
    //     std::lock_guard< std::mutex > lock( m_mutex );
    //     if ( m_map_3d_hash_map.find( x ) == m_map_3d_hash_map.end() )
    //     {
    //         return 0;
    //     }
    //     else if ( m_map_3d_hash_map[ x ].find( y ) == m_map_3d_hash_map[ x ].end() )
    //     {
    //         return 0;
    //     }
    //     else if ( m_map_3d_hash_map[ x ][ y ].find( z ) == m_map_3d_hash_map[ x ][ y ].end() )
    //     {
    //         return 0;
    //     }
    //     return 1;
    // }

    // void clear()
    // {
    //     std::lock_guard< std::mutex > lock( m_mutex );
    //     m_map_3d_hash_map.clear();
    // }

    // int total_size()
    // {
    //     std::lock_guard< std::mutex > lock( m_mutex );
    //     int                            count = 0;
    //     for ( auto it : m_map_3d_hash_map )
    //     {
    //         for ( auto it_it : it.second )
    //         {
    //             for ( auto it_it_it : it_it.second )
    //             {
    //                 count++;
    //             }
    //         }
    //     }
    //     return count;
    // }
};




}

