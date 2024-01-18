
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

class RGB_pts
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#if 0
    std::atomic<double> m_pos[3];
    std::atomic<double> m_rgb[3];
    std::atomic<double> m_cov_rgb[3];
    std::atomic<double> m_gray;
    std::atomic<double> m_cov_gray;
    std::atomic<int> m_N_gray;
    std::atomic<int> m_N_rgb;
#else
    double m_pos[ 3 ] = { 0 };
    float m_rgb[ 3 ] = { 0 };
    float intensity = 0;
    uint8_t bgr_intensity[3] = {0};
    double m_cov_rgb[ 3 ] = { 0 };
    double m_gray = 0;
    double m_cov_gray = 0;
    int    m_N_gray = 0;
    int    m_N_rgb = 0;
    int    m_pt_index = 0;
#endif
    TypeDefs::Vector2Type      m_img_vel;
    TypeDefs::Vector2Type      m_img_pt_in_last_frame;
    TypeDefs::Vector2Type      m_img_pt_in_current_frame;
    int        m_is_out_lier_count = 0;
    cv::Scalar m_dbg_color;
    double     m_obs_dis = 0;
    double     m_last_obs_time = 0;
    void       clear()
    {
        m_rgb[ 0 ] = 0;
        m_rgb[ 1 ] = 0;
        m_rgb[ 2 ] = 0;
        m_gray = 0;
        m_cov_gray = 0;
        m_N_gray = 0;
        m_N_rgb = 0;
        m_obs_dis = 0;
        m_last_obs_time = 0;
        int r = g_rng.uniform( 0, 256 );
        int g = g_rng.uniform( 0, 256 );
        int b = g_rng.uniform( 0, 256 );
        m_dbg_color = cv::Scalar( r, g, b );
        // m_rgb = TypeDefs::Vector3Type(255, 255, 255);
    };

    RGB_pts()
    {
        // m_pt_index = g_pts_index++;
        clear();
    };
    ~RGB_pts(){};

    void set_pos( const TypeDefs::Vector3Type &pos );
    TypeDefs::Vector3Type          get_pos();
    TypeDefs::Vector3Type          get_rgb();
    TypeDefs::Matrix3Type        get_rgb_cov();
    pcl::PointXYZI get_pt();
    void update_gray( const double gray, double obs_dis = 1.0 );
    int update_rgb( const TypeDefs::Vector3Type &rgb, const double obs_dis, const TypeDefs::Vector3Type obs_sigma, const double obs_time );
    void Intensity2Rgb( int colormap_type=cv::COLORMAP_JET);
  private:
    friend class boost::serialization::access;
    template < typename Archive >
    void serialize( Archive &ar, const unsigned int version )
    {
        ar &m_pos;
        ar &m_rgb;
        ar &m_pt_index;
        ar &m_cov_rgb;
        ar &m_gray;
        ar &m_N_rgb;
        ar &m_N_gray;
    }
};

}

