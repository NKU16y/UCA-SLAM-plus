// Declaration-only preview. Function implementations are not included.
#pragma once

/*
 * @Author: Shizhuo Yu
 * @Date: 2025-07-17 20:07:54
 * @Last Modified by: Shizhuo Yu
 * @Last Modified time: 2025-09-25 20:57:45
 */
#ifndef LIDARPARA_H
#define LIDARPARA_H
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <string>
namespace UCA_SLAM
{
class LidarPara
{
public:
    int scan_line;
    bool calib_laser;

    double down_sample_size;
    double down_sample_size_for_BA;
    double voxel_size;
    int max_layer;
    std::vector<int> layer_point_size;
    double plannar_threshold;
    int max_points_size;
    int max_cov_points_size;

    double ranging_cov;
    double angle_cov;

    LidarPara();
    void setLidarParams(const cv::FileStorage fSettings);
    void setGivenParams(const cv::FileStorage fSettings);
};

static inline double BaseLidarAngleCov(const LidarPara* const lp);

static inline double EffectiveLidarRangeCov(const LidarPara* const lp);

static inline double EffectiveLidarAngleCov(const LidarPara* const lp);

struct LidarVarianceCalibration
{
    double kappa_range;
    double kappa_azimuth;
    double kappa_elevation;
};

LidarVarianceCalibration RuntimeLidarVarianceCalibration();
void PrintLidarUncertaintyCalibration(const LidarPara* lp);
}
#endif

/*
 * @Author: Shizhuo Yu
 * @Date: 2025-07-17 20:07:14
 * @Last Modified by: Shizhuo Yu
 * @Last Modified time: 2025-07-23 22:04:52
 */
#ifndef LIDARHANDLERFUSION_H
#define LIDARHANDLERFUSION_H

#include "common.h"
#include <memory>
#include <opencv2/imgproc.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include "voxelMapUtil.hpp"

namespace UCA_SLAM
{
class LidarPara;
class LidarHandler
{
private:
    std::shared_ptr<LidarPara> mOwnedParameters;
public:

    double lidar_beg_time;
    PointCloud::Ptr allPoints, voxelPoints, visualPoints;
    PointCloud::Ptr surfPointsLessFlat;
    PointCloud::Ptr feats_undistort, feats_down_body;
    LidarPara *mpLP;
    cv::Mat depthFromLidar;
    Eigen::Matrix4d Tvl;
    Eigen::Matrix3d K;
    Eigen::MatrixXd depthUncertainty;
    LidarHandler();
    LidarHandler(const int &rows, const int &cols, const Eigen::Matrix4d &Tvl_, const Eigen::Matrix3d &K_);

    void setLidarPara(LidarPara *LP_);
    void getPointCloud(PointCloud::Ptr allPoints_);
    void getLidarTime(const double &timeStamp);

    void scanRegistrationForProjection();
    void projectToVisual();
    void projectToDepthImage();
private:
    void scanRegistrationImpl(bool prepare_dense_body);
};
}
#endif

